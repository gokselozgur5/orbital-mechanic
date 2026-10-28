"""Execute production arm callbacks with controlled ROS transport and clocks.

These tests cover callback ordering, not DDS delivery or physical stopping.
The native counterpart is scripts/check-navigation-arm.py.
"""
from concurrent.futures import Future
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch
import uuid


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Goal:
    def __init__(self):
        self.pose = NS(header=NS(), pose=NS(position=NS(), orientation=NS()))


class Client:
    def __init__(self, *, immediate=False):
        self.immediate = immediate
        self.calls = []
        self.futures = []

    def service_is_ready(self):
        return True

    server_is_ready = service_is_ready

    def call_async(self, request):
        self.calls.append(request)
        future = Future()
        self.futures.append(future)
        if self.immediate:
            future.set_result(NS(success=True))
        return future

    send_goal_async = call_async


class Node:
    def __init__(self, name, **kwargs):
        self.name = name
        self.sim_time = 100.

    def create_client(self, service_type, path):
        return Client(immediate=path == '/motion/stop')

    def create_timer(self, *args, **kwargs):
        return NS()

    def create_subscription(self, *args, **kwargs):
        return NS()

    def create_publisher(self, *args, **kwargs):
        return NS(publish=lambda message: None)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=round(self.sim_time * 1e9), to_msg=lambda: NS()))

    def get_parameter(self, name):
        return NS(value=True)

    def declare_parameter(self, *args, **kwargs):
        pass

    def get_name(self):
        return self.name

    def get_namespace(self):
        return '/'

    def get_publishers_info_by_topic(self, topic):
        if topic == '/navigation/raw_cmd_vel':
            return []
        kind = 'geometry_msgs/msg/TwistStamped' if topic == '/motion/request' else 'std_msgs/msg/String'
        return [NS(node_name=self.name, node_namespace='/', topic_type=kind)]


def load_production():
    """Keep the policies, health interlock and action session real; fake ROS only."""
    exports = {
        'rclpy.node': {'Node': Node},
        'rclpy.clock': {'Clock': lambda **kwargs: NS(), 'ClockType': NS(STEADY_TIME=1, ROS_TIME=2)},
        'rclpy.time': {'Time': Message},
        'rclpy.parameter': {'Parameter': lambda *args, **kwargs: NS()},
        'rclpy.action': {'ActionClient': lambda *args: Client()},
        'rcl_interfaces.msg': {'ParameterDescriptor': Message},
        'geometry_msgs.msg': {'TwistStamped': Message},
        'std_msgs.msg': {'String': Message},
        'std_srvs.srv': {'Trigger': NS(Request=Message)},
        'nav2_msgs.action': {'NavigateToPose': NS(Goal=Goal)},
        'action_msgs.msg': {'GoalStatus': NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)},
        'tf2_ros': {'Buffer': lambda **kwargs: NS(), 'TransformException': LookupError,
                   'TransformListener': lambda *args: NS()},
    }
    modules = {}
    for name, attributes in exports.items():
        parent = name.split('.')[0]
        modules.setdefault(parent, ModuleType(parent))
        module = ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    root = Path(__file__).resolve().parents[1] / 'orbital'
    with patch.dict('sys.modules', modules):
        for stem in ('ros_health_interlock', 'ros_navigation_action', 'ros_navigation_supervisor'):
            name = 'orbital.' + stem
            spec = importlib.util.spec_from_file_location(name, root / (stem + '.py'))
            module = importlib.util.module_from_spec(spec)
            # The outer patch restores all imports together. A nested restoration
            # would create two different Target class identities across imports.
            sys.modules[name] = module
            spec.loader.exec_module(module)
        return module.NavigationSupervisor


class SupervisorArmTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.clock = patch('time.monotonic', lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.supervisor = load_production()({'pump': (2., 1.8, 0.)})
        self.run = str(uuid.uuid4())
        self.health_sequence = 0
        for _ in range(3):
            self.health()
            self.guard(latched=True)
            self.supervisor.tick()
        self.assertFalse(self.supervisor.lease.stop_required)

    def health(self, stationary=True, *, dt=.01):
        self.now += dt
        self.supervisor.sim_time += dt
        self.health_sequence += 1
        self.supervisor.on_health(Message(data=json.dumps({
            'schema_version': 1, 'run_id': self.run, 'epoch': 0,
            'sequence': self.health_sequence, 'sim_time': self.supervisor.sim_time,
            'healthy': True, 'stationary': stationary, 'reason': 'valid',
        })))

    def guard(self, *, latched):
        self.supervisor.on_guard(Message(data=json.dumps({'latched': latched})))

    def command(self, op, **kwargs):
        node = self.supervisor
        node.on_command(Message(data=json.dumps({
            'run_id': node.policy.run_id, 'sequence': node.commands.sequence + 1,
            'op': op, **kwargs,
        })))
        return node.last_reply['accepted']

    def reserve(self):
        self.assertTrue(self.command('request', waypoint='pump'))
        return self.supervisor.session.current

    def arm(self, target):
        self.assertTrue(self.command('arm', target=asdict(target)))
        self.assertEqual(len(self.supervisor.arm_client.calls), 1)
        return self.supervisor.arm_future

    def test_expired_health_cannot_arm_when_periodic_tick_is_delayed(self):
        target = self.reserve()
        self.now += .26  # Health expires before the policy's independent .5 s limit.
        self.guard(latched=True)
        self.assertFalse(self.command('arm', target=asdict(target)))
        self.assertEqual(self.supervisor.arm_client.calls, [])
        self.assertEqual(self.supervisor.session.client.calls, [])
        self.assertTrue(self.command('cancel', target=asdict(target)))

    def test_expired_health_cannot_reserve_new_target(self):
        self.now += .26
        self.guard(latched=True)
        self.assertFalse(self.command('request', waypoint='pump'))
        self.assertIsNone(self.supervisor.session.current)

    def test_moving_feedback_refuses_arm_without_guard_rpc(self):
        target = self.reserve()
        self.health(stationary=False)
        self.assertFalse(self.command('arm', target=asdict(target)))
        self.assertEqual(self.supervisor.arm_client.calls, [])

    def test_pending_arm_remembers_motion_even_if_feedback_recovers_before_tick(self):
        target = self.reserve()
        future = self.arm(target)
        self.health(stationary=False)
        self.health(stationary=True)
        future.set_result(NS(success=True))
        self.guard(latched=False)
        self.supervisor.tick()
        self.assertEqual(self.supervisor.session.client.calls, [])
        self.assertFalse(self.supervisor.policy.armed)
        self.assertFalse(self.command('arm', target=asdict(target)))

    def test_guard_response_with_moving_feedback_cannot_dispatch(self):
        target = self.reserve()
        future = self.arm(target)
        self.health(stationary=False)
        future.set_result(NS(success=True))
        self.guard(latched=False)
        self.supervisor.tick()
        self.assertEqual(self.supervisor.session.client.calls, [])

    def test_dispatch_requires_response_and_fresh_unlatched_guard(self):
        target = self.reserve()
        future = self.arm(target)
        self.guard(latched=False)
        self.supervisor.tick()
        self.assertEqual(self.supervisor.session.client.calls, [])
        future.set_result(NS(success=True))
        self.guard(latched=True)
        self.supervisor.tick()
        self.assertEqual(self.supervisor.session.client.calls, [])
        self.guard(latched=False)
        self.supervisor.tick()
        self.assertEqual(len(self.supervisor.session.client.calls), 1)
        self.assertEqual(self.supervisor.session.state, 'dispatching')

    def test_unresolved_guard_response_retains_target_after_deadline(self):
        target = self.reserve()
        future = self.arm(target)
        for _ in range(6):
            self.health(dt=.1)
            self.guard(latched=True)
            self.supervisor.tick()
        self.assertIs(self.supervisor.arm_future, future)
        self.assertEqual(self.supervisor.session.current, target)
        self.assertFalse(self.command('request', waypoint='pump'))
        future.set_result(NS(success=True))
        for _ in range(5):
            self.health(dt=.05)
            self.guard(latched=True)
            self.supervisor.tick()
        self.assertEqual(self.supervisor.session.client.calls, [])
        self.assertIsNone(self.supervisor.session.current)
        self.assertFalse(self.command('arm', target=asdict(target)))
        self.assertTrue(self.command('request', waypoint='pump'))


if __name__ == '__main__':
    unittest.main()
