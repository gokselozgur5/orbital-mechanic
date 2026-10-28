"""Exercise audit emission through production command/velocity callbacks."""
from concurrent.futures import Future
from dataclasses import asdict
import json
from types import SimpleNamespace as NS
import unittest

import test_supervisor_arm as arm_fixture


class NavigationAuditTests(unittest.TestCase):
    def setUp(self):
        self.fixture = arm_fixture.SupervisorArmTests(methodName='runTest')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.node = self.fixture.supervisor
        self.events, self.states, self.forwarded = [], [], []
        self.node.audit_pub.publish = lambda msg: self.events.append(json.loads(msg.data))
        self.node.nav_state_pub.publish = lambda msg: self.states.append(json.loads(msg.data))
        self.node.request_pub.publish = self.forwarded.append

    def test_request_and_held_arm_record_intent_without_velocity(self):
        token = self.fixture.reserve()
        self.fixture.arm(token)
        self.node.tick()
        self.assertEqual([e['sequence'] for e in self.events], [1, 2])
        self.assertEqual([e['command']['op'] for e in self.events], ['request', 'arm'])
        self.assertEqual([e['target'] for e in self.events], [asdict(token)]*2)
        self.assertEqual([e['forwarded_commands'] for e in self.events], [0, 0])
        self.assertEqual(self.states[-1]['audit_sequence'], 2)
        self.assertTrue(self.states[-1]['audit_ok'])
        self.assertEqual(self.forwarded, [])
        self.assertEqual(self.node.session.client.calls, [])

    def test_expired_health_early_return_and_malformed_command_are_audited(self):
        self.fixture.now += .3
        self.assertFalse(self.fixture.command('request', waypoint='pump'))
        self.assertFalse(self.events[-1]['reply']['accepted'])
        self.assertEqual(self.events[-1]['command_sequence'], 1)
        self.node.on_command(arm_fixture.Message(data='not json'))
        self.assertIsNone(self.events[-1]['command'])
        self.assertEqual(self.events[-1]['command_sequence'], 1)
        self.assertEqual(self.events[-1]['sequence'], 2)

    def test_forwarded_velocity_has_dispatch_target_stamp_and_counter(self):
        token = self.fixture.reserve()
        self.fixture.arm(token).set_result(NS(success=True))
        self.fixture.guard(latched=False)
        self.node.tick()
        result = Future()
        self.node.session.client.futures[-1].set_result(NS(
            accepted=True, get_result_async=lambda: result, cancel_goal_async=Future))
        self.assertEqual(self.node.session.state, 'running')
        original = self.node.get_publishers_info_by_topic
        self.node.get_publishers_info_by_topic = lambda topic: (
            [NS(node_name='controller_server', node_namespace='/', topic_type='geometry_msgs/msg/TwistStamped')]
            if topic == '/navigation/raw_cmd_vel' else original(topic))
        stamp = self.node.get_clock().now().nanoseconds
        msg = NS(header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp % 10**9), frame_id='base_footprint'),
                 twist=NS(linear=NS(x=.1, y=0., z=0.), angular=NS(x=0., y=0., z=.2)))
        self.node.on_velocity(msg)
        self.assertEqual(len(self.forwarded), 1)
        event = self.events[-1]
        self.assertEqual(event['event'], 'velocity_forwarded')
        self.assertEqual(event['target'], asdict(token))
        self.assertEqual(event['velocity_stamp_ns'], stamp)
        self.assertEqual(event['velocity'], [.1, 0., 0., 0., 0., .2])
        self.assertEqual((event['command_sequence'], event['forwarded_commands']), (2, 1))
        self.assertTrue(self.fixture.command('cancel', target=asdict(token)))
        self.node.on_velocity(msg)
        self.assertEqual(len(self.forwarded), 1)
        self.assertEqual(self.events[-1]['event'], 'command_reply')
        self.assertEqual(self.events[-1]['forwarded_commands'], 1)

    def test_audit_failure_cannot_interrupt_cancel_and_is_sticky_in_state(self):
        token = self.fixture.reserve()
        def broken(msg):
            raise RuntimeError('transport failed')
        self.node.audit_pub.publish = broken
        self.assertTrue(self.fixture.command('cancel', target=asdict(token)))
        self.assertEqual(self.node.session.outcome, 'not_dispatched')
        self.node.tick()
        self.assertFalse(self.states[-1]['audit_ok'])
        self.assertEqual(self.states[-1]['audit_sequence'], 2)


if __name__ == '__main__':
    unittest.main()
