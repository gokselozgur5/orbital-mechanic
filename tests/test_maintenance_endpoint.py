"""Actual station/guard/navigation/capture ownership over controlled transport."""
from concurrent.futures import Future
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch
import uuid

from orbital import inspection_owner, ros_machine_telemetry, ros_maintenance
from orbital.inspection_artifacts import read_capture
from orbital.inspection_protocol import canonical
from orbital.maintenance_protocol import result, state
from orbital.isolation_trace import IsolationTrace, TOPICS as TRACE_TOPICS, verify_isolation
from orbital.trajectory import TraceWriter
from inspection_transport_fixture import SharedClock, transport
from test_isolation_trace import sources as trace_sources
import test_map_binding as map_fixture
import test_supervisor_arm as nav_fixture

ROOT = Path(__file__).resolve().parents[1]


class MaintenanceEndpointTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)
        for name in ('config/robot.urdf', 'config/bridge.yaml'):
            shutil.copyfile(ROOT/name, self.root/name)

    def run_endpoint(self, mode='success', *, retained=False, reset=False):
        host_started = time.monotonic()
        clock = SharedClock(self.root, create=True)
        self.addCleanup(clock.close)
        origin, now = clock.now(), clock.now
        h = transport(self.root, origin, clock, writer=True)
        base_create, base_spin = h.modules['rclpy'].create_node, h.modules['rclpy'].spin_once
        base = base_create()
        callbacks, services, publishers, outputs, timers = {}, {}, {}, {}, []
        station = None
        trace = trace_writer = None
        mission_id, action_id = uuid.uuid4().hex, uuid.uuid4().hex
        workers = []
        request = None
        cancelled = False
        last_tick = None
        real_popen = subprocess.Popen
        real_station = ros_maintenance.StationMaintenance

        def deliver(topic, message):
            if trace is not None and topic in TRACE_TOPICS:
                trace.receive(topic, round(now()*1e9), h.sim, message.data)
                if trace.ready_at is None:
                    trace.ready(round(now()*1e9), h.sim)
            for callback in list(callbacks.get(topic, [])): callback(message)

        class Node(type(base)):
            def __init__(self, name, **kw): self.name = name; self.parameters = {'use_sim_time': True}
            def get_name(self): return self.name
            def get_namespace(self): return '/'
            def declare_parameter(self, name, value, *a, **kw): self.parameters.setdefault(name, value)
            def get_parameter(self, name): return NS(value=self.parameters[name])
            def create_subscription(self, kind, topic, callback, qos):
                callbacks.setdefault(topic, []).append(callback)
                if topic in ('/navigation/health', '/odom', '/map', '/camera/image_raw', '/camera/camera_info'):
                    base.create_subscription(kind, topic, lambda msg:deliver(topic, msg), qos)
                return NS(topic=topic)
            def create_publisher(self, kind, topic, qos):
                type_name = ('geometry_msgs/msg/'+kind.__name__ if kind.__name__ in ('Twist', 'TwistStamped')
                             else 'std_msgs/msg/'+('Float32' if topic == '/station/bus_voltage' else 'String'))
                publishers.setdefault(topic, []).append(NS(node_name=self.name, node_namespace='/',
                    topic_type=type_name, endpoint_gid=[9]*24))
                outputs.setdefault(topic, [])
                def publish(message):
                    if topic == '/maintenance/result':
                        self_test.assertTrue(all(child.poll() is not None for child in workers))
                    value = json.loads(message.data) if isinstance(getattr(message, 'data', None), str) else message
                    outputs[topic].append(value)
                    if topic == '/station/telemetry':
                        temporary = self_test.root/'runtime/endpoint-test-telemetry.next'
                        temporary.write_text(message.data)
                        temporary.replace(self_test.root/'runtime/endpoint-test-telemetry.json')
                    deliver(topic, message)
                return NS(publish=publish)
            def get_publishers_info_by_topic(self, topic):
                # Match the child capture's source IDs for the shared sensor and
                # station publishers. Maintenance/control peers are real owners here.
                if topic == '/station/telemetry': return base.get_publishers_info_by_topic(topic)
                if topic in publishers: return publishers[topic]
                return base.get_publishers_info_by_topic(topic)
            def create_service(self, kind, path, callback): services[path] = callback; return NS()
            def create_client(self, kind, path):
                if path not in ('/motion/stop', '/motion/reset_stop'):
                    return base.create_client(kind, path)
                def call(request):
                    future = Future()
                    future.set_result(services[path](request, NS()))
                    return future
                return NS(service_is_ready=lambda:path in services, call_async=call)
            def create_timer(self, period, callback, **kw):
                value = NS(period=period, callback=callback, next=0., name=self.name)
                timers.append(value); return value
            def destroy_timer(self, timer): timers.remove(timer)

        self_test = self
        class Twist:
            def __init__(self):
                self.linear = NS(x=0., y=0., z=0.); self.angular = NS(x=0., y=0., z=0.)
        class TwistStamped:
            def __init__(self): self.header = NS(); self.twist = Twist()
        def module(name, **values):
            value = ModuleType(name); value.__dict__.update(values); h.modules[name] = value
        module('rclpy.node', Node=Node)
        h.modules['rclpy.clock'].Clock = lambda **kw:NS()
        h.modules['rclpy.clock'].ClockType.STEADY_TIME = 2
        h.modules['rclpy.qos'].ReliabilityPolicy.BEST_EFFORT = 'best_effort'
        h.modules['std_msgs.msg'].Float32 = NS
        module('rclpy.executors', ExternalShutdownException=RuntimeError)
        module('geometry_msgs.msg', Twist=Twist, TwistStamped=TwistStamped)
        module('std_srvs.srv', Trigger=NS(Request=NS))
        module('rcl_interfaces.msg', ParameterDescriptor=NS)
        action_clients = []
        def action_client(*a):
            value = nav_fixture.Client(); value.destroy = lambda:None
            action_clients.append(value); return value
        module('rclpy.action', ActionClient=action_client)
        module('nav2_msgs.action', NavigateToPose=NS(Goal=nav_fixture.Goal))
        module('action_msgs.msg', GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6))
        h.modules['rclpy'].create_node = lambda name, **kw:Node(name, **kw)
        h.modules['rclpy'].shutdown = h.modules['rclpy'].try_shutdown

        def load(name, path):
            spec = importlib.util.spec_from_file_location(name, path)
            value = importlib.util.module_from_spec(spec); sys.modules[name] = value
            spec.loader.exec_module(value)
            value.time = NS(monotonic=now)
            return value

        def station_owner(*args, **kw):
            nonlocal station, trace, trace_writer
            station = real_station(*args, **kw)
            if retained:
                registry = station.owner.model.registry
                trace_writer = TraceWriter(self.root, 'composed-isolation')
                self.addCleanup(lambda:trace_writer.stream.close() if not trace_writer.closed else None)
                trace = IsolationTrace(registry, {'run_id': station.owner.run_id,
                    'mission_id': mission_id, 'action_id': action_id, 'machine_id': 'pump'}, writer=trace_writer)
                trace.begin(round(now()*1e9), h.sim, trace_sources(registry))
            return station

        def launch(args, **kwargs):
            self.assertEqual(Path(args[1]).name, 'inspection-worker.py')
            kwargs['env'].update(ORBITAL_TEST_ORIGIN=str(origin), ORBITAL_TEST_SHARED_CLOCK='1',
                                 ORBITAL_TEST_STATION_PUBLICATIONS='1', ORBITAL_TEST_EXIT_DELAY='.05',
                                 ORBITAL_TEST_STORAGE_DELAY='.35' if mode == 'slow_storage' else '0')
            child = real_popen([args[0], str(ROOT/'tests/inspection_transport_fixture.py'), *args[2:]], **kwargs)
            workers.append(child); return child

        def spin(node, timeout_sec):
            nonlocal request, cancelled, last_tick
            base_spin(node, timeout_sec)
            if h.sim != last_tick:
                last_tick = h.sim
                guard.tick(); supervisor.tick()
                for timer in list(timers):
                    if timer.name == 'station_telemetry' and now() >= timer.next:
                        timer.callback(); timer.next = now()+timer.period
            statuses = outputs.get('/maintenance/state', [])
            if (statuses and (request is None or (reset and request['op'] == 'isolate'
                    and len(outputs.get('/maintenance/result', [])) == 1)) and statuses[-1]['ready']
                    and (trace is None or trace.ready_at is not None)):
                status = statuses[-1]
                second = request is not None
                request = {'schema_version': 1, 'run_id': status['run_id'], 'mission_id': mission_id,
                    'action_id': uuid.uuid4().hex if second else action_id,
                    'command_id': uuid.uuid4().hex, 'op': 'reset' if second else 'isolate',
                    'arguments': {'machine_id': 'pump', 'station_run_id': status['station_run_id'],
                                  'control_revision': status['control_revision'], 'timeout_s': 15.}}
                if second: request['arguments']['isolation_action_id'] = action_id
                deliver('/maintenance/command', NS(data=canonical(request)))
            if station is not None and station.owner.active is not None:
                action = station.owner.active
                if action.inspector is not None and action.inspector.active is None:
                    clock.unfreeze()  # Post-effect samples use this same actual station.
                target_operation = not reset or action.operation == 'reset'
                if target_operation and mode == 'cancel' and action.inspector is not None and action.inspector.active is not None and not cancelled:
                    deliver('/maintenance/command', NS(data=canonical({**request, 'op': 'cancel', 'arguments': {}})))
                    cancelled = True
                if target_operation and mode == 'cancel_after_effect' and action.effect is not None and not cancelled:
                    deliver('/maintenance/command', NS(data=canonical({**request, 'op': 'cancel', 'arguments': {}})))
                    cancelled = True
            if time.monotonic()-host_started > 20.:
                diagnostic = self.root/'runtime/test-worker-error.txt'
                raise AssertionError(canonical({'state': statuses[-1] if statuses else None,
                    'guard_failure': station.holds.failure if station else None,
                    'guard': station.holds.guard if station else None, 'nav': station.holds.nav if station else None,
                    'worker': diagnostic.read_text() if diagnostic.exists() else None}))

        real_read = inspection_owner.read_capture
        def read(*args):
            value = real_read(*args)
            if mode == 'late' and (not reset or station.owner.active.operation == 'reset'):
                clock.advance(.25, force=True)
            return value
        h.modules['rclpy'].spin_once = spin
        try:
            with patch.dict(sys.modules, h.modules), patch.object(ros_maintenance, 'StationMaintenance', side_effect=station_owner), patch('orbital.inspection_worker.subprocess.Popen', side_effect=launch), patch.object(inspection_owner, 'read_capture', side_effect=read):
                for name in ('ros_health_interlock', 'ros_navigation_action', 'ros_navigation_supervisor'):
                    navigation = load('orbital.'+name, ROOT/'orbital'/f'{name}.py')
                motion = load('maintenance_test_motion', ROOT/'scripts/motion-guard.py')
                guard = motion.GuardNode()
                supervisor = navigation.NavigationSupervisor({'pump': (2., 1.8, 0.)}, map_config_sha256=h.config_hash)
                ros_machine_telemetry.run(self.root, now=now,
                    stop_requested=lambda:len(outputs.get('/maintenance/result', [])) == (2 if reset else 1) and (
                        trace is None or trace.assess(round(now()*1e9), h.sim) is not None))
                if trace is not None:
                    value = trace.end(round(now()*1e9), h.sim, trace_sources(station.owner.model.registry))
                    trace_writer.finish(True, {'observation': value})
                    self.retained_path = trace_writer.path
                supervisor.destroy_node(); guard.destroy_node()
        finally:
            for child in workers:
                if child.poll() is None: child.kill(); child.wait(timeout=2.)
        self.assertTrue(h.destroyed and h.unregistered and h.shutdown)
        self.assertTrue(all(not client.calls for client in action_clients))
        self.assertTrue(guard.guard.latched)
        self.assertIsNone(guard.guard.maintenance.active)
        self.assertTrue(all(v.linear.x == 0 and v.angular.z == 0 for v in outputs['/motion/guarded_cmd_vel']))
        for status in outputs['/maintenance/state']: state(canonical(status))
        self.assertEqual(len(outputs['/maintenance/result']), 2 if reset else 1)
        terminal = result(outputs['/maintenance/result'][-1])
        action = station.owner.actions[request['action_id']]
        diagnostic = self.root/'runtime/test-worker-error.txt'
        self.endpoint_diagnostic = {'terminal': terminal,
            'inspection': None if action.inspector is None else action.inspector.last_result,
            'worker': diagnostic.read_text() if diagnostic.exists() else None,
            'guard_failure': station.holds.failure}
        self.assertTrue(outputs['/maintenance/reply'][0]['accepted'])
        return terminal, outputs

    def test_owned_reset_uses_second_real_capture_and_same_station(self):
        terminal, output = self.run_endpoint(reset=True)
        self.assertEqual(terminal['status'], 'succeeded', self.endpoint_diagnostic)
        self.assertEqual(terminal['operation'], 'reset')
        self.assertEqual(terminal['isolation'], output['/maintenance/result'][0])
        self.assertEqual(output['/station/telemetry'][-1]['control']['revision'], 2)
        self.assertFalse(output['/station/telemetry'][-1]['machines']['pump']['powered'])
        ref = terminal['observation']
        record, image = read_capture(self.root, ref['observation_id'], ref['manifest_sha256'])
        self.assertEqual(record['telemetry_source']['control_revision'], 1)
        self.assertEqual(record['inspection_action']['action_id'], terminal['action_id'])
        self.assertGreater(len(image), 900000)

    def test_reset_real_capture_cancel_retains_isolation_without_pulse(self):
        terminal, output = self.run_endpoint('cancel', reset=True)
        self.assertEqual(terminal['status'], 'cancelled', self.endpoint_diagnostic)
        self.assertEqual(terminal['state_change']['disposition'], 'not_applied')
        self.assertEqual(output['/station/telemetry'][-1]['control']['revision'], 1)
        self.assertFalse(output['/station/telemetry'][-1]['machines']['pump']['powered'])

    def test_reset_real_artifact_delay_prevents_second_effect(self):
        terminal, output = self.run_endpoint('late', reset=True)
        self.assertEqual(terminal['status'], 'failed', self.endpoint_diagnostic)
        self.assertEqual(terminal['state_change']['disposition'], 'not_applied')
        self.assertEqual(output['/station/telemetry'][-1]['control']['revision'], 1)

    def test_reset_cancellation_after_actual_pulse_waits_for_verification(self):
        terminal, output = self.run_endpoint('cancel_after_effect', reset=True)
        self.assertEqual(terminal['status'], 'cancelled', self.endpoint_diagnostic)
        self.assertEqual(terminal['state_change']['disposition'], 'applied')
        self.assertEqual(output['/station/telemetry'][-1]['control']['revision'], 2)

    def test_same_station_guard_navigation_real_capture_and_verification(self):
        terminal, output = self.run_endpoint()
        self.assertEqual(terminal['status'], 'succeeded', terminal)
        effect = terminal['state_change']['effect']
        packets = output['/station/telemetry']
        self.assertTrue(packets[0]['machines']['pump']['powered'])
        self.assertFalse(packets[-1]['machines']['pump']['powered'])
        self.assertEqual(packets[-1]['control']['last_applied'], effect)
        ref = terminal['observation']
        record, image = read_capture(self.root, ref['observation_id'], ref['manifest_sha256'])
        self.assertEqual(record['telemetry_source']['run_id'], packets[0]['run_id'])
        self.assertEqual(record['telemetry_source']['control_revision'], 0)
        self.assertEqual(record['inspection_action']['action_id'], effect['action_id'])
        self.assertGreater(len(image), 900000)
        self.assertEqual(terminal['station_run_id'], packets[-1]['run_id'])

    def test_cancel_retains_capture_child_then_releases_without_effect(self):
        terminal, outputs = self.run_endpoint('cancel')
        self.assertEqual(terminal['status'], 'cancelled')
        self.assertEqual(terminal['state_change']['disposition'], 'not_applied')
        self.assertTrue(all(p['machines']['pump']['powered'] for p in outputs['/station/telemetry']))

    def test_cancel_after_actual_effect_reports_isolation_and_never_restores(self):
        terminal, outputs = self.run_endpoint('cancel_after_effect')
        self.assertEqual(terminal['status'], 'cancelled')
        self.assertEqual(terminal['state_change']['disposition'], 'applied')
        self.assertFalse(outputs['/station/telemetry'][-1]['machines']['pump']['powered'])

    def test_real_artifact_read_delay_prevents_effect(self):
        terminal, outputs = self.run_endpoint('late')
        self.assertEqual(terminal['status'], 'failed')
        self.assertEqual(terminal['state_change']['disposition'], 'not_applied')
        self.assertTrue(all(p['machines']['pump']['powered'] for p in outputs['/station/telemetry']))

    def test_independent_retention_of_actual_camera_hold_and_station_flow(self):
        terminal, outputs = self.run_endpoint(retained=True)
        value = verify_isolation(self.root, self.retained_path)['observation']
        self.assertEqual(terminal['status'], 'succeeded', self.endpoint_diagnostic)
        self.assertTrue(value['verified_isolation'])
        self.assertEqual(value['station_run_id'], outputs['/station/telemetry'][-1]['run_id'])

    def test_independent_retention_after_actual_effect_cancellation(self):
        terminal, _ = self.run_endpoint('cancel_after_effect', retained=True)
        value = verify_isolation(self.root, self.retained_path)['observation']
        self.assertEqual(value['server_status'], terminal['status'])
        self.assertEqual(value['server_status'], 'cancelled', self.endpoint_diagnostic)
        self.assertTrue(value['verified_isolation'])

    def test_independent_retention_of_cancellation_before_effect(self):
        terminal, _ = self.run_endpoint('cancel', retained=True)
        value = verify_isolation(self.root, self.retained_path)['observation']
        self.assertEqual(value['server_status'], terminal['status'])
        self.assertFalse(value['verified_isolation'])
        self.assertTrue(value['sampled_unchanged'])

    def test_capture_host_io_scheduling_does_not_advance_shared_test_time(self):
        terminal, _ = self.run_endpoint('slow_storage', retained=True)
        self.assertEqual(terminal['status'], 'succeeded', self.endpoint_diagnostic)
        self.assertTrue(verify_isolation(self.root, self.retained_path)['observation']['verified_isolation'])
