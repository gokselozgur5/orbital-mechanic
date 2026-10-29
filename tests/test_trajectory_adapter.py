"""Execute the native adapter with controlled ROS/Gazebo transports.

This covers real adapter wiring, capture lifecycle and refusal paths. Native
Python bindings, DDS delivery and moving simulation acceptance remain separate.
"""
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import hashlib
from io import StringIO
import json
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from orbital import ros_trajectory
from orbital.collision_evaluation import evaluate_trace
from orbital.trajectory import Inbox, TraceWriter, TRUTH_TOPIC, verify_trace
import test_trajectory as fixture
from collision_fixture import scene_message

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('trajectory_cli_test', ROOT/'scripts/record-navigation-trajectory.py')
cli = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cli)


class TrajectoryAdapterTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'runtime').mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT/'runtime')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.world_data = (ROOT/'worlds/maintenance.sdf').read_bytes()
        (self.root/'worlds').mkdir()
        (self.root/'worlds/maintenance.sdf').write_bytes(self.world_data)
        self.world_hash = hashlib.sha256(self.world_data).hexdigest()
        self.runtime_identity = {'manifest_identity': 'a'*64, 'world_sha256': self.world_hash,
                                 'service_invocations': {n: 'b'*32 for n in (
                                     'orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service')}}
        self.binding_identity = {n: {'path': '/fixture/'+n, 'sha256': 'c'*64} for n in (
            'gz.transport13', 'gz.transport13._transport', 'gz.msgs10.pose_v_pb2')}
        self.clock = 1000000000
        self.callbacks = {}
        self.qos = {}
        self.truth_callback = None
        self.truth_enabled = False
        self.polls = 0
        self.lookups = []
        self.shutdowns = 0
        self.destroyed = self.unregistered = False
        self.duplicate_peer = False
        self.wrong_transform = False
        self.missing_transform = False
        self.bad_truth = False
        self.no_truth = False
        self.missing_scene_link = False
        self.drop_audit = False
        self.delay_tf = False
        self.interrupt_at = None
        self.command_sent = False
        owner = self

        class Node:
            def create_subscription(self, kind, topic, callback, depth):
                if topic not in ('/navigation/state', '/navigation/audit'):
                    raise AssertionError('Unexpected evaluator subscription '+topic)
                owner.callbacks[topic] = callback
                owner.qos[topic] = depth
                return NS()

            def create_publisher(self, *args, **kwargs):
                raise AssertionError('Evaluator must not publish controls or truth')

            def create_client(self, *args, **kwargs):
                raise AssertionError('Evaluator must not command services')

            def get_publishers_info_by_topic(self, topic):
                return [NS(node_name='orbital_navigation_supervisor', node_namespace='/',
                           topic_type='std_msgs/msg/String')]*(2 if owner.duplicate_peer else 1)

            def destroy_node(self):
                owner.destroyed = True

        class Gz:
            def subscribe(self, kind, topic, callback):
                if topic != TRUTH_TOPIC:
                    raise AssertionError('Wrong truth source')
                owner.truth_callback = callback
                owner.truth_enabled = True
                return True

            def unsubscribe(self, topic):
                owner.truth_enabled = False
                return True

            def topic_info(self, topic):
                return [NS(msg_type_name='gz.msgs.Pose_V')], []

        class Buffer:
            def __init__(self, **kwargs):
                pass

            def lookup_transform(self, parent, child, stamp):
                owner.lookups.append((parent, child, stamp.nanoseconds))
                if owner.missing_transform or (owner.delay_tf and stamp.nanoseconds > owner.clock-40000000):
                    raise LookupError('Future transform')
                return fixture.transform(stamp.nanoseconds+(1 if owner.wrong_transform else 0))

        def spin(node, timeout_sec):
            self.polls += 1
            self.clock += round(timeout_sec*1e9)
            if self.interrupt_at == self.polls:
                raise KeyboardInterrupt()
            fields = {}
            if self.drop_audit and self.polls >= 5:
                fields = {'command_sequence': 1, 'audit_sequence': 1}
            self.callbacks['/navigation/state'](NS(data=fixture.state(self.clock, **fields)))
            if self.truth_enabled and not self.no_truth and self.polls % 2 == 0:
                msg = scene_message(self.world_data, self.clock)
                if self.missing_scene_link:
                    msg.pose = [p for p in msg.pose if p.name != 'left_wheel']
                if self.bad_truth:
                    next(p for p in msg.pose if p.name == 'maintenance_robot').position.x = float('nan')
                self.truth_callback(msg)

        def unregister():
            self.unregistered = True

        def shutdown():
            self.shutdowns += 1

        exports = {
            'rclpy': {'init': lambda **kwargs: None, 'create_node': lambda *args, **kwargs: Node(),
                      'spin_once': spin, 'try_shutdown': shutdown},
            'rclpy.clock': {'ClockType': NS(ROS_TIME=1)},
            'rclpy.parameter': {'Parameter': lambda *args, **kwargs: NS()},
            'rclpy.time': {'Time': lambda **kwargs: NS(**kwargs)},
            'rclpy.qos': {'QoSProfile': NS, 'ReliabilityPolicy': NS(BEST_EFFORT='best_effort')},
            'std_msgs.msg': {'String': NS},
            'tf2_ros': {'Buffer': Buffer, 'TransformException': LookupError,
                        'TransformListener': lambda *args: NS(unregister=unregister)},
            'gz.transport13': {'Node': Gz},
            'gz.msgs10.pose_v_pb2': {'Pose_V': NS},
        }
        modules = {name: ModuleType(name) for name in ('std_msgs', 'gz', 'gz.msgs10')}
        for name, attributes in exports.items():
            modules[name] = ModuleType(name)
            modules[name].__dict__.update(attributes)
        for context in (patch.dict(sys.modules, modules),
                        patch.object(ros_trajectory, 'runtime_snapshot', return_value=self.runtime_identity),
                        patch.object(ros_trajectory, 'binding_sources', return_value=self.binding_identity),
                        patch.object(ros_trajectory, 'Inbox', lambda: Inbox(now=lambda: self.clock))):
            context.start()
            self.addCleanup(context.stop)
        real_collect = ros_trajectory.collect
        context = patch.object(ros_trajectory, 'collect', lambda *args, **kwargs:
                               real_collect(*args, **kwargs, now=lambda: self.clock))
        context.start()
        self.addCleanup(context.stop)

    def invoke(self, *, duration=.1, warmup=.2):
        writer = TraceWriter(self.root, 'capture')
        self.addCleanup(writer.stream.close)
        self.output = writer.path
        return ros_trajectory.run(self.root, writer, duration=duration, warmup=warmup)

    def test_actual_adapter_completes_without_control_publications(self):
        result, code = self.invoke()
        self.assertEqual(code, 0, result)
        self.assertTrue(result['complete'])
        self.assertGreaterEqual(result['observation']['capture_samples'], 2)
        self.assertEqual(verify_trace(self.output), result)
        self.assertTrue((self.output/'ready.json').is_file())
        self.assertTrue(self.destroyed and self.unregistered)
        self.assertEqual(self.shutdowns, 1)
        self.assertFalse(self.truth_enabled)
        self.assertEqual(set(self.callbacks), {'/navigation/state', '/navigation/audit'})
        self.assertEqual(self.qos['/navigation/audit'].reliability, 'best_effort')
        self.assertEqual(self.qos['/navigation/audit'].depth, 100)
        self.assertTrue(all(p == 'map' and c == 'base_footprint' and t > 0 for p, c, t in self.lookups))

    def test_future_tf_is_waited_for_at_original_truth_time(self):
        self.delay_tf = True
        result, code = self.invoke(warmup=.4)
        self.assertEqual(code, 0, result)
        stamps = [stamp for parent, child, stamp in self.lookups]
        self.assertGreater(len(stamps), len(set(stamps)))

    def test_actual_recorder_artifact_replays_through_independent_collision_evaluator(self):
        result, code = self.invoke()
        self.assertEqual(code, 0, result)
        evaluation = evaluate_trace(self.output)
        self.assertTrue(evaluation['sampled_clear'])
        self.assertEqual(evaluation['trace_sha256'], result['sha256'])
        self.assertEqual(evaluation['samples'], result['observation']['capture_samples'])
        self.assertIsNone(evaluation['continuous_collision_free'])

    def test_missing_robot_link_is_not_replaced_by_its_declared_static_pose(self):
        self.missing_scene_link = True
        result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('topology differs', result['error'])
        self.assertFalse((self.output/'ready.json').exists())

    def test_world_snapshot_must_match_verified_startup_source(self):
        with patch.object(ros_trajectory, 'runtime_snapshot', return_value={
                **self.runtime_identity, 'world_sha256': '0'*64}):
            result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('world source differs', result['error'])
        self.assertEqual(self.callbacks, {})

    def test_missing_truth_times_out_without_ready_marker(self):
        self.no_truth = True
        result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('warmup timed out', result['error'])
        self.assertFalse((self.output/'ready.json').exists())

    def test_wrong_tf_time_refuses_latest_substitution(self):
        self.wrong_transform = True
        result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('requested truth time', result['error'])

    def test_missing_tf_is_not_replaced_by_truth_or_zero_error(self):
        self.missing_transform = True
        result, code = self.invoke(warmup=.8)
        self.assertEqual(code, 1)
        self.assertIn('warmup timed out', result['error'])
        records = [json.loads(line) for line in (self.output/'events.jsonl').read_text().splitlines()]
        samples = [r['data'] for r in records if r['kind'] == 'sample']
        self.assertTrue(samples)
        self.assertTrue(all(s['error'] is None and s['localization_pose'] is None for s in samples))

    def test_missing_audit_tail_refuses_success_after_data_collection(self):
        self.drop_audit = True
        result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('audit tail', result['error'])
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            verify_trace(self.output)

    def test_malformed_callback_is_fatal_even_if_transport_would_swallow_exception(self):
        self.bad_truth = True
        result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('Nonfinite pose', result['error'])

    def test_competing_ros_publisher_refuses_readiness(self):
        self.duplicate_peer = True
        result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('Ambiguous evaluator publisher', result['error'])

    def test_runtime_identity_change_invalidates_otherwise_complete_trace(self):
        with patch.object(ros_trajectory, 'runtime_snapshot', side_effect=[
                {'identity': 'before', 'world_sha256': self.world_hash},
                {'identity': 'after', 'world_sha256': self.world_hash}]):
            result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertFalse(result['complete'])
        self.assertIn('identity changed', result['error'])

    def test_missing_binding_preserves_failed_artifact_before_ros(self):
        with patch.object(ros_trajectory, 'binding_sources', side_effect=ImportError('gz.transport13 missing')):
            result, code = self.invoke()
        self.assertEqual(code, 1)
        self.assertIn('gz.transport13 missing', result['error'])
        self.assertEqual(self.callbacks, {})

    def test_interrupt_preserves_prefix_and_does_not_send_cancel(self):
        self.interrupt_at = 8
        result, code = self.invoke()
        self.assertEqual(code, 130)
        self.assertFalse(result['complete'])
        self.assertGreater(result['records'], 0)
        self.assertTrue(self.destroyed and self.unregistered)


class TrajectoryCliTests(unittest.TestCase):
    def test_help_invalid_duration_and_conflicting_operations_need_no_ros(self):
        with redirect_stdout(StringIO()), self.assertRaises(SystemExit) as raised:
            cli.main(['--help'])
        self.assertEqual(raised.exception.code, 0)
        for args in (['--name', 'a', '--duration', 'nan'], ['--name', 'a', '--duration', '301'],
                     ['--name', 'a', '--verify', 'b']):
            with self.subTest(args=args), redirect_stderr(StringIO()), self.assertRaises(SystemExit) as raised:
                cli.main(args)
            self.assertEqual(raised.exception.code, 2)

    def test_scope_rejected_before_creating_output_or_importing_ros(self):
        for domain, discovery, partition in (('77', 'LOCALHOST', 'orbital-maintenance'),
                                              ('42', 'SUBNET', 'orbital-maintenance'),
                                              ('42', 'LOCALHOST', 'other')):
            output = StringIO()
            with patch.dict(os.environ, {'ROS_DOMAIN_ID': domain, 'ROS_AUTOMATIC_DISCOVERY_RANGE': discovery,
                                         'GZ_PARTITION': partition}), patch.object(cli, 'TraceWriter',
                                         side_effect=AssertionError('Should not write')), redirect_stdout(output):
                self.assertEqual(cli.main(['--name', 'not-created']), 2)
            self.assertFalse(json.loads(output.getvalue())['complete'])


if __name__ == '__main__':
    unittest.main()
