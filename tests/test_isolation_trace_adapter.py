"""Run the production subscriber/CLI with retained actual-model fixture publications."""
import contextlib
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from orbital import ros_isolation_trace
from orbital.isolation_trace import verify_isolation
from orbital.trajectory import TraceWriter
import test_isolation_trace as fixture

ROOT = Path(__file__).resolve().parents[1]


class IsolationAdapterTests(unittest.TestCase):
    def setUp(self):
        self.f = fixture.IsolationTraceTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.root = self.f.root
        self.f.execute(); self.f.finish()
        self.records = [json.loads(line) for line in (self.f.writer.path/'events.jsonl').read_text().splitlines()]
        self.events = [r for r in self.records if r['kind'] == 'sample']
        self.at, self.sim = self.records[0]['received_monotonic_ns'], self.records[0]['data']['sim_stamp_ns']
        self.callbacks, self.effects = {}, []
        self.polls = self.shutdowns = self.destroyed = 0
        self.mode = None
        owner = self
        class Node:
            def create_subscription(self, kind, topic, callback, qos): owner.callbacks[topic] = callback
            def create_publisher(self, *args, **kw): raise AssertionError('Observer cannot publish')
            def create_client(self, *args, **kw): raise AssertionError('Observer cannot call services')
            def get_parameter(self, name): return NS(value=not (owner.mode == 'clock_mode' and owner.polls > 4))
            def get_clock(self): return NS(now=lambda:NS(nanoseconds=owner.sim))
            def get_publishers_info_by_topic(self, topic):
                result = [NS(node_name=p[1], node_namespace=p[2], topic_type=p[3],
                             endpoint_gid=list(bytes.fromhex(p[4])))
                          for p in owner.f.sources['publishers'] if p[0] == topic]
                if owner.mode == 'duplicate' and owner.polls > 4: result *= 2
                if owner.mode == 'gid' and owner.polls > 4:
                    result[0].endpoint_gid = [7]*24
                return result
            def destroy_node(self):
                owner.destroyed += 1
                if owner.mode == 'cleanup': raise RuntimeError('controlled cleanup failure')
        self.modules = {}
        def module(name, **values):
            value = ModuleType(name); value.__dict__.update(values); self.modules[name] = value
        def spin(node, timeout_sec):
            self.polls += 1
            if self.mode == 'interrupt' and self.polls > 4: raise KeyboardInterrupt()
            if self.mode == 'frozen' and self.polls == 5: self.events.clear()
            if self.events:
                record = self.events.pop(0)
                self.at, self.sim = record['received_monotonic_ns'], record['data']['sim_stamp_ns']
                topic, text = record['data']['topic'], record['data']['text']
                if self.mode == 'malformed' and self.polls == 5: text = 'untrusted incomplete json'
                if self.mode != 'drop_result' or topic != '/maintenance/result':
                    self.callbacks[topic](NS(data=text))
            else:
                self.at += 100000000
                if self.mode != 'frozen': self.sim += 100000000
        def shutdown(): self.shutdowns += 1
        def create(name, **kwargs):
            self.assertTrue(kwargs['parameter_overrides'][0].value)
            return Node()
        module('rclpy', init=lambda **kw:None, ok=lambda:True, create_node=create,
               spin_once=spin, try_shutdown=shutdown)
        module('rclpy.parameter', Parameter=lambda name, value:NS(value=value))
        module('rclpy.signals', SignalHandlerOptions=NS(NO=0))
        module('std_msgs.msg', String=NS)

    def run_adapter(self, mode=None, *, limit=None):
        self.mode = mode
        writer = TraceWriter(self.root, 'adapter', **({'max_records': limit} if limit else {}))
        snapshots = [copy.deepcopy(self.f.sources['runtime']) for _ in range(2)]
        if mode == 'runtime': snapshots[1]['service_invocations']['orbital-demo.service'] = 'c'*32
        with patch.dict(sys.modules, self.modules), patch.object(ros_isolation_trace, 'runtime_snapshot', side_effect=snapshots):
            result, code = ros_isolation_trace.run(self.root, writer, self.f.identity,
                duration=2., warmup=2., now=lambda:self.at)
        self.assertEqual(self.shutdowns, 1)
        self.assertEqual(self.destroyed, 1)
        self.assertEqual(self.f.f.model.control_revision, 1)  # The observer cannot add another effect.
        return result, code, writer.path

    def test_actual_adapter_retains_and_replays_source_packets_without_control_interfaces(self):
        value, code, path = self.run_adapter()
        self.assertEqual(code, 0, value)
        self.assertTrue((path/'ready.json').exists())
        self.assertEqual(verify_isolation(self.root, path)['observation'], value['observation'])
        self.assertEqual(set(self.callbacks), set(ros_isolation_trace.TOPICS))

    def test_ambiguous_graph_after_pin_is_incomplete(self):
        value, code, path = self.run_adapter('duplicate')
        self.assertEqual(code, 1); self.assertFalse(value['complete'])
        with self.assertRaises(ValueError): verify_isolation(self.root, path)

    def test_publisher_generation_change_is_incomplete(self):
        value, code, path = self.run_adapter('gid')
        self.assertEqual(code, 1); self.assertIn('publisher_changed', value['error'])

    def test_clock_mode_change_is_incomplete(self):
        value, code, path = self.run_adapter('clock_mode')
        self.assertEqual(code, 1); self.assertIn('clock_mode_changed', value['error'])

    def test_runtime_restart_cannot_be_recorded_as_complete(self):
        value, code, path = self.run_adapter('runtime')
        self.assertEqual(code, 1); self.assertIn('source_changed', value['error'])

    def test_malformed_packet_is_not_retained_as_valid_source_data(self):
        value, code, path = self.run_adapter('malformed')
        self.assertEqual(code, 1)
        self.assertNotIn('untrusted incomplete json', (path/'events.jsonl').read_text())

    def test_interrupt_preserves_incomplete_prefix_and_never_cancels(self):
        value, code, path = self.run_adapter('interrupt')
        self.assertEqual(code, 130); self.assertFalse(value['complete'])
        self.assertGreater(value['records'], 1)

    def test_missing_result_has_a_wall_deadline(self):
        value, code, path = self.run_adapter('drop_result')
        self.assertEqual(code, 1); self.assertIn('action_timeout', value['error'])

    def test_frozen_simulation_does_not_extend_wall_deadline(self):
        value, code, path = self.run_adapter('frozen')
        self.assertEqual(code, 1); self.assertIn('action_timeout', value['error'])

    def test_cleanup_failure_cannot_seal_an_accepted_trace(self):
        value, code, path = self.run_adapter('cleanup')
        self.assertEqual(code, 1); self.assertFalse(value['complete'])
        with self.assertRaises(ValueError): verify_isolation(self.root, path)

    def test_output_bound_still_retains_an_incomplete_manifest(self):
        value, code, path = self.run_adapter(limit=3)
        self.assertEqual(code, 1); self.assertEqual(value['records'], 3)
        self.assertFalse(value['complete'])

    def test_cli_offline_verification_and_invalid_live_configuration(self):
        spec = importlib.util.spec_from_file_location('isolation_cli', ROOT/'scripts/record-isolation.py')
        cli = importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)
        with patch.object(cli, 'ROOT', self.root), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(cli.main(['--verify', str(self.f.writer.path)]), 0)
        self.assertTrue(json.loads(output.getvalue())['observation']['verified_isolation'])
        args = ['--name', 'forbidden', '--machine-id', 'pump']
        for key in ('run_id', 'mission_id', 'action_id'): args += ['--'+key.replace('_', '-'), self.f.identity[key]]
        with patch.object(cli, 'ROOT', self.root), patch.dict(os.environ, {'ROS_DOMAIN_ID': '77'}), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(args), 2)
        self.assertFalse((self.root/'runtime/evaluation/forbidden').exists())
