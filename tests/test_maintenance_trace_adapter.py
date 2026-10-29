"""Run the shared read-only ROS observer with complete durable maintenance wire traces."""
import contextlib
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from orbital import ros_isolation_trace
from orbital.maintenance_trace import MaintenanceTrace, verify_maintenance
from orbital.trajectory import TraceWriter
import test_isolation_trace_adapter as adapter
import test_maintenance_trace as sequence

ROOT=Path(__file__).resolve().parents[1]


class MaintenanceTraceAdapterTests(unittest.TestCase):
    def setUp(self):
        # Reuse the transport whose publisher/client constructors fail immediately.
        adapter.IsolationAdapterTests.setUp(self)
        self.f=sequence.SequenceFixture(); self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.root=self.f.root
        self.f.execute(); self.f.finish()
        records=[json.loads(line) for line in (self.f.writer.path/'events.jsonl').read_text().splitlines()]
        self.events=[r for r in records if r['kind']=='sample']
        self.at,self.sim=records[0]['received_monotonic_ns'],records[0]['data']['sim_stamp_ns']

    def run_adapter(self,mode=None):
        self.mode=mode
        writer=TraceWriter(self.root,'sequence-adapter')
        snapshots=[copy.deepcopy(self.f.sources['runtime']) for _ in range(2)]
        if mode=='runtime': snapshots[1]['service_invocations']['orbital-demo.service']='c'*32
        with patch.dict(sys.modules,self.modules), patch.object(ros_isolation_trace,'runtime_snapshot',side_effect=snapshots):
            value,code=ros_isolation_trace.run(self.root,writer,self.f.identity,duration=30.,warmup=2.,
                now=lambda:self.at,trace_factory=MaintenanceTrace,node_prefix='orbital_maintenance_observer')
        self.assertEqual((self.destroyed,self.shutdowns),(1,1))
        self.assertEqual(self.f.f.model.control_revision,3)
        return value,code,writer.path

    def test_production_observer_records_and_replays_full_chain_without_control_calls(self):
        value,code,path=self.run_adapter()
        self.assertEqual(code,0,value)
        self.assertTrue((path/'ready.json').exists())
        self.assertEqual(verify_maintenance(self.root,path)['observation'],value['observation'])
        self.assertEqual(len(value['observation']['actions']),3)

    def test_lost_prior_result_leaves_incomplete_prefix(self):
        self.events=[v for v in self.events if not (v['data'].get('topic')=='/maintenance/result'
            and json.loads(v['data']['text'])['operation']=='reset')]
        value,code,path=self.run_adapter()
        self.assertEqual(code,1); self.assertFalse(value['complete'])
        self.assertIn('action_timeout',value['error'])
        with self.assertRaises(ValueError): verify_maintenance(self.root,path)

    def test_runtime_generation_change_refuses_sequence_completion(self):
        value,code,path=self.run_adapter('runtime')
        self.assertEqual(code,1); self.assertIn('source_changed',value['error'])
        with self.assertRaises(ValueError): verify_maintenance(self.root,path)

    def test_observer_interrupt_does_not_cancel_or_retry_a_completed_real_effect(self):
        value,code,path=self.run_adapter('interrupt')
        self.assertEqual(code,130); self.assertFalse(value['complete'])
        self.assertEqual(self.f.f.model.control_revision,3)

    def test_cleanup_failure_cannot_mark_a_sequence_complete(self):
        value,code,path=self.run_adapter('cleanup')
        self.assertEqual(code,1); self.assertFalse(value['complete'])
        with self.assertRaises(ValueError): verify_maintenance(self.root,path)

    def test_actual_cli_replay_and_domain_refusal(self):
        spec=importlib.util.spec_from_file_location('maintenance_trace_cli',ROOT/'scripts/record-maintenance.py')
        cli=importlib.util.module_from_spec(spec); spec.loader.exec_module(cli)
        with patch.object(cli,'ROOT',self.root), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(cli.main(['--verify',str(self.f.writer.path)]),0)
        self.assertTrue(json.loads(output.getvalue())['observation']['observed_reset_restore_chain'])
        args=['--name','forbidden-sequence','--machine-id','pump']
        for key in ('run_id','mission_id','action_id'): args.extend(['--'+key.replace('_','-'),self.f.identity[key]])
        with patch.object(cli,'ROOT',self.root), patch.dict(os.environ,{'ROS_DOMAIN_ID':'77'}), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(args),2)
        self.assertFalse((self.root/'runtime/evaluation/forbidden-sequence').exists())
