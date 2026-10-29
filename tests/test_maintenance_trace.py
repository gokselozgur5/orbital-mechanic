"""Replay actual durable station sequences without consulting owner history or hidden state."""
import copy
import hashlib
import json
import uuid
import unittest
from unittest.mock import patch

from orbital.inspection_protocol import canonical
from orbital.maintenance_trace import MaintenanceTrace, verify_maintenance
from orbital.trajectory import TraceWriter
import test_maintenance_journal as fixture
import test_isolation_trace as isolation


class SequenceFixture(unittest.TestCase):
    def setUp(self):
        self.f = fixture.JournalFixture()
        self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.root, self.clock, self.owner = self.f.root, self.f.clock, self.f.owner
        self.final_id = uuid.uuid4().hex
        self.identity = {'run_id':self.owner.run_id,'mission_id':self.f.request['mission_id'],
                         'action_id':self.final_id,'machine_id':'pump',
                         'require_recovery':getattr(self,'require_recovery',False)}
        self.writer = TraceWriter(self.root,'sequence')
        self.addCleanup(lambda:self.writer.stream.close() if not self.writer.closed else None)
        self.trace = MaintenanceTrace(self.f.model.registry,self.identity,writer=self.writer)
        self.sources = isolation.sources(self.f.model.registry)
        self.trace.begin(self.at(),self.clock.sim,self.sources)
        sample, send, publish = self.f.sample, self.f.send, self.owner.on_result
        def sampled(delta=.5):
            packet = sample(delta)
            self.receive('/station/telemetry',packet)
            self.receive('/maintenance/state',self.owner.state())
            return packet
        def command(request=None,op=None):
            value = copy.deepcopy(self.f.request if request is None else request)
            if op is not None: value.update(op=op,arguments={},command_id=uuid.uuid4().hex)
            self.receive('/maintenance/command',value)
            return send(value)
        def result(value):
            self.receive('/maintenance/result',value)
            publish(value)
        self.f.sample, self.f.send, self.owner.on_result = sampled, command, result
        cancellation_request = self.f.cancellation_request
        def cancellation(target,timeout=20.):
            return {**cancellation_request(target,timeout),'action_id':self.final_id}
        self.f.cancellation_request = cancellation
        self.f.sample(.05); self.f.sample(.05)
        self.assertTrue(self.trace.ready(self.at(),self.clock.sim))

    def at(self): return round(self.clock.wall*1e9)
    def receive(self,topic,value): self.trace.receive(topic,self.at(),self.clock.sim,canonical(value))

    def execute(self, mode='restore', *, recovery=False):
        if mode == 'cancel_after':
            value = self.f.completed()
        else:
            self.f.prepare()
            if mode != 'cancel_before': self.f.restore_request['action_id'] = self.final_id
            self.f.start(self.f.restore_request)
            if mode in ('cancel_before','ordinary_cancel'):
                if mode == 'cancel_before': self.f.cancel_isolate(self.f.restore_request)
                else: self.f.send(self.f.restore_request,'cancel')
                self.f.workers[-1].exited = True; self.owner.tick()
                self.f.hold.released = True; self.owner.tick()
                self.f.sample(.05); self.f.sample(.05)
                value = self.f.results[-1]
            else:
                self.f.apply(); self.f.sample(); self.f.sample()
                value = self.f.finish()
        self.f.sample(.05); self.f.sample(.05)
        if recovery:
            for _ in range(22): self.f.sample(.5)
        return value

    def finish(self):
        value = self.trace.end(self.at(),self.clock.sim,self.sources)
        self.writer.finish(True,{'observation':value})
        return value

    def rewrite(self,edit): isolation.IsolationTraceTests.rewrite(self,edit)

    def wires(self,values,topic):
        return [(v,json.loads(v['data']['text'])) for v in values if v['data'].get('topic') == topic]


class MaintenanceTraceTests(SequenceFixture):
    def test_actual_durable_three_action_chain_replays_public_outcomes(self):
        terminal = self.execute(); observed = self.finish()
        self.assertEqual(verify_maintenance(self.root,self.writer.path)['observation'],observed)
        self.assertEqual([v['operation'] for v in observed['actions']],['isolate','reset','restore'])
        self.assertTrue(observed['observed_reset_restore_chain'])
        self.assertFalse(observed['post_restore_recovery_verified'])
        self.assertEqual(observed['final_control']['last_applied'],terminal['state_change']['effect'])
        self.assertTrue(observed['final_measurements']['powered'])

    def test_full_public_recovery_window_is_separate_from_restoration(self):
        other=SequenceFixture(); other.require_recovery=True; other.setUp()
        try:
            other.execute()
            self.assertIsNone(other.trace.assess(other.at(),other.clock.sim))
            for _ in range(22): other.f.sample(.5)
            value=other.finish()
            self.assertTrue(value['post_restore_recovery_verified'])
            self.assertGreaterEqual(value['recovery_observation']['window']['duration_s'],10.)
            self.assertEqual(verify_maintenance(other.root,other.writer.path)['observation'],value)
        finally: other.doCleanups()

    def test_cancel_after_restore_requires_separate_child_publication_and_packets(self):
        self.execute('cancel_after'); observed = self.finish()
        self.assertEqual(verify_maintenance(self.root,self.writer.path)['observation'],observed)
        self.assertEqual(len(observed['actions']),5)
        child = next(v for v in observed['actions'] if v['request_source'] == 'workflow_child_report')
        self.assertEqual((child['operation'],child['disposition']),('isolate','applied'))
        self.assertFalse(observed['final_measurements']['powered'])
        self.assertTrue(observed['observed_reset_restore_chain'])

    def test_cancel_before_restore_keeps_sampled_off_state_without_child_effect(self):
        self.execute('cancel_before'); observed = self.finish()
        self.assertEqual(verify_maintenance(self.root,self.writer.path)['observation'],observed)
        restore = next(v for v in observed['actions'] if v['operation'] == 'restore')
        self.assertEqual(restore['disposition'],'not_applied')
        self.assertTrue(restore['sampled_unchanged'])
        self.assertFalse(observed['observed_reset_restore_chain'])
        self.assertEqual(observed['final_control']['revision'],2)

    def test_ordinary_cancellation_is_not_reinterpreted_as_restoration(self):
        self.execute('ordinary_cancel'); observed = self.finish()
        self.assertEqual(observed['server_status'],'cancelled')
        self.assertFalse(observed['observed_reset_restore_chain'])
        self.assertFalse(observed['final_measurements']['powered'])
        verify_maintenance(self.root,self.writer.path)

    def test_nested_results_cannot_replace_missing_independent_result_messages(self):
        self.execute(); self.finish()
        def remove(values):
            return [v for v in values if not (v['data'].get('topic') == '/maintenance/result'
                and json.loads(v['data']['text'])['operation'] == 'reset')]
        self.rewrite(remove)
        with self.assertRaisesRegex(ValueError,'incomplete'): verify_maintenance(self.root,self.writer.path)

    def test_missing_observed_command_is_not_supplied_by_later_prior_result(self):
        self.execute(); self.finish()
        def remove(values):
            return [v for v in values if not (v['data'].get('topic') == '/maintenance/command'
                and json.loads(v['data']['text'])['op'] == 'isolate')]
        self.rewrite(remove)
        with self.assertRaisesRegex(ValueError,'incomplete'): verify_maintenance(self.root,self.writer.path)

    def test_workflow_embedded_child_result_does_not_replace_child_publication(self):
        terminal=self.execute('cancel_after'); self.finish()
        def remove(values):
            return [v for v in values if not (v['data'].get('topic') == '/maintenance/result'
                and json.loads(v['data']['text'])['action_id'] == terminal['isolation_action_id'])]
        self.rewrite(remove)
        with self.assertRaisesRegex(ValueError,'incomplete'): verify_maintenance(self.root,self.writer.path)

    def test_source_measurement_contradicts_reset_even_with_recomputed_reference_hashes(self):
        self.execute(); self.finish()
        def corrupt(values):
            changed={}
            for row,packet in self.wires(values,'/station/telemetry'):
                if packet['control']['revision']==2:
                    packet['machines']['pump']['powered']=True
                    changed[packet['sequence']]=hashlib.sha256(canonical(packet).encode()).hexdigest()
                    row['data']['text']=canonical(packet)
            def repair_hashes(value):
                if isinstance(value,dict):
                    if value.get('sample_sha256') is not None and value.get('sequence') in changed:
                        value['sample_sha256']=changed[value['sequence']]
                    for child in value.values(): repair_hashes(child)
                elif isinstance(value,list):
                    for child in value: repair_hashes(child)
            for topic in ('/maintenance/result','/maintenance/state'):
                for row,value in self.wires(values,topic):
                    repair_hashes(value); row['data']['text']=canonical(value)
            return values
        self.rewrite(corrupt)
        with self.assertRaisesRegex(ValueError,'not_observed'): verify_maintenance(self.root,self.writer.path)

    def test_expired_final_samples_cannot_renew_historical_outcomes(self):
        self.execute(); self.clock.advance(1.)
        self.assertIsNone(self.trace.assess(self.at(),self.clock.sim))

    def test_old_final_pair_member_is_not_renewed_by_latest_sample_and_state(self):
        self.execute(); self.clock.advance(.96)
        self.receive('/maintenance/state',self.owner.state())
        self.assertIsNone(self.trace.assess(self.at(),self.clock.sim))

    def test_changed_runtime_and_missing_end_do_not_complete_a_recording(self):
        self.execute()
        changed=copy.deepcopy(self.sources); changed['runtime']['manifest_identity']='e'*64
        with self.assertRaisesRegex(ValueError,'source_changed'): self.trace.end(self.at(),self.clock.sim,changed)
        self.writer.finish(False,{'error':'controlled source mismatch'})
        with self.assertRaises(ValueError): verify_maintenance(self.root,self.writer.path)

    def test_sample_capacity_is_explicit_without_eviction(self):
        before=dict(self.trace.samples)
        with patch('orbital.maintenance_trace.MAX_SAMPLES',len(before)):
            with self.assertRaisesRegex(ValueError,'sample_capacity'): self.f.sample(.05)
        self.assertEqual(self.trace.samples,before)

    def test_replayed_telemetry_is_refused(self):
        packet=next(reversed(self.trace.samples.values()))[0]
        with self.assertRaises(ValueError): self.receive('/station/telemetry',packet)

    def test_changed_command_arguments_cannot_rebind_an_action(self):
        self.receive('/maintenance/command',self.f.request)
        changed=copy.deepcopy(self.f.request); changed['arguments']['timeout_s']=1.
        with self.assertRaisesRegex(ValueError,'conflicting_command'): self.receive('/maintenance/command',changed)

    def test_same_request_with_new_command_id_does_not_create_another_action(self):
        self.execute()
        replay={**self.f.restore_request,'command_id':uuid.uuid4().hex}
        self.receive('/maintenance/command',replay)
        self.assertEqual(len(self.trace.requests),3)
        self.assertIsNotNone(self.trace.assess(self.at(),self.clock.sim))

    def test_historical_result_delivery_does_not_renew_or_expire_observed_effects(self):
        self.execute()
        value=copy.deepcopy(self.f.results[0])
        for _ in range(4): self.f.sample(.5)
        self.receive('/maintenance/result',value)
        observed=self.finish()
        self.assertEqual(verify_maintenance(self.root,self.writer.path)['observation'],observed)

    def test_claimed_reset_basis_must_resolve_to_actual_source_measurements(self):
        self.execute(); self.finish()
        def corrupt(values):
            changed={}
            for row,packet in self.wires(values,'/station/telemetry'):
                if packet['control']['revision']==0:
                    packet['voltage_v']=28.
                    changed[packet['sequence']]=hashlib.sha256(canonical(packet).encode()).hexdigest()
                    row['data']['text']=canonical(packet)
            def replace_hashes(value):
                if isinstance(value,dict):
                    if 'sample_sha256' in value and value.get('sequence') in changed:
                        value['sample_sha256']=changed[value['sequence']]
                    for child in value.values(): replace_hashes(child)
                elif isinstance(value,list):
                    for child in value: replace_hashes(child)
            for topic in ('/maintenance/result','/maintenance/state'):
                for row,value in self.wires(values,topic):
                    replace_hashes(value); row['data']['text']=canonical(value)
            return values
        self.rewrite(corrupt)
        with self.assertRaisesRegex(ValueError,'reset_basis_not_observed'): verify_maintenance(self.root,self.writer.path)

    def test_one_contradictory_intermediate_sample_cannot_hide_between_good_pairs(self):
        self.execute(); self.finish()
        def corrupt(values):
            packets=self.wires(values,'/station/telemetry')
            row,packet=next((row,p) for row,p in packets if p['control']['revision']==3)
            packet['machines']['pump']['powered']=False
            row['data']['text']=canonical(packet)
            return values
        self.rewrite(corrupt)
        with self.assertRaisesRegex(ValueError,'not_observed'): verify_maintenance(self.root,self.writer.path)

    def test_missing_source_packet_is_not_repaired_by_nested_hashes(self):
        self.execute(); self.finish()
        def remove(values):
            row,_=next((row,p) for row,p in self.wires(values,'/station/telemetry') if p['control']['revision']==2)
            values.remove(row); return values
        self.rewrite(remove)
        with self.assertRaisesRegex(ValueError,'gap'): verify_maintenance(self.root,self.writer.path)

    def test_new_observed_fault_keeps_energization_separate_from_recovery(self):
        from orbital.machines import Fault
        self.execute()
        self.f.model.station.machines['pump'].fault=Fault.JAM  # Evaluator-only fault injection.
        for _ in range(24): self.f.sample(.5)
        value=self.finish()
        self.assertTrue(value['observed_reset_restore_chain'])
        self.assertFalse(value['post_restore_recovery_verified'])
        self.assertEqual(value['final_measurements']['vibration_au'],3.)
        verify_maintenance(self.root,self.writer.path)

    def test_pending_unknown_effect_cannot_complete_from_current_telemetry(self):
        self.f.prepare(); self.f.restore_request['action_id']=self.final_id
        self.f.start(self.f.restore_request)
        with self.f.failed_sync(2): self.f.apply()
        self.assertTrue(self.owner.active.unknown)
        self.assertIsNone(self.trace.assess(self.at(),self.clock.sim))
        self.writer.finish(False,{'error':'unknown actual effect'})
        with self.assertRaises(ValueError): verify_maintenance(self.root,self.writer.path)

    def test_unmatched_rejected_command_leaves_incomplete_observation(self):
        self.execute()
        request={**self.f.restore_request,'action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex}
        self.assertFalse(self.f.send(request)['accepted'])
        self.assertIsNone(self.trace.assess(self.at(),self.clock.sim))

    def test_healthy_window_from_an_earlier_mission_is_not_this_sequence_restoration(self):
        self.execute(); self.finish()
        self.identity={**self.identity,'mission_id':uuid.uuid4().hex,'action_id':uuid.uuid4().hex}
        self.writer=TraceWriter(self.root,'later-mission')
        self.trace=MaintenanceTrace(self.f.model.registry,self.identity,writer=self.writer)
        self.trace.begin(self.at(),self.clock.sim,self.sources)
        self.f.sample(.05); self.f.sample(.05)
        self.assertTrue(self.trace.ready(self.at(),self.clock.sim))
        request={**self.f.request,'mission_id':self.identity['mission_id'],'action_id':self.identity['action_id'],
                 'command_id':uuid.uuid4().hex,'arguments':{**self.f.request['arguments'],'control_revision':3}}
        self.f.start(request); self.f.send(request,'cancel')
        self.f.workers[-1].exited=True; self.owner.tick()
        self.f.hold.released=True; self.owner.tick()
        for _ in range(24): self.f.sample(.5)
        value=self.trace.assess(self.at(),self.clock.sim)
        self.assertIsNotNone(value)
        self.assertTrue(value['recovery_observation']['verified'])
        self.assertFalse(value['observed_reset_restore_chain'])
        self.assertFalse(value['post_restore_recovery_verified'])
