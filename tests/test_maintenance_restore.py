"""Restoration is an owned energized effect, separate from sampled recovery."""
import copy
import uuid
from unittest.mock import patch

from orbital.inspection_protocol import canonical
from orbital.maintenance_observation import RecoveryEvidence
from orbital.maintenance_protocol import command, result, state
from orbital.machines import Fault
from orbital.station_control import EffectRejected
import test_maintenance_reset as reset_fixture


class RestoreFixture(reset_fixture.ResetFixture):
    def prepare(self):
        reset = self.reset_complete()
        self.restore_request = {**self.request, 'op': 'restore', 'action_id': uuid.uuid4().hex,
            'command_id': uuid.uuid4().hex, 'arguments': {**self.request['arguments'],
                'control_revision': 2, 'reset_action_id': reset['action_id']}}
        return reset

    def restored(self):
        self.prepare(); self.start(self.restore_request); self.apply()
        self.sample(); self.sample()
        return self.finish()


class RestoreOwnerTests(RestoreFixture):
    def test_actual_station_third_inspection_and_new_energized_effect(self):
        value = self.restored()
        self.assertEqual((value['schema_version'], value['operation'], value['status']), (3, 'restore', 'succeeded'))
        self.assertEqual(value['reset'], self.results[1])
        self.assertEqual(len({v['observation']['observation_id'] for v in self.results}), 3)
        self.assertEqual(len(self.workers), 3)
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertEqual(value['state_change']['effect'], self.model.last_applied)
        self.assertEqual(value['state_change']['verification']['control_revision'], 3)
        self.assertTrue(value['state_change']['verification']['powered'])
        self.assertGreater(value['state_change']['verification']['current_a'], 0.)
        self.assertLess(len(canonical(self.owner.state()).encode()), 16384)

    def test_new_powered_samples_establish_recovery_only_after_full_window(self):
        value = self.restored()
        recovery = RecoveryEvidence(self.model.registry)
        for index in range(22):
            packet = self.sample()
            self.assertEqual(packet['control']['last_applied'], value['state_change']['effect'])
            self.assertGreater(packet['stamp_ns'], value['state_change']['effect']['applied_stamp_ns'])
            recovery.observe(canonical(packet), self.clock.wall, self.clock.sim)
            if index >= 1:
                observed = recovery.measure(self.clock.wall, self.clock.sim)
                self.assertEqual(observed['verified'], index == 21)
        self.assertEqual(observed['window']['duration_s'], 10.)

    def test_new_fault_after_restore_is_energized_without_recovery_claim(self):
        self.prepare(); self.start(self.restore_request); self.apply()
        # Evaluator fault injection after the real effect; never a planner field.
        self.model.station.machines['pump'].fault = Fault.JAM
        self.sample(); self.sample(); value = self.finish()
        self.assertEqual(value['status'], 'succeeded')
        self.assertEqual(value['state_change']['verification']['vibration_au'], 3.)
        self.assertTrue(value['state_change']['verification']['powered'])
        recovery = RecoveryEvidence(self.model.registry)
        for _ in range(24):
            packet = self.sample(); recovery.observe(canonical(packet), self.clock.wall, self.clock.sim)
        self.assertFalse(recovery.measure(self.clock.wall, self.clock.sim)['verified'])

    def test_restore_requires_verified_reset_not_an_isolation_receipt(self):
        self.isolated()
        request = {**self.request, 'op':'restore', 'action_id':uuid.uuid4().hex,
            'arguments':{**self.request['arguments'], 'control_revision':1,
                         'reset_action_id':self.request['action_id']}}
        self.assertEqual(self.send(request)['error_code'], 'reset_required')
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_an_unapplied_cancelled_reset_cannot_authorize_restore(self):
        self.isolated(); self.start(self.reset_request)
        self.send(self.reset_request, 'cancel'); self.apply(); self.finish()
        request = {**self.request, 'op':'restore', 'action_id':uuid.uuid4().hex,
            'arguments':{**self.request['arguments'], 'control_revision':1,
                         'reset_action_id':self.reset_request['action_id']}}
        self.assertEqual(self.send(request)['error_code'], 'reset_required')

    def test_wrong_owner_mission_machine_run_and_revision_are_refused(self):
        self.prepare()
        variants = [({**self.restore_request, 'mission_id':uuid.uuid4().hex}, 'reset_required'),
                    ({**self.restore_request, 'run_id':uuid.uuid4().hex}, 'wrong_run')]
        for field,value,error in (('reset_action_id',uuid.uuid4().hex,'reset_required'),
                ('machine_id','cooler','reset_required'), ('station_run_id',uuid.uuid4().hex,'wrong_station_state'),
                ('control_revision',1,'wrong_station_state')):
            variants.append(({**self.restore_request,'arguments':{**self.restore_request['arguments'],field:value}},error))
        for value,error in variants:
            with self.subTest(error=error): self.assertEqual(self.send(value)['error_code'],error)
        self.assertEqual(len(self.workers),2)

    def test_retained_high_vibration_refuses_energization(self):
        self.prepare()
        self.owner.reset_evidence.high_vibration.add('pump')
        self.assertEqual(self.send(self.restore_request)['error_code'],'restore_not_eligible')
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_missing_source_after_reset_refuses_energization(self):
        self.prepare(); self.owner.telemetry('{"invalid":1}')
        self.assertEqual(self.send(self.restore_request)['error_code'],'restore_not_eligible')

    def test_cooldown_deadline_never_energizes_or_spends_attempt(self):
        self.prepare()
        self.model.station.machines['pump'].temperature_c = 50.
        self.sample(.05)
        self.restore_request['arguments']['timeout_s'] = .3
        self.start(self.restore_request); self.sample()
        self.assertEqual(self.finish()['status'],'timed_out')
        self.assertEqual(len(self.workers),2)
        self.assertTrue(self.model.restore_available('pump'))
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_capture_delay_after_io_prevents_energization(self):
        self.prepare(); self.start(self.restore_request)
        real_load = self.owner.load
        def late(*args):
            value=real_load(*args); self.clock.advance(.201); return value
        self.owner.load=late; self.apply()
        self.assertEqual(self.finish()['state_change']['disposition'],'not_applied')
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertTrue(self.model.restore_available('pump'))

    def test_final_model_temperature_check_rejects_late_heating(self):
        self.prepare(); self.start(self.restore_request)
        real_load=self.owner.load
        def hot(*args):
            value=real_load(*args); self.model.station.machines['pump'].temperature_c=41.; return value
        self.owner.load=hot; self.apply()
        self.assertEqual(self.finish()['state_change']['disposition'],'not_applied')
        self.assertTrue(self.model.restore_available('pump'))

    def test_cancel_before_capture_reap_preserves_isolation(self):
        self.prepare(); self.start(self.restore_request)
        self.send(self.restore_request,'cancel'); self.owner.tick()
        self.assertIsNone(self.hold.release_at)
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.apply(); value=self.finish()
        self.assertEqual((value['status'],value['state_change']['disposition']),('cancelled','not_applied'))
        self.assertTrue(self.model.restore_available('pump'))

    def test_cancel_after_restore_reports_energized_effect_without_rollback(self):
        self.prepare(); self.start(self.restore_request); self.apply()
        self.send(self.restore_request,'cancel'); self.owner.tick()
        self.assertIsNone(self.hold.release_at)
        self.sample(); self.sample(); value=self.finish()
        self.assertEqual((value['status'],value['state_change']['disposition']),('cancelled','applied'))
        self.assertTrue(value['state_change']['verification']['powered'])
        self.assertEqual(self.model.control_revision,3)
        self.assertTrue(self.model.station.machines['pump'].powered)

    def test_deadline_after_restore_reports_energized_state(self):
        self.prepare(); self.restore_request['arguments']['timeout_s']=.5
        self.start(self.restore_request); self.apply(); self.sample(); self.sample()
        value=self.finish()
        self.assertEqual(value['status'],'timed_out')
        self.assertTrue(value['state_change']['verification']['powered'])

    def test_unknown_exception_after_power_on_keeps_unresolved_ownership(self):
        self.prepare(); self.start(self.restore_request)
        actual=self.model.station.intervene
        def lost(*args,**kwargs):
            actual(*args,**kwargs); raise RuntimeError('controlled post-energization failure')
        with patch.object(self.model.station,'intervene',side_effect=lost): self.apply()
        self.assertEqual(self.owner.active.phase,'unresolved')
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.assertFalse(self.model.restore_available('pump'))
        self.assertEqual(len(self.results),2)
        self.assertIsNone(self.hold.release_at)

    def test_unknown_uncommitted_restore_still_reserves_attempt(self):
        self.prepare(); self.start(self.restore_request)
        with patch.object(self.model.station,'intervene',side_effect=RuntimeError('unknown boundary')): self.apply()
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertFalse(self.model.restore_available('pump'))
        self.assertEqual(self.owner.active.phase,'unresolved')

    def test_exact_replay_and_new_request_cannot_repeat_energization(self):
        value=self.restored()
        self.assertEqual(self.send(self.restore_request)['action']['result'],value)
        again={**self.restore_request,'action_id':uuid.uuid4().hex,
            'arguments':{**self.restore_request['arguments'],'control_revision':3}}
        self.assertEqual(self.send(again)['error_code'],'restore_attempt_exhausted')
        self.assertEqual(len(self.workers),3)
        effect=value['state_change']['effect']
        context={k:effect[k] for k in ('owner_run_id','mission_id','action_id','request_sha256','machine_id')}
        def forbidden(): raise AssertionError('replay reached admission')
        self.assertEqual(self.model.restore_machine(context,self.model.run_id,2,
            reset_action_id=self.reset_request['action_id'],admission=forbidden),effect)
        with self.assertRaises(EffectRejected):
            self.model.restore_machine({**context,'owner_run_id':uuid.uuid4().hex},self.model.run_id,3,
                reset_action_id=self.reset_request['action_id'],admission=forbidden)

    def test_old_reset_cannot_be_reused_after_a_later_isolation(self):
        self.restored()
        machine=self.model.station.machines['pump']
        machine.powered=False  # Evaluator simulates later isolation; no new reset budget.
        again={**self.restore_request,'action_id':uuid.uuid4().hex,
            'arguments':{**self.restore_request['arguments'],'control_revision':3}}
        self.assertEqual(self.send(again)['error_code'],'restore_attempt_exhausted')
        self.assertFalse(machine.powered)

    def test_model_refuses_forged_reset_ownership_before_admission(self):
        reset=self.prepare()
        context={k:reset['state_change']['effect'][k] for k in
            ('owner_run_id','mission_id','action_id','request_sha256','machine_id')}
        context.update(action_id=uuid.uuid4().hex,request_sha256='e'*64)
        calls=[]
        def admit():
            calls.append('admission')
            return {'stamp_ns':self.clock.sim,'distance_m':.6,'speed_mps':.001}
        for changes,prior in (({'owner_run_id':uuid.uuid4().hex},self.reset_request['action_id']),
                ({'mission_id':uuid.uuid4().hex},self.reset_request['action_id']),
                ({},self.request['action_id']),({},uuid.uuid4().hex)):
            with self.subTest(changes=changes,prior=prior),self.assertRaises(EffectRejected):
                self.model.restore_machine({**context,**changes},self.model.run_id,2,
                    reset_action_id=prior,admission=admit)
        self.assertEqual(calls,[])
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_first_post_restore_power_measurement_cannot_be_hidden_by_latest(self):
        self.prepare(); self.start(self.restore_request); self.apply()
        self.clock.advance(.5); packet=self.model.sample(self.clock.sim)
        packet['machines']['pump']['powered']=False
        self.owner.telemetry(canonical(packet)); self.owner.tick(); self.sample()
        self.assertEqual(self.owner.active.phase,'unresolved')
        self.assertEqual(self.owner.active.stop_reason[1],'restore_not_verified')
        self.assertIsNone(self.hold.release_at)

    def test_each_post_restore_sample_remains_fresh_in_both_clocks(self):
        for wall,sim in ((.6,.6),(.6,0.),(0.,.6)):
            with self.subTest(wall=wall,sim=sim):
                f=RestoreFixture();f.setUp()
                try:
                    f.prepare();f.start(f.restore_request);f.apply();f.sample();f.sample()
                    f.clock.wall+=wall;f.clock.sim+=round(sim*1e9)
                    f.hold.released=True;f.owner.tick()
                    self.assertIsNotNone(f.owner.active)
                    self.assertEqual(len(f.results),2)
                finally:f.doCleanups()

    def test_schema_requires_reset_chain_and_reports_measured_power(self):
        value=self.restored()
        for key,changed in (('schema_version',2),('reset',value),('reset',value['reset']['isolation']),
                            ('mission_id','f'*32),('observation',None)):
            bad=copy.deepcopy(value);bad[key]=changed
            with self.subTest(key=key),self.assertRaises(ValueError):result(bad)
        for key,changed in (('powered',False),('current_a',True),('voltage_v',-1),('control_revision',2)):
            bad=copy.deepcopy(value);bad['state_change']['verification'][key]=changed
            with self.subTest(key=key),self.assertRaises(ValueError):result(bad)
        bad=copy.deepcopy(self.restore_request);bad['arguments'].pop('reset_action_id')
        with self.assertRaises(ValueError):command(canonical(bad))
