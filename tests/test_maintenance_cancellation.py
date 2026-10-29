"""Cancellation owns a new isolation effect, or explicitly retains uncertainty."""
import copy
import uuid
from unittest.mock import patch

from orbital.inspection_protocol import canonical
from orbital.maintenance_protocol import command, result, state
from orbital.maintenance_owner import MaintenanceOwner
import test_maintenance_restore as restore_fixture


class CancellationFixture(restore_fixture.RestoreFixture):
    def cancellation_request(self, target, timeout=20.):
        return {**target, 'command_id':uuid.uuid4().hex, 'action_id':uuid.uuid4().hex,
            'op':'cancel_isolate', 'arguments':{'machine_id':target['arguments']['machine_id'],
                'station_run_id':target['arguments']['station_run_id'],
                'target_action_id':target['action_id'], 'timeout_s':timeout}}

    def cancel_isolate(self, target, timeout=20.):
        self.cancel_request = self.cancellation_request(target, timeout)
        reply = self.send(self.cancel_request)
        self.assertTrue(reply['accepted'], reply)
        self.workflow = self.owner.cancellation
        self.owner.tick()
        state(canonical(self.owner.state()))
        return reply

    def launch_isolation(self):
        self.sample(.05); self.sample(.05)
        self.assertIsNotNone(self.owner.active)
        self.assertEqual(self.owner.active.operation, 'isolate')
        self.assertEqual(self.owner.active.request['action_id'], self.workflow.isolation_id)
        self.hold.acknowledged = True
        for _ in range(3): self.owner.tick()

    def complete_isolation(self):
        self.apply()
        self.sample(.05); self.sample(.05)
        self.hold.released = True; self.owner.tick()
        self.assertIsNotNone(self.owner.cancellation)  # New observations after release are required.
        self.sample(.05); self.sample(.05)
        self.assertIsNone(self.owner.cancellation)
        return result(self.results[-1])

    def completed(self):
        self.original = copy.deepcopy(self.restored())
        self.cancel_isolate(self.restore_request)
        self.launch_isolation()
        return self.complete_isolation()


class CancellationTests(CancellationFixture):
    def test_cancelled_restore_gets_distinct_inspection_effect_and_final_pair(self):
        self.prepare(); self.start(self.restore_request); self.apply()
        self.cancel_isolate(self.restore_request)
        self.assertEqual(self.model.control_revision, 3)
        self.sample(.05); self.sample(.05)
        self.hold.released = True; self.owner.tick()
        original = copy.deepcopy(self.results[-1])
        self.assertEqual(original['status'], 'cancelled')
        self.assertTrue(original['state_change']['verification']['powered'])
        self.launch_isolation()
        self.assertEqual(len(self.workers), 4)
        value = self.complete_isolation()
        self.assertEqual((value['status'], value['final_state']), ('succeeded', 'isolated'))
        self.assertEqual(value['target_result'], original)
        self.assertEqual(value['isolation_result']['operation'], 'isolate')
        self.assertNotEqual(value['isolation_action_id'], value['action_id'])
        self.assertEqual(self.model.control_revision, 4)
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertFalse(self.model.reset_available('pump'))
        self.assertFalse(self.model.restore_available('pump'))
        self.assertEqual(len(value['verification']['samples']), 2)
        self.assertGreater(value['verification']['samples'][0]['stamp_ns'], value['isolation_result']['finished_sim_stamp_ns'])
        self.assertLess(len(canonical(self.owner.state()).encode()), 16384)

    def test_cancel_before_restore_reap_observes_isolation_without_another_effect(self):
        self.prepare(); self.start(self.restore_request)
        worker = self.workers[-1]
        self.cancel_isolate(self.restore_request)
        self.assertEqual(worker.stopped, 1)
        self.assertIsNotNone(self.owner.active)
        self.assertIsNone(self.workflow.result)
        worker.exited = True; self.owner.tick()
        self.hold.released = True; self.owner.tick()
        self.assertIsNone(self.workflow.result)
        self.sample(.05); self.sample(.05)
        value = result(self.results[-1])
        self.assertEqual(value['final_state'], 'isolated')
        self.assertEqual(value['target_result']['state_change']['disposition'], 'not_applied')
        self.assertIsNone(value['isolation_request'])
        self.assertIsNone(value['isolation_result'])
        self.assertEqual((self.model.control_revision, len(self.workers)), (2, 3))

    def test_cancel_before_original_isolate_actuation_still_isolates_powered_target(self):
        self.sample(.05); self.sample()
        self.start(self.request)
        self.cancel_isolate(self.request)
        self.workers[-1].exited = True; self.owner.tick()
        self.hold.released = True; self.owner.tick()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.launch_isolation()
        value = self.complete_isolation()
        self.assertEqual(value['target_result']['state_change']['disposition'], 'not_applied')
        self.assertEqual(value['isolation_result']['state_change']['disposition'], 'applied')
        self.assertEqual(self.model.control_revision, 1)

    def test_completed_restore_history_is_preserved_and_exact_replay_has_no_effect(self):
        value = self.completed()
        self.assertEqual(value['target_result'], self.original)
        self.assertEqual(self.send(self.restore_request, 'query')['action']['result'], self.original)
        replay = {**self.cancel_request, 'command_id':uuid.uuid4().hex}
        reply = self.send(replay)
        self.assertEqual(reply['action']['result'], value)
        reply['action']['result']['final_state'] = 'energized'
        self.assertEqual(self.send(self.cancel_request, 'query')['action']['result']['final_state'], 'isolated')
        self.assertEqual((self.model.control_revision, len(self.workers)), (4, 4))

    def test_reusing_target_under_new_workflow_id_does_not_repeat_isolation(self):
        self.completed()
        self.assertEqual(self.send(self.cancellation_request(self.restore_request))['error_code'], 'cancellation_conflict')
        changed = copy.deepcopy(self.cancel_request); changed['arguments']['timeout_s'] = 3.
        self.assertEqual(self.send(changed)['error_code'], 'action_conflict')
        self.assertEqual(self.model.control_revision, 4)

    def test_target_owner_mission_machine_generation_and_nested_workflow_refused(self):
        self.restored()
        request = self.cancellation_request(self.restore_request)
        variants = [({**request,'run_id':uuid.uuid4().hex}, 'wrong_run'),
                    ({**request,'mission_id':uuid.uuid4().hex}, 'cancellation_target_required')]
        for key,value in [('machine_id','cooler'), ('station_run_id',uuid.uuid4().hex), ('target_action_id',uuid.uuid4().hex)]:
            variants.append(({**request,'arguments':{**request['arguments'],key:value}}, 'cancellation_target_required'))
        for changed,error in variants:
            self.assertEqual(self.send(changed)['error_code'], error)
        self.cancel_isolate(self.restore_request)
        nested = self.cancellation_request(self.cancel_request)
        self.assertEqual(self.send(nested)['error_code'], 'cancellation_target_required')
        self.assertEqual(self.model.control_revision, 3)

    def test_old_superseded_action_cannot_authorize_a_new_isolation(self):
        self.restored()
        self.assertEqual(self.send(self.cancellation_request(self.request))['error_code'], 'target_superseded')
        self.model.reset()
        self.assertEqual(self.send(self.cancellation_request(self.restore_request))['error_code'], 'target_superseded')

    def test_reserves_history_and_child_identity_before_cancelling(self):
        self.restored()
        while self.owner.capacity() > 1: self.owner.actions[uuid.uuid4().hex] = self.owner.actions[self.request['action_id']]
        self.assertEqual(self.send(self.cancellation_request(self.restore_request))['error_code'], 'history_full')
        self.assertIsNone(self.owner.cancellation)

    def test_reserved_final_slot_can_run_one_child_and_never_go_negative(self):
        self.restored()
        while self.owner.capacity() > 2: self.owner.actions[uuid.uuid4().hex] = self.owner.actions[self.request['action_id']]
        self.cancel_isolate(self.restore_request)
        self.assertEqual(self.owner.capacity(), 0)
        spoof = {**self.request, 'action_id':self.workflow.isolation_id,
                 'arguments':{**self.request['arguments'],'control_revision':3}}
        self.assertEqual(self.send(spoof)['error_code'], 'action_conflict')
        self.launch_isolation(); value = self.complete_isolation()
        self.assertEqual(value['final_state'], 'isolated')
        self.assertEqual(self.owner.capacity(), 0)
        state(canonical(self.owner.state()))

    def test_unrelated_admission_is_blocked_between_original_and_compensation(self):
        self.restored(); self.cancel_isolate(self.restore_request)
        foreign = {**self.request,'action_id':uuid.uuid4().hex,'arguments':{**self.request['arguments'],'control_revision':3}}
        self.assertEqual(self.send(foreign)['error_code'], 'busy')
        self.assertFalse(self.owner.ready())
        self.assertTrue(self.send(self.restore_request,'query')['accepted'])

    def test_unknown_restore_outcome_never_authorizes_compensation(self):
        self.prepare(); self.start(self.restore_request)
        actual = self.model.station.intervene
        def unknown(*args, **kwargs):
            actual(*args, **kwargs)
            raise OSError('controlled loss after energization')
        with patch.object(self.model.station,'intervene',side_effect=unknown): self.apply()
        self.assertTrue(self.model.station.machines['pump'].powered)
        self.cancel_isolate(self.restore_request, .5)
        for _ in range(5): self.sample(.5)
        self.assertEqual(self.workflow.phase, 'unresolved')
        self.assertIsNone(self.workflow.result)
        self.assertIsNone(self.workflow.child_request)
        self.assertIsNotNone(self.hold.owner)
        self.assertEqual(len(self.workers), 3)

    def test_unknown_compensation_keeps_ownership_even_when_power_appears_off(self):
        self.restored(); self.cancel_isolate(self.restore_request); self.launch_isolation()
        actual = self.model.station.intervene
        def unknown(*args, **kwargs):
            actual(*args, **kwargs)
            raise OSError('controlled loss after isolation')
        with patch.object(self.model.station,'intervene',side_effect=unknown): self.apply()
        for _ in range(3): self.sample(.5)
        self.assertFalse(self.model.station.machines['pump'].powered)
        self.assertEqual(self.workflow.phase, 'unresolved')
        self.assertIsNone(self.workflow.result)
        self.assertIsNotNone(self.hold.owner)

    def test_late_capture_expiry_reports_energized_failure_without_retries(self):
        self.restored(); self.cancel_isolate(self.restore_request); self.launch_isolation()
        actual = self.owner.load
        def delayed(*args):
            value = actual(*args); self.clock.advance(.201); return value
        self.owner.load = delayed
        self.apply(); self.hold.released = True; self.owner.tick()
        self.sample(.05); self.sample(.05)
        value = result(self.results[-1])
        self.assertEqual((value['status'],value['final_state']), ('failed','energized'))
        self.assertEqual(value['isolation_result']['state_change']['disposition'], 'not_applied')
        self.assertEqual((self.model.control_revision,len(self.workers)), (3,4))

    def test_missing_pose_rejects_one_child_admission_and_preserves_energized_measurement(self):
        self.restored(); self.cancel_isolate(self.restore_request)
        self.live.available = False
        self.sample(.05); self.sample(.05); self.owner.tick()
        value = result(self.results[-1])
        self.assertEqual((value['status'],value['final_state']), ('failed','energized'))
        self.assertEqual(value['isolation_rejection'], 'unavailable')
        self.assertIsNone(value['isolation_result'])
        self.assertEqual(len(self.workers),3)

    def test_second_cancel_stops_future_compensation_and_reports_actual_power(self):
        self.restored(); self.cancel_isolate(self.restore_request)
        self.send(self.cancel_request,'cancel')
        self.sample(.05); self.sample(.05)
        value = result(self.results[-1])
        self.assertEqual(value['status'], 'cancelled')
        self.assertEqual(value['final_state'], 'unverified')
        self.assertIsNone(value['isolation_request'])
        self.assertTrue(self.model.station.machines['pump'].powered)

    def test_deadline_after_applied_isolation_waits_for_known_child_release(self):
        self.restored(); self.cancel_isolate(self.restore_request, .5); self.launch_isolation()
        self.apply(); self.clock.advance(.5); self.owner.tick()
        self.assertIsNotNone(self.owner.cancellation)
        self.assertIsNotNone(self.owner.active)
        self.sample(.05); self.sample(.05)
        self.hold.released=True; self.owner.tick()
        value = result(self.results[-1])
        self.assertEqual(value['status'],'timed_out')
        self.assertEqual(value['final_state'],'unverified')
        self.assertEqual(value['isolation_result']['state_change']['disposition'],'applied')
        self.assertFalse(self.model.station.machines['pump'].powered)

    def test_source_loss_never_claims_a_fresh_safe_state(self):
        self.restored(); self.cancel_isolate(self.restore_request)
        self.owner.authority = lambda: (_ for _ in ()).throw(ValueError('controlled source change'))
        self.owner.tick()
        value = result(self.results[-1])
        self.assertEqual((value['status'],value['final_state']),('failed','unverified'))
        self.assertIsNone(value['verification']); self.assertIsNone(value['isolation_request'])

    def test_noop_confirmation_needs_new_pair_and_rejects_contradictory_member(self):
        self.isolated(); self.cancel_isolate(self.request)
        self.assertIsNone(self.workflow.result)
        self.clock.advance(.05); first=self.model.sample(self.clock.sim)
        first['machines']['pump']['powered']=True
        self.owner.telemetry(canonical(first)); self.owner.tick()
        self.assertIsNone(self.workflow.result)
        self.sample(.05)
        value = result(self.results[-1])
        self.assertEqual((value['status'],value['final_state']),('failed','unverified'))
        self.assertEqual(self.model.control_revision,1)

    def test_both_confirmation_samples_must_remain_fresh_in_both_clocks(self):
        for clock in ('wall','simulation','both'):
            with self.subTest(clock=clock):
                f=CancellationFixture(); f.setUp(); self.addCleanup(f.doCleanups)
                f.isolated(); f.cancel_isolate(f.request)
                for delta in (.05,.95):
                    f.clock.advance(delta); f.owner.telemetry(canonical(f.model.sample(f.clock.sim)))
                if clock in ('wall','both'): f.clock.wall += .1
                if clock in ('simulation','both'): f.clock.sim += 100000000
                f.owner.tick()
                self.assertIsNone(f.workflow.result)
                f.sample(.05); f.sample(.05)
                self.assertEqual(result(f.results[-1])['final_state'],'isolated')

    def test_authoritative_control_change_after_pair_prevents_terminal_claim(self):
        self.isolated(); self.cancel_isolate(self.request)
        for _ in range(2):
            self.clock.advance(.05); self.owner.telemetry(canonical(self.model.sample(self.clock.sim)))
        # Another actual model effect occurs after those packets but before the
        # workflow tick. Their age alone cannot prove the control is current.
        context={'owner_run_id':uuid.uuid4().hex,'mission_id':uuid.uuid4().hex,
            'action_id':uuid.uuid4().hex,'request_sha256':'a'*64,'machine_id':'cooler'}
        self.model.isolate(context,self.model.run_id,self.model.control_revision,
            admission=lambda:{'stamp_ns':self.clock.sim,'distance_m':.1,'speed_mps':0.})
        self.owner.tick()
        value=result(self.results[-1])
        self.assertEqual((value['status'],value['final_state']),('failed','unverified'))
        self.assertEqual(value['error_code'],'precondition_failed')
        self.assertIsNone(value['verification'])

    def test_deadline_crossed_during_authority_io_cannot_report_success(self):
        self.isolated(); self.cancel_isolate(self.request,timeout=.2)
        for _ in range(2):
            self.clock.advance(.05); self.owner.telemetry(canonical(self.model.sample(self.clock.sim)))
        self.owner.authority=lambda:self.clock.advance(.11)
        self.owner.tick()
        value=result(self.results[-1])
        self.assertGreater(self.clock.wall,self.workflow.deadline)
        self.assertEqual((value['status'],value['error_code']),('timed_out','deadline_exceeded'))
        self.assertEqual(value['final_state'],'isolated')
        self.assertIsNone(value['isolation_request'])
        self.assertEqual(self.model.control_revision,1)

    def test_final_pair_after_child_cannot_be_replaced_by_cached_isolation_result(self):
        self.restored(); self.cancel_isolate(self.restore_request); self.launch_isolation(); self.apply()
        self.sample(.05); self.sample(.05)
        self.hold.released=True; self.owner.tick()
        self.assertIsNotNone(self.workflow.child.result)
        self.assertIsNone(self.workflow.result)
        self.clock.advance(1.1); self.owner.tick()
        self.assertIsNone(self.workflow.result)
        self.sample(.05); self.sample(.05)
        self.assertEqual(result(self.results[-1])['final_state'],'isolated')

    def test_schema_binds_request_children_and_both_fresh_measurements(self):
        value=self.completed()
        variants=[]
        for key,changed in [('target_result',value),('isolation_result',value['target_result']),
                            ('request_sha256','0'*64),('final_state','energized'),('hold_released',False)]:
            candidate=copy.deepcopy(value); candidate[key]=changed; variants.append(candidate)
        for key,changed in [('received_age_s',1.),('simulation_age_s',1.),('powered',True),('current_a',1.)]:
            candidate=copy.deepcopy(value); candidate['verification']['samples'][0][key]=changed; variants.append(candidate)
        for candidate in variants:
            with self.assertRaises(ValueError): result(candidate)
        for key,changed in [('target_action_id',value['action_id']),('timeout_s',31.)]:
            request=copy.deepcopy(self.cancel_request); request['arguments'][key]=changed
            with self.assertRaises(ValueError): command(canonical(request))

    def test_owner_restart_does_not_adopt_workflow_or_renew_old_identity(self):
        self.restored(); self.cancel_isolate(self.restore_request)
        fresh=MaintenanceOwner(self.root,self.model,self.live,self.hold,lambda _:None,
            now=lambda:self.clock.wall,sim_ns=lambda:self.clock.sim)
        self.assertEqual(fresh.handle(canonical(self.cancel_request))['error_code'],'wrong_run')
        changed={**self.cancel_request,'run_id':fresh.run_id}
        self.assertEqual(fresh.handle(canonical(changed))['error_code'],'cancellation_target_required')

    def test_shutdown_retains_unknown_effect_and_never_launches_compensation(self):
        self.prepare(); self.start(self.restore_request)
        with patch.object(self.model.station,'intervene',side_effect=RuntimeError('unknown')): self.apply()
        self.cancel_isolate(self.restore_request); self.owner.shutdown(); self.owner.tick()
        self.assertIsNone(self.workflow.result)
        self.assertIsNone(self.workflow.child_request)
        self.assertIsNotNone(self.hold.owner)
