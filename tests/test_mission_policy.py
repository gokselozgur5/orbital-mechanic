"""Policy decisions consume real bound captures; hidden setup stays in tests."""
import copy
import hashlib
import json
import unittest
import uuid
from unittest.mock import patch

from orbital import mission_policy as policy_module
from orbital.inspection_protocol import canonical

from orbital.machines import Fault
from mission_policy_fixture import PolicyFixture


class MissionPolicyTests(PolicyFixture):
    def test_recoverable_policy_uses_actual_capture_then_owned_effects_and_recovery(self):
        decision=self.inspection_step()
        self.assertEqual(decision['decision'],'recoverable')
        self.assertEqual(self.m.phase,'intervene')
        self.assertEqual(decision['candidate_machine'],'pump')
        isolate=self.maintain(self.f.request)['result']
        self.assertEqual(isolate['status'],'succeeded')
        reset_request={**self.f.request,'op':'reset','action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex,
            'arguments':{**self.f.request['arguments'],'control_revision':1,'isolation_action_id':isolate['action_id']}}
        reset=self.maintain(reset_request)['result']
        self.assertEqual(reset['status'],'succeeded')
        restore_request={**self.f.request,'op':'restore','action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex,
            'arguments':{**self.f.request['arguments'],'control_revision':2,'reset_action_id':reset['action_id']}}
        self.assertEqual(self.maintain(restore_request)['result']['status'],'succeeded')
        self.assertTrue(self.recovery())
        self.assertEqual(self.m.outcome,'recovered')
        self.assertEqual(self.f.model.control_revision,3)

    def test_healthy_policy_inspects_catalog_without_intervention_before_verifying(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        self.f.sample(.05);self.f.sample(.05)
        choices=[]
        for _ in range(3):
            report=self.inspection_step();choices.append(report['machine_id'])
        self.assertEqual(choices,['pump','cooler','power_unit'])
        self.assertEqual(report['decision'],'healthy')
        self.assertEqual(self.f.model.control_revision,0)
        self.assertTrue(self.recovery())
        self.assertEqual(self.m.outcome,'healthy')
        self.assertEqual(len(self.capture_records),3)

    def test_persistent_public_vibration_escalates_without_reset(self):
        self.f.model.station.machines['pump'].fault=Fault.JAM
        self.f.sample(.05);self.f.sample(.05)
        report=self.inspection_step()
        self.assertEqual(report['decision'],'persistent')
        self.m.escalate()
        self.assertEqual(self.m.outcome,'escalated')
        self.assertEqual(self.f.model.control_revision,0)

    def test_ambiguous_multiple_loads_get_one_repeat_then_escalate(self):
        self.f.model.station.machines['cooler'].fault=Fault.LATCHED
        self.f.sample(.05);self.f.sample(.05)
        first=self.inspection_step();second=self.inspection_step()
        self.assertEqual((first['decision'],second['decision']),('inspect_again','ambiguous'))
        self.assertEqual(first['machine_id'],second['machine_id'])
        self.m.escalate()
        self.assertEqual(self.f.model.control_revision,0)

    def test_blocked_navigation_never_admits_inspection_or_maintenance(self):
        plan=self.policy.next_machine()
        self.navigate_to(plan['machine_id'],blocked=True)
        self.assertEqual(self.m.phase,'escalate')
        with self.assertRaises(ValueError):self.policy.apply()
        self.m.escalate()
        self.assertEqual(self.m.outcome,'escalated')
        self.assertEqual(self.capture_records,[])
        self.assertEqual(self.f.model.control_revision,0)

    def test_load_priority_uses_measured_catalog_target_instead_of_pump_assumption(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        self.f.model.station.machines['cooler'].fault=Fault.LATCHED
        self.f.sample(.05);self.f.sample(.05)
        report=self.inspection_step()
        self.assertEqual((report['machine_id'],report['candidate_machine']),('cooler','cooler'))
        self.assertEqual(report['decision'],'recoverable')

    def test_hidden_fault_change_without_new_measurements_cannot_change_decision(self):
        self.capture_step()
        self.f.model.station.machines['pump'].fault=Fault.JAM
        report=self.policy.apply()
        self.assertEqual(report['decision'],'recoverable')
        self.assertEqual(self.f.model.control_revision,0)

    def test_fresh_new_measurements_contradicting_capture_force_reinspection(self):
        self.capture_step()
        self.f.model.station.machines['pump'].fault=Fault.NONE
        self.f.sample(.05);self.f.sample(.05)
        report=self.policy.apply()
        self.assertEqual(report['decision'],'inspect_again')
        self.assertEqual(report['reason'],'changing_public_measurements')
        self.assertEqual(self.f.model.control_revision,0)

    def test_earlier_high_vibration_cannot_be_erased_by_later_low_samples(self):
        self.f.model.station.machines['pump'].fault=Fault.JAM
        self.f.sample(.05)
        self.f.model.station.machines['pump'].fault=Fault.LATCHED
        self.f.sample(.05);self.f.sample(.05)
        first=self.inspection_step()
        self.assertEqual(first['decision'],'inspect_again')
        self.assertEqual(first['reason'],'contradictory_vibration_history')
        second=self.inspection_step()
        self.assertEqual(second['decision'],'ambiguous')
        self.assertEqual(self.f.model.control_revision,0)

    def test_changing_candidate_inspection_loop_terminates_at_global_budget(self):
        names=('pump','cooler','power_unit')
        for index in range(6):
            plan=self.policy.next_machine()
            self.navigate_to(plan['machine_id'])
            other=names[(names.index(plan['machine_id'])+1)%3]
            for name,value in self.f.model.station.machines.items():
                value.fault=Fault.LATCHED if name==other else Fault.NONE
            self.actual_inspect(plan['machine_id'])
            report=self.policy.apply()
        self.assertEqual(report['decision'],'ambiguous')
        self.assertEqual(self.m.phase,'escalate')
        self.assertEqual(sum(self.policy.visits.values()),6)
        self.assertEqual(self.f.model.control_revision,0)

    def rewrite_record(self,change):
        reference=self.m.inspection['observation']
        path=self.f.root/'runtime/inspection'/reference['observation_id']/'observation.json'
        record=json.loads(path.read_text())
        change(record)
        raw=canonical(record).encode()
        path.write_bytes(raw)
        # Deliberately coherent reference mutation isolates source binding from
        # the artifact reader's already tested byte-integrity check.
        reference['manifest_sha256']=hashlib.sha256(raw).hexdigest()

    def test_coherently_rehashed_measurement_cannot_replace_observed_source(self):
        self.capture_step()
        self.rewrite_record(lambda value:value['measurements'].update(vibration_au=3.))
        with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.f.model.control_revision,0)

    def test_coherently_rehashed_boolean_measurement_type_is_refused(self):
        self.capture_step()
        self.rewrite_record(lambda value:value['measurements'].update(powered=1))
        with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()

    def test_coherently_rehashed_foreign_action_context_is_refused(self):
        self.capture_step()
        self.rewrite_record(lambda value:value['inspection_action'].update(mission_id=uuid.uuid4().hex))
        with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.m.phase,'diagnose')

    def test_changed_image_bytes_cannot_supply_an_inspection(self):
        self.capture_step()
        path=self.f.root/'runtime/inspection'/self.m.inspection['observation']['observation_id']/'image.ppm'
        before=path.read_bytes()
        path.write_bytes(before[:-1]+bytes([before[-1]^1]))
        with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        path.write_bytes(before)
        with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.f.model.control_revision,0)

    def test_missing_selected_raw_packet_cannot_be_reconstructed_from_artifact(self):
        self.capture_step()
        self.policy.samples.clear()
        with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.m.phase,'diagnose')

    def test_bounded_retention_refuses_an_evicted_capture_source(self):
        self.capture_step()
        for _ in range(70):self.f.sample(.001)
        self.assertEqual(len(self.policy.samples),64)
        with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.f.model.control_revision,0)

    def delayed_read(self,after):
        actual=policy_module.read_capture
        def delayed(*args):
            value=actual(*args);after();return value
        return patch.object(policy_module,'read_capture',side_effect=delayed)

    def test_wall_expiry_during_artifact_io_refuses_decision(self):
        self.capture_step()
        with self.delayed_read(lambda:setattr(self.clock,'wall',self.clock.wall+1.)):
            with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.m.phase,'diagnose')

    def test_simulation_expiry_during_artifact_io_refuses_decision(self):
        self.capture_step()
        with self.delayed_read(lambda:setattr(self.clock,'sim',self.clock.sim+1000000000)):
            with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.m.phase,'diagnose')

    def test_cancellation_during_artifact_io_cannot_turn_into_a_decision(self):
        self.capture_step()
        with self.delayed_read(self.m.cancel):
            with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertEqual(self.m.outcome,'cancelled')
        self.assertIsNone(self.m.decision)

    def test_registry_change_during_artifact_io_refuses_decision(self):
        self.capture_step()
        path=self.f.root/'config/known-map.json'
        with self.delayed_read(lambda:path.write_bytes(path.read_bytes()+b'\n')):
            with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertIsNone(self.m.decision)

    def test_snapshot_mutation_cannot_change_live_plan_or_budget(self):
        plan=self.policy.next_machine()
        plan['machine_id']='power_unit'
        snapshot=self.policy.snapshot();snapshot['visits']['pump']=2
        self.assertEqual(self.policy.plan['machine_id'],'pump')
        self.assertEqual(self.policy.visits['pump'],0)

    def test_policy_never_observes_an_ack_as_a_completed_inspection(self):
        self.policy.next_machine()
        with self.assertRaises(ValueError):self.policy.apply()
        self.assertEqual(self.f.model.control_revision,0)

    def delayed_registry(self,after):
        actual=type(self.m.registry).verify_current
        def delayed(registry):
            actual(registry);after()
        return patch.object(type(self.m.registry),'verify_current',delayed)

    def test_wall_expiry_during_registry_io_refuses_decision(self):
        self.capture_step()
        with self.delayed_registry(lambda:setattr(self.clock,'wall',self.clock.wall+1.)):
            with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertIsNone(self.m.decision)

    def test_simulation_expiry_during_registry_io_refuses_decision(self):
        self.capture_step()
        with self.delayed_registry(lambda:setattr(self.clock,'sim',self.clock.sim+1000000000)):
            with self.assertRaisesRegex(ValueError,'policy_inspection_unavailable'):self.policy.apply()
        self.assertIsNone(self.m.decision)

    def test_expired_pair_after_registry_io_cannot_choose_next_machine(self):
        with self.delayed_registry(lambda:setattr(self.clock,'wall',self.clock.wall+1.)):
            with self.assertRaises(ValueError):self.policy.next_machine()
        self.assertIsNone(self.policy.plan)
