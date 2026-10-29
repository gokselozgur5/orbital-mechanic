"""Mission transitions consume actual clients/owners over controlled transport.

Navigation peers, hold and camera inputs are controlled here. The durable station
and its real causal effects are not replaced by fabricated success receipts.
"""
import copy
from types import SimpleNamespace as NS
import unittest
import uuid

from orbital.inspection_client import InspectionClient
from orbital.inspection_owner import InspectionOwner
from orbital.inspection_protocol import canonical
from orbital.maintenance_client import MaintenanceClient
from orbital.maintenance_protocol import result as maintenance_result
from orbital.machines import Fault
from orbital.mission_state import MissionState
from orbital.navigation_client import execute
from orbital.navigation_command import CommandGate
import test_inspection_owner as inspection
import test_maintenance_journal as maintenance
import test_navigation_client as navigation
import test_supervisor_arm as supervisor


class MissionFixture(unittest.TestCase):
    def setUp(self):
        self.f = maintenance.JournalFixture()
        self.f.setUp(); self.addCleanup(self.f.doCleanups)
        self.clock = self.f.clock
        self.f.sample(.05); baseline = self.f.sample(.05)
        self.m = MissionState(self.f.model.registry, self.f.request['mission_id'], baseline,
            now=lambda:self.clock.wall, sim_ns=lambda:self.clock.sim)
        sample = self.f.sample
        def observed(delta=.05):
            packet = sample(delta)
            self.m.observe_telemetry(canonical(packet))
            return packet
        self.f.sample = observed

    def navigate(self, *, complete=True):
        request = {'run_id':navigation.RUN,'sequence':1,'op':'request','waypoint':'pump'}
        channel = navigation.Channel(navigation.state(), navigation.reply('request'))
        receipt = self.m.dispatch(request, lambda _:channel.execute(waypoint='pump')[0])
        self.assertEqual(receipt['delivery'],'acknowledged')
        if not complete:
            return
        after = navigation.reply('arm'); after['command_sequence'] = 2; after['last_reply']['sequence'] = 2
        arm = navigation.Channel(navigation.state(command_sequence=1,target=navigation.TARGET,
            action_state='waiting_for_arm'), after)
        arm_command={'run_id':navigation.RUN,'sequence':2,'op':'arm','target':copy.deepcopy(navigation.TARGET)}
        value=self.m.dispatch_arm(arm_command,lambda _:arm.execute('arm',target=navigation.TARGET)[0])
        self.assertEqual(value['delivery'],'acknowledged')
        self.clock.advance(.02)
        self.assertTrue(self.m.navigation_state(self.nav_finished(),self.clock.wall))

    def nav_finished(self, **changes):
        return navigation.state(command_sequence=2, sim_stamp_ns=self.clock.sim,
            last_completion={'target':copy.deepcopy(navigation.TARGET),'outcome':'arrived',
                             'measured':{'within_tolerance':True}}, **changes)

    def inspect(self):
        children = []
        def worker(payload):
            value = inspection.Worker(payload)
            value.reply['observation_id'] = uuid.uuid4().hex
            children.append(value)
            return value
        owner = InspectionOwner(self.f.root,self.f.live,worker,now=lambda:self.clock.wall,
            sim_ns=lambda:self.clock.sim,load=self.f.load)
        client = None
        def publish(text):
            client.observe_reply(canonical(owner.handle(text)))
        def spin(_):
            self.f.sample(.02)
            if children: children[-1].exited = True
            owner.tick()
            client.observe_state(canonical(owner.state()))
        client = InspectionClient(publish,spin,lambda:{'owner':'inspection'},now=lambda:self.clock.wall)
        client.discover()
        request = {'schema_version':1,'run_id':owner.run_id,'mission_id':self.m.mission_id,
            'action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex,'op':'inspect',
            'arguments':{'machine_id':'pump','timeout_s':5.}}
        receipt = self.m.dispatch(request,client.execute)
        self.assertEqual(receipt['transport'],'terminal',receipt)
        self.assertEqual(receipt['result']['status'],'succeeded',receipt)
        return request,receipt

    def ready(self, decision='recoverable'):
        self.navigate(); request,receipt = self.inspect(); self.m.decide(decision)
        return request,receipt

    def maintain(self, request):
        client = None
        def publish(text):
            client.observe_reply(canonical(self.f.owner.handle(text)))
        def spin(_):
            self.f.hold.acknowledged = True
            if self.f.workers: self.f.workers[-1].exited = True
            if self.f.hold.release_at is not None: self.f.hold.released = True
            self.f.sample(.05)
            client.observe_state(canonical(self.f.owner.state()))
        client = MaintenanceClient(publish,spin,lambda:{'owner':'station'},now=lambda:self.clock.wall)
        client.discover()
        return self.m.dispatch(request,client.execute)

    def restored(self):
        self.ready()
        isolate = self.maintain(self.f.request)['result']
        self.assertEqual(isolate['status'],'succeeded')
        reset_request = {**self.f.request,'op':'reset','action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex,
            'arguments':{**self.f.request['arguments'],'control_revision':1,'isolation_action_id':isolate['action_id']}}
        reset = self.maintain(reset_request)['result']
        self.assertEqual(reset['status'],'succeeded')
        restore_request = {**self.f.request,'op':'restore','action_id':uuid.uuid4().hex,'command_id':uuid.uuid4().hex,
            'arguments':{**self.f.request['arguments'],'control_revision':2,'reset_action_id':reset['action_id']}}
        restore = self.maintain(restore_request)['result']
        self.assertEqual(restore['status'],'succeeded')
        return restore

    def recovery(self):
        for _ in range(23): self.f.sample(.5)
        return self.m.verify()


class MissionStateTests(MissionFixture):
    def test_actual_clients_station_and_complete_new_recovery_window(self):
        value = self.restored()
        self.assertEqual(self.m.phase,'verify')
        self.assertIsNone(self.m.outcome)
        self.assertFalse(self.m.verify())
        self.assertTrue(self.recovery())
        self.assertEqual(self.m.outcome,'recovered')
        self.assertEqual(self.m.control['last_applied'],value['state_change']['effect'])
        self.assertEqual(self.f.model.station.machines['pump'].reset_count,1)
        self.assertEqual(len(self.m.actions),5)

    def test_healthy_decision_needs_new_public_window_without_maintenance(self):
        self.f.model.station.machines['pump'].fault = Fault.NONE
        self.ready('healthy')
        self.assertFalse(self.m.verify())
        self.assertTrue(self.recovery())
        self.assertEqual(self.m.outcome,'healthy')
        self.assertEqual(self.f.model.control_revision,0)
        self.assertFalse(any(v['family']=='maintenance' for v in self.m.actions.values()))

    def test_request_acknowledgement_is_not_arrival_or_inspection_permission(self):
        self.navigate(complete=False)
        self.assertEqual(self.m.phase,'navigating')
        with self.assertRaisesRegex(ValueError,'not_available'): self.inspect()
        with self.assertRaisesRegex(ValueError,'not_available'): self.m.escalate()

    def test_unarmed_success_and_stale_or_wrong_target_do_not_complete_navigation(self):
        self.navigate(complete=False)
        with self.assertRaisesRegex(ValueError,'arrival_not_observed'):
            self.m.navigation_state(self.nav_finished(),self.clock.wall)
        self.clock.advance(.01)
        wrong=self.nav_finished(); wrong['last_completion']['target']['epoch']+=1
        self.assertFalse(self.m.navigation_state(wrong,self.clock.wall))
        with self.assertRaisesRegex(ValueError,'stale_or_changed'):
            self.m.navigation_state(self.nav_finished(),self.clock.wall-.3)
        self.assertIsNotNone(self.m.pending)

    def test_gateway_exception_keeps_possible_delivery_and_never_retries(self):
        calls=[]
        command={'run_id':navigation.RUN,'sequence':1,'op':'request','waypoint':'pump'}
        def broken(value):
            calls.append(value)
            self.assertIsNotNone(self.m.pending)
            raise OSError('controlled loss after publication')
        with self.assertRaises(OSError): self.m.dispatch(command,broken)
        with self.assertRaisesRegex(ValueError,'not_available'): self.m.dispatch(command,broken)
        self.assertEqual(len(calls),1)
        self.assertEqual(self.m.phase,'unresolved')
        self.assertEqual(self.m.cancellation_target(),{'family':'navigation','target':None})
        with self.assertRaisesRegex(ValueError,'contradiction'):
            self.m.navigation_receipt({'operation':'request','command':command,'delivery':'not_sent','accepted':None})

    def test_failed_navigation_escalates_and_cannot_admit_inspection(self):
        self.navigate(complete=False)
        value=self.nav_finished(); value['last_completion']['outcome']='not_dispatched'; value['last_completion']['measured']=None
        self.assertTrue(self.m.navigation_state(value,self.clock.wall))
        self.assertEqual(self.m.phase,'escalate')
        self.m.escalate()
        self.assertEqual(self.m.outcome,'escalated')
        with self.assertRaises(ValueError): self.inspect()

    def test_maintenance_cannot_skip_diagnosis_or_prerequisite_actions(self):
        with self.assertRaises(ValueError): self.m.begin_action(self.f.request)
        self.ready()
        request={**self.f.request,'op':'reset','arguments':{**self.f.request['arguments'],'isolation_action_id':uuid.uuid4().hex}}
        with self.assertRaisesRegex(ValueError,'maintenance_transition'): self.m.begin_action(request)
        self.assertEqual(self.f.model.control_revision,0)

    def test_mismatched_result_retains_pending_step_instead_of_advancing(self):
        self.ready()
        self.m.begin_action(self.f.request)
        value=self.f.isolated()
        wrong=copy.deepcopy(value); wrong['request_sha256']='f'*64
        with self.assertRaises(ValueError): self.m.action_result(wrong)
        self.assertIsNotNone(self.m.pending)
        self.m.action_result(value)
        self.assertEqual(self.m.next_operation,'reset')

    def test_uncertain_receipt_and_cleanup_error_do_not_release_ownership(self):
        self.ready(); self.m.begin_action(self.f.request)
        identity={k:self.f.request[k] for k in ('run_id','mission_id','action_id')}
        receipt={**identity,'transport':'uncertain','result':None,'acknowledgement':None,'error_code':'terminal_result_unconfirmed'}
        self.m.action_receipt(receipt)
        with self.assertRaisesRegex(ValueError,'contradiction'):
            self.m.action_receipt({**receipt,'transport':'not_sent'})
        value=self.f.isolated()
        self.m.action_receipt({**receipt,'transport':'terminal','result':value,'client_cleanup_failed':True})
        self.assertIsNotNone(self.m.pending)
        self.m.action_result(value)
        self.assertEqual(self.m.next_operation,'reset')

    def test_cancel_remains_nonterminal_until_exact_action_result(self):
        self.ready(); self.m.begin_action(self.f.request)
        target=self.m.cancel()
        self.assertEqual(target['action_id'],self.f.request['action_id'])
        self.assertEqual(self.m.phase,'unresolved')
        self.assertIsNone(self.m.outcome)
        value=self.f.isolated()
        self.m.action_result(value)
        self.assertEqual(self.m.outcome,'cancelled')
        self.assertEqual(self.m.control['last_applied'],value['state_change']['effect'])
        self.assertFalse(self.f.model.station.machines['pump'].powered)

    def test_late_success_resolves_step_but_does_not_extend_mission_budget(self):
        self.ready(); self.m.begin_action(self.f.request)
        value=self.f.isolated()
        self.clock.advance(self.m.deadline-self.clock.wall)
        self.m.tick()
        self.assertEqual(self.m.phase,'unresolved')
        self.assertIsNone(self.m.outcome)
        self.m.action_result(value)
        self.assertEqual(self.m.outcome,'timed_out')
        self.assertIsNone(self.m.pending)
        self.assertEqual(self.m.control['last_applied'],value['state_change']['effect'])

    def test_clock_regression_and_station_change_do_not_grant_new_work(self):
        self.navigate(complete=False)
        self.clock.wall-=1.
        self.m.tick()
        self.assertEqual(self.m.stop_reason,'clock_invalid')
        self.assertIsNotNone(self.m.pending)
        self.clock.wall+=1.
        packet=copy.deepcopy(self.f.packet); packet['run_id']=uuid.uuid4().hex
        with self.assertRaisesRegex(ValueError,'station_generation_changed'):
            self.m.observe_telemetry(canonical(packet))
        with self.assertRaises(ValueError): self.inspect()

    def test_persistent_decision_escalates_without_intervention(self):
        self.ready('persistent')
        self.m.escalate()
        self.assertEqual(self.m.outcome,'escalated')
        self.assertEqual(self.f.model.control_revision,0)

    def test_new_fault_after_restoration_cannot_be_called_recovered(self):
        self.restored()
        self.f.model.station.machines['pump'].fault=Fault.JAM
        self.assertFalse(self.recovery())
        self.assertIsNone(self.m.outcome)
        self.assertEqual(self.m.phase,'verify')

    def test_action_budget_is_not_silently_reset(self):
        self.m.max_steps=1
        self.navigate()
        with self.assertRaisesRegex(ValueError,'step_limit'): self.inspect()
        self.assertEqual(len(self.m.actions),1)

    def test_result_snapshot_cannot_change_live_state_or_finished_history(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        self.ready('healthy'); self.assertTrue(self.recovery())
        snapshot=self.m.snapshot()
        snapshot['outcome']='recovered'; snapshot['actions'].clear()
        original=self.m.snapshot()
        self.clock.advance(1000.)
        self.assertEqual(self.m.snapshot(),original)

    def test_command_identity_and_remaining_budget_are_enforced_before_send(self):
        self.ready()
        for field,value in (('mission_id',uuid.uuid4().hex),('run_id','invalid')):
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.m.begin_action({**self.f.request,field:value})
        self.clock.advance(self.m.deadline-self.clock.wall-.5)
        with self.assertRaisesRegex(ValueError,'remaining_budget'): self.m.begin_action(self.f.request)
        self.assertIsNone(self.m.pending)

    def test_telemetry_after_completion_cannot_change_the_frozen_result(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        self.ready('healthy'); self.assertTrue(self.recovery())
        before=self.m.snapshot()
        self.f.sample(.5)
        self.assertEqual(self.m.snapshot(),before)

    def test_telemetry_that_triggers_idle_deadline_does_not_enter_closed_session(self):
        self.clock.advance(self.m.deadline-self.clock.wall)
        self.assertFalse(self.m.observe_telemetry('invalid after deadline'))
        self.assertEqual(self.m.outcome,'timed_out')
        self.assertEqual(self.m.stop_reason,'deadline_exceeded')
        self.assertIsNone(self.m.last_sample)

    def test_malformed_arm_receipt_keeps_reserved_target_unresolved(self):
        self.navigate(complete=False)
        command={'run_id':navigation.RUN,'sequence':2,'op':'arm','target':copy.deepcopy(navigation.TARGET)}
        calls=[]
        def malformed(value):
            calls.append(value)
            return {'operation':'arm','delivery':'acknowledged'}
        with self.assertRaises(ValueError): self.m.dispatch_arm(command,malformed)
        self.assertEqual(self.m.phase,'unresolved')
        self.assertEqual(self.m.pending['arm_command'],command)
        self.assertIsNone(self.m.outcome)
        with self.assertRaisesRegex(ValueError,'arm_not_available'):
            self.m.dispatch_arm(command,malformed)
        self.assertEqual(len(calls),1)

    def test_arm_is_reserved_before_gateway_and_cannot_be_dispatched_twice(self):
        self.navigate(complete=False)
        command={'run_id':navigation.RUN,'sequence':2,'op':'arm','target':copy.deepcopy(navigation.TARGET)}
        calls=[]
        def lost(value):
            calls.append(value)
            self.assertEqual(self.m.pending['arm_command'],value)
            raise OSError('controlled arm response loss')
        with self.assertRaises(OSError): self.m.dispatch_arm(command,lost)
        with self.assertRaisesRegex(ValueError,'arm_not_available'): self.m.dispatch_arm(command,lost)
        self.assertEqual(len(calls),1)
        self.assertIsNone(self.m.outcome)
        self.assertEqual(self.m.cancellation_target()['target'],navigation.TARGET)

    def test_cancel_after_reserved_target_prevents_first_arm(self):
        self.navigate(complete=False); self.m.cancel()
        command={'run_id':navigation.RUN,'sequence':2,'op':'arm','target':copy.deepcopy(navigation.TARGET)}
        with self.assertRaisesRegex(ValueError,'arm_not_available'):
            self.m.begin_navigation_arm(command)

    def test_valid_owner_result_for_different_request_does_not_resolve_mission_step(self):
        self.ready()
        promised=copy.deepcopy(self.f.request); promised['arguments']['timeout_s']=9.
        self.m.begin_action(promised)
        actual=self.f.isolated()
        maintenance_result(actual)
        with self.assertRaisesRegex(ValueError,'mission_result_binding'):
            self.m.action_result(actual)
        self.assertIsNotNone(self.m.pending)
        self.assertEqual(self.m.control['revision'],0)

    def test_actual_client_not_sent_refusal_does_not_claim_effect_or_completion(self):
        self.ready(); self.f.live.available=False
        value=self.maintain(self.f.request)
        self.assertEqual(value['transport'],'not_sent')
        self.assertEqual(self.m.phase,'escalate')
        self.assertEqual(self.f.model.control_revision,0)

    def test_old_healthy_window_does_not_complete_a_new_verification_phase(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        for _ in range(23): self.f.sample(.5)
        self.ready('healthy')
        self.assertFalse(self.m.verify())
        self.assertTrue(self.recovery())

    def test_reused_inspection_identity_cannot_start_a_second_step(self):
        request,_=self.ready('inspect_again')
        with self.assertRaisesRegex(ValueError,'identity_reused'): self.m.begin_action(request)
        self.assertEqual(len(self.m.actions),2)

    def test_ambiguous_decision_never_selects_reset_or_restore(self):
        self.ready('ambiguous')
        with self.assertRaises(ValueError): self.m.begin_action(self.f.request)
        self.m.escalate()
        self.assertEqual(self.m.outcome,'escalated')
        self.assertEqual(self.f.model.control_revision,0)

    def test_oversized_gateway_receipt_cannot_release_pending_action(self):
        self.ready(); self.m.begin_action(self.f.request)
        with self.assertRaises(ValueError): self.m.action_receipt({'data':'x'*65537})
        self.assertIsNotNone(self.m.pending)

    def test_real_effect_then_gateway_loss_retains_exact_step_until_result(self):
        self.ready()
        actual=[]
        def lost(_):
            actual.append(self.f.isolated())
            raise OSError('controlled reply loss after actual isolation')
        with self.assertRaises(OSError): self.m.dispatch(self.f.request,lost)
        self.assertFalse(self.f.model.station.machines['pump'].powered)
        self.assertEqual(self.f.model.control_revision,1)
        self.assertEqual(self.m.control['revision'],0)
        self.assertIsNotNone(self.m.pending)
        with self.assertRaises(ValueError): self.m.begin_action(self.f.reset_request)
        self.m.action_result(actual[0])
        self.assertEqual(self.m.control['revision'],1)
        self.assertEqual(self.m.next_operation,'reset')

    def test_unconfirmed_rejection_cannot_clear_possible_action_ownership(self):
        self.ready(); self.m.begin_action(self.f.request)
        identity={k:self.f.request[k] for k in ('run_id','mission_id','action_id')}
        with self.assertRaisesRegex(ValueError,'rejection_unconfirmed'):
            self.m.action_receipt({**identity,'transport':'rejected','acknowledgement':None,'result':None})
        self.assertIsNotNone(self.m.pending)

    def test_real_supervisor_hold_refusal_resolves_rejected_navigation(self):
        request={'run_id':navigation.RUN,'sequence':1,'op':'request','waypoint':'pump'}
        node=NS(command_authority_clear=lambda:True,commands=CommandGate(navigation.RUN,('pump','cooler','battery')),
                maintenance_hold={'controlled':True},maintenance_error=False,last_reply=None,audit=lambda *a,**kw:None)
        supervisor.load_production().on_command(node,supervisor.Message(data=canonical(request)))
        self.assertEqual(node.last_reply,{'sequence':1,'op':'request','accepted':False,'error':'maintenance_hold'})
        response=navigation.state(command_sequence=1,last_reply=node.last_reply)
        channel=navigation.Channel(navigation.state(),response)
        receipt=self.m.dispatch(request,lambda _:channel.execute(waypoint='pump')[0])
        self.assertEqual(receipt['delivery'],'rejected')
        self.assertIsNone(self.m.pending)
        self.assertEqual(self.m.phase,'escalate')
