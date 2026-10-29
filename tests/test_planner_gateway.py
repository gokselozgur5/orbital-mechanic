import copy
import json
import threading
from unittest.mock import patch

from orbital.inspection_protocol import canonical
from orbital.machines import Fault
from orbital.mission_journal import MissionJournal
from orbital.planner_gateway import PlannerGateway
from orbital.planner_history import ToolHistory
from orbital.planner_protocol import PlannerError
from orbital import mission_journal as storage, planner_runtime
from orbital import ros_mission
from planner_gateway_fixture import GatewayFixture


class PlannerGatewayTests(GatewayFixture):
    def test_wrong_phase_is_a_bounded_error_before_durable_admission(self):
        self.driver.start()
        value=self.command('reset_machine',{'machine_id':'pump','timeout_s':5.})
        before=self.mission_journal.sequence
        with self.assertRaisesRegex(PlannerError,'^precondition_failed$'):
            self.driver.gateway.execute(canonical(value))
        self.assertEqual(self.mission_journal.sequence,before)
        self.assertEqual(self.bus.sent,[])

    def test_stop_latch_survives_reopen_and_needs_explicit_closed_mission_reset(self):
        def stop():
            self.driver.gateway.execute(canonical(self.request('stop',{'reason':'operator_request'})))
            with self.assertRaisesRegex(ValueError,'reset_unavailable'):
                self.mission_journal.reset_planner_stop()
        with patch.object(self.driver,'drive',side_effect=stop):report=self.driver.run()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'cancelled')
        path=self.f.root/'runtime/mission-service/events.jsonl';before=path.read_bytes()
        self.mission_journal.close();self.mission_lock.close()
        def never(**kwargs):self.fail('a stopped journal must refuse before transport construction')
        with self.assertRaisesRegex(ValueError,'planner_operator_reset_required'):
            ros_mission.run(self.f.root,now=self.bus.now,transport_factory=never,tool_contract=True)
        self.assertEqual(path.read_bytes(),before)
        from orbital.inspection_worker import OwnerLock
        lock=OwnerLock(self.f.root,service='mission-service');self.addCleanup(lock.close)
        journal=MissionJournal(lock);self.addCleanup(journal.close)
        self.assertEqual(journal.tools.stop_owner,self.driver.mission_id)
        receipt=journal.reset_planner_stop()
        self.assertTrue(receipt['cleared']);self.assertIsNone(journal.tools.stop_owner)
        events=[json.loads(v) for v in path.read_text().splitlines()]
        self.assertNotEqual(events[-1]['mission_id'],self.driver.mission_id)
        self.assertEqual(events[-1]['kind'],'operator_tool_reset')
        own=[v for v in events if v['mission_id']==self.driver.mission_id]
        self.assertEqual(own[-1]['kind'],'terminal')
        journal.close();journal=MissionJournal(lock);self.addCleanup(journal.close)
        self.assertIsNone(journal.tools.stop_owner)
        with self.assertRaisesRegex(ValueError,'reset_unavailable'):journal.reset_planner_stop()
        called=[]
        # Constructor admission is allowed again; no run, motion or guard reset.
        with patch.object(self.bus,'install',side_effect=lambda callbacks:called.append(callbacks)):
            planner_runtime.PlannerRuntime(self.f.model.registry,self.bus,journal,timeout_s=60.)
        self.assertEqual(len(called),1)
        self.assertEqual(self.bus.sent,[])

    def test_separate_thread_stop_cancels_exact_owner_and_preserves_identity(self):
        self.driver.start();entered=threading.Event();release=threading.Event();answers=[];errors=[]
        value=self.command('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        stop=self.request('stop',{'reason':'operator_request'})
        def hold(family,command):
            if family=='navigation' and command['op']=='request':
                entered.set()
                if not release.wait(5.):raise RuntimeError('controlled stop observer expired')
        def work():
            try:answers.append(self.driver.gateway.execute(canonical(value)))
            except BaseException as error:errors.append(error)
        self.bus.before_publish=hold
        worker=threading.Thread(target=work);worker.start()
        try:
            self.assertTrue(entered.wait(5.))
            ack=self.driver.gateway.execute(canonical(stop))
            self.assertFalse(ack['terminal']);self.assertFalse(ack['durable'])
            changed=copy.deepcopy(stop);changed['arguments']['reason']='uncertain'
            with self.assertRaisesRegex(PlannerError,'^action_conflict$'):
                self.driver.gateway.execute(canonical(changed))
            self.assertEqual(self.driver.gateway.execute(canonical(stop)),ack)
        finally:release.set();worker.join(10.)
        self.assertFalse(worker.is_alive());self.assertEqual(errors,[])
        self.assertEqual(answers[0]['result']['status'],'cancelled')
        self.assertEqual([v['op'] for family,v in self.bus.sent],['request','cancel'])
        terminal=self.driver.gateway.execute(canonical(stop))
        self.assertTrue(terminal['replayed']);self.assertEqual(terminal['result']['status'],'succeeded')
        self.assertEqual(self.mission_journal.tools.stop_owner,self.driver.mission_id)

    def test_actual_client_recovery_uses_the_tool_contract(self):
        self.driver.start()
        self.driver.drive()
        self.assertEqual(self.driver.mission.outcome,'recovered')
        self.assertEqual(self.f.model.control_revision,3)
        state=self.driver.gateway.state()
        self.assertEqual(state['reported'],'recovered')
        self.assertTrue(all(v['response'] is not None for v in state['calls'].values()))

    def test_healthy_complete_run_closes_history_without_intervening(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        report=self.driver.run()
        self.assertTrue(report['complete'],report['failure'])
        self.assertEqual(report['mission']['outcome'],'healthy')
        self.assertEqual(self.f.model.control_revision,0)
        self.assertEqual(set(self.driver.policy.healthy_inspections),{'pump','cooler','power_unit'})
        self.mission_journal.close()
        reopened=MissionJournal(self.mission_lock);self.addCleanup(reopened.close)
        self.assertIsNone(reopened.active)
        self.assertEqual(reopened.tools.get(self.driver.mission_id)['reported'],'healthy')

    def test_navigation_replay_returns_original_evidence_without_another_command(self):
        self.driver.start()
        value=self.command('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        original=self.driver.gateway.execute(canonical(value));sent=copy.deepcopy(self.bus.sent)
        saved=copy.deepcopy(original);original['observations'].clear()
        self.clock.advance(2.)  # A replay is historical and does not renew old evidence.
        again=self.driver.gateway.execute(canonical(value))
        self.assertTrue(again['replayed'])
        self.assertEqual(again,{**saved,'replayed':True})
        self.assertEqual(self.bus.sent,sent)
        self.mission_journal.close();self.mission_journal=MissionJournal(self.mission_lock)
        self.addCleanup(self.mission_journal.close);self.driver.journal=self.mission_journal
        self.assertEqual(self.driver.gateway.execute(canonical(value)),again)
        self.assertEqual(self.bus.sent,sent)

    def test_reset_replay_and_conflicting_identity_cannot_repeat_a_pulse(self):
        self.isolated()
        value=self.command('reset_machine',{'machine_id':'pump','timeout_s':5.})
        first=self.driver.gateway.execute(canonical(value));sent=copy.deepcopy(self.bus.sent)
        self.assertEqual(first['result']['effect'],'reported')
        self.assertEqual(self.f.model.station.machines['pump'].reset_count,1)
        self.assertTrue(self.driver.gateway.execute(canonical(value))['replayed'])
        value['arguments']['timeout_s']=4.
        with self.assertRaisesRegex(PlannerError,'^action_conflict$'):
            self.driver.gateway.execute(canonical(value))
        self.assertEqual(self.bus.sent,sent)
        self.assertEqual(self.f.model.station.machines['pump'].reset_count,1)

    def test_lost_terminal_keeps_call_owned_until_exact_stop_observes_its_result(self):
        self.isolated()
        value=self.command('reset_machine',{'machine_id':'pump','timeout_s':5.})
        deliver=self.bus.deliver
        def drop(topic,text):
            packet=json.loads(text)
            if topic=='/maintenance/result' and packet.get('operation')=='reset':return
            if topic=='/maintenance/state' and (packet.get('last_result') or {}).get('operation')=='reset':return
            deliver(topic,text)
        with patch.object(self.bus,'deliver',side_effect=drop):
            with self.assertRaisesRegex(PlannerError,'^unresolved$'):
                self.driver.gateway.execute(canonical(value))
        self.assertEqual(self.f.model.control_revision,2)
        self.assertIsNotNone(self.driver.mission.pending)
        self.assertIsNone(self.driver.gateway.state()['calls'][value['action_id']]['response'])
        before=copy.deepcopy(self.bus.sent)
        with self.assertRaisesRegex(PlannerError,'^unresolved$'):
            self.driver.gateway.execute(canonical(value))
        another=copy.deepcopy(value);another['action_id']='f'*32
        with self.assertRaisesRegex(PlannerError,'^busy$'):
            self.driver.gateway.execute(canonical(another))
        self.assertEqual(self.bus.sent,before)
        stop=self.request('stop',{'reason':'uncertain'})
        stopped=self.driver.gateway.execute(canonical(stop))
        self.assertEqual(stopped['result']['status'],'succeeded')
        self.assertIsNone(self.driver.mission.pending)
        prior=self.driver.gateway.execute(canonical(value))
        self.assertTrue(prior['replayed']);self.assertEqual(prior['result']['effect'],'reported')
        self.assertEqual(self.f.model.control_revision,2)
        self.assertFalse(self.f.model.station.machines['pump'].powered)

    def test_post_intent_expiry_cannot_receive_a_new_navigation_timeout(self):
        self.driver.start()
        value=self.command('navigate_to',{'waypoint_id':'pump','timeout_s':.1})
        append=self.mission_journal.append
        def delayed(mission,kind,data):
            answer=append(mission,kind,data)
            if kind=='tool_intent':self.clock.advance(.2)
            return answer
        with patch.object(self.mission_journal,'append',side_effect=delayed):
            answer=self.driver.gateway.execute(canonical(value))
        self.assertEqual(answer['result']['status'],'timed_out')
        self.assertEqual(answer['result']['error_code'],'deadline_exceeded')
        self.assertEqual(self.bus.sent,[])

    def test_public_precondition_expires_after_native_binding_io_before_send(self):
        self.driver.start()
        value=self.command('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        bind=self.driver.gateway.bind
        def delayed(family,command):
            bind(family,command);self.clock.advance(1.1)
        with patch.object(self.driver.gateway,'bind',side_effect=delayed):
            answer=self.driver.gateway.execute(canonical(value))
        self.assertEqual(answer['result']['status'],'rejected')
        self.assertEqual(answer['result']['error_code'],'stale_observation')
        self.assertEqual(self.bus.sent,[])

    def test_new_public_packet_after_binding_does_not_replace_original_precondition(self):
        self.driver.start()
        value=self.command('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        bind=self.driver.gateway.bind
        def changed(family,command):
            bind(family,command);self.bus.spin(.05)
        with patch.object(self.driver.gateway,'bind',side_effect=changed):
            answer=self.driver.gateway.execute(canonical(value))
        self.assertEqual(answer['result']['status'],'rejected')
        self.assertEqual(answer['result']['error_code'],'stale_observation')
        self.assertEqual(self.bus.sent,[])

    def test_failed_intent_sync_never_publishes_or_grants_in_memory_admission(self):
        self.driver.start()
        value=self.command('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        append=self.mission_journal.append;sync=storage.os.fsync
        def fail(fd):
            if fd==self.mission_journal.fd:raise OSError('controlled intent sync failure')
            return sync(fd)
        def admission(mission,kind,data):
            if kind=='tool_intent':
                with patch.object(storage.os,'fsync',side_effect=fail):return append(mission,kind,data)
            return append(mission,kind,data)
        with patch.object(self.mission_journal,'append',side_effect=admission):
            with self.assertRaises(OSError):self.driver.gateway.execute(canonical(value))
        self.assertEqual(self.bus.sent,[])
        self.assertNotIn(value['action_id'],self.mission_journal.tools.get(self.driver.mission_id)['calls'])
        self.mission_journal.close();reopened=MissionJournal(self.mission_lock);self.addCleanup(reopened.close)
        self.assertEqual(reopened.active,self.driver.mission_id)
        self.assertIsNone(reopened.tools.get(reopened.active)['calls'][value['action_id']]['response'])

    def test_navigation_deadline_cancels_one_owned_goal_and_retains_terminal_evidence(self):
        self.driver.start();self.bus.hold_navigation=True
        value=self.command('navigate_to',{'waypoint_id':'pump','timeout_s':.2})
        response=self.driver.gateway.execute(canonical(value))
        self.assertEqual(response['result']['status'],'timed_out')
        self.assertEqual([v['op'] for family,v in self.bus.sent],['request','arm','cancel'])
        self.assertIsNone(self.driver.mission.pending)
        self.assertEqual(self.bus.nav['action_state'],'idle')

    def test_host_cancellation_and_reentrant_call_do_not_publish_an_arm(self):
        self.driver.start();rejections=[]
        def interrupt(family,value):
            if family=='navigation' and value['op']=='request':
                try:self.driver.gateway.execute(canonical(self.request('observe_station',{})))
                except PlannerError as error:rejections.append(error.code)
                self.cancel_requested=True
        self.bus.before_publish=interrupt
        response=self.driver.call('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        self.assertEqual(rejections,['busy'])
        self.assertEqual(response['result']['status'],'cancelled')
        self.assertEqual([v['op'] for family,v in self.bus.sent],['request','cancel'])
        self.assertIsNone(self.driver.mission.pending)

    def test_tool_stop_interrupts_inflight_call_then_exposes_one_durable_result(self):
        self.driver.start();pending=[]
        stop=self.request('stop',{'reason':'operator_request'})
        def interrupt(family,value):
            if family=='navigation' and value['op']=='request':
                pending.append(self.driver.gateway.execute(canonical(stop)))
                pending.append(self.driver.gateway.execute(canonical(stop)))
        self.bus.before_publish=interrupt
        response=self.driver.call('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        self.assertEqual(response['result']['status'],'cancelled')
        self.assertEqual(pending[0],pending[1])
        self.assertFalse(pending[0]['terminal']);self.assertFalse(pending[0]['durable'])
        self.assertEqual([v['op'] for family,v in self.bus.sent],['request','cancel'])
        terminal=self.driver.gateway.execute(canonical(stop))
        self.assertTrue(terminal['replayed'])
        self.assertEqual(terminal['result']['status'],'succeeded')
        self.assertFalse(self.driver.gateway.stop_delivery_failed)
        self.assertTrue(self.driver.stop_signal.is_set())

    def test_broken_journal_still_allows_owned_navigation_cleanup(self):
        self.driver.start();self.bus.hold_navigation=True
        def break_after_arm():
            if self.bus.nav['action_state']=='running':self.mission_journal.broken=True
        self.bus.after_spin=break_after_arm
        with self.assertRaises(ValueError):
            self.driver.call('navigate_to',{'waypoint_id':'pump','timeout_s':5.})
        self.assertEqual([v['op'] for family,v in self.bus.sent],['request','arm','cancel'])
        self.assertIsNone(self.driver.mission.pending)
        stop=self.request('stop',{'reason':'uncertain'})
        with self.assertRaisesRegex(PlannerError,'^unavailable$'):
            self.driver.gateway.execute(canonical(stop))
        self.assertEqual(self.bus.nav['action_state'],'idle')

    def test_result_sync_failure_after_real_isolation_never_returns_success(self):
        self.diagnose()
        value=self.command('isolate_power',{'machine_id':'pump','timeout_s':5.})
        append=self.mission_journal.append;sync=storage.os.fsync
        def fail(fd):
            if fd==self.mission_journal.fd:raise OSError('controlled result sync failure')
            return sync(fd)
        def fail_result(mission,kind,data):
            if kind=='tool_result':
                with patch.object(storage.os,'fsync',side_effect=fail):return append(mission,kind,data)
            return append(mission,kind,data)
        with patch.object(self.mission_journal,'append',side_effect=fail_result):
            with self.assertRaises(OSError):self.driver.gateway.execute(canonical(value))
        self.assertEqual(self.f.model.control_revision,1)
        self.assertIsNone(self.mission_journal.tools.get(self.driver.mission_id)['calls'][value['action_id']]['response'])
        self.assertTrue(self.mission_journal.broken)
        self.mission_journal.close();reopened=MissionJournal(self.mission_lock);self.addCleanup(reopened.close)
        self.assertEqual(reopened.active,self.driver.mission_id)
        self.assertEqual(reopened.tools.get(self.driver.mission_id)['calls'][value['action_id']]['response']['result']['effect'],'reported')
        self.assertEqual(self.f.model.control_revision,1)

    def test_call_budget_reserves_stop_and_does_not_renew_on_replay(self):
        with patch.object(planner_runtime,'PlannerGateway',side_effect=lambda *a:PlannerGateway(*a,max_calls=1)):
            self.driver.start()
        original=self.request('observe_station',{})
        self.driver.gateway.execute(canonical(original))
        self.assertTrue(self.driver.gateway.execute(canonical(original))['replayed'])
        with self.assertRaisesRegex(PlannerError,'^budget_exhausted$'):self.observe()
        answer=self.driver.gateway.execute(canonical(self.request('stop',{'reason':'budget_exhausted'})))
        self.assertEqual(answer['result']['status'],'succeeded')
        self.assertEqual(len(self.driver.gateway.state()['calls']),2)

    def test_outcome_and_unknown_evidence_cannot_override_the_mission(self):
        self.driver.start()
        with self.assertRaisesRegex(PlannerError,'^precondition_failed$'):
            self.driver.gateway.execute(canonical(self.request('report_outcome',
                {'outcome':'recovered','evidence_ids':[],'summary':'claim'})))
        self.assertIsNone(self.driver.mission.outcome)
        self.assertEqual(self.bus.sent,[])

    def test_requested_recovery_window_is_not_shortened_to_default_ten_seconds(self):
        self.isolated()
        self.driver.call('reset_machine',{'machine_id':'pump','timeout_s':5.})
        self.driver.call('restore_power',{'machine_id':'pump','timeout_s':5.})
        answer=self.driver.call('verify_recovery',{'machine_id':'pump','observation_window_s':20.})
        self.assertEqual(answer['result']['status'],'succeeded')
        measured=answer['observations'][0]['data']
        self.assertGreaterEqual(measured['window']['duration_s'],20.)
        self.assertEqual(measured['required_window_s'],20.)

    def test_native_result_and_reported_effect_must_match_the_durable_binding(self):
        self.isolated()
        path=self.f.root/'runtime/mission-service/events.jsonl'
        events=[json.loads(v) for v in path.read_text().splitlines()]
        history=ToolHistory();context=events[0]['data'];response=None
        for event in events[1:]:
            if event['kind']=='tool_result' and event['data']['result']['tool']=='isolate_power':
                response=event['data'];break
            history=history.advance(event['mission_id'],event['kind'],event['data'],context)
        self.assertIsNotNone(response)
        history.advance(self.driver.mission_id,'tool_result',response,context)
        for change in ('owner_action','reported_revision','private_field','missing_binding'):
            candidate=copy.deepcopy(response);trial=history
            if change=='owner_action':candidate['observations'][0]['data']['action_id']='f'*32
            elif change=='reported_revision':candidate['result']['state_change']['control_revision']+=1
            elif change=='private_field':candidate['observations'][0]['data']['secret']='canary'
            else:
                trial=copy.deepcopy(history)
                trial._sessions[self.driver.mission_id]['calls'][candidate['result']['action_id']]['bindings']=[]
            with self.subTest(change=change),self.assertRaises(ValueError):
                trial.advance(self.driver.mission_id,'tool_result',candidate,context)
