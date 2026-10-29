"""Whole scripted driver over actual public capture and durable maintenance."""
import json
from unittest.mock import patch

from orbital.machines import Fault
from mission_runtime_fixture import RuntimeFixture


class MissionRuntimeTests(RuntimeFixture):
    def test_recovered_mission_drives_actual_clients_and_durable_station(self):
        report=self.run_mission()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'recovered')
        self.assertEqual(self.f.model.control_revision,3)
        self.assertEqual([c['op'] for family,c in self.bus.sent],['request','arm','inspect','isolate','reset','restore'])
        self.assertIsNone(self.mission_journal.active)
        self.assertEqual(len(self.capture_records),1)

    def test_healthy_mission_inspects_all_three_without_maintenance(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        report=self.run_mission()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'healthy')
        self.assertEqual(len(self.capture_records),3)
        self.assertEqual(self.f.model.control_revision,0)
        self.assertFalse(any(family=='maintenance' for family,c in self.bus.sent))

    def test_persistent_condition_escalates_without_reset(self):
        self.f.model.station.machines['pump'].fault=Fault.JAM
        report=self.run_mission()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertEqual(self.f.model.control_revision,0)
        self.assertEqual(len(self.capture_records),1)

    def test_blocked_navigation_does_not_send_inspection(self):
        self.bus.blocked_navigation=True
        report=self.run_mission()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertEqual([family for family,c in self.bus.sent],['navigation','navigation'])

    def test_every_original_publication_has_exact_durable_prior_intent(self):
        def before(family,command):
            path=self.f.root/'runtime/mission-service/events.jsonl'
            events=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertTrue(any(v['kind']=='command' and v['data']=={'family':family,'command':command} for v in events))
            step=self.driver.mission.pending
            self.assertIsNotNone(step)
            self.assertEqual(step['family'],family)
        self.bus.before_publish=before
        report=self.run_mission()
        self.assertTrue(report['complete'],report)

    def test_cancellation_propagates_to_exact_navigation_target_and_observes_completion(self):
        self.bus.hold_navigation=True
        def after(family,command):
            if family=='navigation' and command['op']=='arm':self.cancel_requested=True
        self.bus.after_publish=after
        report=self.run_mission()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'cancelled')
        commands=[c for family,c in self.bus.sent]
        self.assertEqual([v['op'] for v in commands],['request','arm','cancel'])
        self.assertEqual(commands[1]['target'],commands[2]['target'])
        self.assertEqual(self.bus.nav['last_completion']['outcome'],'cancelled')

    def test_lost_navigation_ack_preserves_unknown_target_and_unfinished_journal(self):
        def after(family,command):
            if family=='navigation' and command['op']=='request':self.bus.drop_navigation=True
        self.bus.after_publish=after
        report=self.run_mission()
        self.assertFalse(report['complete'],report)
        self.assertIsNone(report['mission']['pending']['target'])
        self.assertEqual(self.mission_journal.active,self.driver.mission_id)
        self.assertEqual([c['op'] for family,c in self.bus.sent],['request'])

    def test_ambiguous_condition_collects_two_captures_then_escalates(self):
        self.f.model.station.machines['cooler'].fault=Fault.LATCHED
        report=self.run_mission()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertEqual(len(self.capture_records),2)
        self.assertEqual(self.f.model.control_revision,0)

    def test_measured_cooler_candidate_uses_the_same_gateway(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        self.f.model.station.machines['cooler'].fault=Fault.LATCHED
        report=self.run_mission()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'recovered')
        targets=[c['arguments']['machine_id'] for family,c in self.bus.sent if family!='navigation']
        self.assertEqual(targets,['cooler']*4)

    def test_final_graph_io_cannot_publish_terminal_with_stale_owner_states(self):
        self.f.model.station.machines['pump'].fault=Fault.NONE
        actual=self.bus.fence
        def fence(family=None):
            value=actual(family)
            mission=self.driver.mission
            if family=='maintenance' and mission is not None and mission.outcome=='healthy':
                self.clock.wall+=1.
            return value
        self.bus.fence=fence
        report=self.run_mission()
        self.assertFalse(report['complete'],report)
        self.assertEqual(self.mission_journal.active,self.driver.mission_id)

    def test_late_authority_read_before_maintenance_does_not_publish_stale_intent(self):
        actual=self.bus.fence
        delayed=False
        def fence(family=None):
            nonlocal delayed
            value=actual(family)
            mission=self.driver.mission
            if (not delayed and family is None and mission is not None and mission.pending is not None
                    and mission.pending['request'].get('op')=='isolate'):
                delayed=True
                self.clock.wall+=1.
            return value
        self.bus.fence=fence
        self.run_mission()
        self.assertTrue(delayed)
        self.assertFalse(any(family=='maintenance' and c['op']=='isolate' for family,c in self.bus.sent))
        self.assertEqual(self.f.model.control_revision,0)

    def test_cancellation_after_isolation_retains_effect_without_reset_or_restore(self):
        def after():
            if self.f.model.control_revision==1:self.cancel_requested=True
        self.bus.after_spin=after
        report=self.run_mission()
        self.assertEqual(report['mission']['outcome'],'cancelled',report)
        self.assertEqual(report['mission']['control']['revision'],1)
        self.assertEqual(self.f.model.control_revision,1)
        self.assertFalse(any(c['op'] in ('reset','restore') for family,c in self.bus.sent))

    def test_inspection_cancel_keeps_exact_owned_identity_and_never_intervenes(self):
        def after(family,command):
            if family=='inspection' and command['op']=='inspect':self.cancel_requested=True
        self.bus.after_publish=after
        report=self.run_mission()
        self.assertEqual(report['mission']['outcome'],'cancelled',report)
        commands=[c for family,c in self.bus.sent if family=='inspection']
        self.assertEqual([c['op'] for c in commands],['inspect','cancel'])
        for key in ('run_id','mission_id','action_id'):self.assertEqual(commands[0][key],commands[1][key])
        self.assertFalse(any(family=='maintenance' for family,c in self.bus.sent))
        self.assertTrue(report['complete'],report)

    def test_lost_maintenance_observations_retain_effect_and_pending_identity(self):
        deliver=self.bus.deliver
        def drop(topic,text):
            if topic.startswith('/maintenance/') and self.f.model.control_revision>0:return
            deliver(topic,text)
        self.bus.deliver=drop
        report=self.run_mission()
        self.assertFalse(report['complete'],report)
        self.assertEqual(self.f.model.control_revision,1)
        self.assertEqual(report['mission']['pending']['request']['op'],'isolate')
        commands=[c for family,c in self.bus.sent if family=='maintenance']
        self.assertEqual([c['op'] for c in commands],['isolate','cancel'])
        for key in ('run_id','mission_id','action_id'):self.assertEqual(commands[0][key],commands[1][key])
        self.assertEqual(self.mission_journal.active,self.driver.mission_id)

    def test_command_sync_failure_sends_nothing_and_keeps_durable_barrier(self):
        from orbital import mission_journal as storage
        actual=storage.os.fsync
        def sync(fd):
            if fd==self.mission_journal.fd:
                events=(self.f.root/'runtime/mission-service/events.jsonl').read_text().splitlines()
                if events and json.loads(events[-1])['kind']=='command':
                    raise OSError('controlled_command_sync_failure')
            return actual(fd)
        with patch.object(storage.os,'fsync',side_effect=sync):report=self.run_mission()
        self.assertFalse(report['complete'],report)
        self.assertEqual(self.bus.sent,[])
        self.assertTrue(self.mission_journal.broken)
        self.assertEqual(self.mission_journal.active,self.driver.mission_id)

    def test_public_source_loss_still_allows_exact_navigation_cancellation(self):
        self.bus.hold_navigation=True
        def after(family,command):
            if family=='navigation' and command['op']=='arm':self.bus.broken_sources.add('public')
        self.bus.after_publish=after
        report=self.run_mission()
        self.assertFalse(report['complete'],report)
        commands=[c for family,c in self.bus.sent]
        self.assertEqual([c['op'] for c in commands],['request','arm','cancel'])
        self.assertEqual(commands[1]['target'],commands[2]['target'])
        self.assertEqual(self.bus.nav['last_completion']['outcome'],'cancelled')
        self.assertIsNone(report['mission']['pending'])
        self.assertEqual(self.mission_journal.active,self.driver.mission_id)
