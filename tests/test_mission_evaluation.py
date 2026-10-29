"""An actual mission witness and actual private station stream share exact packets."""
import json
from unittest.mock import patch

from orbital.inspection_worker import OwnerLock
from orbital.mission_witness_replay import read
from orbital.station_evaluation import evidence
from orbital.station_truth_protocol import digest
from orbital.mission_evaluation import evaluate
from orbital.mission_state import MissionState
from orbital import mission_policy, station_evaluation
import signal
from mission_evaluation_fixture import MissionEvaluationFixture


class MissionEvidenceBindingTests(MissionEvaluationFixture):
    def run_and_score(self,profile):
        trial=self.configure(profile)
        report=self.composed();self.assertTrue(report['complete'],report)
        self.close_station()
        return evaluate(self.f.root,trial,report['mission_id'])

    def test_healthy_mission_has_three_real_captures_and_no_mutation_attempt(self):
        scored=self.run_and_score('healthy')
        self.assertTrue(scored['outcome']['success_claim_supported'])
        self.assertEqual(scored['outcome']['claimed_outcome'],'healthy')
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],0)
        self.assertEqual(scored['captured_observations'],3)

    def test_persistent_fault_reports_escalation_without_claiming_safety(self):
        scored=self.run_and_score('persistent')
        self.assertTrue(scored['outcome']['reported_escalation'])
        self.assertIsNone(scored['outcome']['success_claim_supported'])
        self.assertFalse(scored['outcome']['station_criteria_passed'])
        self.assertIsNone(scored['overall_acceptance'])

    def test_ambiguous_fault_reports_escalation_and_no_intervention(self):
        scored=self.run_and_score('ambiguous')
        self.assertTrue(scored['outcome']['reported_escalation'])
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],0)
        self.assertEqual(scored['captured_observations'],2)

    def test_later_real_recovery_does_not_validate_an_early_success_claim(self):
        trial=self.configure()
        def early(mission):
            if mission.phase=='verify' and mission.pending is None and mission.outcome is None:
                mission._finish('recovered','controlled_early_claim')
                return True
            return False
        with patch.object(MissionState,'verify',early):report=self.composed()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'recovered')
        # Advance the actual station only after the immutable mission claim.
        for _ in range(24):self.f.sample(.5)
        self.close_station()
        self.assertEqual(station_evaluation.evaluate(self.f.root,trial)['outcome'],'recovered')
        scored=evaluate(self.f.root,trial,report['mission_id'])
        self.assertTrue(scored['outcome']['false_success_claim'])
        self.assertFalse(scored['outcome']['sustained_healthy_state_at_claim'])
        self.assertLess(scored['outcome']['healthy_window_s_at_claim'],10.)

    def test_mistaken_repair_of_a_healthy_machine_is_counted_as_unnecessary(self):
        trial=self.configure('healthy')
        with patch.object(mission_policy,'classification',return_value=('recoverable','pump','controlled_wrong_diagnosis')):
            report=self.composed()
        self.assertTrue(report['complete'],report)
        self.close_station()
        scored=evaluate(self.f.root,trial,report['mission_id'])
        self.assertGreater(scored['outcome']['unnecessary_mutation_attempts'],0)
        self.assertFalse(scored['outcome']['station_criteria_passed'])

    def test_actual_pre_admission_rejection_is_counted_without_an_applied_effect(self):
        trial=self.configure()
        def closing(family,request):
            if family=='maintenance':self.f.owner.closing=True
        self.bus.before_publish=closing
        report=self.composed();self.assertTrue(report['complete'],report)
        self.close_station()
        scored=evaluate(self.f.root,trial,report['mission_id'])
        commands=[c for c in scored['commands'] if c['family']=='maintenance']
        self.assertEqual(len(commands),1)
        self.assertIs(commands[0]['accepted'],False)
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],1)
        self.assertEqual(scored['outcome']['applied_interventions'],0)

    def test_real_gateway_signal_cancels_exact_navigation_and_retains_claim(self):
        trial=self.configure();self.bus.hold_navigation=True
        def stop(family,request):
            if family=='navigation' and request['op']=='arm':
                signal.getsignal(signal.SIGINT)(signal.SIGINT,None)
        self.bus.after_publish=stop
        report=self.composed();self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'cancelled')
        self.close_station()
        scored=evaluate(self.f.root,trial,report['mission_id'])
        self.assertEqual([v['operation'] for v in scored['commands']],['request','arm','cancel'])
        self.assertIsNone(scored['outcome']['success_claim_supported'])
        self.assertEqual(scored['outcome']['applied_interventions'],0)

    def test_real_configured_station_and_gateway_share_exact_observed_packets(self):
        trial=self.configure()
        report=self.composed();self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'recovered')
        self.assertEqual(len(self.capture_records),4)
        self.close_station()
        with evidence(self.f.root,trial) as station:
            lock=OwnerLock(self.f.root,service='mission-service')
            try:
                events,_=read(lock,report['mission_id'],station['registry'])
                private={v['value']['snapshot']['sequence']:v['value'] for line in station['observations'].splitlines()
                         if (v:=json.loads(line))['kind']=='sample'}
                packets=[json.loads(v['data']['text']) for v in events if v['kind']=='received'
                         and v['data']['topic']=='/station/telemetry']
                self.assertGreater(len(packets),20)
                for packet in packets:
                    self.assertEqual(packet['run_id'],station['report']['station_run_id'])
                    self.assertEqual(digest(packet),private[packet['sequence']]['public_sha256'])
                self.assertEqual(station['report']['outcome'],'recovered')
                self.assertEqual(station['report']['applied_interventions'],3)
            finally:lock.close()
        scored=evaluate(self.f.root,trial,report['mission_id'])
        self.assertTrue(scored['outcome']['success_claim_supported'],scored)
        self.assertEqual(scored['captured_observations'],4)
        self.assertEqual(scored['publication_attempts'],len(self.bus.sent))
        self.assertEqual(scored['scenario_input']['profile'],'recoverable')
        self.assertEqual(scored['scenario_input']['seed'],0)
        self.assertEqual(len(scored['scenario_input']['matched_conditions_sha256']),64)
        self.assertIsNone(scored['overall_acceptance'])
