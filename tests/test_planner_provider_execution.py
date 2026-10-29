"""Offline Messages turns over actual ROS adapter, station and capture files."""
import copy
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

from orbital import ros_mission
from orbital.mission_evaluation import evaluate
from orbital.planner_recording import RecordedProvider
from mission_evaluation_fixture import MissionEvaluationFixture
from mission_ros_fixture import MissionRos,runtime_identity
from planner_provider_fixture import recording,recovery_steps,step


def profile_steps(profile,delay=1.5):
    if profile=='recoverable':return recovery_steps(delay)
    if profile=='stopped':return [step('stop',{'reason':'operator_request'},'stop_1',delay)]
    if profile=='malformed':
        value=recovery_steps(delay)[0]
        value['response']['content'].append(copy.deepcopy(value['response']['content'][0]))
        value['response']['content'][1]['id']='parallel_2'
        return [value]
    machines=['pump','cooler','power_unit'] if profile=='healthy' else ['pump']*(2 if profile=='ambiguous' else 1)
    result=[]
    for machine in machines:
        result.append(step('navigate_to',{'waypoint_id':machine,'timeout_s':5.},'call_'+str(len(result)),delay))
        result.append(step('inspect_machine',{'machine_id':machine,'timeout_s':5.},'call_'+str(len(result)),delay))
    if profile=='healthy':
        result.append(step('verify_recovery',{'machine_id':machines[-1],'observation_window_s':10.},'verify_1',delay))
    result.append(step('report_outcome',{'outcome':'healthy' if profile=='healthy' else 'escalated',
        'evidence_ids':['$known_evidence'],'summary':'Synthetic recorded profile claim.'},'report_1',delay))
    return result


class PlannerProviderExecutionTests(MissionEvaluationFixture):
    def composed_provider(self,steps):
        self.release_initial_gateway();self.ros=MissionRos(self.bus)
        self.provider=RecordedProvider(recording(steps))
        with patch.dict(sys.modules,self.ros.modules),patch.object(ros_mission,'runtime_snapshot',
                return_value=runtime_identity(self.f.model.registry)):
            return ros_mission.run(self.f.root,mission_id=self.f.request['mission_id'],
                timeout_s=60.,now=self.bus.now,provider=self.provider)

    def run_and_score(self,profile,delay=1.5):
        trial=self.configure('recoverable' if profile in ('stopped','malformed') else profile)
        report=self.composed_provider(profile_steps(profile,delay));self.assertTrue(report['complete'],report)
        self.assertTrue(report['witness']['complete'])
        self.assertEqual(report['planner']['mode'],'recorded-tools-v1')
        self.assertEqual(report['planner']['calls']['mode'],'recorded-tools-v1')
        self.assertTrue(report['provider']['history']['closed'])
        self.assertTrue(self.provider.closed)
        self.assertTrue(self.ros.destroyed and self.ros.shutdown)
        self.assertEqual(self.bus.callbacks,{})
        self.close_station()
        return report,evaluate(self.f.root,trial,report['mission_id'])

    def test_five_second_responses_recover_actual_station_with_four_captures(self):
        report,scored=self.run_and_score('recoverable',delay=5.)
        self.assertEqual(report['mission']['outcome'],'recovered')
        self.assertTrue(scored['outcome']['success_claim_supported'])
        self.assertEqual(scored['captured_observations'],4)
        self.assertEqual(scored['outcome']['applied_interventions'],3)
        self.assertIsNone(scored['overall_acceptance'])

    def test_healthy_replay_inspects_three_machines_without_mutation(self):
        report,scored=self.run_and_score('healthy')
        self.assertEqual(report['mission']['outcome'],'healthy')
        self.assertTrue(scored['outcome']['success_claim_supported'])
        self.assertEqual(scored['captured_observations'],3)
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],0)

    def test_persistent_replay_escalates_actual_jam_without_reset(self):
        report,scored=self.run_and_score('persistent')
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertTrue(scored['outcome']['reported_escalation'])
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],0)

    def test_ambiguous_replay_escalates_after_actual_bounded_inspections(self):
        report,scored=self.run_and_score('ambiguous')
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertEqual(scored['captured_observations'],2)
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],0)

    def test_recorded_stop_has_no_native_publication_or_success_claim(self):
        report,scored=self.run_and_score('stopped')
        self.assertEqual(report['mission']['outcome'],'cancelled')
        self.assertEqual(scored['commands'],[])
        self.assertIsNone(scored['outcome']['success_claim_supported'])

    def test_parallel_recording_is_rejected_before_either_native_publication(self):
        report,scored=self.run_and_score('malformed')
        self.assertEqual(report['provider']['failure'],'invalid_reply')
        self.assertEqual(scored['commands'],[])
        self.assertIsNone(scored['outcome']['success_claim_supported'])

    def test_cli_requires_exclusive_explicit_recording_and_valid_bounds_before_io(self):
        script=Path(__file__).resolve().parents[1]/'scripts/run-scripted-mission.py'
        cases=[['run','--provider-max-turns','1'],['run','--mission-text','probe'],
            ['run','--provider-replay','missing','--tool-contract'],
            ['reset-tool-stop','--acknowledge-stop','--provider-replay','missing'],
            ['run','--provider-replay','missing','--provider-max-turns','33'],
            ['run','--provider-replay','missing','--provider-max-rejections','-1']]
        for args in cases:
            with self.subTest(args=args):
                result=subprocess.run([sys.executable,'-B',str(script),*args],capture_output=True,text=True,
                    env={'PATH':'/usr/bin:/bin','TMPDIR':'/Users/gok/codex/.tmp','PYTHONDONTWRITEBYTECODE':'1'},timeout=10.)
                self.assertEqual(result.returncode,2,result)
                self.assertNotIn('ModuleNotFoundError',result.stderr)
                self.assertEqual(result.stdout,'')

    def test_cli_missing_recording_is_refused_before_ros_import(self):
        script=Path(__file__).resolve().parents[1]/'scripts/run-scripted-mission.py'
        result=subprocess.run([sys.executable,'-B',str(script),'run','--provider-replay',str(self.f.root/'missing')],
            capture_output=True,text=True,env={'PATH':'/usr/bin:/bin','ROS_DOMAIN_ID':'84',
                'ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST','TMPDIR':'/Users/gok/codex/.tmp',
                'PYTHONDONTWRITEBYTECODE':'1'},timeout=10.)
        self.assertEqual(result.returncode,1,result)
        self.assertEqual(json.loads(result.stdout)['error'],'invalid_recording')
        self.assertEqual(result.stderr,'')
