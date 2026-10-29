"""Real mission adapter, actual station history and capture files over tool calls."""
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

from orbital import ros_mission
from orbital.mission_evaluation import evaluate
from orbital.inspection_protocol import canonical
from orbital.inspection_worker import OwnerLock
from orbital.mission_journal import MissionJournal
from orbital.planner_runtime import PlannerRuntime
from mission_evaluation_fixture import MissionEvaluationFixture
from mission_ros_fixture import MissionRos, runtime_identity


class PlannerExecutionTests(MissionEvaluationFixture):
    def composed_tools(self):
        self.release_initial_gateway()
        self.ros=MissionRos(self.bus)
        with patch.dict(sys.modules,self.ros.modules),patch.object(ros_mission,'runtime_snapshot',
                return_value=runtime_identity(self.f.model.registry)):
            return ros_mission.run(self.f.root,mission_id=self.f.request['mission_id'],
                timeout_s=60.,now=self.bus.now,tool_contract=True)

    def run_and_score(self,profile):
        trial=self.configure(profile)
        report=self.composed_tools();self.assertTrue(report['complete'],report)
        self.assertTrue(report['witness']['complete'])
        self.assertEqual(report['planner']['mode'],'scripted-tools-v1')
        self.assertTrue(all(c['response'] is not None for c in report['planner']['calls']['calls'].values()))
        self.assertTrue(self.ros.destroyed and self.ros.shutdown)
        self.assertEqual(len(self.ros.subscribers),8)
        self.assertEqual(self.bus.callbacks,{})
        self.close_station()
        return report,evaluate(self.f.root,trial,report['mission_id'])

    def test_recoverable_tool_mission_has_four_actual_captures_and_three_effects(self):
        report,scored=self.run_and_score('recoverable')
        self.assertEqual(report['mission']['outcome'],'recovered')
        self.assertTrue(scored['outcome']['success_claim_supported'])
        self.assertEqual(scored['captured_observations'],4)
        self.assertEqual(scored['outcome']['applied_interventions'],3)
        self.assertIsNone(scored['overall_acceptance'])

    def test_healthy_tool_mission_inspects_three_machines_without_mutating(self):
        report,scored=self.run_and_score('healthy')
        self.assertEqual(report['mission']['outcome'],'healthy')
        self.assertTrue(scored['outcome']['success_claim_supported'])
        self.assertEqual(scored['captured_observations'],3)
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],0)

    def test_persistent_tool_mission_escalates_without_false_success(self):
        report,scored=self.run_and_score('persistent')
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertTrue(scored['outcome']['reported_escalation'])
        self.assertIsNone(scored['outcome']['success_claim_supported'])

    def test_ambiguous_tool_mission_escalates_without_intervention(self):
        report,scored=self.run_and_score('ambiguous')
        self.assertEqual(report['mission']['outcome'],'escalated')
        self.assertEqual(scored['captured_observations'],2)
        self.assertEqual(scored['outcome']['mutation_publication_attempts'],0)

    def test_tool_stop_and_operator_reset_preserve_the_original_mission_evaluation(self):
        trial=self.configure();self.bus.hold_navigation=True;pending=[]
        original=PlannerRuntime.start
        def start(runtime):
            original(runtime)
            def stop(family,command):
                if family=='navigation' and command['op']=='arm':
                    import uuid
                    pending.append(runtime.gateway.execute(canonical({'schema_version':1,
                        'kind':'planner_tool_request','run_id':runtime.inputs.run_id,
                        'mission_id':runtime.mission_id,'action_id':uuid.uuid4().hex,
                        'tool':'stop','observation_id':None,'arguments':{'reason':'operator_request'}})))
            self.bus.after_publish=stop
        with patch.object(PlannerRuntime,'start',start):report=self.composed_tools()
        self.assertTrue(report['complete'],report)
        self.assertEqual(report['mission']['outcome'],'cancelled')
        self.assertEqual(len(pending),1);self.assertFalse(pending[0]['terminal'])
        self.close_station()
        before=evaluate(self.f.root,trial,report['mission_id'])
        self.assertEqual([v['operation'] for v in before['commands']],['request','arm','cancel'])
        lock=OwnerLock(self.f.root,service='mission-service')
        try:
            journal=MissionJournal(lock)
            try:journal.reset_planner_stop()
            finally:journal.close()
        finally:lock.close()
        self.assertEqual(evaluate(self.f.root,trial,report['mission_id']),before)

    def test_cli_requires_explicit_reset_ack_and_rejects_wrong_domain_before_io(self):
        script=Path(__file__).resolve().parents[1]/'scripts/run-scripted-mission.py'
        cases=[['reset-tool-stop'],['reset-tool-stop','--acknowledge-stop','--tool-contract'],
               ['run','--acknowledge-stop'],['run','--tool-contract'],['reset-tool-stop','--acknowledge-stop']]
        for args in cases:
            with self.subTest(args=args):
                process=subprocess.run([sys.executable,'-B',str(script),*args],capture_output=True,text=True,
                    env={'PATH':'/usr/bin:/bin','ROS_DOMAIN_ID':'77','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
                         'TMPDIR':'/Users/gok/codex/.tmp','PYTHONDONTWRITEBYTECODE':'1'},timeout=10.)
                self.assertNotEqual(process.returncode,0)
                self.assertNotIn('ModuleNotFoundError',process.stderr)
                if process.stdout:self.assertIn('requires_localhost',json.loads(process.stdout)['error'])
