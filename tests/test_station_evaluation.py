"""An observed successful station outcome cannot come from mission assertions."""
import copy
import json
import os
import subprocess
import sys
from unittest.mock import patch

from orbital.inspection_protocol import canonical
from orbital.maintenance_journal import prepare_configured_scenario, read_history
from orbital.machines import Station, Fault
from orbital.station_evaluation import evaluate
from orbital import station_evaluation as evaluation
from orbital.station_truth_protocol import digest
from station_evaluation_fixture import StationFixture
from test_machine_registry import RegistryFixture, ROOT
from test_machine_telemetry_adapter import AdapterHarness


def rewrite(path, change):
    events=[json.loads(line) for line in path.read_text().splitlines()]
    change(events)
    previous='0'*64
    for sequence,event in enumerate(events,1):
        event['sequence'],event['previous']=sequence,previous
        event['sha256']=digest({k:v for k,v in event.items() if k!='sha256'})
        previous=event['sha256']
    path.write_text(''.join(canonical(e)+'\n' for e in events))


class StationEvaluationTests(StationFixture):
    def test_actual_owned_repair_requires_new_sustained_operating_measurements(self):
        self.attach();result=self.restored()
        self.assertEqual(result['status'],'succeeded')
        for _ in range(23):self.sample()
        report=self.score()
        self.assertEqual(report['outcome'],'recovered')
        self.assertTrue(report['sustained_healthy_state'])
        self.assertEqual(report['applied_interventions'],3)
        self.assertEqual(report['operations'],{'isolate':1,'reset':1,'restore':1})
        self.assertEqual(report['unnecessary_interventions'],0)
        self.assertEqual(report['recorded_precondition_or_effect_violations'],[])
        self.assertIsNone(report['overall_acceptance'])
        self.assertIsNone(report['coverage']['collision'])
        self.assertIsNone(report['coverage']['independent_robot_interlocks'])

    def test_success_receipts_and_a_recovered_flag_cannot_replace_actual_clearance(self):
        self.attach()
        original=self.model.station.intervene
        def pulse_fails(*args,**kwargs):
            result=original(*args,**kwargs)
            if args[1]=='reset':self.model.station.machines['pump'].fault=Fault.LATCHED
            return result
        with patch.object(self.model.station,'intervene',side_effect=pulse_fails), patch.object(Station,'recovered',property(lambda _:True)):
            result=self.restored()
            self.assertEqual(result['status'],'succeeded')
            for _ in range(23):self.sample()
            self.assertTrue(self.model.station.recovered)
            report=self.score()
        self.assertFalse(report['sustained_healthy_state'])
        self.assertEqual(report['outcome'],'not_recovered')
        self.assertEqual(report['applied_interventions'],3)
        self.assertIn('observed_effect_did_not_match_operation',[v['reason'] for v in report['recorded_precondition_or_effect_violations']])

    def test_isolation_with_a_healthy_bus_is_not_repair(self):
        self.attach();self.isolated()
        for _ in range(24):self.sample()
        self.assertEqual(self.model.station.voltage_v,28.)
        report=self.score()
        self.assertEqual(report['outcome'],'not_recovered')
        self.assertFalse(report['sustained_healthy_state'])
        self.assertEqual(report['operations'],{'isolate':1,'reset':0,'restore':0})

    def test_actual_intervention_on_an_initially_healthy_machine_is_counted(self):
        self.attach('healthy')
        self.sample(.05);self.sample();self.start(self.request);self.apply()
        self.sample();self.sample();self.finish()
        for _ in range(24):self.sample()
        report=self.score()
        self.assertEqual(report['outcome'],'healthy_with_intervention')
        self.assertEqual(report['unnecessary_interventions'],1)
        self.assertEqual(report['unnecessary_action_ids'],[self.request['action_id']])

    def test_measurement_gap_restarts_the_window_even_if_the_model_counter_says_recovered(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        self.sample(3.)
        for _ in range(8):self.sample()
        self.assertTrue(self.model.station.recovered)
        report=self.score()
        self.assertFalse(report['sustained_healthy_state'])
        self.assertEqual(report['healthy_window_s'],4.)

    def test_unobserved_effect_or_coherent_control_rewrite_is_refused(self):
        self.attach();self.restored()
        for _ in range(23):self.sample()
        self.close_station()
        def drop(events):events[:]=[e for e in events if not(e['kind']=='effect' and e['value']['effect']['operation']=='reset')]
        rewrite(self.path,drop)
        with self.assertRaisesRegex(ValueError,'history'):
            evaluate(self.root,self.assignment['trial_id'])

    def test_wrong_trial_binding_and_coherently_rewritten_initial_truth_are_refused(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        self.close_station();original=self.path.read_bytes()
        for field in ('owner_run_id','configuration_sha256','initial'):
            self.path.write_bytes(original)
            def change(events):
                value=events[0]['value']
                if field=='initial':value['initial']['machines']['pump']['fault']='mechanical_jam'
                else:value[field]='f'*(64 if field=='configuration_sha256' else 32)
            rewrite(self.path,change)
            with self.subTest(field=field),self.assertRaises(ValueError):evaluate(self.root,self.assignment['trial_id'])

    def test_truncation_duplicate_json_fields_and_nonfinite_data_never_score(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        self.close_station();raw=self.path.read_bytes()
        for changed in (raw[:-1],raw.replace(b'"schema_version":1',b'"schema_version":1,"schema_version":1',1),
                        raw.replace(b'"model_time_s":0.0',b'"model_time_s":NaN',1)):
            self.path.write_bytes(changed)
            with self.assertRaises(ValueError):evaluate(self.root,self.assignment['trial_id'])

    def test_incomplete_close_and_live_owner_cannot_be_scored(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        with self.assertRaises((ValueError,OSError,RuntimeError)):evaluate(self.root,self.assignment['trial_id'])
        self.close_station(False)
        with self.assertRaisesRegex(ValueError,'incomplete'):evaluate(self.root,self.assignment['trial_id'])

    def test_unrecorded_private_fault_change_is_not_an_accepted_timing_scenario(self):
        self.attach('healthy');self.sample()
        self.model.station.machines['pump'].fault=Fault.JAM
        self.sample();self.close_station()
        with self.assertRaisesRegex(ValueError,'sample_history'):evaluate(self.root,self.assignment['trial_id'])

    def test_source_change_during_scoring_cannot_return_a_report(self):
        self.attach('healthy')
        for _ in range(24):self.sample()
        self.close_station()
        revision=evaluation.source_revision()
        with patch.object(evaluation,'source_revision',side_effect=[revision,'f'*64]):
            with self.assertRaisesRegex(ValueError,'source_changed'):evaluate(self.root,self.assignment['trial_id'])


class StationEvaluationAdapterTests(RegistryFixture):
    def run_profile(self, profile, seed=0):
        assignment=prepare_configured_scenario(self.root,profile,seed)
        harness=AdapterHarness(times=tuple(n*500_000_000 for n in range(1,26)))
        harness.run(self.root,prepare=False)
        return assignment,harness,evaluate(self.root,assignment['trial_id'])

    def test_actual_adapter_scores_all_four_profiles_without_full_acceptance_or_truth_leak(self):
        for profile in ('healthy','recoverable','persistent','ambiguous'):
            with self.subTest(profile=profile):
                assignment,harness,report=self.run_profile(profile)
                expected='healthy_no_applied_intervention' if profile=='healthy' else 'not_recovered'
                self.assertEqual(report['outcome'],expected)
                self.assertEqual(report['sustained_healthy_state'],profile=='healthy')
                self.assertEqual(report['applied_interventions'],0)
                self.assertIsNone(report['overall_acceptance'])
                for field in ('latched_controller','mechanical_jam','configuration_sha256','initial_state','"seed"'):
                    self.assertNotIn(field,canonical(harness.messages))
                    self.assertNotIn(field,canonical(report))
                self.assertEqual(read_history(self.root).current,assignment['trial_id'])

    def test_historical_trial_remains_bound_after_a_new_generation_is_prepared(self):
        assignment,harness,report=self.run_profile('healthy')
        prepare_configured_scenario(self.root,'persistent',2)
        self.assertEqual(evaluate(self.root,assignment['trial_id']),report)

    def test_actual_cli_is_read_only_bounded_and_does_not_print_private_errors(self):
        assignment,harness,report=self.run_profile('healthy')
        script=self.root/'scripts/evaluate-station.py';script.write_bytes((ROOT/'scripts/evaluate-station.py').read_bytes())
        path=self.root/'runtime/station-service/events.jsonl';before=path.read_bytes()
        env={**os.environ,'PYTHONPATH':str(ROOT),'PYTHONDONTWRITEBYTECODE':'1'}
        for trial,code in ((assignment['trial_id'],0),('../bad',2),('f'*32,2)):
            value=subprocess.run([sys.executable,'-B',str(script),'--trial-id',trial],env=env,
                capture_output=True,text=True,timeout=5.)
            self.assertEqual(value.returncode,code,value.stderr)
            self.assertLess(len(value.stdout+value.stderr),8192)
            if code==0:self.assertEqual(json.loads(value.stdout),report)
            else:self.assertEqual(json.loads(value.stderr),{'error':'station_evaluation_unavailable'})
        self.assertEqual(path.read_bytes(),before)
