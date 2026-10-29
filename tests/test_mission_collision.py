"""A valid collision file alone cannot establish correspondence to a mission."""
from unittest.mock import patch

from orbital import mission_collision
from orbital.collision_evaluation import evaluate_trace
from mission_collision_fixture import trace
from mission_evidence_corpus import MissionCorpus


class MissionCollisionTests(MissionCorpus):
    def test_complete_generation_command_and_interval_join_remains_sampled(self):
        path=trace(self.root,self.mission)
        report=self.score(collision_trace=path)
        self.assertTrue(report['coverage']['collision'])
        self.assertEqual(report['collision']['matched_navigation_commands'],2)
        self.assertTrue(report['collision']['covering_trace']['sampled_clear'])
        self.assertIsNone(report['collision']['continuous_collision_free'])
        self.assertIsNone(report['overall_acceptance'])
        self.assertTrue(all(v['accepted'] for v in report['commands'] if v['family']=='navigation'))

    def test_same_world_with_different_service_invocation_refuses_join(self):
        path=trace(self.root,self.mission,runtime_change=True)
        self.assertTrue(evaluate_trace(path)['sampled_clear'])
        with self.assertRaisesRegex(ValueError,'runtime_binding'):self.score(collision_trace=path)

    def test_same_runtime_with_different_navigation_owner_refuses_join(self):
        path=trace(self.root,self.mission,wrong_run=True)
        self.assertTrue(evaluate_trace(path)['sampled_clear'])
        with self.assertRaisesRegex(ValueError,'navigation_generation'):self.score(collision_trace=path)

    def test_coincident_sim_time_without_wall_interval_coverage_refuses(self):
        path=trace(self.root,self.mission,wall_shift=100000000000)
        self.assertTrue(evaluate_trace(path)['sampled_clear'])
        with self.assertRaisesRegex(ValueError,'does_not_cover_mission'):self.score(collision_trace=path)

    def test_clear_same_generation_trace_without_actual_commands_refuses(self):
        path=trace(self.root,self.mission,omit_commands=True)
        self.assertTrue(evaluate_trace(path)['sampled_clear'])
        with self.assertRaisesRegex(ValueError,'command_correspondence'):self.score(collision_trace=path)

    def test_adverse_geometry_cannot_be_hidden_by_station_success(self):
        path=trace(self.root,self.mission,adverse=True)
        report=self.score(collision_trace=path)
        self.assertTrue(report['outcome']['success_claim_supported'])
        self.assertFalse(report['collision']['covering_trace']['sampled_clear'])
        self.assertGreater(report['collision']['covering_trace']['overlap_or_touch_samples'],0)
        self.assertIsNone(report['overall_acceptance'])

    def test_trace_replacement_after_scoring_refuses_final_report(self):
        path=trace(self.root,self.mission);real=mission_collision.evaluate
        def changing(*args,**kwargs):
            result=real(*args,**kwargs)
            with (path/'events.jsonl').open('ab') as stream:stream.write(b'\n')
            return result
        with patch.object(mission_collision,'evaluate',changing):
            with self.assertRaisesRegex(ValueError,'files_changed'):self.score(collision_trace=path)
