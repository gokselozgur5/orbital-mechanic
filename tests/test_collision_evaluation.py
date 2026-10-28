"""Replay complete production-format traces; mutate semantics beyond file hashes."""
from contextlib import redirect_stdout
import copy
import hashlib
import importlib.util
from io import StringIO
import json
import math
from pathlib import Path
import tempfile
import unittest

from orbital.collision_evaluation import evaluate_trace
from orbital.collision_geometry import Scene, IDENTITY
from orbital.trajectory import TraceWriter, Trajectory, TRUTH_TOPIC, truth_from_message, verify_trace
from collision_fixture import scene_message
from test_trajectory import state

ROOT = Path(__file__).resolve().parents[1]
WORLD = (ROOT/'worlds/maintenance.sdf').read_bytes()
SPEC = importlib.util.spec_from_file_location('collision_cli_test', ROOT/'scripts/evaluate-navigation-collisions.py')
cli = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cli)


class CollisionEvaluationTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'runtime').mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT/'runtime')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.scene = Scene(WORLD)

    def trace(self, *, poses=None, models=None, count=2):
        writer = TraceWriter(self.root, 'capture')
        self.addCleanup(writer.stream.close)
        trace = Trajectory(writer)
        at = 10**9
        def sample(pose, model):
            nonlocal at
            trace.state(at, state(at))
            truth = truth_from_message(scene_message(WORLD, at, robot_pose=pose, model_poses=model), self.scene)
            trace.truth(at, truth)
            # Deliberately fixed localization: collision must follow Gazebo truth.
            trace.match(lambda stamp: list(IDENTITY), at)
        sample(IDENTITY, None)
        self.assertTrue(trace.start(at))
        for i in range(count):
            at += 20000000
            sample(poses[i] if poses else IDENTITY, models[i] if models else None)
        observation = trace.finish(at)
        fence = {'manifest_identity': 'a'*64, 'world_sha256': self.scene.sha256,
                 'service_invocations': {name: 'b'*32 for name in (
                     'orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service')}}
        bindings = {name: {'path': '/synthetic-fixture/'+name, 'sha256': 'c'*64} for name in (
            'gz.transport13', 'gz.transport13._transport', 'gz.msgs10.pose_v_pb2')}
        writer.finish(True, {'scope': 'Synthetic trace fixture; no native evidence',
                             'truth_topic': TRUTH_TOPIC, 'before': fence, 'after': copy.deepcopy(fence),
                             'bindings_before': bindings, 'bindings_after': copy.deepcopy(bindings),
                             'observation': observation, 'geometry': {
                                 'schema_version': 1, 'world_sdf': WORLD.decode(), 'world_sha256': self.scene.sha256,
                                 'pose_convention': 'world model poses; model-relative robot links'}})
        self.path = writer.path
        return writer.path

    def rewrite(self, change_records=None, change_summary=None):
        events = self.path/'events.jsonl'
        summary_path = self.path/'summary.json'
        rows = [json.loads(line) for line in events.read_text().splitlines()]
        summary = json.loads(summary_path.read_text())
        if change_records:
            change_records(rows)
        for number, row in enumerate(rows, 1):
            row['sequence'] = number
        raw = ''.join(json.dumps(row, separators=(',', ':'))+'\n' for row in rows).encode()
        events.write_bytes(raw)
        summary.update(records=len(rows), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())
        if change_summary:
            change_summary(summary)
        summary_path.write_text(json.dumps(summary))
        self.assertTrue(verify_trace(self.path)['complete'])  # Integrity alone is insufficient.

    def test_clear_trace_reports_all_parts_pairs_hashes_and_sampled_scope(self):
        result = evaluate_trace(self.trace())
        self.assertTrue(result['evaluation_complete'])
        self.assertTrue(result['sampled_clear'])
        self.assertEqual((result['samples'], result['checked_obstacle_pairs']), (2, 56))
        self.assertIsNone(result['continuous_collision_free'])
        self.assertEqual(result['world_sha256'], self.scene.sha256)
        self.assertEqual(len(result['evaluator_source_sha256']), 5)

    def test_body_overlap_follows_truth_while_localization_stays_at_clear_origin(self):
        result = evaluate_trace(self.trace(poses=[(2.5, 0., 0., 0., 0., 0., 1.)]*2))
        self.assertFalse(result['sampled_clear'])
        self.assertEqual(result['overlap_or_touch_samples'], 2)
        self.assertTrue(any(e['part'] == 'base_link/collision' and e['obstacle'].startswith('cooler/')
                            for e in result['event_examples']))

    def test_changed_obstacle_detects_wheel_contact_in_complete_trace(self):
        result = evaluate_trace(self.trace(models=[{'cooler': (0., .66, .6, 0., 0., 0., 1.)}]*2))
        self.assertFalse(result['sampled_clear'])
        self.assertEqual({e['part'] for e in result['event_examples']}, {'left_wheel/collision'})

    def test_tipping_and_deep_floor_penetration_cannot_pass(self):
        tilted = (0., 0., 0., math.sin(math.pi/8), 0., 0., math.cos(math.pi/8))
        result = evaluate_trace(self.trace(poses=[tilted, (0., 0., -.01, 0., 0., 0., 1.)]))
        self.assertFalse(result['sampled_clear'])
        self.assertEqual(result['floor_violation_samples'], 2)
        self.assertGreaterEqual(max(result['max_support_penetration_m'].values()), .01)

    def test_clear_endpoints_never_claim_continuous_collision_freedom(self):
        result = evaluate_trace(self.trace(poses=[(1., 0., 0., 0., 0., 0., 1.), (4., 0., 0., 0., 0., 0., 1.)]))
        self.assertTrue(result['sampled_clear'])
        self.assertIsNone(result['continuous_collision_free'])

    def test_legacy_model_only_trace_is_refused(self):
        self.trace()
        self.rewrite(change_summary=lambda value: value.pop('geometry'))
        with self.assertRaisesRegex(ValueError, 'full-scene geometry'):
            evaluate_trace(self.path)

    def test_missing_truth_sample_is_refused_even_after_rehashing(self):
        self.trace()
        self.rewrite(lambda rows: rows.pop(next(i for i, row in enumerate(rows)
                                               if row['kind'] == 'sample' and row['data']['capture'])))
        with self.assertRaises(ValueError):
            evaluate_trace(self.path)

    def test_wrong_sample_state_reference_is_refused_even_after_rehashing(self):
        self.trace()
        def change(rows):
            row = next(r for r in rows if r['kind'] == 'sample' and r['data']['capture'])
            row['data']['state_record_at_truth_receipt'] = 1
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError, 'truth-time observation'):
            evaluate_trace(self.path)

    def test_false_error_and_localization_timestamp_are_rejected(self):
        self.trace()
        def change(rows):
            row = next(r for r in rows if r['kind'] == 'sample' and r['data']['capture'])
            row['data']['error']['xy_m'] = .5
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError, 'error does not match'):
            evaluate_trace(self.path)

    def test_wrong_world_hash_and_changed_deployment_fences_are_rejected(self):
        self.trace()
        self.rewrite(change_summary=lambda s: s['after'].update(world_sha256='0'*64))
        with self.assertRaisesRegex(ValueError, 'fences'):
            evaluate_trace(self.path)

    def test_summary_count_cannot_override_actual_replay(self):
        self.trace()
        self.rewrite(change_summary=lambda s: s['observation'].update(capture_samples=999))
        with self.assertRaisesRegex(ValueError, 'summary differs'):
            evaluate_trace(self.path)

    def test_changed_static_link_identity_cannot_hide_behind_signature_text(self):
        self.trace()
        def change(rows):
            row = [r for r in rows if r['kind'] == 'truth'][-1]
            row['data']['static_links']['body'][0]['entity_id'] = 999
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError, 'entity generation'):
            evaluate_trace(self.path)

    def test_missing_audit_event_is_rejected_by_shared_protocol_validator(self):
        self.trace()
        def change(rows):
            for row in rows:
                if row['kind'] == 'state' and row['received_monotonic_ns'] > 10**9:
                    row['data']['audit_sequence'] = 1
        self.rewrite(change)
        with self.assertRaisesRegex(ValueError, 'audit tail'):
            evaluate_trace(self.path)

    def test_integrity_failure_never_returns_a_partial_clear_result(self):
        self.trace()
        with (self.path/'events.jsonl').open('ab') as stream:
            stream.write(b'\n')
        output = StringIO()
        with redirect_stdout(output):
            code = cli.main([str(self.path)])
        self.assertEqual(code, 2)
        self.assertFalse(json.loads(output.getvalue())['evaluation_complete'])

    def test_cli_adverse_geometry_has_distinct_exit_from_invalid_evidence(self):
        self.trace(poses=[(2.5, 0., 0., 0., 0., 0., 1.)]*2)
        output = StringIO()
        with redirect_stdout(output):
            code = cli.main([str(self.path)])
        result = json.loads(output.getvalue())
        self.assertEqual(code, 1)
        self.assertTrue(result['evaluation_complete'])
        self.assertFalse(result['sampled_clear'])

    def test_report_examples_are_bounded_without_losing_counts(self):
        result = evaluate_trace(self.trace(poses=[(2.5, 0., 0., 0., 0., 0., 1.)]*150, count=150))
        self.assertEqual(result['overlap_or_touch_samples'], 150)
        self.assertGreater(result['event_count'], 100)
        self.assertEqual(len(result['event_examples']), 100)
        self.assertTrue(result['examples_truncated'])


if __name__ == '__main__':
    unittest.main()
