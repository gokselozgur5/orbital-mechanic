"""Evaluator data loss, timestamp alignment and artifact boundaries without ROS."""
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from orbital.trajectory import (Inbox, TraceWriter, Trajectory, pose_error, stamp_ns,
                                transform_at, truth_from_message, verify_trace)

ROOT = Path(__file__).resolve().parents[1]
POSE = [0., 0., .02, 0., 0., 0., 1.]
TARGET = {'run_id': 'run-a', 'epoch': 1, 'sequence': 1, 'waypoint': 'pump'}


class Proto(NS):
    def HasField(self, name):
        return hasattr(self, name)

    def ByteSize(self):
        return 512


def truth_message(stamp=1000000000):
    return Proto(header=Proto(stamp=Proto(sec=stamp//10**9, nsec=stamp % 10**9)), pose=[
        Proto(name='maintenance_robot', id=42, position=Proto(x=0., y=0., z=.02),
              orientation=Proto(x=0., y=0., z=0., w=1.)),
        Proto(name='base_link', id=43)])


def transform(stamp):
    return NS(header=NS(frame_id='map', stamp=NS(sec=stamp//10**9, nanosec=stamp % 10**9)),
              child_frame_id='base_footprint',
              transform=NS(translation=NS(x=.03, y=.04, z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))


def state(stamp=1000000000, **values):
    return json.dumps({'run_id': 'run-a', 'command_sequence': 0, 'target': None,
                       'action_state': 'idle', 'healthy': True, 'guard_latched': True,
                       'last_reply': None, 'last_completion': None, 'forwarded_commands': 0,
                       'audit_sequence': 0, 'audit_ok': True, 'sim_stamp_ns': stamp, **values})


def request_audit(sequence=1, **values):
    return json.dumps({'schema_version': 1, 'run_id': 'run-a', 'sequence': sequence,
                       'sim_stamp_ns': 1010000000, 'command_sequence': 1, 'forwarded_commands': 0,
                       'target': TARGET, 'event': 'command_reply',
                       'command': {'run_id': 'run-a', 'sequence': 1, 'op': 'request', 'waypoint': 'pump'},
                       'reply': {'sequence': 1, 'op': 'request', 'accepted': True}, **values})


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'runtime').mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT/'runtime')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.writer = TraceWriter(self.root, 'capture')
        self.addCleanup(self.writer.stream.close)
        self.trace = Trajectory(self.writer)
        self.now = 1000000000

    def begin(self):
        self.trace.state(self.now, state())
        self.trace.truth(self.now, truth_from_message(truth_message(self.now)))
        self.trace.match(lambda stamp: transform_at(transform(stamp), stamp), self.now)
        self.assertTrue(self.trace.start(self.now))

    def sample(self, **state_values):
        self.now += 20000000
        self.trace.state(self.now, state(self.now, **state_values))
        self.trace.truth(self.now, truth_from_message(truth_message(self.now)))
        self.trace.match(lambda stamp: transform_at(transform(stamp), stamp), self.now)

    def test_independent_truth_is_retained_and_error_uses_exact_time_estimate(self):
        self.begin()
        self.sample()
        self.sample()
        details = self.trace.finish(self.now)
        self.writer.finish(True, {'observation': details})
        self.assertEqual(verify_trace(self.writer.path)['observation']['capture_samples'], 2)
        records = [json.loads(line) for line in (self.writer.path/'events.jsonl').read_text().splitlines()]
        samples = [r['data'] for r in records if r['kind'] == 'sample']
        self.assertAlmostEqual(samples[-1]['error']['xy_m'], .05)
        truth = [r for r in records if r['kind'] == 'truth'][-1]
        self.assertEqual(truth['data']['pose'], POSE)
        self.assertEqual(samples[-1]['truth_record'], truth['sequence'])
        self.assertLess(samples[-1]['state_record_at_truth_receipt'], truth['sequence'])

    def test_exact_command_audit_window_and_final_state_counter(self):
        self.begin()
        self.trace.audit(self.now+1, request_audit())
        fields = dict(command_sequence=1, audit_sequence=1, target=TARGET, action_state='waiting_for_arm')
        self.sample(**fields)
        self.sample(**fields)
        details = self.trace.finish(self.now)
        self.assertEqual(details['baseline']['audit_sequence'], 0)
        self.assertEqual(details['final_audit_sequence'], 1)

    def test_missing_audit_tail_cannot_pass_as_no_commands(self):
        self.begin()
        self.sample(audit_sequence=1, command_sequence=1)
        self.sample(audit_sequence=1, command_sequence=1)
        with self.assertRaisesRegex(ValueError, 'audit tail'):
            self.trace.finish(self.now)

    def test_velocity_audit_retains_exact_target_and_counts_forwarding_only(self):
        self.begin()
        self.trace.audit(self.now+1, request_audit())
        velocity = {'schema_version': 1, 'event': 'velocity_forwarded', 'run_id': 'run-a',
                    'sequence': 2, 'sim_stamp_ns': self.now+20000000, 'command_sequence': 1,
                    'forwarded_commands': 1, 'target': TARGET, 'velocity_stamp_ns': self.now+10000000,
                    'frame_id': 'base_link', 'velocity': [.1, 0, 0, 0, 0, .2]}
        self.trace.audit(self.now+2, json.dumps(velocity))
        fields = dict(command_sequence=1, audit_sequence=2, forwarded_commands=1,
                      target=TARGET, action_state='running', guard_latched=False)
        self.sample(**fields)
        self.sample(**fields)
        details = self.trace.finish(self.now)
        self.assertEqual(details['final_forwarded_commands'], 1)
        self.assertNotIn('arrived', details)
        self.assertNotIn('stopped', details)

    def test_foreign_target_missing_velocity_and_counter_gap_are_fatal(self):
        for changed in ({'target': None}, {'target': {**TARGET, 'run_id': 'other'}},
                        {'velocity': [0]}, {'forwarded_commands': 3}):
            trace = Trajectory(self.writer)
            trace.state(self.now, state())
            velocity = {'schema_version': 1, 'event': 'velocity_forwarded', 'run_id': 'run-a',
                        'sequence': 1, 'sim_stamp_ns': self.now, 'command_sequence': 0,
                        'forwarded_commands': 1, 'target': TARGET, 'velocity_stamp_ns': self.now,
                        'frame_id': 'base_link', 'velocity': [.1, 0, 0, 0, 0, .2], **changed}
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                trace.audit(self.now+1, json.dumps(velocity))

    def test_audit_clock_regression_is_not_hidden_by_increasing_counter(self):
        self.begin()
        with self.assertRaisesRegex(ValueError, 'time regressed'):
            self.trace.audit(self.now+1, request_audit(sim_stamp_ns=1))

    def test_gap_duplicate_and_foreign_run_audit_are_rejected(self):
        self.begin()
        for event in (request_audit(2), request_audit(run_id='other')):
            with self.subTest(event=event), self.assertRaises(ValueError):
                self.trace.audit(self.now+1, event)
        self.trace.audit(self.now+2, request_audit())
        with self.assertRaisesRegex(ValueError, 'sequence gap'):
            self.trace.audit(self.now+3, request_audit())

    def test_restart_and_audit_publication_failure_reject_state(self):
        for fields in ({'run_id': 'other'}, {'audit_ok': False}):
            trace = Trajectory(self.writer)
            trace.state(self.now, state())
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                trace.state(self.now+1, state(**fields))

    def test_truth_reset_duplicate_and_replaced_entity_are_rejected(self):
        self.begin()
        original = truth_from_message(truth_message(self.now))
        for fields in ({'sim_stamp_ns': 1}, {}, {'sim_stamp_ns': self.now+1, 'entity_id': 44}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.trace.truth(self.now+1, {**original, **fields})

    def test_missing_exact_time_transform_is_preserved_and_fails_capture(self):
        self.begin()
        self.trace.truth(self.now+20000000, truth_from_message(truth_message(self.now+20000000)))
        def missing(stamp):
            raise LookupError('future transform')
        self.trace.match(missing, self.now+30000000)
        self.assertEqual(len(self.trace.pending), 1)
        with self.assertRaisesRegex(ValueError, 'Localization missing'):
            self.trace.match(missing, self.now+600000000)
        self.writer.sync()
        last = json.loads((self.writer.path/'events.jsonl').read_text().splitlines()[-1])
        self.assertIsNone(last['data']['localization_pose'])
        self.assertEqual(last['data']['missing_reason'], 'exact_time_transform_unavailable')

    def test_warmup_can_start_with_newer_truth_waiting_for_interpolation(self):
        self.trace.state(self.now, state())
        self.trace.truth(self.now, truth_from_message(truth_message(self.now)))
        self.trace.match(lambda stamp: POSE, self.now)
        self.trace.truth(self.now+1, truth_from_message(truth_message(self.now+1)))
        self.assertTrue(self.trace.start(self.now+1))

    def test_late_available_transform_cannot_escape_the_capture_wait_bound(self):
        self.begin()
        at = self.now+20000000
        self.trace.truth(at, truth_from_message(truth_message(at)))
        # A delayed executor must not accept a transform after the declared wait
        # simply because lookup happens to succeed on its first resumed callback.
        with self.assertRaisesRegex(ValueError, 'Localization missing'):
            self.trace.match(lambda stamp: POSE, at+500000000)

    def test_warmup_requires_idle_latched_and_no_command_activity(self):
        self.trace.state(self.now, state(guard_latched=False))
        self.trace.truth(self.now, truth_from_message(truth_message(self.now)))
        self.trace.match(lambda stamp: POSE, self.now)
        self.assertFalse(self.trace.start(self.now))
        self.trace.state(self.now+1, state(command_sequence=1, audit_sequence=1))
        with self.assertRaisesRegex(ValueError, 'Command activity'):
            self.trace.start(self.now+1)

    def test_sampling_gap_invalidates_capture(self):
        self.begin()
        with self.assertRaisesRegex(ValueError, 'sampling gap'):
            self.trace.truth(self.now+300000000, truth_from_message(truth_message(self.now+300000000)))
    def test_stale_state_invalidates_capture_even_with_recent_truth(self):
        self.begin()
        # Small truth time increments do not disguise stale navigation receipt.
        self.trace.last_truth = (self.now+310000000, truth_from_message(truth_message(self.now+310000000)))
        with self.assertRaisesRegex(ValueError, 'fresh navigation'):
            self.trace.truth(self.now+320000000, truth_from_message(truth_message(self.now+320000000)))

    def test_trace_tamper_truncation_and_incomplete_manifest_are_refused(self):
        self.begin()
        self.sample()
        self.sample()
        self.writer.finish(True, {'observation': self.trace.finish(self.now)})
        path = self.writer.path/'events.jsonl'
        original = path.read_bytes()
        for changed in (original[:-1], original.replace(b'run-a', b'run-b')):
            path.write_bytes(changed)
            with self.assertRaises(ValueError):
                verify_trace(self.writer.path)
        path.write_bytes(original)
        summary_path = self.writer.path/'summary.json'
        summary = json.loads(summary_path.read_text())
        summary['complete'] = False
        summary_path.write_text(json.dumps(summary))
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            verify_trace(self.writer.path)

    def test_exclusive_output_refuses_path_escape_symlinks_and_overwrite(self):
        for name in ('../escape', '/tmp/escape', 'capture', '..'):
            with self.subTest(name=name), self.assertRaises((ValueError, FileExistsError)):
                TraceWriter(self.root, name)
        other = self.root/'another'
        other.mkdir()
        (other/'runtime').symlink_to(self.root/'runtime', target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            TraceWriter(other, 'new')

    def test_output_record_and_byte_caps_preserve_existing_prefix(self):
        for name, kwargs in (('count', {'max_records': 1}), ('bytes', {'max_bytes': 150})):
            writer = TraceWriter(self.root, name, **kwargs)
            self.addCleanup(writer.stream.close)
            writer.append('boundary', self.now, {})
            with self.assertRaisesRegex(ValueError, 'bound'):
                writer.append('boundary', self.now, {'large': 'a'*100})
            result = writer.finish(False, {'error': 'limit'})
            self.assertEqual(result['records'], 1)


class TrajectoryInputTests(unittest.TestCase):
    def test_exact_model_and_all_pose_components_are_required(self):
        good = truth_message()
        self.assertEqual(truth_from_message(good)['entity_id'], 42)
        variants = []
        missing = copy.deepcopy(good)
        missing.pose = missing.pose[1:]
        variants.append(missing)
        duplicate = copy.deepcopy(good)
        duplicate.pose.append(duplicate.pose[0])
        variants.append(duplicate)
        for field in ('position', 'orientation'):
            bad = copy.deepcopy(good)
            delattr(bad.pose[0], field)
            variants.append(bad)
        nan = copy.deepcopy(good)
        nan.pose[0].position.x = float('nan')
        variants.append(nan)
        bad_q = copy.deepcopy(good)
        bad_q.pose[0].orientation.w = .2
        variants.append(bad_q)
        for value in variants:
            with self.subTest(value=value), self.assertRaises(ValueError):
                truth_from_message(value)

    def test_zero_stamp_never_becomes_latest_tf_and_nsec_range_is_checked(self):
        for stamp in (NS(sec=0, nsec=0), NS(sec=1, nsec=10**9), NS(sec=-1, nsec=0)):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                stamp_ns(stamp)

    def test_transform_latest_wrong_frame_and_wrong_child_are_rejected(self):
        for field, value in (('time', 2000000000), ('parent', 'odom'), ('child', 'base_link')):
            msg = transform(1000000000)
            if field == 'time':
                msg = transform(value)
            elif field == 'parent':
                msg.header.frame_id = value
            else:
                msg.child_frame_id = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                transform_at(msg, 1000000000)

    def test_yaw_wrap_and_nonplanar_truth_remain_independent(self):
        def yaw_pose(angle):
            return [0, 0, 0, 0, 0, math.sin(angle/2), math.cos(angle/2)]
        self.assertAlmostEqual(pose_error(yaw_pose(math.pi-.01), yaw_pose(-math.pi+.01))['yaw_rad'], .02)
        msg = truth_message()
        msg.pose[0].orientation.x = math.sin(.2)
        msg.pose[0].orientation.w = math.cos(.2)
        self.assertAlmostEqual(truth_from_message(msg)['pose'][3], math.sin(.2))

    def test_threaded_receipts_have_one_order_and_overflow_is_sticky(self):
        counter = iter(range(100))
        inbox = Inbox(capacity=30, now=lambda: next(counter))
        with ThreadPoolExecutor(max_workers=4) as pool:
            list(pool.map(lambda number: inbox.receive('state', lambda: str(number)), range(30)))
        items, error = inbox.drain()
        self.assertIsNone(error)
        self.assertEqual([at for kind, at, data in items], list(range(30)))
        for _ in range(31):
            inbox.receive('state', lambda: '{}')
        items, error = inbox.drain()
        self.assertEqual(len(items), 30)
        self.assertIn('overflow', error)
        inbox.receive('state', lambda: '{}')
        self.assertEqual(inbox.drain()[0], [])

    def test_callback_error_and_closure_cannot_silently_lose_capture_data(self):
        inbox = Inbox(now=lambda: 1)
        inbox.close_kind('truth')
        inbox.receive('truth', lambda: self.fail('Closed input was evaluated'))
        inbox.receive('state', lambda: (_ for _ in ()).throw(ValueError('bad JSON')))
        self.assertIn('bad JSON', inbox.drain()[1])
        inbox.close()
        inbox.receive('audit', lambda: self.fail('Closed input was evaluated'))


if __name__ == '__main__':
    unittest.main()
