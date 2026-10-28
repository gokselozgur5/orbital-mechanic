"""Observe the real station/owner, then replay and corrupt retained source packets."""
import copy
import hashlib
import json
import unittest

from orbital.inspection_protocol import canonical
from orbital.isolation_trace import IsolationTrace, verify_isolation
from orbital.trajectory import TraceWriter
import test_maintenance_owner as fixture


def sources(registry):
    peers = [(topic, 'station_telemetry', '/', 'std_msgs/msg/String', '09'*24) for topic in (
        '/maintenance/state', '/maintenance/result', '/station/telemetry')]
    peers.append(('/clock', 'ros_gz_bridge', '/', 'rosgraph_msgs/msg/Clock', '08'*24))
    return {'runtime': {'manifest_identity': 'a'*64,
        'world_sha256': registry.public()['source_sha256']['world'],
        'service_invocations': {unit:'b'*32 for unit in (
            'orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service')}},
        'publishers': sorted(peers)}


class IsolationTraceTests(unittest.TestCase):
    def setUp(self):
        self.f = fixture.MaintenanceOwnerTests()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        self.root, self.clock, self.owner = self.f.root, self.f.clock, self.f.owner
        self.writer = TraceWriter(self.root, 'isolation')
        self.addCleanup(lambda:self.writer.stream.close() if not self.writer.closed else None)
        self.identity = {k:self.f.request[k] for k in ('run_id', 'mission_id', 'action_id')}
        self.identity['machine_id'] = 'pump'
        self.trace = IsolationTrace(self.f.model.registry, self.identity, writer=self.writer)
        self.sources = sources(self.f.model.registry)
        self.trace.begin(self.at(), self.clock.sim, self.sources)
        for _ in range(2):
            self.sample(.25)
            self.receive('/maintenance/state', self.owner.state())
        self.assertTrue(self.trace.ready(self.at(), self.clock.sim))

    def at(self): return round(self.clock.wall*1e9)

    def receive(self, topic, value):
        self.trace.receive(topic, self.at(), self.clock.sim, canonical(value))

    def sample(self, delta=.25):
        packet = self.f.sample(delta)
        self.receive('/station/telemetry', packet)
        return packet

    def execute(self, mode='success', *, deliver=True):
        self.receive('/maintenance/command', self.f.request)
        if mode == 'cancel_before':
            self.f.send(); self.f.send('cancel'); self.owner.tick()
            result = self.f.finish()
            if deliver: self.receive('/maintenance/result', result)
            self.sample(); self.sample()
        else:
            self.f.apply()
            if mode == 'cancel_after': self.f.send('cancel'); self.owner.tick()
            self.sample(); self.sample()
            result = self.f.finish()
            if deliver: self.receive('/maintenance/result', result)
        return result

    def finish(self):
        value = self.trace.end(self.at(), self.clock.sim, self.sources)
        self.writer.finish(True, {'observation': value})
        return value

    def rewrite(self, edit):
        path = self.writer.path/'events.jsonl'
        values = [json.loads(line) for line in path.read_text().splitlines()]
        values = edit(values)
        for index, value in enumerate(values, 1): value['sequence'] = index
        raw = ''.join(canonical(value)+'\n' for value in values).encode()
        path.write_bytes(raw)
        summary_path = self.writer.path/'summary.json'
        summary = json.loads(summary_path.read_text())
        summary.update(records=len(values), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())
        summary_path.write_text(canonical(summary))

    def test_actual_station_effect_replays_from_retained_source_bytes(self):
        result = self.execute()
        value = self.finish()
        verified = verify_isolation(self.root, self.writer.path)
        self.assertEqual(verified['observation'], value)
        self.assertTrue(value['verified_isolation'])
        self.assertEqual(value['server_status'], 'succeeded')
        self.assertEqual(result['state_change']['effect'], self.f.model.last_applied)
        self.assertEqual(self.f.model.control_revision, 1)

    def test_cancel_after_effect_preserves_verified_isolation(self):
        self.execute('cancel_after'); value = self.finish()
        self.assertEqual(value['server_status'], 'cancelled')
        self.assertTrue(verify_isolation(self.root, self.writer.path)['observation']['verified_isolation'])

    def test_cancel_before_effect_reports_only_sampled_unchanged(self):
        self.execute('cancel_before'); value = self.finish()
        self.assertFalse(value['verified_isolation'])
        self.assertTrue(value['sampled_unchanged'])
        self.assertEqual(value['server_status'], 'cancelled')
        self.assertEqual(verify_isolation(self.root, self.writer.path)['observation'], value)

    def test_result_before_its_reference_packet_is_bounded_not_an_implied_failure(self):
        self.receive('/maintenance/command', self.f.request)
        self.f.apply(); self.sample()
        packet = self.f.sample(.25)
        result = self.f.finish()
        self.receive('/maintenance/result', result)
        self.assertIsNone(self.trace.assess(self.at(), self.clock.sim))
        self.clock.advance(.01)
        self.receive('/station/telemetry', packet)
        self.finish()
        self.assertTrue(verify_isolation(self.root, self.writer.path)['observation']['verified_isolation'])

    def test_missing_command_cannot_be_replaced_by_a_success_result(self):
        self.execute(); self.finish()
        self.rewrite(lambda values:[v for v in values if v['data'].get('topic') != '/maintenance/command'])
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            verify_isolation(self.root, self.writer.path)

    def test_missing_post_sample_refuses_even_with_recomputed_trace_hash(self):
        self.execute(); self.finish()
        def edit(values):
            post = [v for v in values if v['data'].get('topic') == '/station/telemetry'
                    and json.loads(v['data']['text'])['control']['revision'] == 1]
            values.remove(post[0]); return values
        self.rewrite(edit)
        with self.assertRaisesRegex(ValueError, 'gap'):
            verify_isolation(self.root, self.writer.path)

    def test_server_success_with_powered_source_packets_is_not_isolation(self):
        self.execute(); self.finish()
        def edit(values):
            for v in values:
                if v['data'].get('topic') == '/station/telemetry':
                    packet = json.loads(v['data']['text'])
                    if packet['control']['revision'] == 1:
                        packet['machines']['pump']['powered'] = True
                        v['data']['text'] = canonical(packet)
            return values
        self.rewrite(edit)
        with self.assertRaisesRegex(ValueError, 'reference|not_observed'):
            verify_isolation(self.root, self.writer.path)

    def test_wrong_applied_action_cannot_be_hidden_by_rehashing(self):
        self.execute(); self.finish()
        def edit(values):
            for v in values:
                if v['data'].get('topic') == '/station/telemetry':
                    packet = json.loads(v['data']['text'])
                    if packet['control']['revision']:
                        packet['control']['last_applied']['action_id'] = 'f'*32
                        v['data']['text'] = canonical(packet)
            return values
        self.rewrite(edit)
        with self.assertRaisesRegex(ValueError, 'reference'):
            verify_isolation(self.root, self.writer.path)

    def test_result_for_another_request_does_not_verify(self):
        result = self.execute(deliver=False)
        result['request_sha256'] = result['state_change']['effect']['request_sha256'] = 'e'*64
        self.receive('/maintenance/result', result)
        with self.assertRaisesRegex(ValueError, 'result_binding'):
            self.trace.assess(self.at(), self.clock.sim)

    def test_forged_success_digest_cannot_override_source_power_measurement(self):
        self.receive('/maintenance/command', self.f.request)
        self.f.apply()
        for _ in range(2):
            packet = self.f.sample(.25)
            packet['machines']['pump']['powered'] = True
            self.receive('/station/telemetry', packet)
        result = self.f.finish()
        result['state_change']['verification']['sample_sha256'] = hashlib.sha256(canonical(packet).encode()).hexdigest()
        self.receive('/maintenance/result', result)
        with self.assertRaisesRegex(ValueError, 'reference|not_observed'):
            self.trace.assess(self.at(), self.clock.sim)

    def test_source_run_change_and_sequence_replay_fail(self):
        packet = self.f.sample(.25)
        changed = copy.deepcopy(packet); changed['run_id'] = 'f'*32
        with self.assertRaisesRegex(ValueError, 'run_changed'):
            self.receive('/station/telemetry', changed)
        # Reader failure remains sticky even after a later valid packet.
        with self.assertRaisesRegex(ValueError, 'run_changed'):
            self.receive('/station/telemetry', packet)

    def test_an_extra_control_revision_cannot_be_reported_as_one_isolation(self):
        self.receive('/maintenance/command', self.f.request)
        self.f.apply()
        packet = self.f.sample(.25)
        packet['control']['revision'] = packet['control']['last_applied']['control_revision'] = 2
        with self.assertRaisesRegex(ValueError, 'unexpected_control_revision'):
            self.receive('/station/telemetry', packet)

    def test_receipt_regression_and_frozen_telemetry_are_refused(self):
        packet = self.trace.baseline
        with self.assertRaisesRegex(ValueError, 'clock_regressed'):
            self.trace.receive('/station/telemetry', self.at()-1, self.clock.sim, canonical(packet))
        with self.assertRaisesRegex(ValueError, 'replayed'):
            self.receive('/station/telemetry', packet)

    def test_expired_postcondition_never_becomes_complete(self):
        self.execute(); self.clock.advance(1.01)
        self.assertIsNone(self.trace.assess(self.at(), self.clock.sim))
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            self.finish()

    def test_changed_runtime_refuses_historical_replay(self):
        self.execute(); self.finish()
        def edit(values):
            values[-1]['data']['source']['runtime']['manifest_identity'] = 'c'*64
            return values
        self.rewrite(edit)
        with self.assertRaisesRegex(ValueError, 'source_changed'):
            verify_isolation(self.root, self.writer.path)

    def test_runtime_pin_does_not_alias_the_callers_mutable_snapshot(self):
        self.execute()
        self.sources['runtime']['manifest_identity'] = 'c'*64
        with self.assertRaisesRegex(ValueError, 'source_changed'):
            self.finish()

    def test_offline_replay_requires_the_same_registry_bytes(self):
        self.execute(); self.finish()
        path = self.root/'worlds/maintenance.sdf'
        path.write_bytes(path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError, 'header'):
            verify_isolation(self.root, self.writer.path)

    def test_invalid_record_structure_is_a_bounded_refusal(self):
        self.execute(); self.finish()
        def edit(values):
            values[0]['data']['source']['publishers'][0][0] = []
            return values
        self.rewrite(edit)
        with self.assertRaisesRegex(ValueError, 'structure'):
            verify_isolation(self.root, self.writer.path)

    def test_integrity_and_manifest_result_are_checked_after_replay(self):
        self.execute(); self.finish()
        path = self.writer.path/'summary.json'
        original = json.loads(path.read_text())
        for field, value in (('sha256', 'f'*64), ('complete', False)):
            summary = copy.deepcopy(original); summary[field] = value
            path.write_text(canonical(summary))
            with self.assertRaises(ValueError): verify_isolation(self.root, self.writer.path)
        summary = copy.deepcopy(original); summary['observation']['verified_isolation'] = False
        path.write_text(canonical(summary))
        with self.assertRaisesRegex(ValueError, 'final_observation'):
            verify_isolation(self.root, self.writer.path)

    def test_late_start_and_conflicting_exact_identity_are_refused(self):
        other = IsolationTrace(self.f.model.registry, self.identity)
        other.begin(self.at(), self.clock.sim, self.sources)
        with self.assertRaisesRegex(ValueError, 'before_ready'):
            other.receive('/maintenance/command', self.at(), self.clock.sim, canonical(self.f.request))
        self.receive('/maintenance/command', self.f.request)
        changed = copy.deepcopy(self.f.request); changed['arguments']['timeout_s'] = 6.
        with self.assertRaisesRegex(ValueError, 'conflicting_command'):
            self.receive('/maintenance/command', changed)

    def test_matching_replay_does_not_repeat_an_effect(self):
        self.execute()
        changed = copy.deepcopy(self.f.request); changed['command_id'] = 'e'*32
        self.receive('/maintenance/command', changed)
        self.assertEqual(self.f.model.control_revision, 1)
        self.finish()
        self.assertTrue(verify_isolation(self.root, self.writer.path)['observation']['verified_isolation'])
