"""Evaluate every captured truth sample against its observed supported scene.

An integrity-checked sampled-clear result is not continuous collision freedom,
authenticated telemetry or physics contact-force evidence.
"""
import hashlib
import json
import math
from pathlib import Path
import re

from orbital.collision_geometry import Scene, Transform, CYLINDER_SIDES, EPS, SUPPORT_PENETRATION_M
from orbital.trajectory import Trajectory, MAX_GAP_NS, TRUTH_TOPIC, integer, pose_error, verify_trace

MAX_EXAMPLES = 100


class ProtocolSink:
    """Reuse the production state/audit validator with original record identities."""
    def append(self, kind, at, value):
        if (kind, at, value) != (self.record['kind'], self.record['received_monotonic_ns'], self.record['data']):
            raise ValueError('Replayed protocol record differs from recorded input')
        return self.record['sequence']


class CollisionEvaluation:
    def __init__(self):
        self.scene = None
        self.phase = 'warmup'
        self.sink = ProtocolSink()
        self.protocol = Trajectory(self.sink)
        self.pending = {}
        self.last_truth = None
        self.signature = None
        self.identity = None
        self.last_matched_ns = None
        self.matched = self.samples = self.checked_pairs = 0
        self.overlap_samples = self.uncertain_samples = self.floor_samples = 0
        self.event_count = 0
        self.examples = []
        self.max_penetration = {}
        self.max_wall_gap = self.max_sim_gap = 0
        self.started = self.cutoff = self.finished = None
        self.last_capture_receipt = None

    def begin(self, summary):
        geometry = summary.get('geometry')
        if (not isinstance(geometry, dict) or type(geometry.get('schema_version')) is not int
                or geometry['schema_version'] != 1 or not isinstance(geometry.get('world_sdf'), str)
                or geometry.get('pose_convention') != 'world model poses; model-relative robot links'
                or summary.get('truth_topic') != TRUTH_TOPIC):
            raise ValueError('Trace lacks the supported full-scene geometry contract')
        self.scene = Scene(geometry['world_sdf'].encode())
        if geometry.get('world_sha256') != self.scene.sha256:
            raise ValueError('World source hash mismatch')
        before, after = summary.get('before'), summary.get('after')
        if not isinstance(before, dict) or before != after or before.get('world_sha256') != self.scene.sha256:
            raise ValueError('World/deployment fences do not agree')
        if not re.fullmatch('[0-9a-f]{64}', before.get('manifest_identity', '')):
            raise ValueError('Missing navigation deployment identity')
        units = before.get('service_invocations', {})
        if (set(units) != {'orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service'}
                or any(not isinstance(v, str) or not re.fullmatch('[0-9a-f]{32}', v) for v in units.values())):
            raise ValueError('Missing service generation fences')
        bindings = summary.get('bindings_before')
        if not isinstance(bindings, dict) or not bindings or bindings != summary.get('bindings_after'):
            raise ValueError('Missing or changed observer binding identities')
        for name in ('gz.transport13', 'gz.transport13._transport', 'gz.msgs10.pose_v_pb2'):
            value = bindings.get(name, {})
            if not isinstance(value, dict) or not re.fullmatch('[0-9a-f]{64}', value.get('sha256', '')):
                raise ValueError('Missing observer binding hash')

    def visit(self, record, summary):
        if self.scene is None:
            self.begin(summary)
        if self.phase == 'ended':
            raise ValueError('Records after the final observation fence')
        at, value, kind = record['received_monotonic_ns'], record.get('data'), record['kind']
        if not isinstance(value, dict):
            raise ValueError('Trace record data must be an object')
        self.sink.record = record
        if kind in ('state', 'audit'):
            getattr(self.protocol, kind)(at, json.dumps(value, allow_nan=False))
        elif kind == 'boundary':
            self.boundary(at, value)
        elif kind == 'truth':
            self.truth(record)
        elif kind == 'sample':
            self.sample(at, value)

    def boundary(self, at, value):
        if value.get('phase') == 'capture_start' and self.phase == 'warmup':
            if (not self.protocol.idle_baseline(at) or self.last_matched_ns is None
                    or not 0 <= at-self.last_matched_ns <= MAX_GAP_NS
                    or value.get('run_id') != self.protocol.client.run_id
                    or value.get('baseline') != self.protocol.baseline):
                raise ValueError('Invalid capture start baseline')
            self.phase, self.started = 'capture', at
            self.protocol.started_ns = at
        elif value.get('phase') == 'capture_end' and self.phase == 'capture':
            self.cutoff = integer(value.get('truth_cutoff_monotonic_ns'), 'truth cutoff', 1)
            self.protocol.now = at
            self.protocol.audit_fence()
            if (self.pending or self.samples < 2 or not self.protocol.client.fresh()
                    or value.get('state_record') != self.protocol.state_ref
                    or not self.started < self.cutoff <= at
                    or self.last_capture_receipt is None
                    or not 0 <= self.cutoff-self.last_capture_receipt <= MAX_GAP_NS):
                raise ValueError('Incomplete capture end fence')
            self.phase, self.finished = 'ended', at
        else:
            raise ValueError('Missing, duplicate or out-of-order capture boundary')

    def truth(self, record):
        value, at = record['data'], record['received_monotonic_ns']
        stamp = integer(value.get('sim_stamp_ns'), 'truth simulation time', 1)
        entity = integer(value.get('entity_id'), 'robot entity', 1)
        measured = self.scene.evaluate(value)  # Includes all parts and scene observations.
        identity = (entity, tuple((n, v['entity_id']) for n, v in sorted(value['links'].items())),
                    tuple((n, v['entity_id']) for n, v in sorted(value['obstacles'].items())),
                    tuple((n, tuple(sorted(v['entity_id'] for v in rows))) for n, rows in sorted(value['static_links'].items())))
        if self.last_truth:
            previous_at, previous_stamp = self.last_truth
            if at < previous_at or stamp <= previous_stamp or identity != self.identity or value['entity_signature'] != self.signature:
                raise ValueError('Truth clock or scene entity generation changed')
            if self.phase == 'capture':
                self.max_wall_gap = max(self.max_wall_gap, at-previous_at)
                self.max_sim_gap = max(self.max_sim_gap, stamp-previous_stamp)
                if max(self.max_wall_gap, self.max_sim_gap) > MAX_GAP_NS:
                    raise ValueError('Truth sampling gap exceeds evaluator coverage bound')
        self.last_truth, self.signature, self.identity = (at, stamp), value['entity_signature'], identity
        if len(self.pending) >= 512:
            raise ValueError('Too many truth samples without matching localization records')
        capture = self.phase == 'capture'
        self.pending[record['sequence']] = {'at': at, 'truth': value, 'capture': capture,
                                            'state_record': self.protocol.state_ref}
        if capture:
            self.protocol.now = at
            if not self.protocol.client.fresh():
                raise ValueError('Stale navigation state at captured truth')
            self.last_capture_receipt = at
            self.checked_pairs += measured['checked_pairs']
            events = measured['events']
            self.overlap_samples += any(e['result'] == 'overlap_or_touch' for e in events)
            self.uncertain_samples += any(e['result'] == 'indeterminate' for e in events)
            self.floor_samples += any(e['result'] in ('support_penetration', 'body_at_or_below_floor', 'outside_floor') for e in events)
            self.event_count += len(events)
            for event in events:
                if len(self.examples) < MAX_EXAMPLES:
                    self.examples.append({'truth_record': record['sequence'], 'sim_stamp_ns': stamp, **event})
            for name, penetration in measured['support_penetration_m'].items():
                self.max_penetration[name] = max(self.max_penetration.get(name, 0.), penetration)

    def sample(self, at, value):
        reference = integer(value.get('truth_record'), 'sample truth reference', 1)
        if not self.pending or reference != next(iter(self.pending)):
            raise ValueError('Missing, duplicate or out-of-order truth/sample correspondence')
        item = self.pending.pop(reference)
        stamp = item['truth']['sim_stamp_ns']
        if (type(value.get('capture')) is not bool or value['capture'] != item['capture']
                or value.get('state_record_at_truth_receipt') != item['state_record']
                or integer(value.get('sim_stamp_ns'), 'sample timestamp', 1) != stamp or at < item['at']):
            raise ValueError('Sample does not match its truth-time observation')
        pose = value.get('localization_pose')
        if pose is None:
            if (item['capture'] or value.get('missing_reason') != 'exact_time_transform_unavailable'
                    or value.get('error') is not None or at-item['at'] < 500000000):
                raise ValueError('Missing localization in capture or invalid warmup timeout')
        else:
            Transform.pose(pose)
            if at-item['at'] >= 500000000 or value.get('missing_reason') is not None:
                raise ValueError('Late localization sample or contradictory missing reason')
            expected, reported = pose_error(item['truth']['pose'], pose), value.get('error')
            if not isinstance(reported, dict) or set(reported) != set(expected) or any(
                    type(reported[k]) not in (int, float) or not math.isfinite(reported[k])
                    or not math.isclose(expected[k], reported[k], rel_tol=1e-9, abs_tol=1e-9) for k in expected):
                raise ValueError('Localization error does not match recorded poses')
            self.last_matched_ns = item['at']
            self.matched += 1
            self.samples += int(item['capture'])

    def finish(self, summary):
        if self.phase != 'ended' or self.pending:
            raise ValueError('Observation window was not closed')
        observed = summary.get('observation')
        expected = {'run_id': self.protocol.client.run_id, 'entity_id': self.identity[0],
                    'baseline': self.protocol.baseline, 'final_audit_sequence': self.protocol.audit_sequence,
                    'final_command_sequence': self.protocol.command_sequence,
                    'final_forwarded_commands': self.protocol.forwarded,
                    'capture_samples': self.samples, 'matched_samples_including_warmup': self.matched,
                    'started_monotonic_ns': self.started, 'truth_cutoff_monotonic_ns': self.cutoff,
                    'finished_monotonic_ns': self.finished,
                    'max_sim_gap_ns': self.max_sim_gap, 'max_wall_gap_ns': self.max_wall_gap}
        if not isinstance(observed, dict) or observed != expected:
            raise ValueError('Recorded summary differs from independently replayed evidence')
        clear = self.overlap_samples == self.uncertain_samples == self.floor_samples == 0
        return {'schema_version': 1, 'evaluation_complete': True, 'sampled_clear': clear,
                'continuous_collision_free': None,
                'scope': 'All captured truth samples against declared primitive geometry at observed poses; not continuous contacts or authenticated loaded geometry',
                'trace_sha256': summary['sha256'], 'world_sha256': self.scene.sha256,
                'navigation_manifest_identity': summary['before']['manifest_identity'],
                'samples': self.samples, 'checked_obstacle_pairs': self.checked_pairs,
                'overlap_or_touch_samples': self.overlap_samples, 'indeterminate_samples': self.uncertain_samples,
                'floor_violation_samples': self.floor_samples, 'max_support_penetration_m': self.max_penetration,
                'support_penetration_tolerance_m': SUPPORT_PENETRATION_M, 'numeric_tolerance_m': EPS,
                'cylinder_prism_sides': CYLINDER_SIDES, 'event_count': self.event_count,
                'event_examples': self.examples, 'examples_truncated': self.event_count > len(self.examples),
                'observation': expected}


def evaluate_trace(path):
    evaluator = CollisionEvaluation()
    summary = verify_trace(path, evaluator.visit)
    result = evaluator.finish(summary)
    root = Path(__file__).resolve().parent
    result['evaluator_source_sha256'] = {name: hashlib.sha256((root/name).read_bytes()).hexdigest()
                                       for name in ('collision_geometry.py', 'collision_evaluation.py',
                                                    'trajectory.py', 'navigation_client.py', 'navigation_command.py')}
    return result
