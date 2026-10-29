"""Read-only evaluator trace: simulator truth never enters a control interface.

Input receipt timestamps and record write order remain distinct from source
simulation timestamps and audit counters. None authenticates a publisher.
"""
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import re
import threading
import time

from orbital.navigation_client import NavigationClient, _json, _target
from orbital.navigation_command import CommandGate
from orbital.machines import MACHINE_IDS

MAX_BYTES = 64 * 1024 * 1024
MAX_RECORDS = 100000
MAX_LINE = 65536
MAX_GAP_NS = 250000000
TRUTH_TOPIC = '/world/maintenance/pose/info'


def integer(value, name, minimum=0):
    if type(value) is not int or not minimum <= value < 2**63:
        raise ValueError('Invalid '+name)
    return value


def stamp_ns(stamp):
    sec = integer(stamp.sec, 'seconds')
    nsec = integer(stamp.nanosec if hasattr(stamp, 'nanosec') else stamp.nsec, 'nanoseconds')
    if nsec >= 1000000000:
        raise ValueError('Nanoseconds outside clock range')
    return integer(sec * 1000000000 + nsec, 'simulation stamp', 1)


def pose_values(position, orientation):
    values = [position.x, position.y, position.z, orientation.x,
              orientation.y, orientation.z, orientation.w]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Nonfinite pose')
    if abs(sum(v*v for v in values[3:])-1.) > .001:
        raise ValueError('Pose quaternion is not normalized')
    return values


def truth_from_message(message, scene=None):
    if message.ByteSize() > 1024*1024:
        raise ValueError('Truth message exceeds 1 MiB')
    if not message.HasField('header') or not message.header.HasField('stamp'):
        raise ValueError('Truth has no simulation timestamp')
    matches = [p for p in message.pose if p.name == 'maintenance_robot']
    if len(matches) != 1:
        raise ValueError('Expected one exact maintenance_robot model pose')
    pose = matches[0]
    if not pose.HasField('position') or not pose.HasField('orientation'):
        raise ValueError('Truth pose fields missing')
    value = {'sim_stamp_ns': stamp_ns(message.header.stamp),
             'entity_id': integer(pose.id, 'entity id', 1),
             'pose': pose_values(pose.position, pose.orientation)}
    if scene is not None:
        value.update(scene.observe(message))
    return value


def transform_at(message, requested_ns):
    if (message.header.frame_id != 'map' or message.child_frame_id != 'base_footprint'
            or stamp_ns(message.header.stamp) != requested_ns):
        raise ValueError('Localization transform is not at the requested truth time/frame')
    return pose_values(message.transform.translation, message.transform.rotation)


def pose_error(truth, estimate):
    def yaw(p):
        x, y, z, w = p[3:]
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return {'xy_m': math.hypot(truth[0]-estimate[0], truth[1]-estimate[1]),
            'yaw_rad': abs(math.remainder(yaw(truth)-yaw(estimate), 2*math.pi))}


class Inbox:
    """Bound callbacks from Gazebo's threads; errors cannot disappear in its wrapper."""
    def __init__(self, *, capacity=2048, now=time.monotonic_ns):
        self.capacity, self.now = capacity, now
        self.items = deque()
        self.lock = threading.Lock()
        self.error = None
        self.closed = False
        self.closed_kinds = set()

    def receive(self, kind, convert):
        with self.lock:
            if self.closed or self.error or kind in self.closed_kinds:
                return
            try:
                if len(self.items) >= self.capacity:
                    raise ValueError('Observer queue overflow')
                self.items.append((kind, self.now(), convert()))
            except Exception as error:
                self.error = f'{type(error).__name__}: {error}'

    def drain(self):
        with self.lock:
            result = list(self.items)
            self.items.clear()
            return result, self.error

    def close(self):
        with self.lock:
            self.closed = True

    def close_kind(self, kind):
        with self.lock:
            self.closed_kinds.add(kind)
            return self.now()


class TraceWriter:
    """Exclusive local output with bounded JSONL and a final integrity manifest."""
    def __init__(self, root, name, *, max_bytes=MAX_BYTES, max_records=MAX_RECORDS):
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', name):
            raise ValueError('Use one simple run name, not a path')
        root = Path(root).resolve()
        parent = root
        for component in ('runtime', 'evaluation'):
            parent = parent/component
            if parent.is_symlink():
                raise ValueError('Evaluation output parent cannot be a symlink')
            parent.mkdir(exist_ok=True, mode=0o700)
        self.path = parent/name
        self.path.mkdir(mode=0o700)  # Never reuse an old run, including a symlink.
        self.stream = (self.path/'events.jsonl').open('xb')
        os.chmod(self.path/'events.jsonl', 0o600)
        self.max_bytes, self.max_records = max_bytes, max_records
        self.count, self.size = 0, 0
        self.hash = hashlib.sha256()
        self.closed = False

    def append(self, kind, received_ns, data):
        record = {'sequence': self.count+1, 'kind': kind,
                  'received_monotonic_ns': integer(received_ns, 'receipt time'), 'data': data}
        raw = (json.dumps(record, separators=(',', ':'), allow_nan=False)+'\n').encode()
        if len(raw) > MAX_LINE or self.size+len(raw) > self.max_bytes or self.count >= self.max_records:
            raise ValueError('Trace output bound reached')
        self.stream.write(raw)
        self.hash.update(raw)
        self.count += 1
        self.size += len(raw)
        return self.count

    def sync(self):
        self.stream.flush()
        os.fsync(self.stream.fileno())

    def marker(self, name, value):
        # The containing directory is new and private. Exclusive creation means
        # an interrupted/partial marker can never be mistaken for a later run.
        with (self.path/name).open('x') as stream:
            os.chmod(self.path/name, 0o600)
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())

    def finish(self, complete, details):
        if self.closed:
            raise ValueError('Trace already finalized')
        self.sync()
        self.stream.close()
        self.closed = True
        result = {**details, 'schema_version': 1, 'complete': complete, 'records': self.count,
                  'bytes': self.size, 'sha256': self.hash.hexdigest()}
        self.marker('summary.json', result)
        return result


def verify_trace(path, visit=None):
    """Check the exact consumed bytes; a visitor must defer results until return.

    Integrity is not authenticity or mission acceptance. A visitor sees records
    before the final hash check and must not publish an accepted result early.
    """
    path = Path(path)
    for name in ('summary.json', 'events.jsonl'):
        if (path/name).is_symlink():
            raise ValueError('Trace files cannot be symlinks')
    with (path/'summary.json').open() as stream:
        summary = _json(stream.read(MAX_LINE+1), MAX_LINE)
    if type(summary.get('schema_version')) is not int or summary['schema_version'] != 1 or summary.get('complete') is not True:
        raise ValueError('Trace is incomplete or has an unsupported schema')
    integer(summary.get('records'), 'manifest record count', 1)
    integer(summary.get('bytes'), 'manifest byte count', 1)
    count, size, digest = 0, 0, hashlib.sha256()
    with (path/'events.jsonl').open('rb') as stream:
        while raw := stream.readline(MAX_LINE+1):
            size += len(raw)
            count += 1
            if size > MAX_BYTES or count > MAX_RECORDS or len(raw) > MAX_LINE or not raw.endswith(b'\n'):
                raise ValueError('Trace file exceeds bounds or has a partial record')
            record = _json(raw.decode(), MAX_LINE)
            if integer(record.get('sequence'), 'record sequence', 1) != count:
                raise ValueError('Trace record sequence gap')
            integer(record.get('received_monotonic_ns'), 'receipt time')
            if record.get('kind') not in ('state', 'audit', 'truth', 'sample', 'boundary'):
                raise ValueError('Unknown trace record')
            digest.update(raw)
            if visit is not None:
                visit(record, summary)
    if not count or (count, size, digest.hexdigest()) != (summary.get('records'), summary.get('bytes'), summary.get('sha256')):
        raise ValueError('Trace integrity mismatch')
    return summary


class Trajectory:
    def __init__(self, writer):
        self.writer = writer
        self.now = 0
        self.client = NavigationClient(lambda: self.now/1e9)
        self.state_ref = None
        self.baseline = None
        self.audit_sequence = self.command_sequence = self.forwarded = None
        self.audit_stamp_ns = None
        self.last_state = None
        self.last_truth = None
        self.pending = deque()
        self.started_ns = None
        self.matched = 0
        self.capture_samples = 0
        self.max_sim_gap_ns = self.max_wall_gap_ns = 0
        self.last_matched_ns = None

    def state(self, at, text):
        self.now = at
        self.client.observe(text)
        if self.client.failure:
            raise ValueError(self.client.failure)
        state = self.client.state
        for key in ('audit_sequence', 'forwarded_commands', 'sim_stamp_ns'):
            integer(state.get(key), key)
            if self.last_state and state[key] < self.last_state[key]:
                raise ValueError('Supervisor state counter/time regressed')
        if state.get('audit_ok') is not True:
            raise ValueError('Supervisor audit publication failed or is unavailable')
        self.last_state = state
        self.state_ref = self.writer.append('state', at, state)
        if self.baseline is None:
            self.audit_sequence = state['audit_sequence']
            self.command_sequence = state['command_sequence']
            self.forwarded = state['forwarded_commands']
            self.audit_stamp_ns = state['sim_stamp_ns']
            self.baseline = {'audit_sequence': self.audit_sequence, 'command_sequence': self.command_sequence,
                             'forwarded_commands': self.forwarded, 'state_record': self.state_ref}

    def audit(self, at, text):
        event = _json(text)
        seq = integer(event.get('sequence'), 'audit sequence', 1)
        stamp = integer(event.get('sim_stamp_ns'), 'audit timestamp')
        if (type(event.get('schema_version')) is not int or event['schema_version'] != 1
                or event.get('event') not in ('command_reply', 'velocity_forwarded')):
            raise ValueError('Unknown audit schema/event')
        self.writer.append('audit', at, event)
        if self.baseline is None or (self.started_ns is None and seq <= self.baseline['audit_sequence']):
            return  # Explicit warmup prefix, outside the declared counter window.
        if event.get('run_id') != self.client.run_id or seq != self.audit_sequence+1:
            raise ValueError('Audit restart, duplicate or sequence gap')
        if stamp < self.audit_stamp_ns:
            raise ValueError('Audit simulation time regressed')
        target = event.get('target')
        if target is not None and _target(target)['run_id'] != self.client.run_id:
            raise ValueError('Audit target belongs to another run')
        if event['event'] == 'command_reply':
            command, reply = event.get('command'), event.get('reply')
            if not isinstance(reply, dict) or type(reply.get('accepted')) is not bool:
                raise ValueError('Invalid command audit reply')
            if command is not None:
                gate = CommandGate(self.client.run_id, MACHINE_IDS)
                gate.sequence = self.command_sequence
                gate.decode(json.dumps(command, allow_nan=False))
                self.command_sequence = gate.sequence
                if integer(reply.get('sequence'), 'reply sequence', 1) != gate.sequence or reply.get('op') != command['op']:
                    raise ValueError('Audit reply does not match command')
            elif reply['accepted']:
                raise ValueError('Accepted audit command is missing')
        else:
            if target is None:
                raise ValueError('Forwarded velocity has no exact target')
            integer(event.get('velocity_stamp_ns'), 'velocity stamp', 1)
            velocity = event.get('velocity')
            if (event.get('frame_id') != 'base_link' or not isinstance(velocity, list) or len(velocity) != 6
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in velocity)):
                raise ValueError('Invalid forwarded velocity')
            self.forwarded += 1
        if (integer(event.get('command_sequence'), 'command sequence') != self.command_sequence
                or integer(event.get('forwarded_commands'), 'forwarded counter') != self.forwarded):
            raise ValueError('Audit command/velocity counter gap')
        self.audit_sequence = seq
        self.audit_stamp_ns = stamp

    def truth(self, at, value):
        if self.last_truth:
            old_at, old = self.last_truth
            if value['entity_id'] != old['entity_id'] or value['sim_stamp_ns'] <= old['sim_stamp_ns']:
                raise ValueError('Truth entity changed or simulation time did not advance')
            if value.get('entity_signature') != old.get('entity_signature'):
                raise ValueError('Scene entity generation changed')
            if self.started_ns is not None:
                self.max_sim_gap_ns = max(self.max_sim_gap_ns, value['sim_stamp_ns']-old['sim_stamp_ns'])
                self.max_wall_gap_ns = max(self.max_wall_gap_ns, at-old_at)
                if max(self.max_sim_gap_ns, self.max_wall_gap_ns) > MAX_GAP_NS:
                    raise ValueError('Truth sampling gap exceeds 250 ms')
        if self.started_ns is not None:
            self.now = at
            if not self.client.fresh():
                raise ValueError('No fresh navigation state at truth receipt')
        if len(self.pending) >= 512:
            raise ValueError('Unmatched truth queue overflow')
        ref = self.writer.append('truth', at, value)
        self.pending.append({'truth_record': ref, 'truth': value, 'received_ns': at,
                             'state_record': self.state_ref, 'capture': self.started_ns is not None})
        self.last_truth = at, value

    def match(self, lookup, now):
        while self.pending:
            item = self.pending[0]
            stamp = item['truth']['sim_stamp_ns']
            if now-item['received_ns'] >= 500000000:
                pose = None
            else:
                try:
                    pose = lookup(stamp)
                except LookupError:
                    break
            self.pending.popleft()
            self.writer.append('sample', now, {
                'truth_record': item['truth_record'], 'state_record_at_truth_receipt': item['state_record'],
                'sim_stamp_ns': stamp, 'capture': item['capture'], 'localization_pose': pose,
                'error': pose_error(item['truth']['pose'], pose) if pose is not None else None,
                'missing_reason': 'exact_time_transform_unavailable' if pose is None else None})
            if pose is None and item['capture']:
                raise ValueError('Localization missing at exact truth timestamp')
            if pose is not None:
                self.matched += 1
                self.last_matched_ns = item['received_ns']
                self.capture_samples += int(item['capture'])

    def idle_baseline(self, now):
        self.now = now
        state = self.client.state
        if (not self.client.fresh() or not state['healthy'] or state['action_state'] != 'idle'
                or state['guard_latched'] is not True):
            return False
        if (state['audit_sequence'] != self.baseline['audit_sequence']
                or self.audit_sequence != self.baseline['audit_sequence']
                or state['command_sequence'] != self.baseline['command_sequence']
                or state['forwarded_commands'] != self.baseline['forwarded_commands']):
            raise ValueError('Command activity during recorder warmup; retry while idle/latched')
        return True

    def start(self, now):
        if (not self.idle_baseline(now) or self.last_matched_ns is None
                or now-self.last_matched_ns > MAX_GAP_NS):
            return False
        self.started_ns = now
        boundary = {'phase': 'capture_start', 'run_id': self.client.run_id, 'baseline': self.baseline}
        self.writer.append('boundary', now, boundary)
        self.writer.sync()
        self.writer.marker('ready.json', {'schema_version': 1, 'started_monotonic_ns': now, **boundary})
        return True

    def finish(self, now, *, ended_ns=None):
        self.now = now
        ended_ns = now if ended_ns is None else ended_ns
        if (self.started_ns is None or self.capture_samples < 2 or self.pending or not self.client.fresh()
                or self.last_truth is None or ended_ns-self.last_truth[0] > MAX_GAP_NS):
            raise ValueError('Incomplete observation window')
        self.audit_fence()
        self.writer.append('boundary', now, {'phase': 'capture_end', 'state_record': self.state_ref,
                                            'truth_cutoff_monotonic_ns': ended_ns})
        return {'run_id': self.client.run_id, 'entity_id': self.last_truth[1]['entity_id'],
                'baseline': self.baseline, 'final_audit_sequence': self.audit_sequence,
                'final_command_sequence': self.command_sequence, 'final_forwarded_commands': self.forwarded,
                'capture_samples': self.capture_samples, 'matched_samples_including_warmup': self.matched,
                'started_monotonic_ns': self.started_ns, 'truth_cutoff_monotonic_ns': ended_ns,
                'finished_monotonic_ns': now,
                'max_sim_gap_ns': self.max_sim_gap_ns, 'max_wall_gap_ns': self.max_wall_gap_ns}

    def audit_fence(self):
        state = self.client.state
        if state is None or (state['audit_sequence'], state['command_sequence'], state['forwarded_commands']) != (
                self.audit_sequence, self.command_sequence, self.forwarded):
            raise ValueError('Unobserved audit tail or state has not confirmed the audit tail')
