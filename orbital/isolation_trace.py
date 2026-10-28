"""Independent replay of retained public isolation observations.

This checks a sampled model postcondition. It never grants action permission,
re-evaluates camera/pose admission, or certifies electrical isolation.
"""
from collections import deque
import copy
import hashlib

from orbital import maintenance_protocol as protocol
from orbital.inspection_protocol import canonical, decode, identifier
from orbital.machine_registry import MachineRegistry
from orbital.machines import MACHINE_IDS
from orbital.proximity import Unavailable
from orbital.telemetry_observation import TelemetryEvidence, decode as telemetry_decode
from orbital.trajectory import integer, verify_trace

TOPICS = ('/maintenance/command', '/maintenance/state', '/maintenance/result', '/station/telemetry')
SCOPE = ('Sampled public model isolation; not continuous health, camera/pose admission, '
         'independent hold-release proof, publisher authentication or electrical isolation')


def target(value):
    if not isinstance(value, dict) or set(value) != {'run_id', 'mission_id', 'action_id', 'machine_id'}:
        raise ValueError('invalid_trace_target')
    for name in ('run_id', 'mission_id', 'action_id'):
        identifier(value[name])
    if value['machine_id'] not in MACHINE_IDS:
        raise ValueError('invalid_trace_target')
    return dict(value)


def source(value, registry):
    if not isinstance(value, dict) or set(value) != {'runtime', 'publishers'}:
        raise ValueError('invalid_trace_source')
    runtime = value['runtime']
    if (not isinstance(runtime, dict)
            or set(runtime) != {'manifest_identity', 'world_sha256', 'service_invocations'}
            or runtime['world_sha256'] != registry.public()['source_sha256']['world']):
        raise ValueError('invalid_trace_runtime')
    identifier(runtime['manifest_identity'], 64)
    services = runtime['service_invocations']
    if not isinstance(services, dict) or set(services) != {
            'orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service'}:
        raise ValueError('invalid_trace_services')
    for invocation in services.values():
        identifier(invocation)
    expected = {name:('station_telemetry', 'std_msgs/msg/String') for name in TOPICS[1:]}
    expected['/clock'] = ('ros_gz_bridge', 'rosgraph_msgs/msg/Clock')
    peers = value['publishers']
    if not isinstance(peers, list) or len(peers) != len(expected):
        raise ValueError('invalid_trace_publishers')
    seen = set()
    for peer in peers:
        if not isinstance(peer, (list, tuple)) or len(peer) != 5:
            raise ValueError('invalid_trace_publishers')
        topic, owner, namespace, kind, gid = peer
        if (topic not in expected or topic in seen or (owner, kind) != expected[topic]
                or namespace != '/' or not isinstance(gid, str) or not 2 <= len(gid) <= 256
                or len(gid) % 2 or any(c not in '0123456789abcdef' for c in gid) or int(gid, 16) == 0):
            raise ValueError('invalid_trace_publishers')
        seen.add(topic)
    return value


class IsolationTrace:
    def __init__(self, registry, identity, *, writer=None):
        self.registry, self.target, self.writer = registry, target(identity), writer
        self.reader = TelemetryEvidence(registry)
        self.status = self.status_at = self.last_wall = self.last_sim = None
        self.baseline = self.ready_at = self.request = self.terminal = self.terminal_at = None
        self.samples = deque(maxlen=8)
        self.source = None

    def clocks(self, at, sim):
        integer(at, 'receipt time'); integer(sim, 'simulation clock', 1)
        if ((self.last_wall is not None and at < self.last_wall)
                or (self.last_sim is not None and sim < self.last_sim)):
            raise ValueError('trace_clock_regressed')
        self.last_wall, self.last_sim = at, sim

    def write(self, kind, at, data):
        if self.writer is not None:
            self.writer.append(kind, at, data)

    def begin(self, at, sim, sources):
        if self.source is not None:
            raise ValueError('trace_already_started')
        self.clocks(at, sim)
        self.source = copy.deepcopy(source(sources, self.registry))
        self.write('boundary', at, {'event': 'begin', 'sim_stamp_ns': sim,
            'schema_version': 1, 'kind': 'isolation_observation', 'target': self.target,
            'registry_identity': self.registry.identity, 'source': self.source})

    def matches(self, value):
        return all(value.get(k) == self.target[k] for k in ('run_id', 'mission_id', 'action_id'))

    def receive(self, topic, at, sim, text):
        if self.source is None or topic not in TOPICS:
            raise ValueError('invalid_trace_message')
        self.clocks(at, sim)
        if topic == '/station/telemetry':
            packet = telemetry_decode(text, self.registry)
            if packet['schema_version'] != 2:
                raise ValueError('trace_requires_control_telemetry')
            if not self.reader.observe(text, at/1e9, sim):
                raise ValueError(self.reader.failure)
            if self.samples:
                old, current = self.samples[-1][0]['control'], packet['control']
                if (current['revision'] < old['revision']
                        or (current['revision'] == old['revision'] and current != old)):
                    raise ValueError('trace_control_history_changed')
            if self.ready_at is not None and self.reader.missed:
                raise ValueError('trace_telemetry_gap')
            if self.baseline is not None and not self.baseline['control']['revision'] <= packet['control']['revision'] <= self.baseline['control']['revision']+1:
                raise ValueError('trace_unexpected_control_revision')
            self.samples.append((packet, at))
        elif topic == '/maintenance/state':
            value = protocol.state(text)
            if value['run_id'] != self.target['run_id']:
                raise ValueError('trace_owner_run_changed')
            if not 0 <= sim-value['sim_stamp_ns'] < 500000000:
                raise ValueError('trace_stale_state')
            if self.status is not None:
                old = self.status
                if (value['station_run_id'] != old['station_run_id'] or value['sequence'] <= old['sequence']
                        or value['sim_stamp_ns'] < old['sim_stamp_ns']
                        or value['control_revision'] < old['control_revision']):
                    raise ValueError('trace_state_replayed_or_changed')
            if self.ready_at is not None and value['active'] is not None and not self.matches(value['active']):
                raise ValueError('trace_other_action')
            if self.status is None or value['sim_stamp_ns'] > self.status['sim_stamp_ns']:
                self.status_at = at  # Repeated frozen timestamps never renew readiness.
            self.status = value
        elif topic == '/maintenance/command':
            value = protocol.command(text)
            if self.matches(value) and value['op'] == 'isolate':
                if self.ready_at is None:
                    raise ValueError('trace_command_before_ready')
                args = value['arguments']
                if (args['machine_id'] != self.target['machine_id']
                        or args['station_run_id'] != self.baseline['run_id']
                        or args['control_revision'] != self.baseline['control']['revision']):
                    raise ValueError('trace_command_binding')
                if self.request is not None and protocol.request_digest(value) != protocol.request_digest(self.request):
                    raise ValueError('trace_conflicting_command')
                self.request = value
        else:
            value = protocol.result(decode(text, 16384))
            if self.matches(value):
                if self.ready_at is None:
                    raise ValueError('trace_result_before_ready')
                if self.terminal is not None and value != self.terminal:
                    raise ValueError('trace_conflicting_result')
                if self.terminal is None:
                    self.terminal, self.terminal_at = value, at
        # Retain exact validated wire text, not just the server's sample digest.
        self.write('sample', at, {'topic': topic, 'sim_stamp_ns': sim, 'text': text})

    def ready(self, at, sim):
        self.clocks(at, sim)
        if self.ready_at is not None:
            return True
        if (self.status is None or not self.status['ready'] or self.status['active'] is not None
                or at-self.status_at >= 500000000 or not 0 <= sim-self.status['sim_stamp_ns'] < 500000000):
            return False
        try:
            packet = self.reader.measure(at/1e9, sim)['telemetry']
        except Unavailable:
            return False
        if (self.reader.missed or packet['run_id'] != self.status['station_run_id']
                or packet['control']['revision'] != self.status['control_revision']
                or packet['machines'][self.target['machine_id']]['powered'] is not True
                or (self.status['last_result'] is not None and self.matches(self.status['last_result']))):
            raise ValueError('trace_baseline_unavailable')
        self.baseline, self.ready_at = packet, at
        self.write('boundary', at, {'event': 'ready', 'sim_stamp_ns': sim})
        return True

    def assess(self, at, sim):
        self.clocks(at, sim)
        if self.request is None or self.terminal is None:
            return None
        terminal = self.terminal
        digest = protocol.request_digest(self.request)
        if (terminal['operation'] != 'isolate' or terminal['request_sha256'] != digest
                or terminal['machine_id'] != self.target['machine_id']
                or terminal['station_run_id'] != self.baseline['run_id']
                or not self.baseline['stamp_ns'] <= terminal['started_sim_stamp_ns'] <= terminal['finished_sim_stamp_ns']):
            raise ValueError('trace_result_binding')
        try:
            self.reader.measure(at/1e9, sim)
        except Unavailable:
            return None
        if len(self.samples) < 2:
            return None
        pair = list(self.samples)[-2:]
        if (pair[0][0]['sequence']+1 != pair[1][0]['sequence']
                or any(not 0 <= at-received < 1000000000 or not 0 <= sim-packet['stamp_ns'] < 1000000000
                       for packet, received in pair)):
            return None
        change = terminal['state_change']
        applied = change['disposition'] == 'applied'
        if applied:
            effect = change['effect']
            expected = {'owner_run_id': self.target['run_id'], 'mission_id': self.target['mission_id'],
                'action_id': self.target['action_id'], 'request_sha256': digest,
                'machine_id': self.target['machine_id'], 'operation': 'isolate',
                'control_revision': self.baseline['control']['revision']+1,
                'applied_stamp_ns': effect['applied_stamp_ns']}
            if effect != expected or effect['applied_stamp_ns'] < terminal['started_sim_stamp_ns']:
                raise ValueError('trace_effect_binding')
            verified = change['verification']
            references = [(p, receipt) for p, receipt in self.samples if p['sequence'] == verified['sequence']]
            if not references:
                return None  # Topic callback order is not a cross-topic source ordering guarantee.
            reference, receipt = references[0]
            measured = reference['machines'][self.target['machine_id']]
            if (hashlib.sha256(canonical(reference).encode()).hexdigest() != verified['sample_sha256']
                    or reference['stamp_ns'] != verified['stamp_ns'] or reference['control']['last_applied'] != effect
                    or measured['powered'] is not False or measured['current_a'] != 0 or measured['vibration_au'] != 0
                    or not 0 <= terminal['finished_sim_stamp_ns']-reference['stamp_ns'] < 1000000000
                    or abs(self.terminal_at-receipt) >= 1000000000):
                raise ValueError('trace_verification_reference')
            for packet, _ in pair:
                if packet['stamp_ns'] <= effect['applied_stamp_ns']:
                    return None
                machine = packet['machines'][self.target['machine_id']]
                if (packet['control'] != {'revision': effect['control_revision'], 'last_applied': effect}
                        or machine['powered'] is not False or machine['current_a'] != 0 or machine['vibration_au'] != 0):
                    raise ValueError('trace_isolation_not_observed')
        else:
            # Unchanged samples do not prove that no transient effect occurred.
            for packet, _ in pair:
                if packet['stamp_ns'] < terminal['finished_sim_stamp_ns']:
                    return None
                if (packet['control'] != self.baseline['control']
                        or packet['machines'][self.target['machine_id']]['powered'] is not True):
                    raise ValueError('trace_nonapplication_contradicted')
        return {'target': self.target, 'request_sha256': digest,
                'station_run_id': self.baseline['run_id'], 'server_status': terminal['status'],
                'server_disposition': change['disposition'], 'verified_isolation': applied,
                'sampled_unchanged': not applied, 'baseline_sequence': self.baseline['sequence'],
                'post_sequences': [p['sequence'] for p, _ in pair],
                'post_sample_sha256': [hashlib.sha256(canonical(p).encode()).hexdigest() for p, _ in pair],
                'hold_released_reported': terminal['hold_released'], 'scope': SCOPE}

    def end(self, at, sim, sources):
        if canonical(source(sources, self.registry)) != canonical(self.source):
            raise ValueError('trace_source_changed')
        result = self.assess(at, sim)
        if result is None:
            raise ValueError('trace_evidence_incomplete')
        self.write('boundary', at, {'event': 'end', 'sim_stamp_ns': sim, 'source': sources, 'observation': result})
        return result


def verify_isolation(root, path):
    """Read-only replay with matching configured registry; no live ROS needed."""
    registry = MachineRegistry.load(root)
    trace = observation = None
    def visit(record, summary):
        nonlocal trace, observation
        if observation is not None:
            raise ValueError('trace_record_after_end')
        value, at = record['data'], record['received_monotonic_ns']
        if not isinstance(value, dict):
            raise ValueError('invalid_trace_record')
        sim = value.get('sim_stamp_ns')
        if record['sequence'] == 1:
            if (record['kind'] != 'boundary' or set(value) != {'event', 'sim_stamp_ns', 'schema_version',
                    'kind', 'target', 'registry_identity', 'source'} or value['event'] != 'begin'
                    or type(value['schema_version']) is not int or value['schema_version'] != 1
                    or value['kind'] != 'isolation_observation' or value['registry_identity'] != registry.identity):
                raise ValueError('invalid_isolation_trace_header')
            trace = IsolationTrace(registry, value['target'])
            trace.begin(at, sim, value['source'])
        elif record['kind'] == 'sample' and set(value) == {'topic', 'sim_stamp_ns', 'text'}:
            trace.receive(value['topic'], at, sim, value['text'])
        elif record['kind'] == 'boundary' and value.get('event') == 'ready' and set(value) == {'event', 'sim_stamp_ns'}:
            if trace.ready_at is not None or not trace.ready(at, sim):
                raise ValueError('invalid_trace_readiness')
        elif record['kind'] == 'boundary' and value.get('event') == 'end' and set(value) == {
                'event', 'sim_stamp_ns', 'source', 'observation'}:
            observation = trace.end(at, sim, value['source'])
            if observation != value['observation']:
                raise ValueError('trace_observation_mismatch')
        else:
            raise ValueError('invalid_isolation_trace_record')
    try:
        summary = verify_trace(path, visit)
    except (KeyError, TypeError, OverflowError, RecursionError):
        raise ValueError('invalid_isolation_trace_structure') from None
    if observation is None or summary.get('observation') != observation:
        raise ValueError('trace_missing_final_observation')
    registry.verify_current()
    return {'complete': True, 'integrity_verified': True, 'observation': observation}
