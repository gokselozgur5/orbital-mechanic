"""Read-only maintenance sequence retention; historical effects are not permission."""
import copy

from orbital import maintenance_protocol as protocol
from orbital.inspection_protocol import canonical, decode, MAX_ACTIONS, MAX_REPLY_BYTES
from orbital.isolation_trace import IsolationTrace, TOPICS, source, target as action_target
from orbital.maintenance_observation import RecoveryEvidence
from orbital.maintenance_trace_checks import SequenceChecks, digest
from orbital.proximity import Unavailable
from orbital.machine_registry import MachineRegistry
from orbital.telemetry_observation import decode as telemetry_decode
from orbital.trajectory import verify_trace

MAX_SAMPLES = 4096
SCOPE = ('Retained public model sequence and sampled post-effect recovery; not camera/pose admission, '
         'independent hold-release proof, continuous physical health, repair causation or authentication')


def target(value):
    if not isinstance(value, dict) or set(value) != {'run_id','mission_id','action_id','machine_id','require_recovery'}:
        raise ValueError('invalid_sequence_target')
    if type(value['require_recovery']) is not bool: raise ValueError('invalid_sequence_target')
    return {**action_target({k:v for k,v in value.items() if k != 'require_recovery'}),
            'require_recovery':value['require_recovery']}


class MaintenanceTrace(IsolationTrace):
    """Reuse recorder clocks/readiness, while retaining every bounded sequence reference."""
    def __init__(self, registry, identity, *, writer=None):
        identity = target(identity)
        super().__init__(registry, {k:v for k,v in identity.items() if k != 'require_recovery'}, writer=writer)
        self.target = identity
        self.samples, self.requests, self.results = {}, {}, {}
        self.recovery = RecoveryEvidence(registry)

    def begin(self, at, sim, sources):
        if self.source is not None: raise ValueError('trace_already_started')
        self.clocks(at, sim)
        self.source = copy.deepcopy(source(sources, self.registry))
        self.write('boundary', at, {'event':'begin','sim_stamp_ns':sim,'schema_version':1,
            'kind':'maintenance_sequence','target':self.target,'registry_identity':self.registry.identity,'source':self.source})

    def mission_matches(self, value):
        return all(value.get(k) == self.target[k] for k in ('run_id','mission_id'))

    def ready(self, at, sim):
        if (self.ready_at is None and self.status is not None and self.status['last_result'] is not None
                and self.mission_matches(self.status['last_result'])):
            raise ValueError('sequence_mission_already_started')
        return super().ready(at, sim)

    def receive(self, topic, at, sim, text):
        if self.source is None or topic not in TOPICS: raise ValueError('invalid_trace_message')
        self.clocks(at, sim)
        if topic == '/station/telemetry':
            packet = telemetry_decode(text, self.registry)
            if packet['schema_version'] != 2: raise ValueError('trace_requires_control_telemetry')
            if not self.reader.observe(text, at/1e9, sim): raise ValueError(self.reader.failure)
            if len(self.samples) >= MAX_SAMPLES: raise ValueError('sequence_sample_capacity')
            if self.samples:
                previous = next(reversed(self.samples.values()))[0]['control']
                current = packet['control']
                if (current['revision'] < previous['revision'] or current['revision'] > previous['revision']+1
                        or (current['revision'] == previous['revision'] and current != previous)):
                    raise ValueError('sequence_control_history_changed')
                if current != previous: self.recovery = RecoveryEvidence(self.registry)
            if self.ready_at is not None and self.reader.missed: raise ValueError('trace_telemetry_gap')
            self.samples[packet['sequence']] = (packet, at, sim)
            if not self.recovery.observe(text, at/1e9, sim): raise ValueError('sequence_recovery_source_invalid')
        elif topic == '/maintenance/state':
            value = protocol.state(text)
            if value['run_id'] != self.target['run_id']: raise ValueError('trace_owner_run_changed')
            if not 0 <= sim-value['sim_stamp_ns'] < 500000000: raise ValueError('trace_stale_state')
            if self.status is not None:
                old = self.status
                if (value['station_run_id'] != old['station_run_id'] or value['sequence'] <= old['sequence']
                        or value['sim_stamp_ns'] < old['sim_stamp_ns'] or value['control_revision'] < old['control_revision']):
                    raise ValueError('trace_state_replayed_or_changed')
            if self.ready_at is not None and value['active'] is not None and (
                    not self.mission_matches(value['active']) or value['active']['machine_id'] != self.target['machine_id']):
                raise ValueError('trace_other_action')
            if self.status is None or value['sim_stamp_ns'] > self.status['sim_stamp_ns']: self.status_at = at
            self.status = value
        elif topic == '/maintenance/command':
            value = protocol.command(text)
            if self.mission_matches(value) and value['op'] in ('isolate','reset','restore','cancel_isolate'):
                if self.ready_at is None: raise ValueError('trace_command_before_ready')
                if (value['arguments']['machine_id'] != self.target['machine_id']
                        or value['arguments']['station_run_id'] != self.baseline['run_id']):
                    raise ValueError('trace_command_binding')
                key = value['action_id']
                if key in self.requests and protocol.request_digest(value) != protocol.request_digest(self.requests[key]):
                    raise ValueError('trace_conflicting_command')
                self.requests.setdefault(key, value)
        else:
            value = protocol.result(decode(text, MAX_REPLY_BYTES))
            if self.mission_matches(value):
                if self.ready_at is None: raise ValueError('trace_result_before_ready')
                if value['machine_id'] != self.target['machine_id']: raise ValueError('sequence_other_machine')
                key = value['action_id']
                if key in self.results and value != self.results[key]: raise ValueError('trace_conflicting_result')
                self.results.setdefault(key, value)
        if len(set(self.requests)|set(self.results)) > MAX_ACTIONS: raise ValueError('sequence_action_capacity')
        self.write('sample', at, {'topic':topic,'sim_stamp_ns':sim,'text':text})

    def assess(self, at, sim):
        self.clocks(at, sim)
        key = self.target['action_id']
        if self.ready_at is None or key not in self.requests or key not in self.results: return None
        terminal = self.results[key]
        if any(v['finished_sim_stamp_ns'] > terminal['finished_sim_stamp_ns'] for v in self.results.values()):
            raise ValueError('sequence_target_not_final')
        try:
            actions, control = SequenceChecks(self).check()
            self.reader.measure(at/1e9, sim)
            recovery = self.recovery.measure(at/1e9, sim)
        except Unavailable:
            return None
        pair = list(self.samples.values())[-2:]
        if len(pair) != 2 or any(not 0 <= at-received < 1000000000 or not 0 <= sim-p['stamp_ns'] < 1000000000
                or p['stamp_ns'] < terminal['finished_sim_stamp_ns'] for p, received, _ in pair):
            return None
        if any(p['control'] != control for p, _, _ in pair): raise ValueError('sequence_final_control_changed')
        if (self.status is None or self.status['active'] is not None or self.status['last_result'] != terminal
                or self.status['control_revision'] != control['revision']
                or self.status['station_run_id'] != self.baseline['run_id']
                or at-self.status_at >= 500000000 or not 0 <= sim-self.status['sim_stamp_ns'] < 500000000):
            return None
        after_restore = (control['revision'] > self.baseline['control']['revision']
                         and control['last_applied']['operation'] == 'restore')
        recovery_verified = after_restore and recovery['verified']
        if self.target['require_recovery'] and not recovery_verified: return None
        packet = pair[-1][0]
        ordered = sorted(actions, key=lambda action:(self.results[action]['started_sim_stamp_ns'],
                                                   self.results[action]['finished_sim_stamp_ns'],action))
        return {'target':self.target,'station_run_id':self.baseline['run_id'],'server_status':terminal['status'],
            'actions':[actions[action] for action in ordered],
            'observed_reset_restore_chain':any(v['operation'] == 'restore' and v.get('disposition') == 'applied' for v in actions.values()),
            'final_control':control,'final_measurements':{'voltage_v':packet['voltage_v'], **packet['machines'][self.target['machine_id']]},
            'final_sequences':[p['sequence'] for p, _, _ in pair], 'final_sha256':[digest(p) for p, _, _ in pair],
            'post_restore_recovery_verified':recovery_verified,'recovery_observation':recovery,'scope':SCOPE}


def verify_maintenance(root, path):
    registry = MachineRegistry.load(root)
    trace = observation = None
    def visit(record, summary):
        nonlocal trace, observation
        if observation is not None: raise ValueError('trace_record_after_end')
        value, at = record['data'], record['received_monotonic_ns']
        if not isinstance(value, dict): raise ValueError('invalid_trace_record')
        sim = value.get('sim_stamp_ns')
        if record['sequence'] == 1:
            if (record['kind'] != 'boundary' or set(value) != {'event','sim_stamp_ns','schema_version',
                    'kind','target','registry_identity','source'} or value['event'] != 'begin'
                    or type(value['schema_version']) is not int or value['schema_version'] != 1
                    or value['kind'] != 'maintenance_sequence' or value['registry_identity'] != registry.identity):
                raise ValueError('invalid_maintenance_trace_header')
            trace = MaintenanceTrace(registry, value['target'])
            trace.begin(at, sim, value['source'])
        elif record['kind'] == 'sample' and set(value) == {'topic','sim_stamp_ns','text'}:
            trace.receive(value['topic'], at, sim, value['text'])
        elif record['kind'] == 'boundary' and value.get('event') == 'ready' and set(value) == {'event','sim_stamp_ns'}:
            if trace.ready_at is not None or not trace.ready(at, sim): raise ValueError('invalid_trace_readiness')
        elif record['kind'] == 'boundary' and value.get('event') == 'end' and set(value) == {'event','sim_stamp_ns','source','observation'}:
            observation = trace.end(at, sim, value['source'])
            if observation != value['observation']: raise ValueError('trace_observation_mismatch')
        else: raise ValueError('invalid_maintenance_trace_record')
    try:
        summary = verify_trace(path, visit)
    except (KeyError, TypeError, OverflowError, RecursionError):
        raise ValueError('invalid_maintenance_trace_structure') from None
    if observation is None or summary.get('observation') != observation: raise ValueError('trace_missing_final_observation')
    registry.verify_current()
    return {'complete':True,'integrity_verified':True,'observation':observation}
