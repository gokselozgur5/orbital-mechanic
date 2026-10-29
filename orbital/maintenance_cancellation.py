"""Server-owned cancellation with one separately identified isolation attempt."""
import copy
import hashlib
import json
import uuid

from orbital.inspection_protocol import canonical
from orbital import maintenance_protocol as protocol
from orbital import cancellation_protocol
from orbital.proximity import Unavailable
from orbital.telemetry_observation import TelemetryEvidence


class Cancellation:
    def __init__(self, owner, request, target, isolation_id):
        self.owner, self.request, self.target = owner, copy.deepcopy(request), target
        self.digest = protocol.request_digest(request)
        self.isolation_id = isolation_id
        self.started, self.started_sim = owner.now(), owner.sim_ns()
        self.deadline = self.started+request['arguments']['timeout_s']
        self.phase, self.stop_reason, self.result = 'cancelling', None, None
        self.child_request = self.child_rejection = None
        self.reader = self.observing = self.expected_control = None
        self.after = None
        self.pair = []
        self.evidence_failed = False

    @property
    def args(self): return self.request['arguments']

    @property
    def child(self): return self.owner.actions.get(self.isolation_id)

    def identity(self): return {k:self.request[k] for k in ('run_id', 'mission_id', 'action_id')}

    def view(self):
        return {**self.identity(), 'request_sha256': self.digest, 'machine_id': self.args['machine_id'],
            'phase': self.phase, 'stop_reason': self.stop_reason, 'result': copy.deepcopy(self.result),
            'cancellation': {'target_action_id': self.target.request['action_id'],
                'isolation_action_id': self.isolation_id, 'target_phase': self.target.phase,
                'isolation_phase': None if self.child is None else self.child.phase}}

    def stop(self, status, code):
        self.stop_reason = self.stop_reason or (status, code)
        if self.owner.active is not None and (self.owner.active is self.target or self.owner.active is self.child):
            self.owner.stop(status, code)

    def guard(self):
        owner = self.owner
        if owner.now() < self.started or owner.sim_ns() < self.started_sim:
            self.evidence_failed = True
            self.stop('failed', 'clock_regressed')
        if owner.model.run_id != self.args['station_run_id']:
            self.evidence_failed = True
            self.stop('failed', 'station_run_changed')
        if owner.now() >= self.deadline:
            self.stop('timed_out', 'deadline_exceeded')
        if owner.closing:
            self.stop('cancelled', 'owner_shutdown')
        try:
            owner.authority()
            owner.model.registry.verify_current()
        except (ValueError, OSError, RuntimeError):
            self.evidence_failed = True
            self.stop('failed', 'cancellation_evidence_unavailable')
        if self.reader is not None and self.reader.failure:
            self.evidence_failed = True
            self.stop('failed', 'cancellation_evidence_unavailable')

    def telemetry(self, text, now, sim):
        if self.reader is not None and self.reader.observe(text, now, sim):
            self.pair.append((json.loads(self.reader.candidate), now))
            self.pair[:] = self.pair[-2:]

    def _begin_observing(self, action):
        protocol.result(action.result)
        if not current_target(self.owner, action):
            self.evidence_failed = True
            self.stop('failed', 'precondition_failed')
            return
        self.observing = action
        self.expected_control = {'revision': self.owner.model.control_revision,
                                 'last_applied': copy.deepcopy(self.owner.model.last_applied)}
        self.reader = TelemetryEvidence(self.owner.model.registry)
        self.after = self.owner.sim_ns()
        self.pair = []
        self.phase = 'confirming' if action is self.child else 'assessing'

    def _verification(self):
        owner = self.owner
        if not current_target(owner, self.observing):
            self.evidence_failed = True
            self.stop('failed', 'precondition_failed')
            return None
        try:
            self.reader.measure(owner.now(), owner.sim_ns())
        except Unavailable:
            return None
        now, sim = owner.now(), owner.sim_ns()
        if len(self.pair) < 2:
            return None
        # Check both members independently; a fresh latest packet cannot renew
        # an expired first member or hide its contradictory power measurement.
        if any(not 0 <= now-at < 1. or not 0 <= sim-packet['stamp_ns'] < 1000000000
               or packet['stamp_ns'] <= self.after for packet, at in self.pair):
            return None
        packets = [packet for packet, _ in self.pair]
        machine = self.args['machine_id']
        if (packets[1]['sequence'] != packets[0]['sequence']+1
                or any(packet.get('control') != self.expected_control
                    or packet['run_id'] != self.args['station_run_id'] for packet in packets)
                or packets[0]['machines'][machine]['powered'] is not packets[1]['machines'][machine]['powered']):
            self.evidence_failed = True
            self.stop('failed', 'cancellation_evidence_unavailable')
            return None
        powered = packets[-1]['machines'][machine]['powered']
        if not powered and any(packet['machines'][machine]['current_a'] != 0
                               or packet['machines'][machine]['vibration_au'] != 0 for packet in packets):
            self.evidence_failed = True
            self.stop('failed', 'cancellation_evidence_unavailable')
            return None
        samples = [{'sequence':packet['sequence'], 'stamp_ns':packet['stamp_ns'],
            'sample_sha256':hashlib.sha256(canonical(packet).encode()).hexdigest(),
            'received_age_s':now-at, 'simulation_age_s':(sim-packet['stamp_ns'])/1e9,
            **{key:packet['machines'][machine][key] for key in ('powered', 'current_a', 'vibration_au')}}
            for packet, at in self.pair]
        return {'registry_identity':owner.model.registry.identity, 'run_id':self.args['station_run_id'],
            'control':copy.deepcopy(self.expected_control), 'after_sim_stamp_ns':self.after,
            'observed_sim_stamp_ns':sim, 'samples':samples}

    def _isolate(self):
        remaining = self.deadline-self.owner.now()
        if remaining < .1:
            self.stop('timed_out', 'deadline_exceeded')
            return
        request = {**self.identity(), 'schema_version':1, 'action_id':self.isolation_id,
            'command_id':uuid.uuid4().hex, 'op':'isolate', 'arguments':{
                'machine_id':self.args['machine_id'], 'station_run_id':self.args['station_run_id'],
                'control_revision':self.expected_control['revision'], 'timeout_s':min(30., remaining)}}
        self.child_request = request  # Persist intent before the one admission call.
        reply = self.owner._handle(request, cancellation=self)
        if not reply['accepted']:
            self.child_rejection = reply['error_code']
            self.stop('failed', 'cancellation_isolation_failed')
        else:
            self.phase = 'isolating'

    def tick(self):
        # Even after our deadline, an unresolved original/child effect is still
        # owned. Telemetry alone cannot turn an unknown call into a known result.
        if self.target.result is None or (self.child is not None and self.child.result is None):
            pending = self.child or self.target
            self.phase = 'unresolved' if pending.phase == 'unresolved' else ('isolating' if self.child else 'cancelling')
            return
        latest = self.child or self.target
        if self.evidence_failed:
            self.finish(None)
            return
        if self.observing is not latest:
            self._begin_observing(latest)
        if self.evidence_failed:
            self.finish(None)
            return
        verification = self._verification()
        if verification is None:
            if self.stop_reason is not None:
                self.finish(None)
            return
        if verification['samples'][-1]['powered']:
            if self.child_request is None and self.stop_reason is None:
                self._isolate()
                return
            if self.child is not None and self.stop_reason is None:
                self.stop('failed', 'cancellation_isolation_failed')
        self.finish(verification)

    def finish(self, verification):
        final_state = ('isolated' if not verification['samples'][-1]['powered'] else 'energized') if verification else 'unverified'
        status, code = self.stop_reason or ('succeeded', None)
        value = {'schema_version':4, 'kind':'maintenance_result', **self.identity(),
            'request_sha256':self.digest, 'operation':'cancel_isolate', 'machine_id':self.args['machine_id'],
            'station_run_id':self.args['station_run_id'], 'status':status, 'error_code':code,
            'started_sim_stamp_ns':self.started_sim, 'finished_sim_stamp_ns':self.owner.sim_ns(),
            'wall_duration_s':max(0.,self.owner.now()-self.started), 'request':copy.deepcopy(self.request),
            'target_request':copy.deepcopy(self.target.request), 'target_result':copy.deepcopy(self.target.result),
            'isolation_action_id':self.isolation_id, 'isolation_request':copy.deepcopy(self.child_request),
            'isolation_result':None if self.child is None else copy.deepcopy(self.child.result),
            'isolation_rejection':self.child_rejection, 'final_state':final_state,
            'verification':verification, 'hold_released':True}
        cancellation_protocol.result(value)
        self.result, self.phase = value, 'terminal'
        self.owner.cancellation = None
        self.owner.last_result = copy.deepcopy(value)
        self.owner.on_result(copy.deepcopy(value))


def current_target(owner, target):
    if target.result is None:
        return owner.active is target
    if owner.model.run_id != target.args['station_run_id']:
        return False
    revision = cancellation_protocol.expected_revision(target.request, target.result)
    if revision != owner.model.control_revision:
        return False
    return (target.result['state_change']['disposition'] != 'applied'
            or owner.model.last_applied == target.result['state_change']['effect'])


def admit(owner, request):
    args = request['arguments']
    target = owner.actions.get(args['target_action_id'])
    if (target is None or target.request['mission_id'] != request['mission_id']
            or any(target.args[k] != args[k] for k in ('machine_id', 'station_run_id'))):
        raise ValueError('cancellation_target_required')
    if owner.closing:
        raise ValueError('unavailable')
    if owner.cancellation is not None or (owner.active is not None and owner.active is not target):
        raise ValueError('busy')
    if any(item.target is target for item in owner.cancellations.values()):
        raise ValueError('cancellation_conflict')
    if owner.capacity() < 2:
        raise ValueError('history_full')
    if owner.model.run_id != args['station_run_id'] or not current_target(owner, target):
        raise ValueError('target_superseded')
    for _ in range(8):
        child_id = uuid.uuid4().hex
        if child_id not in {*owner.actions, *owner.cancellations, *owner.reservations, request['action_id']}:
            break
    else:
        raise ValueError('unavailable')
    cancellation = Cancellation(owner, request, target, child_id)
    owner.cancellations[request['action_id']] = owner.cancellation = cancellation
    owner.reservations[child_id] = request['action_id']
    if target.result is None:
        owner.stop('timed_out' if owner.now() >= target.deadline else 'cancelled',
                   'deadline_exceeded' if owner.now() >= target.deadline else 'cancel_requested')
    return cancellation
