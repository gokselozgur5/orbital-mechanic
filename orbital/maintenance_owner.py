"""Single-executor station controls with retained observation and drive ownership."""
import copy
from dataclasses import dataclass
import hashlib
import json
import uuid

from orbital.inspection_owner import InspectionOwner, load_result, fresh
from orbital.inspection_protocol import canonical, MAX_ACTIONS
from orbital.maintenance_protocol import command, request_digest
from orbital.proximity import Unavailable
from orbital.station_control import EffectRejected
from orbital.telemetry_observation import TelemetryEvidence
from orbital.reset_evidence import ResetEvidence, MAX_RESET_TEMPERATURE_C
from orbital.maintenance_cancellation import admit as admit_cancellation
from orbital.maintenance_journal import JournalUnavailable


class InspectionGuard:
    """Keep the parent pose history alive after the capture worker completes."""
    def __init__(self, live, hold):
        self.live, self.hold = live, hold

    def ready(self): return self.live.ready() and self.hold.held()
    def begin(self, machine): self.live.begin(machine)
    def end(self): pass  # Only the maintenance owner ends this observation.
    def identity(self): return self.live.identity()

    def sample(self):
        if not self.hold.held():
            raise ValueError('maintenance_hold_lost')
        return self.live.sample()


@dataclass
class Action:
    request: dict
    digest: str
    started: float
    started_sim: int
    deadline: float
    hold: dict
    phase: str = 'holding'
    inspector: object = None
    stop_reason: tuple = None
    effect: dict = None
    unknown: bool = False
    release_requested: bool = False
    verifier: object = None
    verification_packets: object = None
    verification: dict = None
    observation: dict = None
    result: dict = None
    reset_basis: dict = None
    isolation: dict = None
    reset: dict = None

    @property
    def args(self): return self.request['arguments']

    @property
    def operation(self): return self.request['op']

    def identity(self):
        return {key:self.request[key] for key in ('run_id', 'mission_id', 'action_id')}

    def view(self):
        return {**self.identity(), 'request_sha256': self.digest, 'machine_id': self.args['machine_id'],
                'phase': self.phase, 'stop_reason': self.stop_reason, 'result': copy.deepcopy(self.result)}


class MaintenanceOwner:
    def __init__(self, root, model, live, hold, worker, *, now, sim_ns, on_result=lambda _: None,
                 inspection_factory=InspectionOwner, load=load_result, authority=lambda: None, journal=None):
        self.root, self.model, self.live, self.hold, self.worker = root, model, live, hold, worker
        self.now, self.sim_ns, self.on_result = now, sim_ns, on_result
        self.inspection_factory, self.load = inspection_factory, load
        self.authority = authority
        self.run_id = uuid.uuid4().hex
        self.journal = journal
        if journal is not None:
            journal.claim(self.run_id, model.run_id, model.registry.identity)
            model.journal = journal
        elif model.journal is not None:
            raise ValueError('station_journal_required')
        self.actions, self.missions = {}, {}
        self.active = self.last_result = None
        self.cancellations, self.reservations = {}, {}
        self.cancellation = None
        self.closing = False
        self.sequence = 0
        self.reset_evidence = ResetEvidence(model.registry)

    def ready(self):
        return self.cancellation is None and self._ready()

    def capacity(self):
        return MAX_ACTIONS-len(self.actions)-len(self.cancellations)-len(self.reservations)

    def _ready(self, *, reserved=False):
        try:
            return (not self.closing and (self.journal is None or not self.journal.broken)
                    and self.active is None and (self.capacity() > 0 or reserved)
                    and self.model.sequence > 0 and self.live.ready() and self.hold.available())
        except (ValueError, OSError, RuntimeError):
            return False

    def persist_admission(self, request, **identities):
        if self.journal is None: return None
        try:
            self.journal.admit(request, **identities)
        except ValueError as error:
            if str(error) in ('mission_reused', 'action_reused', 'journal_full', 'history_full'):
                return str(error)
            self.journal.broken = True
            return 'journal_unavailable'
        except JournalUnavailable:
            return 'journal_unavailable'
        return None

    def persist_result(self, result):
        if self.journal is None: return True
        try:
            self.journal.terminal(result)
        except (JournalUnavailable, ValueError):
            self.journal.broken = True
            return False
        return True

    def handle(self, text):
        try:
            request = command(text)
        except (ValueError, TypeError, UnicodeError):
            return {'schema_version': 1, 'kind': 'maintenance_reply', 'run_id': self.run_id,
                    'command_id': None, 'accepted': False, 'error_code': 'invalid_argument', 'action': None}
        return self._handle(request)

    def _handle(self, request, *, cancellation=None):
        reply = {'schema_version': 1, 'kind': 'maintenance_reply', 'run_id': self.run_id,
                 'command_id': None, 'accepted': False, 'error_code': 'invalid_argument', 'action': None}
        reply.update({key:request[key] for key in ('command_id', 'mission_id', 'action_id', 'op')})

        def reject(code):
            reply['error_code'] = code
            return reply

        if request['run_id'] != self.run_id:
            return reject('wrong_run')
        workflow = self.cancellations.get(request['action_id'])
        if workflow is not None:
            if (workflow.request['mission_id'] != request['mission_id']
                    or (request['op'] == 'cancel_isolate' and request_digest(request) != workflow.digest)
                    or request['op'] not in ('cancel_isolate', 'cancel', 'query')):
                return reject('action_conflict')
            if request['op'] == 'cancel' and workflow.result is None:
                workflow.stop('cancelled', 'cancel_requested')
            reply.update(accepted=True, error_code=None, action=workflow.view())
            return reply
        internal = (cancellation is not None and cancellation is self.cancellation
                    and request is cancellation.child_request and request['op'] == 'isolate'
                    and self.reservations.get(request['action_id']) == cancellation.request['action_id'])
        if request['action_id'] in self.reservations and not internal:
            return reject('action_conflict')
        if request['op'] == 'cancel_isolate':
            if request['action_id'] in self.actions:
                return reject('action_conflict')
            try:
                workflow = admit_cancellation(self, request)
            except ValueError as error:
                return reject(str(error))
            reply.update(accepted=True, error_code=None, action=workflow.view())
            return reply
        action = self.actions.get(request['action_id'])
        if action is not None and action.request['mission_id'] != request['mission_id']:
            return reject('action_conflict')
        if request['op'] in ('isolate', 'reset', 'restore'):
            digest = request_digest(request)
            if action is not None:
                if digest != action.digest:
                    return reject('action_conflict')
            else:
                if self.active is not None or (self.cancellation is not None and not internal):
                    return reject('busy')
                if self.capacity() <= 0 and not internal:
                    return reject('history_full')
                if not self._ready(reserved=internal):
                    return reject('unavailable')
                args = request['arguments']
                if (args['station_run_id'] != self.model.run_id or args['control_revision'] != self.model.control_revision
                        or self.missions.get(request['mission_id'], self.model.run_id) != self.model.run_id):
                    return reject('wrong_station_state')
                prior = None
                if request['op'] in ('reset', 'restore'):
                    resetting = request['op'] == 'reset'
                    prior = self.actions.get(args['isolation_action_id' if resetting else 'reset_action_id'])
                    if (prior is None or prior.operation != ('isolate' if resetting else 'reset') or prior.result is None
                            or prior.request['mission_id'] != request['mission_id']
                            or prior.args['station_run_id'] != args['station_run_id']
                            or prior.args['machine_id'] != args['machine_id']
                            or prior.result['state_change']['disposition'] != 'applied'):
                        return reject('isolation_required' if resetting else 'reset_required')
                    if (prior.reset_basis is None or self.reset_evidence.failure
                            or args['machine_id'] in self.reset_evidence.high_vibration):
                        return reject(request['op']+'_not_eligible')
                    available = self.model.reset_available if resetting else self.model.restore_available
                    if not available(args['machine_id']):
                        return reject(request['op']+'_attempt_exhausted')
                started, sim = self.now(), self.sim_ns()
                token = {'owner_run_id': self.run_id, 'mission_id': request['mission_id'],
                         'action_id': request['action_id'], 'token': uuid.uuid4().hex}
                action = Action(request, digest, started, sim, started+args['timeout_s'], token)
                if prior is not None:
                    action.reset_basis = copy.deepcopy(prior.reset_basis)
                    if request['op'] == 'reset': action.isolation = copy.deepcopy(prior.result)
                    else: action.reset = copy.deepcopy(prior.result)
                persistence_error = self.persist_admission(request,
                    parent_action_id=cancellation.request['action_id'] if internal else None)
                if persistence_error is not None:
                    return reject(persistence_error)
                # Record ownership before publication: a transport exception is
                # not proof that the guard could not receive this acquire.
                self.actions[request['action_id']] = self.active = action
                if internal: self.reservations.pop(request['action_id'])
                self.missions[request['mission_id']] = self.model.run_id
                try:
                    self.hold.begin(token)
                except (ValueError, OSError, RuntimeError):
                    self.stop('failed', 'hold_acquisition_unavailable')
        elif action is None:
            return reject('unknown_action')
        elif request['op'] == 'cancel' and action.result is None:
            self.stop('timed_out' if self.now() >= action.deadline else 'cancelled',
                      'deadline_exceeded' if self.now() >= action.deadline else 'cancel_requested')
        reply.update(accepted=True, error_code=None, action=action.view())
        return reply

    def stop(self, status, reason):
        action = self.active
        if action is None or action.stop_reason is not None:
            return
        action.stop_reason = (status, reason)
        if action.inspector is not None:
            action.inspector.stop(status, 'owner_shutdown')

    def shutdown(self):
        self.closing = True
        self.stop('cancelled', 'owner_shutdown')
        if self.cancellation is not None: self.cancellation.stop('cancelled', 'owner_shutdown')

    def telemetry(self, text):
        now, sim = self.now(), self.sim_ns()
        self.reset_evidence.observe(text, now, sim)
        if self.cancellation is not None: self.cancellation.telemetry(text, now, sim)
        action = self.active
        if action is not None and action.verifier is not None:
            if action.verifier.observe(text, now, sim):
                action.verification_packets.append((json.loads(action.verifier.candidate), now))
                action.verification_packets[:] = action.verification_packets[-2:]

    def _cooldown(self, action):
        if self.live.sample() is None:
            return
        try:
            values = self.reset_evidence.isolated(action.args['machine_id'], action.args['station_run_id'],
                action.args['control_revision'], self.now(), self.sim_ns())
        except Unavailable:
            return
        except ValueError:
            self.stop('failed', 'precondition_failed')
            return
        if values['temperature_c'] <= MAX_RESET_TEMPERATURE_C:
            self._capture(action)

    def _capture(self, action):
        action.inspector = self.inspection_factory(self.root, InspectionGuard(self.live, self.hold), self.worker,
            now=self.now, sim_ns=self.sim_ns, run_id=self.run_id)
        remaining = min(10., action.deadline-self.now())
        if remaining < .1:
            self.stop('timed_out', 'deadline_exceeded')
            return
        request = {**action.identity(), 'schema_version': 1, 'command_id': uuid.uuid4().hex, 'op': 'inspect',
                   'arguments': {'machine_id': action.args['machine_id'], 'timeout_s': remaining}}
        if action.inspector.handle(canonical(request))['accepted'] is not True:
            self.stop('failed', 'inspection_unavailable')
        action.phase = 'inspecting'

    def _apply(self, action):
        inspection = action.inspector.actions[action.request['action_id']]
        result = inspection.result
        if result['status'] != 'succeeded':
            self.stop('failed', 'inspection_unavailable')
            return
        try:
            reference = result['observation']
            record = self.load(self.root, reference, inspection)
            source = record['telemetry_source']
            if (source['schema_version'] != 2 or source['run_id'] != action.args['station_run_id']
                    or source['control_revision'] != action.args['control_revision']
                    or record['measurements']['powered'] is not (action.operation == 'isolate')):
                raise ValueError('wrong_inspection_station_state')
            if action.operation in ('reset', 'restore') and (record['measurements']['temperature_c'] > MAX_RESET_TEMPERATURE_C
                    or record['measurements']['current_a'] != 0 or record['measurements']['vibration_au'] != 0):
                raise ValueError('control_requires_cool_isolation')
            action.observation = copy.deepcopy(reference)
            if action.operation == 'isolate':
                action.reset_basis = self.reset_evidence.retain(action.args['machine_id'], record, self.now(), self.sim_ns())
            context = {'owner_run_id': self.run_id, 'mission_id': action.request['mission_id'],
                       'action_id': action.request['action_id'], 'request_sha256': action.digest,
                       'machine_id': action.args['machine_id']}

            def admit():
                current = self.live.sample()
                if current is None or not self.hold.held() or action.stop_reason is not None:
                    raise ValueError('maintenance_precondition_lost')
                now, sim = self.now(), self.sim_ns()
                if action.operation == 'isolate' and not self.reset_evidence.confirm(action.reset_basis, now, sim):
                    action.reset_basis = None
                if action.operation in ('reset', 'restore'):
                    values = self.reset_evidence.isolated(action.args['machine_id'], action.args['station_run_id'],
                        action.args['control_revision'], now, sim)
                    if (values['temperature_c'] > MAX_RESET_TEMPERATURE_C
                            or action.args['machine_id'] in self.reset_evidence.high_vibration):
                        raise ValueError('control_requires_cool_isolation')
                now, sim = self.now(), self.sim_ns()
                fresh(record, now, sim, current)
                if now >= action.deadline:
                    raise ValueError('deadline_exceeded')
                return {'stamp_ns': sim, 'distance_m': current['distance_m'],
                        'speed_mps': current['stationarity']['max_linear_m_s']}

            try:
                if action.operation == 'isolate':
                    action.effect = self.model.isolate(context, action.args['station_run_id'],
                                                       action.args['control_revision'], admission=admit)
                elif action.operation == 'reset':
                    action.effect = self.model.reset_machine(context, action.args['station_run_id'],
                        action.args['control_revision'], isolation_action_id=action.args['isolation_action_id'], admission=admit)
                else:
                    action.effect = self.model.restore_machine(context, action.args['station_run_id'],
                        action.args['control_revision'], reset_action_id=action.args['reset_action_id'], admission=admit)
            except EffectRejected:
                self.stop('timed_out' if self.now() >= action.deadline else 'failed',
                          'deadline_exceeded' if self.now() >= action.deadline else 'precondition_failed')
                return
            except Exception:
                action.unknown = True
                self.stop('failed', 'effect_unknown')
                return
            action.verifier = TelemetryEvidence(self.model.registry)
            action.verification_packets = []
            action.phase = 'verifying'
        except (EffectRejected, ValueError, OSError, KeyError, TypeError) as error:
            self.stop('timed_out' if self.now() >= action.deadline else 'failed',
                      'deadline_exceeded' if self.now() >= action.deadline else 'precondition_failed')
        except Exception:
            # An unexpected exception around a synchronous actuator is not a
            # mutation-free rejection. Keep the hold and exact action unresolved.
            action.unknown = True
            self.stop('failed', 'effect_unknown')

    @staticmethod
    def _postcondition(action, machine):
        if action.operation == 'restore':
            return machine['powered'] is True  # Energization is distinct from recovery.
        return (machine['powered'] is False and machine['current_a'] == 0 and machine['vibration_au'] == 0
                and (action.operation != 'reset' or machine['temperature_c'] <= MAX_RESET_TEMPERATURE_C))

    def _verify(self, action):
        action.verification = None  # A cached postcondition cannot renew freshness.
        try:
            observation = action.verifier.measure(self.now(), self.sim_ns())
        except Unavailable:
            return
        packet = observation['telemetry']
        if packet['run_id'] != action.args['station_run_id']:
            self.stop('failed', 'station_run_changed')
            return
        if packet['stamp_ns'] <= action.effect['applied_stamp_ns']:
            return
        if action.operation in ('reset', 'restore'):
            pair = action.verification_packets
            now, sim = self.now(), self.sim_ns()
            if len(pair) < 2 or any(p['stamp_ns'] <= action.effect['applied_stamp_ns']
                    or not 0 <= now-received < 1. or not 0 <= sim-p['stamp_ns'] < 1000000000
                    for p, received in pair):
                return
            if (pair[-1][0]['sequence'] != pair[0][0]['sequence']+1
                    or any(p['control']['last_applied'] != action.effect
                        or not self._postcondition(action, p['machines'][action.args['machine_id']])
                        for p, _ in pair)):
                self.stop('failed', action.operation+'_not_verified')
                return
        control = packet.get('control')
        machine = packet['machines'][action.args['machine_id']]
        if (control is None or control['last_applied'] != action.effect
                or not self._postcondition(action, machine)):
            self.stop('failed', 'isolation_not_verified' if action.operation == 'isolate' else action.operation+'_not_verified')
            return
        action.verification = {'run_id': packet['run_id'], 'sequence': packet['sequence'],
                               'stamp_ns': packet['stamp_ns'], 'control_revision': control['revision'],
                               'sample_sha256': hashlib.sha256(canonical(packet).encode()).hexdigest(),
                               'machine_id': action.args['machine_id'],
                               **{key:machine[key] for key in ('powered', 'current_a', 'vibration_au')}}
        if action.operation in ('reset', 'restore'): action.verification['temperature_c'] = machine['temperature_c']
        if action.operation == 'restore': action.verification['voltage_v'] = packet['voltage_v']
        action.phase = 'releasing'

    def tick(self):
        if self.cancellation is not None: self.cancellation.guard()
        self._tick_action()
        if self.cancellation is not None: self.cancellation.tick()

    def _tick_action(self):
        action = self.active
        if action is None:
            return
        if self.journal is not None and self.journal.broken:
            self.stop('failed', 'journal_unavailable')
        if self.now() < action.started or self.sim_ns() < action.started_sim:
            self.stop('failed', 'clock_regressed')
        if self.now() >= action.deadline:
            self.stop('timed_out', 'deadline_exceeded')
        if self.model.run_id != action.args['station_run_id']:
            self.stop('failed', 'station_run_changed')
        if action.inspector is not None and action.inspector.active is not None:
            action.inspector.tick()
        try:
            held = self.hold.held()
            if action.stop_reason is None:
                if action.phase != 'holding' and not held and not action.release_requested:
                    self.stop('failed', 'maintenance_hold_lost')
                elif action.phase == 'holding' and held:
                    if action.operation in ('reset', 'restore'):
                        self.live.begin(action.args['machine_id'])
                        action.phase = 'cooling'
                    else:
                        self._capture(action)
                elif action.phase == 'cooling':
                    self._cooldown(action)
                elif action.phase == 'inspecting' and action.inspector.active is None:
                    self._apply(action)
                elif action.phase == 'verifying' and self.live.sample() is None:
                    self.stop('failed', 'precondition_lost')
            if action.effect is not None:
                self._verify(action)
        except (ValueError, OSError, RuntimeError, KeyError, TypeError):
            self.stop('failed', 'maintenance_evidence_unavailable')
        child_owned = action.inspector is not None and action.inspector.active is not None
        if action.unknown or (action.effect is not None and action.verification is None and action.stop_reason):
            action.phase = 'unresolved'
        if child_owned or action.unknown or (action.effect is not None and action.verification is None):
            return
        if action.stop_reason is None and action.phase != 'releasing':
            return
        try:
            released = self.hold.owner is None or self.hold.release()
            action.release_requested = self.hold.release_at is not None
        except (ValueError, OSError, RuntimeError):
            released = False
        if not released:
            action.phase = 'unresolved' if action.stop_reason else 'releasing'
            return
        self.finish(action)

    def finish(self, action):
        # Exact hold release can perform I/O after the tick's initial checks.
        # Decide against the current clocks before freezing a historical result.
        if self.now() >= action.deadline:
            self.stop('timed_out', 'deadline_exceeded')
        if action.effect is not None:
            self._verify(action)
            if (action.verification is None or self.model.run_id != action.args['station_run_id']
                    or self.model.control_revision != action.effect['control_revision']
                    or self.model.last_applied != action.effect):
                action.verification, action.phase = None, 'unresolved'
                return
        status, code = action.stop_reason or ('succeeded', None)
        result = {'schema_version': {'isolate':1, 'reset':2, 'restore':3}[action.operation],
                  'kind': 'maintenance_result', **action.identity(),
                  'request_sha256': action.digest, 'operation': action.operation, 'machine_id': action.args['machine_id'],
                  'station_run_id': action.args['station_run_id'], 'status': status, 'error_code': code,
                  'started_sim_stamp_ns': action.started_sim, 'finished_sim_stamp_ns': self.sim_ns(),
                  'wall_duration_s': max(0., self.now()-action.started), 'observation': action.observation,
                  'state_change': {'disposition': 'applied' if action.effect else 'not_applied',
                                   'effect': action.effect, 'verification': action.verification},
                  'hold_released': True}
        if action.operation == 'reset':
            result.update(isolation=copy.deepcopy(action.isolation), eligibility=copy.deepcopy(action.reset_basis))
        if action.operation == 'restore': result['reset'] = copy.deepcopy(action.reset)
        # The recorded times describe this completed decision. Disk I/O and
        # later query/publication never renew its observation freshness.
        if not self.persist_result(result):
            action.unknown, action.phase = True, 'unresolved'
            self.stop('failed', 'journal_unavailable')
            return
        action.result, action.phase = result, 'terminal'
        self.last_result = copy.deepcopy(result)
        self.active = None
        self.live.end()
        self.on_result(copy.deepcopy(result))

    def state(self):
        self.sequence += 1
        return {'schema_version': 1, 'kind': 'maintenance_state', 'run_id': self.run_id,
                'station_run_id': self.model.run_id, 'control_revision': self.model.control_revision,
                'sequence': self.sequence, 'sim_stamp_ns': self.sim_ns(), 'ready': self.ready(),
                'capacity_remaining': self.capacity(),
                'active': self.cancellation.view() if self.cancellation is not None else (
                    None if self.active is None else self.active.view()), 'last_result': copy.deepcopy(self.last_result)}
