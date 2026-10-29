"""Serialized inspection admission, cancellation and terminal-result ownership."""
import copy
from dataclasses import dataclass
import time
import uuid

from orbital.inspection_artifacts import read_capture
from orbital.inspection_protocol import MAX_ACTIONS, canonical, command, finite, request_digest


def load_result(root, reference, action):
    record, _ = read_capture(root, reference['observation_id'], reference['manifest_sha256'])
    if (record.get('inspection_action') != action.context or record.get('machine_id') != action.machine
            or record.get('registry_identity') != action.context['registry_identity']
            or record.get('authority_identity') != action.context['authority_identity']
            or record.get('kind') != 'machine_inspection_observation'):
        raise ValueError('observation_identity_mismatch')
    if finite(record['observed_monotonic_s']) < action.started:
        raise ValueError('pre_admission_observation')
    return record


def fresh(record, now, sim_ns, current):
    elapsed = now - finite(record['observed_monotonic_s'])
    if not 0 <= elapsed <= .2:
        raise ValueError('stale_observation')
    for key in ('received_age_s', 'info_received_age_s'):
        if finite(record['camera'][key]) + elapsed > .2:
            raise ValueError('stale_observation')
    if finite(record['telemetry_freshness']['received_age_s']) + elapsed >= 1.:
        raise ValueError('stale_observation')
    for stamp in (record['sample_stamp_ns'], record['camera']['stamp_ns'], record['observed_sim_stamp_ns']):
        if type(stamp) is not int or not 0 <= sim_ns - stamp <= 200000000:
            raise ValueError('stale_observation')
    for robot in (record['robot_state'], record['camera']['robot_state']):
        if robot['within_settled_inspection_pose'] is not True:
            raise ValueError('precondition_failed')
        for name in ('run_id', 'epoch'):
            if robot['health'][name] != current['health'][name]:
                raise ValueError('localization_generation_changed')


@dataclass
class Action:
    request: dict
    digest: str
    started: float
    started_sim: int
    deadline: float
    phase: str = 'preparing'
    context: dict = None
    worker: object = None
    stop_reason: tuple = None
    result: dict = None

    @property
    def machine(self):
        return self.request['arguments']['machine_id']

    def identity(self):
        return {key: self.request[key] for key in ('run_id', 'mission_id', 'action_id')}

    def view(self):
        return {**self.identity(), 'request_sha256': self.digest, 'machine_id': self.machine,
                'phase': self.phase, 'result': copy.deepcopy(self.result)}


class InspectionOwner:
    """One executor calls every method; a terminal event follows actual child reap."""
    def __init__(self, root, guard, worker, *, now=time.monotonic, sim_ns, on_result=lambda _: None,
                 run_id=None, load=load_result):
        self.root, self.guard, self.factory = root, guard, worker
        self.now, self.sim_ns, self.on_result, self.load = now, sim_ns, on_result, load
        self.run_id = run_id or uuid.uuid4().hex
        self.actions = {}
        self.active = None
        self.last_result = None
        self.sequence = 0
        self.closing = False

    def handle(self, text):
        response = {'schema_version': 1, 'kind': 'inspection_reply', 'run_id': self.run_id,
                    'command_id': None, 'accepted': False, 'error_code': 'invalid_argument', 'action': None}
        try:
            value = command(text)
        except (ValueError, TypeError):
            return response
        response.update(command_id=value['command_id'], mission_id=value['mission_id'],
                        action_id=value['action_id'], op=value['op'])

        def reject(code):
            response['error_code'] = code
            return response

        if value['run_id'] != self.run_id:
            return reject('wrong_run')
        action = self.actions.get(value['action_id'])
        if action is not None and value['mission_id'] != action.request['mission_id']:
            return reject('action_conflict')
        if value['op'] == 'inspect':
            digest = request_digest(value)
            if action is not None:
                if digest != action.digest:
                    return reject('action_conflict')
            else:
                if self.closing:
                    return reject('unavailable')
                if self.active is not None:
                    return reject('busy')
                if len(self.actions) >= MAX_ACTIONS:
                    return reject('history_full')
                try:
                    if not self.guard.ready():
                        return reject('unavailable')
                    started, started_sim = self.now(), self.sim_ns()
                    if type(started_sim) is not int or not 0 < started_sim < 2**63:
                        return reject('unavailable')
                    self.guard.begin(value['arguments']['machine_id'])
                except (ValueError, RuntimeError, OSError):
                    return reject('unavailable')
                action = Action(value, digest, started, started_sim, started + value['arguments']['timeout_s'])
                self.actions[value['action_id']] = self.active = action
        elif action is None:
            return reject('unknown_action')
        elif value['op'] == 'cancel' and action.result is None:
            # The first terminal intent wins, even if successful bytes are waiting
            # in a pipe or the child has exited before this callback is processed.
            if self.now() >= action.deadline:
                self.stop('timed_out', 'deadline_exceeded')
            else:
                self.stop('cancelled', 'cancel_requested')
        response.update(accepted=True, error_code=None, action=action.view())
        return response

    def stop(self, status, code):
        action = self.active
        if action is None or action.stop_reason is not None:
            return
        action.stop_reason = (status, code)
        action.phase = 'cancelling'
        if action.worker is not None:
            action.worker.stop(self.now())

    def shutdown(self):
        self.closing = True
        self.stop('cancelled', 'owner_shutdown')

    def tick(self):
        action = self.active
        if action is None:
            return
        if self.now() < action.started or self.sim_ns() < action.started_sim:
            self.stop('failed', 'clock_regressed')
        if self.now() >= action.deadline:
            self.stop('timed_out', 'deadline_exceeded')
        current = None
        if action.stop_reason is None:
            try:
                current = self.guard.sample()
                if action.worker is not None and current is None:
                    raise ValueError('precondition_lost')
                if action.worker is None and current is not None:
                    action.context = {**action.identity(), 'request_sha256': action.digest,
                        **self.guard.identity(), 'admitted_monotonic_s': action.started,
                        'admitted_sim_stamp_ns': action.started_sim}
                    remaining = action.deadline - self.now()
                    if remaining < .1:
                        self.stop('timed_out', 'deadline_exceeded')
                    else:
                        action.worker = self.factory({'machine_id': action.machine,
                            'timeout_s': min(10., remaining), 'context': action.context})
                        action.phase = 'running'
            except (ValueError, RuntimeError, OSError, KeyError, TypeError, AttributeError):
                self.stop('failed', 'precondition_failed')
        if action.worker is None:
            if action.stop_reason:
                self.finish(*action.stop_reason)
            return
        done = action.worker.poll(self.now())
        if action.worker.error:
            self.stop('failed', action.worker.error)
        if not done:
            return
        if self.now() >= action.deadline:
            self.stop('timed_out', 'deadline_exceeded')
        if action.stop_reason:
            self.finish(*action.stop_reason)
            return
        if action.worker.reply['status'] != 'succeeded':
            self.finish('failed', 'capture_unavailable')
            return
        try:
            reference = action.worker.reply
            record = self.load(self.root, reference, action)
            observation = {key: reference[key] for key in ('observation_id', 'manifest_sha256')}
            observation.update(sample_stamp_ns=record['sample_stamp_ns'], camera_stamp_ns=record['camera']['stamp_ns'],
                               registry_identity=record['registry_identity'])
            # Finish all artifact I/O, hashing and result preparation before the
            # final live sample and both clocks. Disk files cannot restore a run.
            canonical(observation)
            current = self.guard.sample()
            if current is None:
                raise ValueError('precondition_failed')
            now, sim = self.now(), self.sim_ns()
            fresh(record, now, sim, current)
            if now >= action.deadline:
                self.finish('timed_out', 'deadline_exceeded')
            else:
                self.finish('succeeded', None, observation, now=now, sim=sim)
        except (ValueError, RuntimeError, OSError, KeyError, TypeError, AttributeError):
            if self.now() >= action.deadline:
                self.finish('timed_out', 'deadline_exceeded')
            else:
                self.finish('failed', 'observation_unavailable')

    def finish(self, status, code, observation=None, *, now=None, sim=None):
        action = self.active
        if action.worker is not None and not action.worker.done:
            raise RuntimeError('inspection_worker_still_owned')
        now = self.now() if now is None else now
        sim = self.sim_ns() if sim is None else sim
        result = {'schema_version': 1, 'kind': 'inspection_result', **action.identity(),
                  'request_sha256': action.digest, 'machine_id': action.machine, 'status': status,
                  'error_code': code, 'started_sim_stamp_ns': action.started_sim, 'finished_sim_stamp_ns': sim,
                  'wall_duration_s': max(0., now - action.started), 'observation': observation, 'state_change': None}
        action.result, action.phase = result, 'terminal'
        self.last_result = result
        self.active = None
        self.guard.end()
        self.on_result(copy.deepcopy(result))

    def state(self):
        self.sequence += 1
        try:
            ready = not self.closing and len(self.actions) < MAX_ACTIONS and bool(self.guard.ready())
        except (ValueError, RuntimeError, OSError):
            ready = False
        return {'schema_version': 1, 'kind': 'inspection_state', 'run_id': self.run_id,
                'sequence': self.sequence, 'ready': ready, 'capacity_remaining': MAX_ACTIONS - len(self.actions),
                'active': None if self.active is None else self.active.view(),
                'last_result': copy.deepcopy(self.last_result)}
