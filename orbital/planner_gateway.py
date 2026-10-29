"""Serialized, durable tools over the existing mission state and owned clients.

The current executive uses the validated scripted diagnosis policy. This is the
same-contract baseline, not a provider, independent evaluator or OS boundary.
"""
import copy
import threading
import uuid

from orbital.inspection_protocol import canonical
from orbital.planner_history import MAX_CALLS
from orbital.planner_protocol import PlannerError, request, request_digest, result
from orbital.planner_schema import FRESH_TOOLS, MUTATIONS
from orbital.proximity import Unavailable

OPERATIONS = {'inspect_machine': 'inspect', 'isolate_power': 'isolate',
              'reset_machine': 'reset', 'restore_power': 'restore'}


class PlannerGateway:
    def __init__(self, runtime, inputs, *, max_calls=MAX_CALLS):
        self.runtime, self.inputs = runtime, inputs
        self.serial = threading.Lock()
        self.stops = threading.Lock()
        self.queued_stop = self.busy_request = None
        self.stop_delivery_failed = False
        self.current = None
        runtime._record('tool_session', {'run_id': inputs.run_id,
            'mode': 'scripted-tools-v1', 'max_calls': max_calls})

    def state(self):
        self.runtime.journal.verify()
        return self.runtime.journal.tools.get(self.runtime.mission_id)

    def bind(self, family, command):
        if self.current is None:raise PlannerError('unavailable')
        self.runtime._record('tool_binding', {'action_id': self.current['request']['action_id'],
                                             'family': family, 'command': command})

    def before_publish(self, family, command):
        if self.current is None:raise PlannerError('unavailable')
        self.check_deadline()
        if not self.current['entered']:
            try:self.inputs.current(self.current['request']['observation_id'])
            except PlannerError:
                self.current['rejection'] = 'stale_observation'
                raise
            self.current['entered'] = True

    def check_deadline(self):
        if self.current is not None and self.current['request']['tool'] != 'stop':
            if self.runtime.now() >= self.current['deadline_wall_s']:
                self.current['expired'] = True
                self.runtime.mission.fail('planner_tool_deadline')
                raise PlannerError('unavailable')

    def _record(self, kind, data):
        return {'schema_version': 1, 'kind': kind, 'run_id': self.inputs.run_id,
            'mission_id': self.inputs.mission_id, 'observation_id': uuid.uuid4().hex,
            'observed_sim_stamp_ns': self.runtime.sim_ns(), 'data': copy.deepcopy(data),
            'scope': 'Source-checked executive/owner observations; not independent physical or scenario acceptance.'}

    def _preflight(self, value):
        runtime, mission = self.runtime, self.runtime.mission
        tool, args = value['tool'], value['arguments']
        if tool in FRESH_TOOLS:
            phase = 'inspect' if tool in ('navigate_to', 'inspect_machine') else 'verify' if tool == 'verify_recovery' else 'intervene'
            mission._available(phase)
            if tool == 'navigate_to':runtime.policy.next_machine(args['waypoint_id'])
            elif args['machine_id'] != mission.arrival:raise PlannerError('precondition_failed')
            if tool == 'inspect_machine' and (runtime.policy.plan is None
                    or runtime.policy.plan['machine_id'] != args['machine_id']):
                raise PlannerError('precondition_failed')
            if tool in MUTATIONS and mission.next_operation != OPERATIONS[tool]:
                raise PlannerError('precondition_failed')
            runtime.transport.fence()
            runtime._fresh_sources()
        elif tool == 'report_outcome':
            expected = 'escalated' if mission.phase == 'escalate' else mission.outcome
            if args['outcome'] != expected:raise PlannerError('precondition_failed')
            state = self.state()
            if expected in ('healthy','recovered') and any(c['response'] is None or
                    c['response']['result']['status'] != 'succeeded' for c in state['calls'].values()):
                raise PlannerError('precondition_failed')
            if any(v not in state['observations'] for v in args['evidence_ids']):
                raise PlannerError('precondition_failed')
        elif tool != 'stop' and mission.outcome is not None:raise PlannerError('unavailable')
        self.inputs.validate(canonical(value))

    def _stop(self):
        runtime = self.runtime
        runtime.mission.cancel()
        runtime._settle()
        if runtime.mission.pending is not None:raise PlannerError('unresolved')
        quiet = runtime.quiescence()
        return self._record('planner_stop_observation', {
            'navigation_run_id': quiet['navigation']['run_id'],
            'inspection_run_id': quiet['inspection']['run_id'],
            'maintenance_run_id': quiet['maintenance']['run_id'],
            'control_revision': runtime.mission.control['revision'],
            'guard_latched': True, 'owners_idle': True})

    def _perform(self, value):
        runtime, mission = self.runtime, self.runtime.mission
        tool, args = value['tool'], value['arguments']
        if tool == 'observe_station':return [self.inputs.view()]
        if tool == 'navigate_to':
            runtime.navigate(args['waypoint_id'], timeout_s=min(args['timeout_s'],self.current['deadline_wall_s']-runtime.now()))
        elif tool in OPERATIONS:
            runtime.action(OPERATIONS[tool], args['machine_id'],
                timeout_s=min(args['timeout_s'],self.current['deadline_wall_s']-runtime.now()), ready_only=True)
            if tool == 'inspect_machine' and mission.phase == 'diagnose':
                runtime._record('decision', runtime.policy.apply())
        elif tool == 'verify_recovery':
            deadline = self.current['deadline_wall_s']
            while runtime.now() < deadline and mission.outcome is None:
                runtime._poll()
                if mission.stop_reason:break
                if runtime.policy.high_vibration:raise PlannerError('precondition_failed')
                try: observed = mission.recovery.measure(runtime.now(), runtime.sim_ns())
                except Unavailable:continue
                if (observed['verified'] and observed['window']['duration_s'] >= args['observation_window_s']
                        and mission.verify()):
                    return [self._record('planner_recovery_observation', {
                        'station_run_id': mission.station_run, 'control_revision': mission.control['revision'],
                        'machine_id': args['machine_id'], 'window': observed['window'],
                        'required_window_s': args['observation_window_s']})]
            raise PlannerError('unavailable')
        elif tool == 'stop':return [self._stop()]
        elif tool == 'report_outcome':
            if mission.phase == 'escalate':mission.escalate()
            if mission.outcome != args['outcome']:raise PlannerError('precondition_failed')
            runtime.quiescence()
            return []
        return None  # Action evidence is derived from the exact bound native step.

    def _step(self, call):
        if not call['bindings']:return None
        binding = call['bindings'][0]; command = binding['command']
        key = ('navigation:'+command['run_id']+':'+str(command['sequence'])
               if binding['family'] == 'navigation' else command['action_id'])
        return self.runtime.mission.actions.get(key)

    def _finish(self, action_id, observations=None, *, failed=False):
        runtime = self.runtime
        call = self.state()['calls'][action_id]; value = call['request']; tool = value['tool']
        status, code, effect, change = ('failed', 'unavailable', 'none', None) if failed else ('succeeded', None, 'none', None)
        observations = [] if observations is None else observations
        if tool in OPERATIONS or tool == 'navigate_to':
            step = self._step(call)
            if step is None:
                if runtime.mission.pending is not None:raise PlannerError('unresolved')
                status, code = 'rejected', 'unavailable'
            elif step['result'] is None:raise PlannerError('unresolved')
            else:
                native = step['result']
                if native.get('kind') in ('action_delivery', 'navigation_delivery'):
                    status, code = 'rejected', 'precondition_failed'
                elif tool == 'navigate_to':
                    arrived = native['outcome'] == 'arrived'
                    status = 'succeeded' if arrived else 'cancelled' if native['outcome'] == 'cancelled' else 'failed'
                    code = None if arrived else 'cancel_requested' if status == 'cancelled' else 'unreachable'
                    observations = [self._record('planner_navigation_observation', {'target': native['target'],
                        'arrived': arrived, 'within_tolerance': native.get('measured', {}).get('within_tolerance') is True})]
                else:
                    status = native['status']
                    code = {'succeeded': None, 'cancelled': 'cancel_requested',
                            'timed_out': 'deadline_exceeded', 'failed': 'unavailable'}[status]
                    observations = [self._record('planner_'+step['family']+'_observation', native)]
                    if tool in MUTATIONS and native['state_change']['disposition'] == 'applied':
                        effect = 'reported'
                        change = {'machine_id': native['machine_id'], 'operation': tool,
                            'control_revision': native['state_change']['effect']['control_revision'],
                            'powered': native['state_change']['verification']['powered']}
                if status != 'rejected':
                    try:observations.append(self.inputs.view())
                    except PlannerError:pass  # Historical owner result does not promise a fresh public sample.
        if tool != 'stop' and runtime.now() >= call['deadline_wall_s']:
            status, code = 'timed_out', 'deadline_exceeded'
            runtime.failure = runtime.failure or 'planner_tool_deadline'
            runtime.mission.fail('planner_tool_deadline')
        elif status == 'rejected' and self.current is not None and self.current['request']['action_id'] == action_id:
            code = self.current.get('rejection', code)
        response = {'result': {'schema_version': 1, 'kind': 'planner_tool_result',
            **{k: value[k] for k in ('run_id', 'mission_id', 'action_id', 'tool')},
            'request_sha256': request_digest(value), 'status': status, 'error_code': code,
            'started_sim_stamp_ns': call['started_sim_ns'], 'finished_sim_stamp_ns': runtime.sim_ns(),
            'wall_duration_s': runtime.now()-call['started_wall_s'],
            'observation_ids': [v['observation_id'] for v in observations], 'effect': effect, 'state_change': change},
            'observations': observations}
        result(canonical(response['result']), original=value)
        runtime._record('tool_result', response)
        return copy.deepcopy(response)

    def _execute(self, value):
        try:state = self.state()
        except (ValueError, OSError):
            if value['tool'] == 'stop':self._stop()
            raise PlannerError('unavailable') from None
        prior = state['calls'].get(value['action_id'])
        if prior is not None:
            if request_digest(value) != request_digest(prior['request']):raise PlannerError('action_conflict')
            if prior['response'] is None:raise PlannerError('unresolved')
            return {'schema_version': 1, 'kind': 'planner_tool_reply', 'replayed': True, **prior['response']}
        pending = [key for key, call in state['calls'].items() if call['response'] is None]
        if pending and (value['tool'] != 'stop' or any(state['calls'][k]['request']['tool'] == 'stop' for k in pending)):
            raise PlannerError('busy')
        if len(state['calls']) >= state['max_calls']+int(value['tool'] == 'stop'):
            raise PlannerError('budget_exhausted')
        try:self._preflight(value)
        except PlannerError:raise
        except (ValueError, OSError, RuntimeError):
            raise PlannerError('precondition_failed') from None
        started = self.runtime.now()
        budget = value['arguments'].get('timeout_s',
            value['arguments'].get('observation_window_s',0.)+5.)
        deadline = started+budget if value['tool'] == 'stop' else min(self.runtime.deadline,started+budget)
        self.runtime._record('tool_intent', {'request': value,
            'started_wall_s': started, 'started_sim_ns': self.runtime.sim_ns(), 'deadline_wall_s':deadline})
        self.current = {'request': value, 'entered': False, 'expired':False, 'deadline_wall_s':deadline}
        failed = False; observations = None
        try:
            self.check_deadline()
            observations = self._perform(value)
        except (ValueError, OSError, RuntimeError, KeyboardInterrupt):
            failed = True
            self.runtime.failure = self.runtime.failure or 'planner_tool_execution_unavailable'
            self.runtime.mission.fail(self.runtime.failure)
            self.runtime._settle()
        if value['tool'] == 'stop':
            for key in pending:self._finish(key, failed=True)
        response = self._finish(value['action_id'], observations, failed=failed)
        return {'schema_version': 1, 'kind': 'planner_tool_reply', 'replayed': False, **response}

    def execute(self, text):
        value = request(text, run_id=self.inputs.run_id, mission_id=self.inputs.mission_id)
        while not self.serial.acquire(blocking=False):
            if value['tool'] != 'stop':raise PlannerError('busy')
            with self.stops:
                if not self.serial.locked():continue
                known = self.runtime.journal.tools.get(self.runtime.mission_id)['calls'].get(value['action_id'])
                identities = [v for v in (self.busy_request,
                    None if known is None else known['request']) if v is not None]
                if any(v['action_id'] == value['action_id'] and request_digest(v) != request_digest(value)
                       for v in identities):raise PlannerError('action_conflict')
                if self.queued_stop is not None and self.queued_stop != value:
                    raise PlannerError('action_conflict' if self.queued_stop['action_id'] == value['action_id'] else 'busy')
                self.queued_stop = copy.deepcopy(value)
                self.runtime.stop_signal.set()
                # This is an immediate in-memory cancellation request, never a
                # durable admission, terminal result or proof of physical stop.
                return {'schema_version': 1, 'kind': 'planner_tool_pending',
                    **{k:value[k] for k in ('run_id','mission_id','action_id')},
                    'request_sha256': request_digest(value), 'reason':'cancellation_requested',
                    'terminal':False, 'durable':False}
        with self.stops:self.busy_request = value
        try:
            return self._execute(value)
        finally:
            self.current = None
            with self.stops:
                queued = self.queued_stop
                if queued is None:
                    self.busy_request = None
                    self.serial.release()
            if queued is not None:
                try:self._execute(queued)
                except (ValueError, OSError, RuntimeError, KeyboardInterrupt):
                    self.stop_delivery_failed = True
                finally:
                    with self.stops:
                        self.current = self.busy_request = self.queued_stop = None
                        self.serial.release()
