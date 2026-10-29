"""Replayable logical tool ownership inside the existing fsynced mission journal."""
import copy

from orbital import inspection_protocol, maintenance_protocol, planner_protocol
from orbital.inspection_protocol import canonical, finite, identifier
from orbital.navigation_command import CommandGate
from orbital.machines import MACHINE_IDS
from orbital.planner_evidence import observation

KINDS = frozenset(('tool_session', 'tool_intent', 'tool_binding', 'tool_result'))
MAX_CALLS = 32


class ToolHistory:
    def __init__(self):
        self._sessions = {}
        self._last_command = None
        self.stop_owner = None
        self.audit_ids = set()

    def get(self, mission_id):
        return copy.deepcopy(self._sessions.get(mission_id))

    def operator_reset(self, audit_id, data):
        if (self.stop_owner is None or data != {'stop_mission_id': self.stop_owner, 'operator_requested': True}
                or type(data.get('operator_requested')) is not bool
                or audit_id in self._sessions or audit_id in self.audit_ids):
            raise ValueError('invalid_planner_stop_reset')
        new = copy.copy(self)
        new.stop_owner = None
        new.audit_ids = self.audit_ids | {audit_id}
        return new

    def advance(self, mission_id, kind, data, context):
        try:return self._advance(mission_id, kind, data, context)
        except (KeyError, TypeError, AttributeError, OverflowError, RecursionError):
            raise ValueError('invalid_planner_history') from None

    def _advance(self, mission_id, kind, data, context):
        """Copy only changed history; failed append must not mutate admitted state."""
        if kind == 'command':
            new = copy.copy(self); new._last_command = copy.deepcopy(data)
            return new
        if kind == 'terminal':
            state = self._sessions.get(mission_id)
            if state is not None:
                if any(c['response'] is None for c in state['calls'].values()):
                    raise ValueError('planner_call_still_owned')
                if data['outcome'] in ('healthy', 'recovered', 'escalated') and state['reported'] != data['outcome']:
                    raise ValueError('planner_outcome_not_reported')
            return self
        if kind not in KINDS:return self
        if kind == 'tool_session':
            if (mission_id in self._sessions or set(data) != {'run_id', 'mode', 'max_calls'}
                    or data['mode'] != 'scripted-tools-v1' or type(data['max_calls']) is not int
                    or not 1 <= data['max_calls'] <= MAX_CALLS):
                raise ValueError('invalid_planner_session')
            identifier(data['run_id'])
            if any(s['run_id'] == data['run_id'] for s in self._sessions.values()):
                raise ValueError('planner_run_reused')
            state = {**data, 'calls': {}, 'observations': {}, 'reported': None}
        else:
            state = self.get(mission_id)
            if state is None:raise ValueError('planner_session_missing')
            calls = state['calls']
            if kind == 'tool_intent':
                if set(data) != {'request', 'started_wall_s', 'started_sim_ns', 'deadline_wall_s'}:
                    raise ValueError('invalid_tool_intent')
                value = planner_protocol.request(canonical(data['request']),
                    run_id=state['run_id'], mission_id=mission_id)
                action = value['action_id']; stop = value['tool'] == 'stop'
                if action in calls:raise ValueError('planner_call_identity_reused')
                pending = [c for c in calls.values() if c['response'] is None]
                if pending and (not stop or any(c['request']['tool'] == 'stop' for c in pending)):
                    raise ValueError('planner_call_still_owned')
                if len(calls) >= state['max_calls'] + int(stop):raise ValueError('planner_call_limit')
                finite(data['started_wall_s'], context['started_wall_s'], float('inf'))
                finite(data['deadline_wall_s']-data['started_wall_s'], 0., 600.)
                budget = value['arguments'].get('timeout_s',value['arguments'].get('observation_window_s',0.)+5.)
                ceiling = data['started_wall_s']+budget
                if not stop:ceiling = min(ceiling,context['deadline_wall_s'])
                if data['deadline_wall_s'] > ceiling:raise ValueError('planner_deadline_extended')
                maintenance_protocol.integer(data['started_sim_ns'], context['started_sim_ns'])
                calls[action] = {**copy.deepcopy(data), 'bindings': [], 'response': None}
            elif kind == 'tool_binding':
                if set(data) != {'action_id', 'family', 'command'}:raise ValueError('invalid_tool_binding')
                call = calls.get(data['action_id'])
                if call is None or call['response'] is not None:raise ValueError('planner_call_not_pending')
                if self._last_command != {k: data[k] for k in ('family', 'command')}:
                    raise ValueError('planner_binding_without_prior_command')
                req = call['request']; command = data['command']; family = data['family']
                if family == 'navigation':
                    gate = CommandGate(command['run_id'], MACHINE_IDS)
                    gate.sequence = maintenance_protocol.integer(command['sequence'], 1)-1
                    gate.decode(canonical(command))
                    expected = 'request' if not call['bindings'] else 'arm'
                    if (req['tool'] != 'navigate_to' or len(call['bindings']) >= 2 or command['op'] != expected
                            or (command.get('waypoint') if expected == 'request' else command['target']['waypoint'])
                               != req['arguments']['waypoint_id']):
                        raise ValueError('planner_navigation_binding')
                else:
                    protocol = inspection_protocol if family == 'inspection' else maintenance_protocol
                    command = protocol.command(canonical(command))
                    tool = {'inspect': 'inspect_machine', 'isolate': 'isolate_power',
                            'reset': 'reset_machine', 'restore': 'restore_power'}.get(command['op'])
                    if (family not in ('inspection', 'maintenance') or call['bindings'] or tool != req['tool']
                            or command['mission_id'] != mission_id
                            or command['arguments']['machine_id'] != req['arguments']['machine_id']
                            or command['arguments']['timeout_s'] > req['arguments']['timeout_s']):
                        raise ValueError('planner_action_binding')
                call['bindings'].append(copy.deepcopy(data))
            elif kind == 'tool_result':
                if set(data) != {'result', 'observations'}:raise ValueError('invalid_tool_response')
                action = data['result'].get('action_id'); call = calls.get(action)
                if call is None or call['response'] is not None:raise ValueError('planner_call_not_pending')
                result = planner_protocol.result(canonical(data['result']), original=call['request'])
                if (result['started_sim_stamp_ns'] != call['started_sim_ns']
                        or not isinstance(data['observations'], list) or len(data['observations']) > 3):
                    raise ValueError('invalid_tool_response')
                records = [observation(v, baseline=context['baseline'], run_id=state['run_id'],
                                       mission_id=mission_id) for v in data['observations']]
                ids = [v['observation_id'] for v in records]
                if len(ids) != len(set(ids)) or ids != result['observation_ids']:
                    raise ValueError('planner_observation_references')
                self._execution(call, result, records, calls)
                for item in records:
                    key = item['observation_id']; prior = state['observations'].get(key)
                    if prior is not None and prior != item:
                        # A packet ID identifies its measurement, not query-time
                        # age metadata. Cached call responses stay unchanged.
                        ignored = {'freshness', 'observed_sim_stamp_ns'}
                        if (item['kind'] != 'planner_observation'
                                or {k:v for k,v in item.items() if k not in ignored} !=
                                   {k:v for k,v in prior.items() if k not in ignored}
                                or item['observed_sim_stamp_ns'] < prior['observed_sim_stamp_ns']
                                or item['freshness']['received_age_s'] < prior['freshness']['received_age_s']):
                            raise ValueError('planner_observation_conflict')
                    state['observations'][key] = item
                if call['request']['tool'] == 'report_outcome' and result['status'] == 'succeeded':
                    if any(k not in state['observations'] for k in call['request']['arguments']['evidence_ids']):
                        raise ValueError('planner_claim_unknown_evidence')
                    state['reported'] = call['request']['arguments']['outcome']
                call['response'] = copy.deepcopy(data)
        new = copy.copy(self); new._sessions = {**self._sessions, mission_id: state}
        if kind == 'tool_intent' and data['request']['tool'] == 'stop':new.stop_owner = mission_id
        return new

    @staticmethod
    def _execution(call, result, records, calls):
        req = call['request']; tool = req['tool']; bindings = call['bindings']
        owners = [v for v in records if v['kind'] != 'planner_observation']
        operations = {'inspect_machine': 'inspect', 'isolate_power': 'isolate',
                      'reset_machine': 'reset', 'restore_power': 'restore'}
        if tool in operations or tool == 'navigate_to':
            if result['status'] == 'succeeded' and not bindings:
                raise ValueError('planner_result_without_binding')
            if not owners:
                if result['status'] == 'succeeded' or result['effect'] != 'none':
                    raise ValueError('planner_result_without_owner_evidence')
                return
            if len(owners) != 1 or not bindings:raise ValueError('planner_owner_evidence_mismatch')
            command = bindings[0]['command']; native = owners[0]['data']
            if tool == 'navigate_to':
                target = native['target']
                if (owners[0]['kind'] != 'planner_navigation_observation'
                        or target['run_id'] != command['run_id'] or target['sequence'] != command['sequence']
                        or target['waypoint'] != req['arguments']['waypoint_id']
                        or (len(bindings) == 2 and target != bindings[1]['command']['target'])
                        or (result['status'] == 'succeeded' and (len(bindings) != 2 or not native['arrived']))):
                    raise ValueError('planner_owner_evidence_mismatch')
            else:
                family = 'inspection' if tool == 'inspect_machine' else 'maintenance'
                protocol = inspection_protocol if family == 'inspection' else maintenance_protocol
                if (owners[0]['kind'] != 'planner_'+family+'_observation'
                        or any(native[k] != command[k] for k in ('run_id', 'mission_id', 'action_id'))
                        or native['request_sha256'] != protocol.request_digest(command)
                        or native['machine_id'] != req['arguments']['machine_id']
                        or (result['status'] == 'succeeded' and native['status'] != 'succeeded')):
                    raise ValueError('planner_owner_evidence_mismatch')
                if family == 'maintenance':
                    applied = native['state_change']['disposition'] == 'applied'
                    if result['effect'] != ('reported' if applied else 'none'):
                        raise ValueError('planner_effect_evidence_mismatch')
                    if applied and result['state_change'] != {'machine_id': native['machine_id'],
                            'operation': tool, 'control_revision': native['state_change']['effect']['control_revision'],
                            'powered': native['state_change']['verification']['powered']}:
                        raise ValueError('planner_effect_evidence_mismatch')
        elif result['status'] == 'succeeded':
            if tool == 'observe_station' and (len(records) != 1 or owners):
                raise ValueError('planner_public_evidence_missing')
            if tool == 'verify_recovery' and (len(owners) != 1 or owners[0]['kind'] != 'planner_recovery_observation'
                    or owners[0]['data']['machine_id'] != req['arguments']['machine_id']
                    or owners[0]['data']['required_window_s'] != req['arguments']['observation_window_s']):
                raise ValueError('planner_recovery_evidence_missing')
            if tool == 'stop' and (len(owners) != 1 or owners[0]['kind'] != 'planner_stop_observation'
                    or any(c['response'] is None for key,c in calls.items() if key != req['action_id'])):
                raise ValueError('planner_stop_still_owned')
            if tool == 'report_outcome' and records:raise ValueError('planner_claim_has_fabricated_observations')
