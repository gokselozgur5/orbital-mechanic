"""Strict retained scenario/action history; never live permission or state adoption."""
import copy

from orbital.inspection_protocol import canonical, identifier, MAX_ACTIONS
from orbital import maintenance_protocol as protocol
from orbital.station_control import control

MAX_SCENARIOS = 256


class History:
    def __init__(self):
        self.scenarios, self.missions, self.actions, self.reserved, self.budgets = {}, {}, {}, {}, {}
        self.current = None

    def apply(self, kind, value):
        if not isinstance(value, dict): raise ValueError('invalid_journal_event')
        fields = {
            'scenario': {'scenario_id', 'registry_identity'},
            'scenario_v2': {'scenario_id', 'registry_identity', 'configuration_sha256'},
            'claim': {'scenario_id', 'run_id', 'station_run_id'},
            'admit': {'scenario_id', 'request', 'child_id', 'parent_action_id'},
            'intent': {'scenario_id', 'action_id', 'context', 'operation', 'station_run_id',
                       'control_revision', 'isolation_action_id', 'reset_action_id'},
            'applied': {'scenario_id', 'action_id', 'effect'},
            'rejected': {'scenario_id', 'action_id'},
            'terminal': {'scenario_id', 'result'},
            'observations_closed': {'scenario_id','station_run_id','chain_sha256','records','source_revision'},
        }
        if kind not in fields or set(value) != fields[kind]: raise ValueError('invalid_journal_event')
        scenario_id = identifier(value['scenario_id'])
        if kind in ('scenario', 'scenario_v2'):
            identifier(value['registry_identity'], 64)
            if kind == 'scenario_v2': identifier(value['configuration_sha256'], 64)
            if scenario_id in self.scenarios or len(self.scenarios) >= MAX_SCENARIOS:
                raise ValueError('scenario_conflict_or_capacity')
            self.scenarios[scenario_id] = {**value, 'claim': None, 'actions': [], 'control_revision': 0}
            self.current = scenario_id
            return
        if scenario_id != self.current or scenario_id not in self.scenarios:
            raise ValueError('wrong_journal_scenario')
        scenario = self.scenarios[scenario_id]
        if kind == 'claim':
            identifier(value['run_id']); identifier(value['station_run_id'])
            if scenario['claim'] is not None: raise ValueError('scenario_already_claimed')
            if any(s['claim'] is not None and any(s['claim'][k] == value[k]
                    for k in ('run_id', 'station_run_id')) for s in self.scenarios.values()):
                raise ValueError('owner_identity_reused')
            scenario['claim'] = copy.deepcopy(value)
            return
        if scenario['claim'] is None: raise ValueError('scenario_not_claimed')
        if 'observations_closed' in scenario: raise ValueError('scenario_observations_closed')
        if kind == 'observations_closed':
            if ('configuration_sha256' not in scenario or value['station_run_id'] != scenario['claim']['station_run_id']
                    or type(value['records']) is not int or not 2 <= value['records'] <= 20000
                    or any(self.actions[a]['result'] is None for a in scenario['actions'])):
                raise ValueError('invalid_observation_closure')
            identifier(value['chain_sha256'],64);identifier(value['source_revision'],64)
            scenario['observations_closed'] = copy.deepcopy(value)
            return
        if kind == 'admit':
            self._admit(value, scenario)
            return
        result = protocol.result(value['result']) if kind == 'terminal' else None
        action_id = identifier(result['action_id'] if result is not None else value['action_id'])
        action = self.actions.get(action_id)
        if action is None or action['scenario_id'] != scenario_id or action['result'] is not None:
            raise ValueError('journal_action_unavailable')
        if kind == 'intent': self._intent(value, action, scenario)
        elif kind == 'terminal': self._terminal(result, action)
        else: self._outcome(kind, value, action)

    def _admit(self, value, scenario):
        request = protocol.command(canonical(value['request']))
        if canonical(request) != canonical(value['request']) or request['op'] not in ('isolate', 'reset', 'restore', 'cancel_isolate'):
            raise ValueError('invalid_journal_request')
        if request['run_id'] != scenario['claim']['run_id'] or request['arguments']['station_run_id'] != scenario['claim']['station_run_id']:
            raise ValueError('journal_owner_mismatch')
        if request['op'] != 'cancel_isolate' and request['arguments']['control_revision'] != scenario['control_revision']:
            raise ValueError('journal_revision_mismatch')
        mission, action_id = request['mission_id'], request['action_id']
        if self.missions.get(mission, self.current) != self.current: raise ValueError('mission_reused')
        if action_id in self.actions: raise ValueError('action_reused')
        parent, child = value['parent_action_id'], value['child_id']
        if parent is not None:
            identifier(parent)
            prior = self.actions.get(parent)
            if (self.reserved.get(action_id) != parent or prior is None
                    or prior['scenario_id'] != self.current or prior['request']['op'] != 'cancel_isolate'
                    or request['op'] != 'isolate' or prior['result'] is not None
                    or prior['request']['mission_id'] != mission
                    or prior['request']['arguments']['machine_id'] != request['arguments']['machine_id']):
                raise ValueError('invalid_reserved_child')
        elif action_id in self.reserved:
            raise ValueError('action_reused')
        if child is not None:
            identifier(child)
            target = self.actions.get(request['arguments'].get('target_action_id'))
            if (request['op'] != 'cancel_isolate' or child in self.actions or child in self.reserved
                    or child == action_id or target is None or target['scenario_id'] != self.current
                    or target['request']['op'] == 'cancel_isolate'
                    or target['request']['mission_id'] != mission
                    or target['request']['arguments']['machine_id'] != request['arguments']['machine_id']):
                raise ValueError('invalid_cancellation_reservation')
        elif request['op'] == 'cancel_isolate':
            raise ValueError('missing_cancellation_reservation')
        used = len(scenario['actions']) + sum(self.actions[p]['scenario_id'] == self.current for p in self.reserved.values())
        if used + 1 + (child is not None) - (parent is not None) > MAX_ACTIONS:
            raise ValueError('history_full')
        self.missions[mission] = self.current
        self.actions[action_id] = {**copy.deepcopy(value), 'intent': None, 'effect': None, 'rejected': False, 'result': None}
        scenario['actions'].append(action_id)
        if parent is not None: del self.reserved[action_id]
        if child is not None: self.reserved[child] = action_id

    def _intent(self, value, action, scenario):
        request, context = action['request'], value['context']
        expected_context = {'owner_run_id': request['run_id'], 'mission_id': request['mission_id'],
            'action_id': request['action_id'], 'request_sha256': protocol.request_digest(request),
            'machine_id': request['arguments']['machine_id']}
        if (context != expected_context or value['operation'] != request['op']
                or request['op'] not in ('isolate', 'reset', 'restore')
                or value['station_run_id'] != scenario['claim']['station_run_id']
                or type(value['control_revision']) is not int
                or value['control_revision'] != request['arguments']['control_revision']
                or value['control_revision'] != scenario['control_revision']
                or action['intent'] is not None):
            raise ValueError('journal_effect_conflict')
        for key in ('isolation_action_id', 'reset_action_id'):
            if value[key] != request['arguments'].get(key): raise ValueError('journal_effect_conflict')
        operation = request['op']
        if operation in ('reset', 'restore'):
            key = (self.current, context['machine_id'], operation)
            if key in self.budgets: raise ValueError('journal_attempt_exhausted')
            prior_id = value['isolation_action_id' if operation == 'reset' else 'reset_action_id']
            prior = self.actions.get(prior_id)
            if (prior is None or prior['effect'] is None or prior['scenario_id'] != self.current
                    or prior['request']['op'] != ('isolate' if operation == 'reset' else 'reset')
                    or any(prior['effect'][k] != context[k] for k in ('owner_run_id', 'mission_id', 'machine_id'))):
                raise ValueError('journal_prior_effect_required')
            self.budgets[key] = request['action_id']
        action['intent'] = copy.deepcopy(value)

    def _outcome(self, kind, value, action):
        if action['intent'] is None or action['effect'] is not None or action['rejected']:
            raise ValueError('journal_effect_outcome_conflict')
        intent = action['intent']
        if kind == 'applied':
            effect = value['effect']
            if not isinstance(effect, dict): raise ValueError('invalid_journal_effect')
            control({'revision': effect.get('control_revision'), 'last_applied': effect}, effect.get('applied_stamp_ns'))
            if (any(effect[k] != v for k, v in intent['context'].items())
                    or effect['operation'] != intent['operation']
                    or effect['control_revision'] != intent['control_revision']+1):
                raise ValueError('journal_effect_outcome_conflict')
            action['effect'] = copy.deepcopy(effect)
            self.scenarios[self.current]['control_revision'] = effect['control_revision']
        else:
            action['rejected'] = True
            if intent['operation'] in ('reset', 'restore'):
                del self.budgets[(self.current, intent['context']['machine_id'], intent['operation'])]

    def _terminal(self, result, action):
        request = action['request']
        if (any(result[k] != request[k] for k in ('run_id', 'mission_id', 'action_id'))
                or result['request_sha256'] != protocol.request_digest(request)
                or result['operation'] != request['op']
                or result['station_run_id'] != request['arguments']['station_run_id']
                or result['machine_id'] != request['arguments']['machine_id']):
            raise ValueError('journal_terminal_conflict')
        if request['op'] == 'cancel_isolate':
            original = self.actions[request['arguments']['target_action_id']]
            child = self.actions.get(action['child_id'])
            if (result['request'] != request or result['target_request'] != original['request']
                    or result['target_result'] != original['result']
                    or result['isolation_action_id'] != action['child_id']
                    or (child is not None and result['isolation_request'] != child['request'])
                    or result['isolation_result'] != (None if child is None else child['result'])):
                raise ValueError('journal_terminal_conflict')
        elif result['state_change']['effect'] != action['effect'] or (
                action['intent'] is not None and action['effect'] is None and not action['rejected']):
            raise ValueError('journal_terminal_conflict')
        if request['op'] in ('reset', 'restore'):
            key, argument = ('isolation', 'isolation_action_id') if request['op'] == 'reset' else ('reset', 'reset_action_id')
            prior = self.actions.get(request['arguments'][argument])
            if prior is None or prior['result'] != result[key]: raise ValueError('journal_terminal_conflict')
        action['result'] = copy.deepcopy(result)

    def status(self):
        unresolved = []
        for key, action in self.actions.items():
            if action['result'] is not None: continue
            state = ('applied_without_terminal_record' if action['effect'] is not None else
                     'rejected_without_terminal_record' if action['rejected'] else
                     'effect_intent_without_outcome' if action['intent'] is not None else 'admitted_without_effect_intent')
            unresolved.append({'action_id': key, 'mission_id': action['request']['mission_id'],
                'scenario_id': action['scenario_id'], 'operation': action['request']['op'], 'retained_state': state})
        current = self.scenarios.get(self.current)
        return {'schema_version': 1, 'kind': 'maintenance_history', 'historical_only': True,
            'current_scenario': None if current is None else {k:copy.deepcopy(current[k]) for k in ('scenario_id','registry_identity','claim')},
            'scenario_count': len(self.scenarios), 'mission_count': len(self.missions), 'action_count': len(self.actions),
            'unresolved_count': len(unresolved), 'unresolved': unresolved[-MAX_ACTIONS:],
            'unresolved_truncated': len(unresolved) > MAX_ACTIONS,
            'restart_requires_new_scenario': current is not None and current['claim'] is not None}
