"""Bounded station controls; geometry and permission stay server-side."""
import hashlib

from orbital.inspection_protocol import canonical, decode, finite, identifier, MAX_REPLY_BYTES, MAX_ACTIONS, TERMINAL
from orbital.machines import MACHINE_IDS
from orbital.station_control import control
from orbital.reset_evidence import basis as reset_basis

REJECTION_CODES = frozenset(('invalid_argument', 'wrong_run', 'action_conflict', 'unavailable',
                             'busy', 'history_full', 'unknown_action', 'wrong_station_state',
                             'isolation_required', 'reset_not_eligible', 'reset_attempt_exhausted',
                             'reset_required', 'restore_not_eligible', 'restore_attempt_exhausted',
                             'cancellation_target_required', 'cancellation_conflict', 'target_superseded'))
REJECTION_CODES |= {'mission_reused', 'action_reused', 'journal_full', 'journal_unavailable'}
ERROR_CODES = frozenset(('hold_acquisition_unavailable', 'deadline_exceeded', 'cancel_requested',
    'owner_shutdown', 'inspection_unavailable', 'precondition_failed', 'effect_unknown',
    'station_run_changed', 'isolation_not_verified', 'clock_regressed', 'maintenance_hold_lost',
    'precondition_lost', 'maintenance_evidence_unavailable', 'reset_not_verified', 'restore_not_verified',
    'cancellation_evidence_unavailable', 'cancellation_isolation_failed'))
ERROR_CODES |= {'journal_unavailable'}


def command(text):
    value = decode(text)
    if (set(value) != {'schema_version', 'run_id', 'mission_id', 'action_id', 'command_id', 'op', 'arguments'}
            or type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['op'] not in ('isolate', 'reset', 'restore', 'cancel_isolate', 'cancel', 'query') or not isinstance(value['arguments'], dict)):
        raise ValueError('invalid_argument')
    for name in ('run_id', 'mission_id', 'action_id', 'command_id'):
        identifier(value[name])
    args = value['arguments']
    if value['op'] == 'cancel_isolate':
        if set(args) != {'machine_id', 'station_run_id', 'target_action_id', 'timeout_s'} or args['machine_id'] not in MACHINE_IDS:
            raise ValueError('invalid_argument')
        identifier(args['station_run_id']); identifier(args['target_action_id'])
        if args['target_action_id'] == value['action_id']:
            raise ValueError('invalid_argument')
        args['timeout_s'] = finite(args['timeout_s'], .1, 30.)
    elif value['op'] in ('isolate', 'reset', 'restore'):
        fields = {'machine_id', 'station_run_id', 'control_revision', 'timeout_s'}
        if value['op'] == 'reset': fields.add('isolation_action_id')
        if value['op'] == 'restore': fields.add('reset_action_id')
        if (set(args) != fields
                or args['machine_id'] not in MACHINE_IDS or type(args['control_revision']) is not int
                or not 0 <= args['control_revision'] < 2**63):
            raise ValueError('invalid_argument')
        identifier(args['station_run_id'])
        if value['op'] == 'reset': identifier(args['isolation_action_id'])
        if value['op'] == 'restore': identifier(args['reset_action_id'])
        args['timeout_s'] = finite(args['timeout_s'], .1, 30.)
    elif args:
        raise ValueError('invalid_argument')
    return value


def request_digest(value):
    return hashlib.sha256(canonical({k:v for k,v in value.items() if k != 'command_id'}).encode()).hexdigest()


def integer(value, minimum=0):
    if type(value) is not int or not minimum <= value < 2**63:
        raise ValueError('invalid_integer')
    return value


def observation(value):
    if not isinstance(value, dict) or set(value) != {
            'observation_id', 'manifest_sha256', 'registry_identity', 'sample_stamp_ns', 'camera_stamp_ns'}:
        raise ValueError('invalid_observation_reference')
    identifier(value['observation_id'])
    for key in ('manifest_sha256', 'registry_identity'): identifier(value[key], 64)
    for key in ('sample_stamp_ns', 'camera_stamp_ns'): integer(value[key], 1)


def result(value):
    if isinstance(value, dict) and value.get('operation') == 'cancel_isolate':
        from orbital.cancellation_protocol import result as cancellation_result
        return cancellation_result(value)
    fields = {'schema_version', 'kind', 'run_id', 'mission_id', 'action_id', 'request_sha256', 'operation',
              'machine_id', 'station_run_id', 'status', 'error_code', 'started_sim_stamp_ns',
              'finished_sim_stamp_ns', 'wall_duration_s', 'observation', 'state_change', 'hold_released'}
    operation = value.get('operation') if isinstance(value, dict) else None
    if operation == 'reset': fields |= {'isolation', 'eligibility'}
    if operation == 'restore': fields.add('reset')
    if (not isinstance(value, dict) or set(value) != fields or type(value['schema_version']) is not int
            or (value['schema_version'], operation) not in ((1, 'isolate'), (2, 'reset'), (3, 'restore'))
            or value['kind'] != 'maintenance_result' or value['machine_id'] not in MACHINE_IDS
            or value['status'] not in TERMINAL or value['hold_released'] is not True):
        raise ValueError('invalid_result')
    for key in ('run_id', 'mission_id', 'action_id', 'station_run_id'): identifier(value[key])
    identifier(value['request_sha256'], 64)
    for key in ('started_sim_stamp_ns', 'finished_sim_stamp_ns'): integer(value[key])
    finite(value['wall_duration_s'])
    if value['status'] == 'succeeded':
        if value['error_code'] is not None: raise ValueError('invalid_result')
    elif value['error_code'] not in ERROR_CODES:
        raise ValueError('invalid_result')
    if value['observation'] is not None: observation(value['observation'])
    if operation == 'reset':
        prior = value['isolation']
        if not isinstance(prior, dict) or prior.get('operation') != 'isolate' or prior.get('schema_version') != 1:
            raise ValueError('invalid_reset_prerequisite')
        result(prior)
        basis = reset_basis(value['eligibility'])
        if (any(prior[k] != value[k] for k in ('run_id', 'mission_id', 'machine_id', 'station_run_id'))
                or prior['action_id'] == value['action_id'] or prior['state_change']['disposition'] != 'applied'
                or prior['finished_sim_stamp_ns'] > value['started_sim_stamp_ns']
                or basis['run_id'] != value['station_run_id'] or basis['machine_id'] != value['machine_id']
                or basis['control_revision']+1 != prior['state_change']['effect']['control_revision']
                or basis['registry_identity'] != prior['observation']['registry_identity']
                or basis['samples'][-1]['stamp_ns'] > prior['state_change']['effect']['applied_stamp_ns']
                or not any(v['stamp_ns'] == prior['observation']['sample_stamp_ns'] for v in basis['samples'])
                or (value['observation'] is not None and value['observation']['registry_identity'] != basis['registry_identity'])):
            raise ValueError('invalid_reset_prerequisite')
    if operation == 'restore':
        prior = value['reset']
        if not isinstance(prior, dict) or prior.get('operation') != 'reset' or prior.get('schema_version') != 2:
            raise ValueError('invalid_restore_prerequisite')
        result(prior)  # The operation/version gates bound the chain to three records.
        if (any(prior[k] != value[k] for k in ('run_id', 'mission_id', 'machine_id', 'station_run_id'))
                or value['action_id'] in (prior['action_id'], prior['isolation']['action_id'])
                or prior['state_change']['disposition'] != 'applied'
                or prior['finished_sim_stamp_ns'] > value['started_sim_stamp_ns']
                or (value['observation'] is not None and value['observation']['registry_identity'] !=
                    prior['observation']['registry_identity'])):
            raise ValueError('invalid_restore_prerequisite')
    change = value['state_change']
    if not isinstance(change, dict) or set(change) != {'disposition', 'effect', 'verification'}:
        raise ValueError('invalid_effect_result')
    if change['disposition'] == 'not_applied':
        if change['effect'] is not None or change['verification'] is not None or value['status'] == 'succeeded':
            raise ValueError('invalid_effect_result')
    elif change['disposition'] == 'applied':
        effect, verified = change['effect'], change['verification']
        if not isinstance(effect, dict): raise ValueError('invalid_effect_result')
        control({'revision': effect.get('control_revision'), 'last_applied': effect}, value['finished_sim_stamp_ns'])
        if (effect['operation'] != operation or effect['owner_run_id'] != value['run_id']
                or any(effect[k] != value[k] for k in ('mission_id', 'action_id', 'request_sha256', 'machine_id'))
                or value['observation'] is None):
            raise ValueError('invalid_effect_result')
        verification_fields = {'run_id', 'sequence', 'stamp_ns', 'control_revision',
                'sample_sha256', 'machine_id', 'powered', 'current_a', 'vibration_au'}
        if operation == 'reset': verification_fields.add('temperature_c')
        if operation == 'restore': verification_fields |= {'temperature_c', 'voltage_v'}
        if not isinstance(verified, dict) or set(verified) != verification_fields:
            raise ValueError('invalid_effect_result')
        identifier(verified['sample_sha256'], 64)
        integer(verified['sequence'], 1); integer(verified['stamp_ns'], 1)
        integer(verified['control_revision'], 1)
        if (verified['run_id'] != value['station_run_id'] or verified['machine_id'] != value['machine_id']
                or verified['control_revision'] != effect['control_revision']
                or verified['powered'] is not (operation == 'restore')
                or verified['stamp_ns'] <= effect['applied_stamp_ns']
                or not 0 <= value['finished_sim_stamp_ns']-verified['stamp_ns'] < 1000000000):
            raise ValueError('invalid_effect_result')
        current, vibration = finite(verified['current_a']), finite(verified['vibration_au'])
        if operation != 'restore' and (current != 0 or vibration != 0):
            raise ValueError('invalid_effect_result')
        if operation == 'reset':
            finite(verified['temperature_c'], -273.15, 40.)
            if effect['control_revision'] <= prior['state_change']['effect']['control_revision']:
                raise ValueError('invalid_reset_revision')
        if operation == 'restore':
            finite(verified['temperature_c'], -273.15, 1e6)
            finite(verified['voltage_v'], 0., 10000.)
            if effect['control_revision'] <= prior['state_change']['effect']['control_revision']:
                raise ValueError('invalid_restore_revision')
    else:
        raise ValueError('invalid_effect_result')
    return value


def action_view(value):
    if isinstance(value, dict) and 'cancellation' in value:
        from orbital.cancellation_protocol import action_view as cancellation_view
        return cancellation_view(value)
    if not isinstance(value, dict) or set(value) != {'run_id', 'mission_id', 'action_id', 'request_sha256',
            'machine_id', 'phase', 'stop_reason', 'result'}:
        raise ValueError('invalid_action')
    for key in ('run_id', 'mission_id', 'action_id'): identifier(value[key])
    identifier(value['request_sha256'], 64)
    if value['machine_id'] not in MACHINE_IDS or value['phase'] not in {
            'holding', 'cooling', 'inspecting', 'verifying', 'releasing', 'unresolved', 'terminal'}:
        raise ValueError('invalid_action')
    stop = value['stop_reason']
    if stop is not None and (not isinstance(stop, (list, tuple)) or len(stop) != 2
            or stop[0] not in ('failed', 'cancelled', 'timed_out') or stop[1] not in ERROR_CODES):
        raise ValueError('invalid_action')
    if value['phase'] == 'terminal':
        result(value['result'])
        if any(value[key] != value['result'][key] for key in ('run_id', 'mission_id', 'action_id', 'request_sha256', 'machine_id')):
            raise ValueError('invalid_action')
    elif value['result'] is not None:
        raise ValueError('invalid_action')
    return value


def state(text):
    value = decode(text, MAX_REPLY_BYTES)
    if (set(value) != {'schema_version', 'kind', 'run_id', 'station_run_id', 'control_revision', 'sequence',
                      'sim_stamp_ns', 'ready', 'capacity_remaining', 'active', 'last_result'}
            or type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['kind'] != 'maintenance_state' or type(value['ready']) is not bool):
        raise ValueError('invalid_state')
    identifier(value['run_id']); identifier(value['station_run_id'])
    integer(value['sequence'], 1); integer(value['sim_stamp_ns']); integer(value['control_revision'])
    integer(value['capacity_remaining'])
    if value['capacity_remaining'] > MAX_ACTIONS or (value['ready'] and (value['sim_stamp_ns'] == 0
            or value['capacity_remaining'] == 0 or value['active'] is not None)):
        raise ValueError('invalid_state')
    if value['active'] is not None:
        action_view(value['active'])
        if value['active']['run_id'] != value['run_id'] or value['active']['phase'] == 'terminal':
            raise ValueError('invalid_state')
    if value['last_result'] is not None:
        result(value['last_result'])
        if value['last_result']['run_id'] != value['run_id']: raise ValueError('invalid_state')
    return value
