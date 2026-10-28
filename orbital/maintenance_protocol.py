"""Bounded isolation requests; all effect geometry and permission stay server-side."""
import hashlib

from orbital.inspection_protocol import canonical, decode, finite, identifier, MAX_REPLY_BYTES, MAX_ACTIONS, TERMINAL
from orbital.machines import MACHINE_IDS
from orbital.station_control import control

REJECTION_CODES = frozenset(('invalid_argument', 'wrong_run', 'action_conflict', 'unavailable',
                             'busy', 'history_full', 'unknown_action', 'wrong_station_state'))
ERROR_CODES = frozenset(('hold_acquisition_unavailable', 'deadline_exceeded', 'cancel_requested',
    'owner_shutdown', 'inspection_unavailable', 'precondition_failed', 'effect_unknown',
    'station_run_changed', 'isolation_not_verified', 'clock_regressed', 'maintenance_hold_lost',
    'precondition_lost', 'maintenance_evidence_unavailable'))


def command(text):
    value = decode(text)
    if (set(value) != {'schema_version', 'run_id', 'mission_id', 'action_id', 'command_id', 'op', 'arguments'}
            or type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['op'] not in ('isolate', 'cancel', 'query') or not isinstance(value['arguments'], dict)):
        raise ValueError('invalid_argument')
    for name in ('run_id', 'mission_id', 'action_id', 'command_id'):
        identifier(value[name])
    args = value['arguments']
    if value['op'] == 'isolate':
        if (set(args) != {'machine_id', 'station_run_id', 'control_revision', 'timeout_s'}
                or args['machine_id'] not in MACHINE_IDS or type(args['control_revision']) is not int
                or not 0 <= args['control_revision'] < 2**63):
            raise ValueError('invalid_argument')
        identifier(args['station_run_id'])
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
    fields = {'schema_version', 'kind', 'run_id', 'mission_id', 'action_id', 'request_sha256', 'operation',
              'machine_id', 'station_run_id', 'status', 'error_code', 'started_sim_stamp_ns',
              'finished_sim_stamp_ns', 'wall_duration_s', 'observation', 'state_change', 'hold_released'}
    if (not isinstance(value, dict) or set(value) != fields or type(value['schema_version']) is not int
            or value['schema_version'] != 1 or value['kind'] != 'maintenance_result'
            or value['operation'] != 'isolate' or value['machine_id'] not in MACHINE_IDS
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
        if (effect['operation'] != 'isolate' or effect['owner_run_id'] != value['run_id']
                or any(effect[k] != value[k] for k in ('mission_id', 'action_id', 'request_sha256', 'machine_id'))
                or value['observation'] is None):
            raise ValueError('invalid_effect_result')
        if not isinstance(verified, dict) or set(verified) != {'run_id', 'sequence', 'stamp_ns', 'control_revision',
                'sample_sha256', 'machine_id', 'powered', 'current_a', 'vibration_au'}:
            raise ValueError('invalid_effect_result')
        identifier(verified['sample_sha256'], 64)
        integer(verified['sequence'], 1); integer(verified['stamp_ns'], 1)
        integer(verified['control_revision'], 1)
        if (verified['run_id'] != value['station_run_id'] or verified['machine_id'] != value['machine_id']
                or verified['control_revision'] != effect['control_revision'] or verified['powered'] is not False
                or finite(verified['current_a']) != 0 or finite(verified['vibration_au']) != 0
                or verified['stamp_ns'] <= effect['applied_stamp_ns']
                or not 0 <= value['finished_sim_stamp_ns']-verified['stamp_ns'] < 1000000000):
            raise ValueError('invalid_effect_result')
    else:
        raise ValueError('invalid_effect_result')
    return value


def action_view(value):
    if not isinstance(value, dict) or set(value) != {'run_id', 'mission_id', 'action_id', 'request_sha256',
            'machine_id', 'phase', 'stop_reason', 'result'}:
        raise ValueError('invalid_action')
    for key in ('run_id', 'mission_id', 'action_id'): identifier(value[key])
    identifier(value['request_sha256'], 64)
    if value['machine_id'] not in MACHINE_IDS or value['phase'] not in {
            'holding', 'inspecting', 'verifying', 'releasing', 'unresolved', 'terminal'}:
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
