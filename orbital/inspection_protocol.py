"""Versioned, bounded inspection messages; no caller-selected paths or robot state."""
import hashlib
import json
import math
import re

from orbital.machines import MACHINE_IDS

MAX_COMMAND_BYTES = 4096
MAX_REPLY_BYTES = 16384
MAX_ACTIONS = 64
TERMINAL = frozenset(('succeeded', 'failed', 'cancelled', 'timed_out'))


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def decode(text, limit=MAX_COMMAND_BYTES):
    if not isinstance(text, str) or len(text) > limit or len(text.encode('utf-8')) > limit:
        raise ValueError('invalid_argument')

    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('invalid_argument')
            result[key] = value
        return result

    def constant(_):
        raise ValueError('invalid_argument')

    try:
        value = json.loads(text, object_pairs_hook=pairs, parse_constant=constant)
    except (ValueError, RecursionError, OverflowError):
        raise ValueError('invalid_argument') from None
    if not isinstance(value, dict):
        raise ValueError('invalid_argument')
    return value


def identifier(value, size=32):
    if not isinstance(value, str) or len(value) != size or re.fullmatch('[0-9a-f]+', value) is None:
        raise ValueError('invalid_argument')
    return value


def finite(value, minimum=0., maximum=1e15):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError('invalid_argument')
    return float(value)


def command(text):
    value = decode(text)
    if set(value) != {'schema_version', 'run_id', 'mission_id', 'action_id', 'command_id', 'op', 'arguments'}:
        raise ValueError('invalid_argument')
    if type(value['schema_version']) is not int or value['schema_version'] != 1:
        raise ValueError('invalid_argument')
    for name in ('run_id', 'mission_id', 'action_id', 'command_id'):
        identifier(value[name])
    if value['op'] not in ('inspect', 'cancel', 'query') or not isinstance(value['arguments'], dict):
        raise ValueError('invalid_argument')
    args = value['arguments']
    if value['op'] == 'inspect':
        if set(args) != {'machine_id', 'timeout_s'} or not isinstance(args['machine_id'], str) or args['machine_id'] not in MACHINE_IDS:
            raise ValueError('invalid_argument')
        args['timeout_s'] = finite(args['timeout_s'], .1, 10.)
    elif args:
        raise ValueError('invalid_argument')
    return value


def request_digest(value):
    intent = {key: value[key] for key in ('schema_version', 'run_id', 'mission_id', 'action_id', 'arguments')}
    return hashlib.sha256(canonical(intent).encode()).hexdigest()


def context(value):
    fields = {'run_id', 'mission_id', 'action_id', 'request_sha256', 'registry_identity',
              'authority_identity', 'admitted_monotonic_s', 'admitted_sim_stamp_ns'}
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('invalid_action_context')
    for name in ('run_id', 'mission_id', 'action_id'):
        identifier(value[name])
    for name in ('request_sha256', 'registry_identity', 'authority_identity'):
        identifier(value[name], 64)
    finite(value['admitted_monotonic_s'])
    if type(value['admitted_sim_stamp_ns']) is not int or not 0 < value['admitted_sim_stamp_ns'] < 2**63:
        raise ValueError('invalid_action_context')
    return dict(value)


def worker_reply(text):
    value = decode(text)
    if value.get('status') == 'succeeded':
        if set(value) != {'status', 'observation_id', 'manifest_sha256'}:
            raise ValueError('invalid_worker_result')
        identifier(value['observation_id'])
        identifier(value['manifest_sha256'], 64)
    elif value != {'status': 'failed', 'error_code': 'capture_unavailable'}:
        raise ValueError('invalid_worker_result')
    return value


def result(value):
    fields = {'schema_version', 'kind', 'run_id', 'mission_id', 'action_id', 'request_sha256',
              'machine_id', 'status', 'error_code', 'started_sim_stamp_ns', 'finished_sim_stamp_ns',
              'wall_duration_s', 'observation', 'state_change'}
    if not isinstance(value, dict) or set(value) != fields or type(value['schema_version']) is not int or value['schema_version'] != 1 or value['kind'] != 'inspection_result':
        raise ValueError('invalid_result')
    for name in ('run_id', 'mission_id', 'action_id'): identifier(value[name])
    identifier(value['request_sha256'], 64)
    if value['machine_id'] not in MACHINE_IDS or value['status'] not in TERMINAL or value['state_change'] is not None:
        raise ValueError('invalid_result')
    for name in ('started_sim_stamp_ns', 'finished_sim_stamp_ns'):
        if type(value[name]) is not int or not 0 <= value[name] < 2**63:
            raise ValueError('invalid_result')
    finite(value['wall_duration_s'])
    if value['status'] == 'succeeded':
        observation = value['observation']
        if value['error_code'] is not None or not isinstance(observation, dict) or set(observation) != {
                'observation_id', 'manifest_sha256', 'registry_identity', 'sample_stamp_ns', 'camera_stamp_ns'}:
            raise ValueError('invalid_result')
        identifier(observation['observation_id'])
        for name in ('manifest_sha256', 'registry_identity'): identifier(observation[name], 64)
        for name in ('sample_stamp_ns', 'camera_stamp_ns'):
            if type(observation[name]) is not int or not 0 < observation[name] < 2**63:
                raise ValueError('invalid_result')
    elif value['observation'] is not None or value['error_code'] not in {
            'clock_regressed', 'deadline_exceeded', 'precondition_failed', 'capture_unavailable',
            'observation_unavailable', 'cancel_requested', 'owner_shutdown', 'worker_setup_failed',
            'worker_pipe_failed', 'worker_output_oversized', 'invalid_worker_result'}:
        raise ValueError('invalid_result')
    return value


def action_view(value):
    if not isinstance(value, dict) or set(value) != {'run_id', 'mission_id', 'action_id', 'request_sha256', 'machine_id', 'phase', 'result'}:
        raise ValueError('invalid_action')
    for name in ('run_id', 'mission_id', 'action_id'): identifier(value[name])
    identifier(value['request_sha256'], 64)
    if value['machine_id'] not in MACHINE_IDS or value['phase'] not in ('preparing', 'running', 'cancelling', 'terminal'):
        raise ValueError('invalid_action')
    if value['phase'] == 'terminal':
        result(value['result'])
        for name in ('run_id', 'mission_id', 'action_id', 'request_sha256', 'machine_id'):
            if value[name] != value['result'][name]: raise ValueError('invalid_action')
    elif value['result'] is not None:
        raise ValueError('invalid_action')
    return value


def state(text):
    value = decode(text, MAX_REPLY_BYTES)
    if set(value) != {'schema_version', 'kind', 'run_id', 'sequence', 'ready', 'capacity_remaining', 'active', 'last_result'}:
        raise ValueError('invalid_state')
    if type(value['schema_version']) is not int or value['schema_version'] != 1 or value['kind'] != 'inspection_state':
        raise ValueError('invalid_state')
    identifier(value['run_id'])
    if (type(value['sequence']) is not int or not 0 < value['sequence'] < 2**63 or type(value['ready']) is not bool
            or type(value['capacity_remaining']) is not int or not 0 <= value['capacity_remaining'] <= MAX_ACTIONS):
        raise ValueError('invalid_state')
    if value['active'] is not None:
        action_view(value['active'])
        if value['active']['run_id'] != value['run_id'] or value['active']['phase'] == 'terminal':
            raise ValueError('invalid_state')
    if value['last_result'] is not None:
        result(value['last_result'])
        if value['last_result']['run_id'] != value['run_id']:
            raise ValueError('invalid_state')
    return value
