"""Bounded evidence for explicit cancellation followed by observed isolation."""
from orbital.inspection_protocol import canonical, finite, identifier
from orbital.station_control import control


def bound_request(request, outcome):
    from orbital import maintenance_protocol as mp
    if not isinstance(outcome, dict) or outcome.get('operation') not in ('isolate', 'reset', 'restore'):
        raise ValueError('invalid_cancellation_child')
    mp.result(outcome)
    parsed = mp.command(canonical(request))
    if (parsed['op'] not in ('isolate', 'reset', 'restore')
            or any(parsed[k] != outcome[k] for k in ('run_id', 'mission_id', 'action_id'))
            or parsed['op'] != outcome['operation']
            or mp.request_digest(parsed) != outcome['request_sha256']
            or any(parsed['arguments'][k] != outcome[k] for k in ('machine_id', 'station_run_id'))):
        raise ValueError('invalid_cancellation_child')


def expected_revision(request, outcome):
    if outcome['state_change']['disposition'] == 'applied':
        return outcome['state_change']['effect']['control_revision']
    return request['arguments']['control_revision']


def result(value):
    from orbital import maintenance_protocol as mp
    fields = {'schema_version', 'kind', 'run_id', 'mission_id', 'action_id', 'request_sha256',
        'operation', 'machine_id', 'station_run_id', 'status', 'error_code', 'started_sim_stamp_ns',
        'finished_sim_stamp_ns', 'wall_duration_s', 'request', 'target_request', 'target_result',
        'isolation_action_id', 'isolation_request', 'isolation_result', 'isolation_rejection',
        'final_state', 'verification', 'hold_released'}
    if (not isinstance(value, dict) or set(value) != fields or type(value['schema_version']) is not int
            or value['schema_version'] != 4 or value['kind'] != 'maintenance_result'
            or value['operation'] != 'cancel_isolate' or value['hold_released'] is not True
            or value['status'] not in ('succeeded', 'failed', 'cancelled', 'timed_out')
            or value['final_state'] not in ('isolated', 'energized', 'unverified')):
        raise ValueError('invalid_cancellation_result')
    request = mp.command(canonical(value['request']))
    if (request['op'] != 'cancel_isolate' or mp.request_digest(request) != value['request_sha256']
            or any(request[k] != value[k] for k in ('run_id', 'mission_id', 'action_id'))
            or any(request['arguments'][k] != value[k] for k in ('machine_id', 'station_run_id'))):
        raise ValueError('invalid_cancellation_request_binding')
    for key in ('started_sim_stamp_ns', 'finished_sim_stamp_ns'): mp.integer(value[key])
    finite(value['wall_duration_s'])
    if value['status'] == 'succeeded':
        if value['error_code'] is not None or value['final_state'] != 'isolated':
            raise ValueError('invalid_cancellation_success')
    elif value['error_code'] not in mp.ERROR_CODES:
        raise ValueError('invalid_cancellation_error')
    target = value['target_result']
    bound_request(value['target_request'], target)
    if (target['action_id'] != request['arguments']['target_action_id']
            or any(target[k] != value[k] for k in ('run_id', 'mission_id', 'machine_id', 'station_run_id'))
            or value['action_id'] == target['action_id']):
        raise ValueError('invalid_cancellation_target')
    identifier(value['isolation_action_id'])
    if value['isolation_action_id'] in (value['action_id'], target['action_id']):
        raise ValueError('invalid_cancellation_child_identity')
    child_request, child = value['isolation_request'], value['isolation_result']
    if child_request is None:
        if child is not None or value['isolation_rejection'] is not None:
            raise ValueError('invalid_cancellation_child')
    else:
        parsed = mp.command(canonical(child_request))
        if (parsed['op'] != 'isolate' or parsed['action_id'] != value['isolation_action_id']
                or any(parsed[k] != value[k] for k in ('run_id', 'mission_id'))
                or any(parsed['arguments'][k] != value[k] for k in ('machine_id', 'station_run_id'))
                or parsed['arguments']['control_revision'] != expected_revision(value['target_request'], target)):
            raise ValueError('invalid_cancellation_child')
        if child is None:
            if value['isolation_rejection'] not in mp.REJECTION_CODES:
                raise ValueError('unresolved_cancellation_child')
        else:
            bound_request(child_request, child)
            if (value['isolation_rejection'] is not None or child['operation'] != 'isolate'
                    or child['started_sim_stamp_ns'] < target['finished_sim_stamp_ns']):
                raise ValueError('invalid_cancellation_child')
    verification = value['verification']
    if value['final_state'] == 'unverified':
        if verification is not None or value['status'] == 'succeeded':
            raise ValueError('invalid_cancellation_verification')
        return value
    if not isinstance(verification, dict) or set(verification) != {
            'registry_identity', 'run_id', 'control', 'after_sim_stamp_ns', 'observed_sim_stamp_ns', 'samples'}:
        raise ValueError('invalid_cancellation_verification')
    identifier(verification['registry_identity'], 64)
    if verification['run_id'] != value['station_run_id']:
        raise ValueError('invalid_cancellation_verification')
    after, observed = verification['after_sim_stamp_ns'], verification['observed_sim_stamp_ns']
    mp.integer(after, 1); mp.integer(observed, 1)
    latest_request, latest = (child_request, child) if child is not None else (value['target_request'], target)
    if (not max(value['started_sim_stamp_ns'], latest['finished_sim_stamp_ns']) <= after < observed
            or observed != value['finished_sim_stamp_ns']):
        raise ValueError('invalid_cancellation_verification_clock')
    control(verification['control'], observed)
    if verification['control']['revision'] != expected_revision(latest_request, latest):
        raise ValueError('invalid_cancellation_verification_revision')
    if (latest['state_change']['disposition'] == 'applied'
            and verification['control']['last_applied'] != latest['state_change']['effect']):
        raise ValueError('invalid_cancellation_verification_effect')
    samples = verification['samples']
    if not isinstance(samples, list) or len(samples) != 2:
        raise ValueError('invalid_cancellation_verification_pair')
    for sample in samples:
        if not isinstance(sample, dict) or set(sample) != {'sequence', 'stamp_ns', 'sample_sha256',
                'received_age_s', 'simulation_age_s', 'powered', 'current_a', 'vibration_au'}:
            raise ValueError('invalid_cancellation_verification_pair')
        mp.integer(sample['sequence'], 1); mp.integer(sample['stamp_ns'], 1)
        identifier(sample['sample_sha256'], 64)
        age = finite(sample['received_age_s'])
        sim_age = finite(sample['simulation_age_s'])
        if (not after < sample['stamp_ns'] <= observed or not 0 <= age < 1.
                or not 0 <= sim_age < 1. or sim_age != (observed-sample['stamp_ns'])/1e9
                or sample['powered'] is not (value['final_state'] == 'energized')):
            raise ValueError('invalid_cancellation_verification_pair')
        current, vibration = finite(sample['current_a']), finite(sample['vibration_au'])
        if value['final_state'] == 'isolated' and (current != 0 or vibration != 0):
            raise ValueError('invalid_cancellation_verification_pair')
    if samples[1]['sequence'] != samples[0]['sequence']+1 or samples[1]['stamp_ns'] <= samples[0]['stamp_ns']:
        raise ValueError('invalid_cancellation_verification_pair')
    return value


def action_view(value):
    from orbital import maintenance_protocol as mp
    if not isinstance(value, dict) or set(value) != {'run_id', 'mission_id', 'action_id', 'request_sha256',
            'machine_id', 'phase', 'stop_reason', 'result', 'cancellation'}:
        raise ValueError('invalid_cancellation_view')
    for key in ('run_id', 'mission_id', 'action_id'): identifier(value[key])
    identifier(value['request_sha256'], 64)
    if value['machine_id'] not in mp.MACHINE_IDS or value['phase'] not in {
            'cancelling', 'assessing', 'isolating', 'confirming', 'unresolved', 'terminal'}:
        raise ValueError('invalid_cancellation_view')
    context = value['cancellation']
    if not isinstance(context, dict) or set(context) != {'target_action_id', 'isolation_action_id', 'target_phase', 'isolation_phase'}:
        raise ValueError('invalid_cancellation_view')
    for key in ('target_action_id', 'isolation_action_id'): identifier(context[key])
    if len({value['action_id'], context['target_action_id'], context['isolation_action_id']}) != 3:
        raise ValueError('invalid_cancellation_view')
    phases = {'holding', 'cooling', 'inspecting', 'verifying', 'releasing', 'unresolved', 'terminal'}
    if context['target_phase'] not in phases or (context['isolation_phase'] is not None and context['isolation_phase'] not in phases):
        raise ValueError('invalid_cancellation_view')
    stop = value['stop_reason']
    if stop is not None and (not isinstance(stop, (list, tuple)) or len(stop) != 2
            or stop[0] not in ('failed', 'cancelled', 'timed_out') or stop[1] not in mp.ERROR_CODES):
        raise ValueError('invalid_cancellation_view')
    if value['phase'] == 'terminal':
        result(value['result'])
        if (any(value[k] != value['result'][k] for k in ('run_id', 'mission_id', 'action_id', 'request_sha256', 'machine_id'))
                or context['target_action_id'] != value['result']['target_result']['action_id']
                or context['isolation_action_id'] != value['result']['isolation_action_id']):
            raise ValueError('invalid_cancellation_view')
    elif value['result'] is not None:
        raise ValueError('invalid_cancellation_view')
    return value
