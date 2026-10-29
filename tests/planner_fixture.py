"""Wire examples with opaque test identities; no successful execution implied."""
from orbital.planner_protocol import request_digest
from orbital.planner_schema import FRESH_TOOLS, MUTATIONS

RUN, MISSION, ACTION, OBSERVATION = ('a'*32, 'b'*32, 'c'*32, 'd'*32)


def call(tool='inspect_machine', **arguments):
    defaults = {
        'observe_station': {},
        'navigate_to': {'waypoint_id': 'pump', 'timeout_s': 120.},
        'inspect_machine': {'machine_id': 'pump', 'timeout_s': 10.},
        **{name: {'machine_id': 'pump', 'timeout_s': 30.} for name in MUTATIONS},
        'verify_recovery': {'machine_id': 'pump', 'observation_window_s': 10.},
        'stop': {'reason': 'uncertain'},
        'report_outcome': {'outcome': 'escalated', 'evidence_ids': [OBSERVATION],
                           'summary': 'Observed persistent high vibration.'},
    }
    return {'schema_version': 1, 'kind': 'planner_tool_request', 'run_id': RUN,
        'mission_id': MISSION, 'action_id': ACTION, 'tool': tool,
        'observation_id': OBSERVATION if tool in FRESH_TOOLS else None,
        'arguments': {**defaults[tool], **arguments}}


def reply(original, status='succeeded'):
    value = {'schema_version': 1, 'kind': 'planner_tool_result', 'run_id': RUN,
        'mission_id': MISSION, 'action_id': ACTION, 'tool': original['tool'],
        'request_sha256': request_digest(original), 'status': status,
        'error_code': None if status == 'succeeded' else 'unavailable',
        'started_sim_stamp_ns': 10**9, 'finished_sim_stamp_ns': 2*10**9,
        'wall_duration_s': 1., 'observation_ids': [OBSERVATION] if status == 'succeeded' else [],
        'effect': 'none', 'state_change': None}
    if original['tool'] in MUTATIONS and status == 'succeeded':
        value.update(effect='reported', state_change={'machine_id': 'pump',
            'operation': original['tool'], 'control_revision': 1,
            'powered': original['tool'] == 'restore_power'})
    return value
