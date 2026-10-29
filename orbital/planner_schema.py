"""Provider-neutral schemas for the fixed, bounded planner vocabulary.

These describe data, not action permission or proof of a successful operation.
ROS owners, the executive and the independent evaluator retain those decisions.
"""
import copy

from orbital.machines import MACHINE_IDS

VERSION = 1
DIALECT = 'https://json-schema.org/draft/2020-12/schema'
MAX_REQUEST_BYTES = 8192
MAX_RESULT_BYTES = 8192
MAX_TEXT_CHARS = 1024
MAX_REFERENCES = 16
MAX_STAMP_NS = 2**63-1
TOOLS = ('observe_station', 'navigate_to', 'inspect_machine', 'isolate_power',
         'reset_machine', 'restore_power', 'verify_recovery', 'stop', 'report_outcome')
MUTATIONS = frozenset(('isolate_power', 'reset_machine', 'restore_power'))
FRESH_TOOLS = frozenset(TOOLS)-{'observe_station', 'stop', 'report_outcome'}
OUTCOMES = ('healthy', 'recovered', 'escalated', 'cancelled', 'failed')
ERRORS = {
    'succeeded': (None,),
    'rejected': ('invalid_argument', 'wrong_run', 'stale_observation',
                 'precondition_failed', 'unavailable', 'busy', 'action_conflict'),
    'failed': ('unreachable', 'precondition_failed', 'unavailable'),
    'cancelled': ('cancel_requested',),
    'timed_out': ('deadline_exceeded',),
}


def enum(values):
    return {'type': 'string', 'enum': sorted(values)}


def object_schema(fields):
    return {'type': 'object', 'properties': fields, 'required': list(fields),
            'additionalProperties': False}


def identifier_schema(size=32):
    return {'type': 'string', 'minLength': size, 'maxLength': size,
            'pattern': '^[0-9a-f]{'+str(size)+'}$'}


def references():
    return {'type': 'array', 'items': identifier_schema(), 'minItems': 0,
            'maxItems': MAX_REFERENCES, 'uniqueItems': True}


def argument_schemas():
    machine = {'machine_id': enum(MACHINE_IDS)}
    text = {'type': 'string', 'minLength': 1, 'maxLength': MAX_TEXT_CHARS}
    return {
        'observe_station': object_schema({}),
        'navigate_to': object_schema({'waypoint_id': enum(MACHINE_IDS),
                                     'timeout_s': {'type': 'number', 'minimum': .1, 'maximum': 120.}}),
        'inspect_machine': object_schema({**machine,
                                         'timeout_s': {'type': 'number', 'minimum': .1, 'maximum': 10.}}),
        **{name: object_schema({**machine,
            'timeout_s': {'type': 'number', 'minimum': .1, 'maximum': 30.}}) for name in MUTATIONS},
        'verify_recovery': object_schema({**machine,
            'observation_window_s': {'type': 'number', 'minimum': 10., 'maximum': 30.}}),
        'stop': object_schema({'reason': enum(('operator_request', 'uncertain',
                                               'unsafe_observation', 'budget_exhausted'))}),
        'report_outcome': object_schema({'outcome': enum(OUTCOMES),
                                        'evidence_ids': references(), 'summary': text}),
    }


def request_schema(tool):
    observation = identifier_schema() if tool in FRESH_TOOLS else {'type': 'null'}
    return {'$schema': DIALECT, **object_schema({
        'schema_version': {'type': 'integer', 'const': VERSION},
        'kind': {'type': 'string', 'const': 'planner_tool_request'},
        'run_id': identifier_schema(), 'mission_id': identifier_schema(),
        'action_id': identifier_schema(), 'tool': enum((tool,)),
        'observation_id': observation, 'arguments': argument_schemas()[tool],
    })}


def result_schema(tool):
    state = object_schema({'machine_id': enum(MACHINE_IDS), 'operation': enum(MUTATIONS),
        'control_revision': {'type': 'integer', 'minimum': 1, 'maximum': MAX_STAMP_NS},
        'powered': {'type': 'boolean'}})
    return {'$schema': DIALECT, **object_schema({
        'schema_version': {'type': 'integer', 'const': VERSION},
        'kind': {'type': 'string', 'const': 'planner_tool_result'},
        'run_id': identifier_schema(), 'mission_id': identifier_schema(),
        'action_id': identifier_schema(), 'tool': enum((tool,)),
        'request_sha256': identifier_schema(64),
        'status': enum(ERRORS),
        'error_code': {'enum': list(dict.fromkeys(code for codes in ERRORS.values() for code in codes))},
        'started_sim_stamp_ns': {'type': 'integer', 'minimum': 0, 'maximum': MAX_STAMP_NS},
        'finished_sim_stamp_ns': {'type': 'integer', 'minimum': 0, 'maximum': MAX_STAMP_NS},
        'wall_duration_s': {'type': 'number', 'minimum': 0., 'maximum': 600.},
        'observation_ids': references(),
        'effect': enum(('none', 'reported', 'unknown') if tool in MUTATIONS else ('none',)),
        'state_change': {'anyOf': [{'type': 'null'}, state]} if tool in MUTATIONS else {'type': 'null'},
    })}


def catalog():
    """Fresh copies; no arbitrary provider extensions or private configuration."""
    return copy.deepcopy({'schema_version': VERSION, 'kind': 'planner_tool_catalog',
        'request_max_utf8_bytes': MAX_REQUEST_BYTES, 'result_max_utf8_bytes': MAX_RESULT_BYTES,
        'tools': [{'name': name, 'requires_fresh_observation': name in FRESH_TOOLS,
                   'input_schema': request_schema(name), 'result_schema': result_schema(name)} for name in TOOLS],
        'scope': 'Wire contracts only; validation is not execution, authorization or independent outcome evidence.'})
