"""Strict JSON admission for the fixed planner schema, with bounded public errors."""
import hashlib
import math
import re

from orbital.inspection_protocol import canonical, decode, identifier
from orbital.planner_schema import (ERRORS, MAX_REQUEST_BYTES, MAX_RESULT_BYTES,
                                   MUTATIONS, TOOLS, request_schema, result_schema)


class PlannerError(ValueError):
    def __init__(self, code):
        if code not in {'invalid_argument', 'wrong_run', 'stale_observation', 'unavailable', 'invalid_result'}:
            code = 'unavailable'
        self.code = code
        super().__init__(code)


def _check(value, schema):
    """Evaluate only our fixed schema vocabulary, never a caller's schema.

    Integer fields additionally require JSON integer tokens, not float tokens.
    The byte bound, cross-field semantics and live context are checked separately.
    """
    if 'anyOf' in schema:
        for alternative in schema['anyOf']:
            try:
                _check(value, alternative)
                return
            except ValueError:
                pass
        raise ValueError
    kind = schema.get('type')
    if kind == 'object':
        if type(value) is not dict or set(value) != set(schema['properties']):
            raise ValueError
        for name, child in schema['properties'].items():
            _check(value[name], child)
    elif kind == 'array':
        if type(value) is not list or not schema['minItems'] <= len(value) <= schema['maxItems']:
            raise ValueError
        for item in value:
            _check(item, schema['items'])
        if schema.get('uniqueItems') and len({canonical(item) for item in value}) != len(value):
            raise ValueError
    elif kind == 'string':
        if type(value) is not str or not schema.get('minLength', 0) <= len(value) <= schema.get('maxLength', 1024):
            raise ValueError
        value.encode('utf-8', errors='strict')
        if 'pattern' in schema and re.fullmatch(schema['pattern'], value) is None:
            raise ValueError
    elif kind in ('number', 'integer'):
        if (type(value) not in ((int,) if kind == 'integer' else (int, float))
                or not math.isfinite(value)
                or not schema.get('minimum', 0) <= value <= schema.get('maximum', 2**63-1)):
            raise ValueError
    elif kind == 'boolean':
        if type(value) is not bool:
            raise ValueError
    elif kind == 'null' and value is not None:
        raise ValueError
    if 'const' in schema and value != schema['const']:
        raise ValueError
    if 'enum' in schema and value not in schema['enum']:
        raise ValueError


def request(text, *, run_id, mission_id):
    """Context comes from the executive, never from the provider's own request."""
    try:
        identifier(run_id)
        identifier(mission_id)
        value = decode(text, MAX_REQUEST_BYTES)
        tool = value.get('tool')
        if type(tool) is not str or tool not in TOOLS:
            raise ValueError
        _check(value, request_schema(tool))
        # Equivalent numeric spellings share one logical request digest.
        for key in ('timeout_s', 'observation_window_s'):
            if key in value['arguments']:
                value['arguments'][key] = float(value['arguments'][key])
        if tool == 'report_outcome' and not value['arguments']['summary'].strip():
            raise ValueError
    except (ValueError, TypeError, OverflowError, RecursionError):
        raise PlannerError('invalid_argument') from None
    if value['run_id'] != run_id or value['mission_id'] != mission_id:
        raise PlannerError('wrong_run')
    return value


def request_digest(value):
    # Revalidate even when a caller passes an object assembled inside the process.
    try:
        normalized = request(canonical(value), run_id=value['run_id'], mission_id=value['mission_id'])
        return hashlib.sha256(canonical(normalized).encode('utf-8')).hexdigest()
    except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
        raise PlannerError('invalid_argument') from None


def result(text, *, original):
    """Check a terminal wire result; this does not authenticate an owner result."""
    try:
        digest = request_digest(original)
        value = decode(text, MAX_RESULT_BYTES)
        _check(value, result_schema(original['tool']))
        for key in ('run_id', 'mission_id', 'action_id', 'tool'):
            if value[key] != original[key]:
                raise ValueError
        if (value['request_sha256'] != digest or value['error_code'] not in ERRORS[value['status']]
                or value['finished_sim_stamp_ns'] < value['started_sim_stamp_ns']):
            raise ValueError
        state = value['state_change']
        if (value['effect'] == 'reported') != (state is not None):
            raise ValueError
        if state is not None and (state['machine_id'] != original['arguments']['machine_id']
                or state['operation'] != original['tool']
                or state['powered'] != (original['tool'] == 'restore_power')
                or not value['observation_ids']):
            raise ValueError
        if value['status'] == 'rejected' and (value['effect'] != 'none' or value['observation_ids']):
            raise ValueError
        if value['status'] == 'succeeded':
            if original['tool'] != 'report_outcome' and not value['observation_ids']:
                raise ValueError
            if original['tool'] in MUTATIONS and value['effect'] != 'reported':
                raise ValueError
        return value
    except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
        raise PlannerError('invalid_result') from None
