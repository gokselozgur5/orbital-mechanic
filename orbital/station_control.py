"""Public applied-operation identities, separate from hidden machine fault state."""
from orbital.inspection_protocol import identifier
from orbital.machines import MACHINE_IDS

OPERATIONS = frozenset(('isolate', 'reset', 'restore'))
EFFECT_FIELDS = frozenset(('owner_run_id', 'mission_id', 'action_id', 'request_sha256',
                           'machine_id', 'operation', 'control_revision', 'applied_stamp_ns'))


class EffectRejected(ValueError):
    """The station owner refused an effect before calling the causal actuator."""


def control(value, stamp_ns):
    if not isinstance(value, dict) or set(value) != {'revision', 'last_applied'}:
        raise ValueError('invalid_control_state')
    revision = value['revision']
    if type(revision) is not int or not 0 <= revision < 2**63:
        raise ValueError('invalid_control_state')
    effect = value['last_applied']
    if revision == 0:
        if effect is not None:
            raise ValueError('invalid_control_state')
        return value
    if not isinstance(effect, dict) or set(effect) != EFFECT_FIELDS:
        raise ValueError('invalid_control_state')
    for name in ('owner_run_id', 'mission_id', 'action_id'):
        identifier(effect[name])
    identifier(effect['request_sha256'], 64)
    if (effect['machine_id'] not in MACHINE_IDS or effect['operation'] not in OPERATIONS
            or type(effect['control_revision']) is not int or effect['control_revision'] != revision
            or type(effect['applied_stamp_ns']) is not int or not 0 < effect['applied_stamp_ns'] <= stamp_ns):
        raise ValueError('invalid_control_state')
    return value
