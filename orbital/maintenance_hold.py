"""Run-bound, non-expiring maintenance inhibition owned by the motion guard."""
import copy
import uuid

from orbital.inspection_protocol import decode, identifier

IDENTITY = frozenset(('owner_run_id', 'mission_id', 'action_id', 'token'))
MAX_HOLDS = 64


def identity(value):
    if not isinstance(value, dict) or set(value) != IDENTITY:
        raise ValueError('invalid_hold_identity')
    return {key: identifier(value[key]) for key in sorted(IDENTITY)}


def command(text):
    value = decode(text)
    if (set(value) != {'schema_version', 'guard_run_id', 'command_id', 'op', 'identity'}
            or type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['op'] not in ('acquire', 'release', 'query')):
        raise ValueError('invalid_hold_command')
    identifier(value['guard_run_id'])
    identifier(value['command_id'])
    value['identity'] = identity(value['identity'])
    return value


def state(value):
    fields = {'schema_version', 'guard_run_id', 'sequence', 'sim_stamp_ns', 'active',
              'last_released', 'last_reply', 'capacity_remaining'}
    if (not isinstance(value, dict) or set(value) != fields
            or type(value['schema_version']) is not int or value['schema_version'] != 1):
        raise ValueError('invalid_hold_state')
    identifier(value['guard_run_id'])
    for key in ('sequence', 'sim_stamp_ns'):
        if type(value[key]) is not int or not 0 <= value[key] < 2**63:
            raise ValueError('invalid_hold_state')
    if value['sequence'] == 0:
        raise ValueError('invalid_hold_state')
    for key in ('active', 'last_released'):
        if value[key] is not None:
            identity(value[key])
    if type(value['capacity_remaining']) is not int or not 0 <= value['capacity_remaining'] <= MAX_HOLDS:
        raise ValueError('invalid_hold_state')
    reply = value['last_reply']
    if reply is not None:
        if (not isinstance(reply, dict) or set(reply) != {'command_id', 'accepted', 'error_code', 'status'}
                or type(reply['accepted']) is not bool
                or reply['status'] not in (None, 'held', 'released')):
            raise ValueError('invalid_hold_state')
        if reply['command_id'] is not None:
            identifier(reply['command_id'])
        if reply['accepted']:
            if reply['error_code'] is not None or reply['status'] is None:
                raise ValueError('invalid_hold_state')
        elif reply['error_code'] not in ('invalid_argument', 'wrong_run', 'hold_conflict',
                                        'busy', 'history_full', 'unknown_hold', 'stationary_required'):
            raise ValueError('invalid_hold_state')
    return copy.deepcopy(value)


class GuardHold:
    """No timeout, lost client or duplicate acquire may remove this inhibitor."""
    def __init__(self):
        self.run_id = uuid.uuid4().hex
        self.active = self.last_released = self.last_reply = None
        self.history = {}
        self.sequence = 0

    def handle(self, text, *, stationary):
        reply = {'command_id': None, 'accepted': False, 'error_code': 'invalid_argument', 'status': None}
        self.last_reply = reply
        try:
            request = command(text)
        except (ValueError, TypeError, UnicodeError):
            return copy.deepcopy(reply)
        reply['command_id'] = request['command_id']

        def reject(code):
            reply['error_code'] = code
            return copy.deepcopy(reply)

        if request['guard_run_id'] != self.run_id:
            return reject('wrong_run')
        owner = request['identity']
        key = (owner['owner_run_id'], owner['action_id'])
        previous = self.history.get(key)
        if previous is not None and previous['identity'] != owner:
            return reject('hold_conflict')
        if request['op'] == 'acquire':
            if previous is None:
                if self.active is not None:
                    return reject('busy')
                if len(self.history) >= MAX_HOLDS:
                    return reject('history_full')
                previous = {'identity': copy.deepcopy(owner), 'status': 'held'}
                self.history[key] = previous
                self.active = copy.deepcopy(owner)
        elif previous is None:
            if request['op'] != 'release':
                return reject('unknown_hold')
            if self.active is not None:
                return reject('busy')
            if stationary is not True:
                return reject('stationary_required')
            if len(self.history) >= MAX_HOLDS:
                return reject('history_full')
            # A cancelled, not-yet-delivered acquire must not arrive later and
            # resurrect ownership after the owner confirmed its release.
            previous = {'identity': copy.deepcopy(owner), 'status': 'released'}
            self.history[key] = previous
            self.last_released = copy.deepcopy(owner)
        elif request['op'] == 'release' and previous['status'] == 'held':
            if stationary is not True:
                return reject('stationary_required')
            self.last_released = copy.deepcopy(owner)
            self.active = None
            previous['status'] = 'released'
        reply.update(accepted=True, error_code=None, status=previous['status'])
        return copy.deepcopy(reply)

    def snapshot(self, sim_ns):
        self.sequence += 1
        return {'schema_version': 1, 'guard_run_id': self.run_id, 'sequence': self.sequence,
                'sim_stamp_ns': sim_ns, 'active': copy.deepcopy(self.active),
                'last_released': copy.deepcopy(self.last_released),
                'last_reply': copy.deepcopy(self.last_reply),
                'capacity_remaining': MAX_HOLDS - len(self.history)}
