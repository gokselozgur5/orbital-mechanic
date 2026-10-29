"""Bounded single-command client protocol, independent of ROS transport.

Acknowledgment means the supervisor accepted intent, not dispatch, arrival or
physical stopping. A missing response never causes an automatic retransmission.
"""
import copy
import json
import math

from orbital.machines import MACHINE_IDS
from orbital.navigation_command import CommandGate

STATE_MAX_AGE = .3
ACTION_STATES = frozenset(('idle', 'waiting_for_arm', 'dispatching', 'running', 'cancelling', 'terminal'))


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError('Duplicate JSON key')
        result[key] = value
    return result


def _json(text, limit=16384):
    if not isinstance(text, str) or len(text) > limit:
        raise ValueError('JSON input exceeds its size bound')
    def finite_float(value):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError('JSON numbers must be finite')
        return number

    def invalid_constant(value):
        raise ValueError('Nonfinite JSON constants are not permitted')

    value = json.loads(text, object_pairs_hook=_pairs, parse_float=finite_float, parse_constant=invalid_constant)
    if not isinstance(value, dict):
        raise ValueError('Expected a JSON object')
    return value


def command_timeout(value):
    result = float(value)
    if not math.isfinite(result) or not .1 <= result <= 10.:
        raise ValueError('Command timeout must be in [0.1, 10] seconds')
    return result


def _target(value):
    if not isinstance(value, dict):
        raise ValueError('Exact target identity is required')
    run = value.get('run_id')
    if not isinstance(run, str) or not 1 <= len(run) <= 128:
        raise ValueError('Invalid target run')
    # Reuse the server's wire-schema validation; this does not authorize a target.
    CommandGate(run, MACHINE_IDS).decode(json.dumps({
        'run_id': run, 'sequence': 1, 'op': 'cancel', 'target': value,
    }, allow_nan=False))
    return copy.deepcopy(value)


def read_target(text):
    """Accept an explicit token or a successful request receipt, never status."""
    value = _json(text)
    if set(value) == {'run_id', 'epoch', 'sequence', 'waypoint'}:
        return _target(value)
    if (type(value.get('schema_version')) is not int or value['schema_version'] != 1
            or value.get('operation') != 'request' or value.get('delivery') != 'acknowledged'
            or value.get('accepted') is not True):
        raise ValueError('Expected an acknowledged request receipt or exact target token')
    target = _target(value.get('target'))
    command = value.get('command')
    if (not isinstance(command, dict) or command.get('op') != 'request'
            or command.get('run_id') != target['run_id'] or command.get('waypoint') != target['waypoint']):
        raise ValueError('Request receipt does not match target')
    sequence = command.get('sequence')
    if type(sequence) is not int or not 0 < sequence < 2**63:
        raise ValueError('Invalid request receipt sequence')
    gate = CommandGate(target['run_id'], MACHINE_IDS)
    gate.sequence = sequence-1
    gate.decode(json.dumps(command, allow_nan=False))
    return target


class NavigationClient:
    def __init__(self, now):
        self.now = now
        self.state = None
        self.received_at = None
        self.run_id = None
        self.failure = None
        self.command = None
        self.published = False

    def observe(self, text):
        """Pin one run and monotonic command sequence; malformed input is fatal."""
        try:
            value = _json(text)
            run, seq = value.get('run_id'), value.get('command_sequence')
            if not isinstance(run, str) or not 1 <= len(run) <= 128:
                raise ValueError('Invalid supervisor run')
            if type(seq) is not int or not 0 <= seq < 2**63:
                raise ValueError('Invalid supervisor command sequence')
            if value.get('action_state') not in ACTION_STATES or type(value.get('healthy')) is not bool:
                raise ValueError('Invalid supervisor action/health state')
            if value.get('guard_latched') is not None and type(value['guard_latched']) is not bool:
                raise ValueError('Invalid supervisor guard state')
            if not {'target', 'guard_latched', 'last_reply', 'last_completion'} <= set(value):
                raise ValueError('Supervisor state fields missing')
            if value['target'] is not None:
                target = _target(value['target'])
                if target['run_id'] != run:
                    raise ValueError('Supervisor target belongs to another run')
            if (value['action_state'] == 'idle') != (value['target'] is None):
                raise ValueError('Supervisor target/action state mismatch')
            completion = value['last_completion']
            if completion is not None:
                if not isinstance(completion, dict) or _target(completion.get('target'))['run_id'] != run:
                    raise ValueError('Invalid supervisor completion target')
                if not isinstance(completion.get('outcome'), str) or not 1 <= len(completion['outcome']) <= 128:
                    raise ValueError('Invalid supervisor completion outcome')
            if self.run_id is not None and run != self.run_id:
                raise ValueError('Supervisor restarted; explicit new invocation required')
            if self.state is not None and seq < self.state['command_sequence']:
                raise ValueError('Supervisor command sequence regressed')
            now = self.now()
            if not math.isfinite(now) or (self.received_at is not None and now < self.received_at):
                raise ValueError('Invalid client clock')
            self.run_id, self.state, self.received_at = run, value, now
        except (ValueError, TypeError, OverflowError, RecursionError) as error:
            self.failure = str(error)

    def fresh(self):
        if self.failure:
            raise ValueError(self.failure)
        if self.state is None or self.received_at is None:
            return False
        age = self.now()-self.received_at
        if not math.isfinite(age) or age < 0:
            raise ValueError('Invalid client clock')
        return age < STATE_MAX_AGE

    def prepare(self, operation, *, waypoint=None, target=None):
        if self.command is not None:
            raise ValueError('Only one command is permitted per invocation')
        self.command = self._make_command(operation, waypoint=waypoint, target=target)

    def _make_command(self, operation, *, waypoint=None, target=None):
        if not self.fresh():
            raise ValueError('Fresh supervisor state is required')
        state = self.state
        command = {'run_id': self.run_id, 'sequence': state['command_sequence']+1, 'op': operation}
        if operation == 'request':
            if state['action_state'] != 'idle' or not state['healthy'] or state['guard_latched'] is not True:
                raise ValueError('Request requires healthy idle supervisor and latched guard')
            command['waypoint'] = waypoint
        else:
            command['target'] = _target(target)
            if command['target'] != state['target']:
                raise ValueError('Target is no longer owned by this supervisor')
            if operation == 'arm' and (state['action_state'] != 'waiting_for_arm'
                    or not state['healthy'] or state['guard_latched'] is not True):
                raise ValueError('Arm requires a healthy waiting target and latched guard')
        gate = CommandGate(self.run_id, MACHINE_IDS)
        gate.sequence = state['command_sequence']
        gate.decode(json.dumps(command, allow_nan=False))
        return command

    def publish_once(self, publish):
        if self.command is None or self.published:
            raise ValueError('Command is absent or was already sent')
        current = self._make_command(self.command['op'], waypoint=self.command.get('waypoint'), target=self.command.get('target'))
        if current != self.command:
            raise ValueError('Supervisor changed before publication')
        # A transport exception may occur after delivery. Mark uncertainty before
        # entering it; never retry a request, arm or cancellation automatically.
        self.published = True
        publish(json.dumps(self.command, separators=(',', ':'), allow_nan=False))

    def acknowledgment(self):
        if not self.published or not self.fresh():
            return None
        state, command = self.state, self.command
        seq = state['command_sequence']
        if seq < command['sequence']:
            return None
        if seq > command['sequence']:
            raise ValueError('Another command advanced the sequence; acknowledgment is ambiguous')
        reply = state.get('last_reply')
        if not isinstance(reply, dict) or reply.get('sequence') != command['sequence']:
            return None
        if type(reply.get('sequence')) is not int or reply.get('op') != command['op'] or type(reply.get('accepted')) is not bool:
            raise ValueError('Mismatched command acknowledgment')
        if reply['accepted']:
            if command['op'] == 'request':
                target = state['target']
                if target is None or target['waypoint'] != command['waypoint']:
                    raise ValueError('Acknowledged request has no matching owned target')
            else:
                target = command['target']
                completion = state.get('last_completion')
                if state['target'] != target and (not isinstance(completion, dict) or completion.get('target') != target):
                    raise ValueError('Acknowledgment lost exact target ownership')
        else:
            target = command.get('target')
        return {'accepted': reply['accepted'], 'target': copy.deepcopy(target)}

    def receipt(self, operation, *, acknowledgment=None, error=None):
        if acknowledgment is not None:
            delivery = 'acknowledged' if acknowledgment['accepted'] else 'rejected'
        else:
            delivery = 'uncertain' if self.published else 'not_sent'
        return {'schema_version': 1, 'operation': operation, 'delivery': delivery,
                'accepted': acknowledgment['accepted'] if acknowledgment is not None else None,
                'command': copy.deepcopy(self.command),
                'target': acknowledgment['target'] if acknowledgment is not None else
                          copy.deepcopy(self.command.get('target')) if self.command else None,
                'state': copy.deepcopy(self.state), 'error': error,
                'scope': 'command acknowledgment only; not dispatch, arrival or physical stopping'}


def execute(client, channel, operation, *, timeout=5., waypoint=None, target=None):
    """The channel spins callbacks into client.observe and checks ROS ownership."""
    deadline = client.now()+command_timeout(timeout)

    def poll():
        remaining = deadline-client.now()
        if remaining <= 0:
            raise TimeoutError('Command discovery/acknowledgment deadline expired')
        channel.poll(min(.02, remaining))
        if client.now() >= deadline:
            raise TimeoutError('Command discovery/acknowledgment deadline expired')
        if client.failure:
            raise ValueError(client.failure)
        clear = channel.authority_clear()
        if client.now() >= deadline:
            raise TimeoutError('Command discovery/acknowledgment deadline expired')
        return clear

    try:
        while not (poll() and client.fresh()):
            pass
        if operation == 'status':
            return {'schema_version': 1, 'operation': 'status', 'delivery': 'observed',
                    'state': copy.deepcopy(client.state), 'scope': 'observed supervisor state only'}, 0
        client.prepare(operation, waypoint=waypoint, target=target)
        if client.now() >= deadline:
            raise TimeoutError('Command deadline expired before publication')
        client.publish_once(channel.publish)
        while True:
            if poll():
                acknowledgment = client.acknowledgment()
                if acknowledgment is not None:
                    return client.receipt(operation, acknowledgment=acknowledgment), 0 if acknowledgment['accepted'] else 1
    except KeyboardInterrupt:
        return client.receipt(operation, error='Interrupted; inspect current state before another command'), 130
    except Exception as error:
        return client.receipt(operation, error=str(error)), 3 if client.published else 2
