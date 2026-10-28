"""Run-bound inspection client; ACK, uncertain transport and terminal result differ."""
import time
import uuid
from orbital import inspection_protocol

from orbital.inspection_protocol import MAX_REPLY_BYTES, canonical, decode


class InspectionClient:
    def __init__(self, publish, spin, authority, *, now=time.monotonic,
                 protocol=inspection_protocol, family='inspection', operation='inspect'):
        self.publish, self.spin, self.authority, self.now = publish, spin, authority, now
        self.protocol, self.family, self.operation = protocol, family, operation
        self.latest = self.received = self.pinned = self.failure = None
        self.state_count = 0
        self.pending = self.ack = self.terminal = None

    def matches(self, value):
        return self.pending is not None and all(value.get(k) == self.pending[k] for k in ('run_id', 'mission_id', 'action_id'))

    def observe_state(self, text):
        try:
            value = self.protocol.state(text)
            if self.latest is not None:
                if value['run_id'] != self.latest['run_id'] or value['sequence'] <= self.latest['sequence']:
                    raise ValueError('server_generation_changed')
            self.latest, self.received = value, self.now()
            self.state_count += 1
            if value['last_result'] is not None:
                self.accept_result(value['last_result'])
            if value['active'] is not None and self.matches(value['active']):
                self.check_digest(value['active'])
        except (ValueError, TypeError, KeyError):
            self.failure = 'invalid_or_changed_server_state'

    def check_digest(self, value):
        if self.pending['op'] == self.operation and value['request_sha256'] != self.protocol.request_digest(self.pending):
            raise ValueError('request_identity_mismatch')

    def accept_result(self, value):
        self.protocol.result(value)
        if self.matches(value):
            self.check_digest(value)
            if self.terminal is not None and self.terminal != value:
                raise ValueError('conflicting_terminal_result')
            self.terminal = value

    def observe_result(self, text):
        try: self.accept_result(decode(text, MAX_REPLY_BYTES))
        except (ValueError, TypeError, KeyError): self.failure = 'invalid_terminal_result'

    def observe_reply(self, text):
        try:
            value = decode(text, MAX_REPLY_BYTES)
            if self.pending is None or value.get('command_id') != self.pending['command_id']:
                return
            if (set(value) != {'schema_version', 'kind', 'run_id', 'mission_id', 'action_id', 'command_id', 'op', 'accepted', 'error_code', 'action'}
                    or type(value['schema_version']) is not int or value['schema_version'] != 1
                    or value['kind'] != self.family+'_reply' or not self.matches(value)
                    or value['op'] != self.pending['op'] or type(value['accepted']) is not bool):
                raise ValueError('invalid_reply')
            if value['accepted']:
                if value['error_code'] is not None:
                    raise ValueError('invalid_reply')
                self.protocol.action_view(value['action'])
                if not self.matches(value['action']):
                    raise ValueError('invalid_reply')
                self.check_digest(value['action'])
                if value['action']['result'] is not None:
                    self.accept_result(value['action']['result'])
            elif value['action'] is not None or value['error_code'] not in self.protocol.REJECTION_CODES:
                raise ValueError('invalid_reply')
            self.ack = value
        except (ValueError, TypeError, KeyError):
            self.failure = 'invalid_reply'

    def valid(self):
        if self.failure or self.latest is None or self.received is None or not 0 <= self.now()-self.received < .3:
            return False
        try:
            peers = self.authority()
            if self.pinned is None:
                self.pinned = peers
            elif peers != self.pinned:
                self.failure = 'publisher_generation_changed'
                return False
        except (ValueError, RuntimeError):
            return False
        return True

    def discover(self, timeout=2.):
        deadline = self.now()+timeout
        while self.now() < deadline:
            self.spin(min(.02, deadline-self.now()))
            if self.failure: break
            if self.state_count >= 2 and self.valid(): return self.latest
        raise ValueError(self.family+'_server_unavailable')

    def receipt(self, transport, code=None):
        identity = {} if self.pending is None else {k: self.pending[k] for k in ('run_id', 'mission_id', 'action_id')}
        return {'transport': transport, **identity, 'error_code': code, 'result': self.terminal,
                'acknowledgement': self.ack}

    def cancel_after_loss(self):
        # One exact cancellation attempt; no inspection retransmission. A lost
        # connection or stuck worker still returns uncertain, never cancelled.
        original = self.pending
        cancellation = {**original, 'command_id': uuid.uuid4().hex, 'op': 'cancel', 'arguments': {}}
        self.publish(canonical(cancellation))
        deadline = self.now()+1.
        while self.now() < deadline and self.terminal is None:
            self.spin(min(.02, deadline-self.now()))
        if self.terminal is not None and self.valid():
            return self.receipt('terminal')
        return self.receipt('uncertain', 'cancellation_unconfirmed')

    def execute(self, value, *, wait=True):
        value = self.protocol.command(canonical(value))
        self.pending, self.ack, self.terminal = value, None, None
        if not self.valid() or value['run_id'] != self.latest['run_id']:
            return self.receipt('not_sent', self.family+'_server_unavailable')
        if value['op'] == self.operation and not self.latest['ready']:
            return self.receipt('not_sent', self.family+'_server_not_ready')
        try:
            self.publish(canonical(value))
            budget = value['arguments']['timeout_s']+2. if value['op'] == self.operation else 2.
            deadline = self.now()+budget
            while self.now() < deadline:
                self.spin(min(.02, deadline-self.now()))
                if not self.valid(): break
                if self.terminal is not None: return self.receipt('terminal')
                if self.ack is not None:
                    if not self.ack['accepted']: return self.receipt('rejected', self.ack['error_code'])
                    if not wait or value['op'] == 'query': return self.receipt('acknowledged')
        except KeyboardInterrupt:
            if value['op'] != self.operation: return self.receipt('uncertain', 'interrupted')
        except (ValueError, RuntimeError, OSError):
            pass
        if value['op'] == self.operation:
            try: return self.cancel_after_loss()
            except (KeyboardInterrupt, ValueError, RuntimeError, OSError): pass
        return self.receipt('uncertain', 'terminal_result_unconfirmed')


def close_client(node, ros, outcome):
    """Cleanup failure cannot turn a transmitted request into `not_sent`."""
    failed = False
    for close in ((node.destroy_node,) if node is not None else ()) + (ros.try_shutdown,):
        try:
            close()
        except (KeyboardInterrupt, ValueError, RuntimeError, OSError):
            failed = True
    if failed and outcome is not None:
        outcome['client_cleanup_failed'] = True


def run_ros(op, *, machine_id=None, timeout_s=10., run_id=None, mission_id=None, action_id=None, wait=True):
    if op in ('cancel', 'query') and not all((run_id, mission_id, action_id)):
        raise ValueError('exact_action_identity_required')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from orbital.ros_proximity import authority
    from orbital.ros_inspection import OWNER, TOPICS
    node = outcome = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node('orbital_inspection_client_'+uuid.uuid4().hex[:12])
        publisher = node.create_publisher(String, '/inspection/command', 10)
        client = InspectionClient(lambda text: publisher.publish(String(data=text)),
            lambda timeout: rclpy.spin_once(node, timeout_sec=timeout),
            lambda: authority(node, {topic: [(OWNER, 'std_msgs/msg/String')] for topic in TOPICS}))
        for topic, callback in (('/inspection/state', client.observe_state), ('/inspection/reply', client.observe_reply),
                                ('/inspection/result', client.observe_result)):
            node.create_subscription(String, topic, lambda msg, callback=callback: callback(msg.data), 10)
        status = client.discover()
        if op == 'status':
            outcome = status
            return outcome
        value = {'schema_version': 1, 'command_id': uuid.uuid4().hex, 'op': op,
                 'run_id': run_id or status['run_id'], 'mission_id': mission_id or uuid.uuid4().hex,
                 'action_id': action_id or uuid.uuid4().hex,
                 'arguments': {'machine_id': machine_id, 'timeout_s': timeout_s} if op == 'inspect' else {}}
        outcome = client.execute(value, wait=wait)
        return outcome
    finally:
        close_client(node, rclpy, outcome)
