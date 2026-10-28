"""Exact maintenance commands; a missing result never authorizes another effect."""
import time
import uuid

from orbital import maintenance_protocol as protocol
from orbital.inspection_client import InspectionClient, close_client
from orbital.inspection_protocol import canonical


class MaintenanceClient(InspectionClient):
    def __init__(self, publish, spin, authority, *, now=time.monotonic):
        super().__init__(publish, spin, authority, now=now, protocol=protocol,
                         family='maintenance', operation='isolate')

    def execute(self, value, *, wait=True):
        self.operation = value['op'] if value.get('op') in ('reset', 'restore') else 'isolate'
        return super().execute(value, wait=wait)

    def observe_state(self, text):
        try:
            value = protocol.state(text)
            if self.latest is not None:
                if (value['run_id'] != self.latest['run_id'] or value['station_run_id'] != self.latest['station_run_id']
                        or value['sequence'] <= self.latest['sequence']
                        or value['sim_stamp_ns'] < self.latest['sim_stamp_ns']
                        or value['control_revision'] < self.latest['control_revision']):
                    raise ValueError('server_generation_changed')
                if value['sim_stamp_ns'] == self.latest['sim_stamp_ns']:
                    return  # A paused clock cannot renew readiness or result evidence.
            if value['sim_stamp_ns'] == 0:
                return
            super().observe_state(text)
        except (ValueError, TypeError, KeyError):
            self.failure = 'invalid_or_changed_server_state'


def run_ros(op, *, machine_id=None, timeout_s=10., run_id=None, mission_id=None, action_id=None,
            isolation_action_id=None, reset_action_id=None, wait=True):
    if op in ('cancel', 'query') and not all((run_id, mission_id, action_id)):
        raise ValueError('exact_action_identity_required')
    if op == 'reset' and not all((run_id, mission_id, isolation_action_id)):
        raise ValueError('exact_isolation_identity_required')
    if op == 'restore' and not all((run_id, mission_id, reset_action_id)):
        raise ValueError('exact_reset_identity_required')
    if (op != 'reset' and isolation_action_id is not None) or (op != 'restore' and reset_action_id is not None):
        raise ValueError('unexpected_prerequisite_identity')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from orbital.ros_proximity import authority
    from orbital.ros_maintenance import OWNER, TOPICS
    node = outcome = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node('orbital_maintenance_client_'+uuid.uuid4().hex[:12])
        publisher = node.create_publisher(String, '/maintenance/command', 10)
        client = MaintenanceClient(lambda text:publisher.publish(String(data=text)),
            lambda timeout:rclpy.spin_once(node, timeout_sec=timeout),
            lambda:authority(node, {topic:[(OWNER, 'std_msgs/msg/String')] for topic in TOPICS}))
        for topic, callback in (('/maintenance/state', client.observe_state), ('/maintenance/reply', client.observe_reply),
                                ('/maintenance/result', client.observe_result)):
            node.create_subscription(String, topic, lambda msg, callback=callback:callback(msg.data), 10)
        status = client.discover()
        if op == 'status':
            outcome = status
            return outcome
        value = {'schema_version': 1, 'command_id': uuid.uuid4().hex, 'op': op,
            'run_id': run_id or status['run_id'], 'mission_id': mission_id or uuid.uuid4().hex,
            'action_id': action_id or uuid.uuid4().hex, 'arguments': {}}
        if op in ('isolate', 'reset', 'restore'):
            value['arguments'] = {'machine_id': machine_id, 'timeout_s': timeout_s,
                'station_run_id': status['station_run_id'], 'control_revision': status['control_revision']}
            if op == 'reset': value['arguments']['isolation_action_id'] = isolation_action_id
            if op == 'restore': value['arguments']['reset_action_id'] = reset_action_id
        outcome = client.execute(value, wait=wait)
        return outcome
    finally:
        close_client(node, rclpy, outcome)
