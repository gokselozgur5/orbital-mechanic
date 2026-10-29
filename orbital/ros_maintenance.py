"""Maintenance runs beside sampling in the exact station-telemetry executor."""
from collections import deque
import time

from orbital.camera_observation import CAMERA_TOPICS
from orbital.inspection_protocol import canonical, MAX_COMMAND_BYTES
from orbital.inspection_worker import OwnerLock, CaptureProcess
from orbital.maintenance_interlock import HoldEvidence
from orbital.maintenance_owner import MaintenanceOwner
from orbital.maintenance_journal import Journal, JournalUnavailable
from orbital.ros_inspection import LiveGuard
from orbital.ros_proximity import authority, proximity_sources

TOPICS = ('/maintenance/state', '/maintenance/reply', '/maintenance/result')
OWNER = 'station_telemetry'


class DeferredGuard:
    """The telemetry publisher can start before the map/navigation prerequisites."""
    def __init__(self, root, node, buffer, lock, registry, holds, *, now, time_type, tf_error):
        self.root, self.node, self.buffer, self.lock, self.registry = root, node, buffer, lock, registry
        self.holds, self.now, self.time_type, self.tf_error = holds, now, time_type, tf_error
        self.real = None
        self.next_attempt = 0.

    def ready(self):
        if self.real is None:
            if self.now() < self.next_attempt:
                return False
            self.next_attempt = self.now()+1.
            expected = {**proximity_sources(), **CAMERA_TOPICS,
                        '/station/telemetry': [(OWNER, 'std_msgs/msg/String')]}
            authority(self.node, expected)
            candidate = LiveGuard(self.root, self.node, self.buffer, self.lock, now=self.now,
                                  time_type=self.time_type, tf_error=self.tf_error, owner=OWNER, topics=TOPICS)
            if candidate.registry.identity != self.registry.identity:
                raise ValueError('station_registry_mismatch')
            self.real = candidate
        self.holds.authority()
        return self.real.ready()

    def begin(self, machine): self.real.begin(machine)
    def end(self):
        if self.real is not None: self.real.end()
    def identity(self): return self.real.identity()
    def sample(self):
        if not self.ready(): raise ValueError('maintenance_sources_unavailable')
        return self.real.sample()
    def health(self, message):
        if self.real is not None: self.real.health(message)
    def odometry(self, message):
        if self.real is not None: self.real.odometry(message)


class StationMaintenance:
    def __init__(self, root, node, model, *, now=time.monotonic, expected_trial_id=None):
        from nav_msgs.msg import Odometry
        from rclpy.clock import ClockType
        from rclpy.qos import QoSProfile, qos_profile_sensor_data
        from rclpy.time import Time
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformException, TransformListener

        self.node, self.now, self.String = node, now, String
        self.lock = OwnerLock(root, service='station-service')
        self.listener = self.owner = None
        self.journal = None
        self.configuration = None
        self.truth = None
        self.pinned = None
        self.broken = False
        self.queue = deque()
        self.next_state = 0.
        try:
            self.journal = Journal(self.lock)
            if model is None:
                from orbital.machine_registry import MachineRegistry
                from orbital.scenario_input import PreparedInput
                self.configuration = PreparedInput(self.journal, MachineRegistry.load(root),
                                                   expected_trial_id=expected_trial_id)
                model = self.configuration.model()
            self.model = model
            self.publishers = {name:node.create_publisher(String, name, QoSProfile(depth=10)) for name in TOPICS}
            hold_pub = node.create_publisher(String, '/motion/maintenance', QoSProfile(depth=8))
            self.holds = HoldEvidence(now=now, sim_ns=lambda:node.get_clock().now().nanoseconds,
                                     publish=lambda text:hold_pub.publish(String(data=text)), authority=self.authority)
            buffer = Buffer(node=node)
            self.listener = TransformListener(buffer, node)
            self.live = DeferredGuard(root, node, buffer, self.lock, model.registry, self.holds, now=now,
                time_type=lambda stamp:Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME), tf_error=TransformException)
            self.owner = MaintenanceOwner(root, model, self.live, self.holds,
                lambda payload:CaptureProcess(root, self.lock.fd, payload, now()), now=now,
                sim_ns=lambda:node.get_clock().now().nanoseconds,
                authority=self.authority,
                journal=self.journal,
                on_result=lambda result:self.publish('/maintenance/result', result))
            if self.configuration is not None:
                from orbital.station_truth import StationTruth
                self.truth = StationTruth(self.lock, self.journal, self.configuration, model, now=now)
                model.truth = self.truth
            node.create_subscription(String, '/maintenance/command', self.receive, QoSProfile(depth=16))
            node.create_subscription(String, '/motion/state', lambda msg:self.holds.on_guard(msg.data), 10)
            node.create_subscription(String, '/navigation/state', lambda msg:self.holds.on_navigation(msg.data), 10)
            node.create_subscription(String, '/navigation/health', self.live.health, 10)
            node.create_subscription(Odometry, '/odom', self.live.odometry, qos_profile_sensor_data)
            node.create_subscription(String, '/station/telemetry', self.telemetry, 5)
        except BaseException:
            self.close()
            raise

    def authority(self):
        if self.broken:
            raise ValueError('maintenance_authority_changed')
        self.lock.verify()
        try:
            self.journal.verify()
            if self.configuration is not None: self.configuration.verify()
        except JournalUnavailable:
            self.broken = True
            raise ValueError('maintenance_journal_unavailable') from None
        expected = {'/motion/state': [('motion_guard', 'std_msgs/msg/String')],
                    '/navigation/state': [('orbital_navigation_supervisor', 'std_msgs/msg/String')],
                    '/station/telemetry': [(OWNER, 'std_msgs/msg/String')],
                    '/motion/maintenance': [(OWNER, 'std_msgs/msg/String')],
                    **{name:[(OWNER, 'std_msgs/msg/String')] for name in TOPICS}}
        try:
            peers = authority(self.node, expected)
            if self.pinned is None:
                self.pinned = peers
            elif peers != self.pinned:
                raise ValueError('maintenance_authority_changed')
        except (ValueError, RuntimeError, OSError):
            if self.pinned is not None:
                self.broken = True
            raise

    def publish(self, topic, value):
        self.publishers[topic].publish(self.String(data=canonical(value)))

    def telemetry(self, message):
        try:
            self.authority()
        except (ValueError, RuntimeError, OSError):
            if self.pinned is not None or self.broken:
                self.owner.reset_evidence.fail('maintenance_authority_changed')
            return
        self.owner.telemetry(message.data)

    def receive(self, message):
        if isinstance(message.data, str) and len(message.data) <= MAX_COMMAND_BYTES and len(self.queue) < 16:
            self.queue.append(message.data)

    def tick(self):
        for _ in range(min(4, len(self.queue))):
            self.publish('/maintenance/reply', self.owner.handle(self.queue.popleft()))
        self.owner.tick()
        if self.now() >= self.next_state:
            self.publish('/maintenance/state', self.owner.state())
            self.next_state = self.now()+.1

    def close(self, *, complete=False):
        cleaned = False
        try:
            if self.owner is not None:
                self.owner.shutdown()
                action = self.owner.active
                if action is not None and action.inspector is not None:
                    inspector = action.inspector
                    inspector.shutdown()
                    # Cancellation/reap needs no live ROS or power mutation.
                    inspector.sim_ns = lambda:action.started_sim
                    while inspector.active is not None:
                        inspector.tick()
                        time.sleep(.01)
                # An unresolved external guard hold survives this process. Do
                # not manufacture a release or terminal result during teardown.
                self.live.end()
            cleaned = True
        finally:
            try:
                if self.truth is not None:
                    current = self.journal.history.scenarios[self.journal.history.current]
                    settled = (self.owner.active is None and self.owner.cancellation is None
                               and all(self.journal.history.actions[a]['result'] is not None for a in current['actions']))
                    self.truth.finish(complete and cleaned and settled)
            finally:
                try:
                    if self.listener is not None: self.listener.unregister()
                finally:
                    try:
                        if self.configuration is not None: self.configuration.close()
                    finally:
                        try:
                            if self.journal is not None: self.journal.close()
                        finally: self.lock.close()
