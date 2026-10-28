"""ROS String transport for typed inspection commands; not a ROSIDL action type."""
from collections import deque
import hashlib
import json
import time

from orbital.camera_observation import CAMERA_TOPICS, CameraContract
from orbital.inspection_owner import InspectionOwner
from orbital.inspection_protocol import MAX_COMMAND_BYTES, canonical
from orbital.inspection_worker import CaptureProcess, OwnerLock
from orbital.machine_registry import MachineRegistry
from orbital.map_snapshot import MapSnapshot
from orbital.proximity import ProximityEvidence, Unavailable
from orbital.ros_proximity import authority, proximity_sources

TOPICS = ('/inspection/state', '/inspection/reply', '/inspection/result')
OWNER = 'orbital_inspection_server'


class LiveGuard:
    def __init__(self, root, node, buffer, lock, *, now=time.monotonic, time_type, tf_error):
        self.registry = MachineRegistry.load(root)
        self.camera = CameraContract.load(self.registry)
        self.mapping = MapSnapshot.prepare(self.registry)
        self.node, self.buffer, self.lock = node, buffer, lock
        self.now, self.time_type, self.tf_error = now, time_type, tf_error
        self.expected = {**proximity_sources(), **CAMERA_TOPICS,
                         '/station/telemetry': [('station_telemetry', 'std_msgs/msg/String')]}
        self.pinned = self.self_pinned = None
        self.digest = None
        self.broken = False
        self.robot = None
        self.generation = None

    def ready(self):
        if self.broken:
            return False
        try:
            self.lock.verify()
            self.registry.verify_current()
            self.camera.verify_current()
            self.mapping.verify_current(self.registry.root, self.mapping.yaml_path)
            if self.node.get_parameter('use_sim_time').value is not True:
                raise ValueError('simulation_clock_mode_changed')
            peers = authority(self.node, self.expected)
            own = authority(self.node, {name: [(OWNER, 'std_msgs/msg/String')] for name in TOPICS})
            if self.pinned is None:
                self.digest = hashlib.sha256(json.dumps(peers, separators=(',', ':')).encode()).hexdigest()
                self.pinned, self.self_pinned = peers, own
            elif peers != self.pinned or own != self.self_pinned:
                raise ValueError('publisher_generation_changed')
            return self.node.get_clock().now().nanoseconds > 0
        except (ValueError, OSError, RuntimeError):
            if self.pinned is not None:
                self.broken = True
            return False

    def begin(self, machine):
        self.robot = ProximityEvidence(self.registry, machine)
        self.generation = None

    def end(self):
        self.robot = None
        self.generation = None

    def identity(self):
        return {'registry_identity': self.registry.identity, 'authority_identity': self.digest}

    def health(self, msg):
        if self.robot is not None:
            self.robot.health(msg.data, self.now())

    def odometry(self, msg):
        if self.robot is not None:
            self.robot.odometry(msg, self.now(), self.node.get_clock().now().nanoseconds)

    def sample(self):
        if not self.ready():
            raise ValueError('inspection_source_unavailable')
        if self.robot.error:
            raise ValueError('robot_evidence_invalid')
        sim = self.node.get_clock().now().nanoseconds
        try:
            settled = self.robot.settled(self.now(), sim)
            if not settled['settled']:
                raise Unavailable('awaiting_settled_robot')
            if self.generation is not None and settled['generation'] != self.generation:
                raise ValueError('inspection_stationarity_changed')
            requested = self.robot.requested(sim)
            tf = self.buffer.lookup_transform('map', 'base_footprint', self.time_type(requested))
            # Source/clock/file checks may finish later than the TF request.
            if not self.ready():
                raise ValueError('inspection_source_unavailable')
            result = self.robot.measure(requested, self.node.get_clock().now().nanoseconds, self.now(), tf)
            if not result['within_settled_inspection_pose']:
                raise ValueError('inspection_pose_required')
            self.generation = settled['generation']
            return result
        except (Unavailable, self.tf_error):
            if self.robot.error or self.generation is not None:
                raise ValueError('inspection_precondition_lost') from None
            return None


def serve(root, *, stop_requested=lambda: False, now=time.monotonic):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.clock import ClockType
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    lock = OwnerLock(root)
    node = listener = owner = None
    try:
        # Keep the context alive while the owner cancels and reaps its worker.
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node(OWNER, parameter_overrides=[Parameter('use_sim_time', value=True)])
        end = now() + 2.
        while now() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if any(node.get_publishers_info_by_topic(topic) for topic in TOPICS):
            raise RuntimeError('inspection_owner_already_present')
        publishers = {topic: node.create_publisher(String, topic, QoSProfile(depth=10)) for topic in TOPICS}
        buffer = Buffer(node=node)
        listener = TransformListener(buffer, node)
        guard = LiveGuard(root, node, buffer, lock, now=now,
            time_type=lambda stamp: Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME), tf_error=TransformException)

        def publish(topic, value):
            publishers[topic].publish(String(data=canonical(value)))

        owner = InspectionOwner(root, guard,
            lambda payload: CaptureProcess(root, lock.fd, payload, now()), now=now,
            sim_ns=lambda: node.get_clock().now().nanoseconds,
            on_result=lambda result: publish('/inspection/result', result))
        queue = deque()

        def receive(msg):
            # ROS has already allocated the incoming message. Bound retention and
            # per-turn work; overflow is absence of an ACK, never silent success.
            if len(queue) < 16 and isinstance(msg.data, str) and len(msg.data) <= MAX_COMMAND_BYTES:
                queue.append(msg.data)

        node.create_subscription(String, '/inspection/command', receive, QoSProfile(depth=16))
        node.create_subscription(String, '/navigation/health', guard.health, 10)
        node.create_subscription(Odometry, '/odom', guard.odometry, qos_profile_sensor_data)
        next_state = 0.
        while rclpy.ok() and not stop_requested():
            rclpy.spin_once(node, timeout_sec=.01)
            # Callbacks and ownership transitions share this single thread.
            for _ in range(min(4, len(queue))):
                publish('/inspection/reply', owner.handle(queue.popleft()))
            owner.tick()
            if now() >= next_state:
                publish('/inspection/state', owner.state())
                next_state = now() + .1
    finally:
        try:
            if owner is not None:
                owner.shutdown()
                # No confirmed cancellation or lock release before exact reap.
                # An uninterruptible child keeps this process stopping; the
                # inherited lock survives a service-level kill of this parent.
                while owner.active is not None:
                    owner.tick()
                    time.sleep(.01)
        finally:
            try:
                if listener is not None: listener.unregister()
                if node is not None: node.destroy_node()
                rclpy.try_shutdown()
            finally:
                lock.close()
