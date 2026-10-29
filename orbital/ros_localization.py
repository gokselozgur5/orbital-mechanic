"""ROS observation adapter. Publishes health only; never publishes motion or TF."""
from dataclasses import asdict
import json
import math
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from orbital.localization import Health, LocalizationHealth, TransformSample


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec*1e-9


def xyz(v):
    return v.x, v.y, v.z


def quaternion(q):
    return q.x, q.y, q.z, q.w


class LocalizationMonitor(Node):
    """Single-executor observer suitable for composition with a future supervisor.

    Consumers must evaluate every health transition and enforce their own steady
    deadline; a lost health publisher is not permission to keep moving.
    """
    def __init__(self):
        super().__init__('orbital_localization_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.validator = LocalizationHealth()
        self.last_clock = None
        self.clock_advanced_at = None
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.health_pub = self.create_publisher(String, '/navigation/health', 10)
        self.last_health = Health(False, False, 'startup', 0.)
        self.sequence = 0
        self.epoch = 0
        self.run_id = str(uuid.uuid4())
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.inputs = [
            self.create_subscription(ClockMessage, '/clock', self.on_clock, qos_profile_sensor_data),
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odom', self.on_odometry, qos_profile_sensor_data),
            self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data),
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, retained),
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_seed, 10),
        ]
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self.evaluate, clock=self.steady_clock)

    def emit(self, health):
        if ((self.last_health.healthy and not health.healthy)
                or health.reason in ('simulation_reset', 'initial_pose_replaced')):
            self.epoch += 1
        self.last_health = health
        self.sequence += 1
        self.health_pub.publish(String(data=json.dumps({**asdict(health), 'schema_version': 1, 'run_id': self.run_id, 'epoch': self.epoch, 'sequence': self.sequence})))

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec*1e-9
        now = time.monotonic()
        if self.last_clock is not None and value < self.last_clock:
            self.buffer.clear()
            self.emit(self.validator.clock_reset(value))
        if self.last_clock is None or value != self.last_clock:
            self.clock_advanced_at = now
        self.last_clock = value

    def on_scan(self, msg):
        geometry_ok = (abs(msg.angle_min+math.pi)<.001 and abs(msg.angle_max-math.pi)<.001
                       and abs(msg.angle_increment-2*math.pi/359)<.00001)
        self.validator.scan(stamp=stamp(msg.header), now=time.monotonic(),
            frame=msg.header.frame_id if geometry_ok else '', ranges=msg.ranges,
            range_min=msg.range_min, range_max=msg.range_max)

    def on_odometry(self, msg):
        p=msg.pose.pose;v=msg.twist.twist
        self.validator.odometry(stamp=stamp(msg.header), now=time.monotonic(),
            parent=msg.header.frame_id, child=msg.child_frame_id, position=xyz(p.position),
            orientation=quaternion(p.orientation), linear=xyz(v.linear), angular=xyz(v.angular))

    def on_imu(self, msg):
        self.validator.imu(stamp=stamp(msg.header), now=time.monotonic(),
            frame=msg.header.frame_id if msg.orientation_covariance[0] != -1 else '',
            orientation=quaternion(msg.orientation))

    def on_pose(self, msg):
        p=msg.pose.pose
        self.validator.pose(stamp=stamp(msg.header), now=time.monotonic(), frame=msg.header.frame_id,
            position=xyz(p.position), orientation=quaternion(p.orientation), covariance=msg.pose.covariance)

    def on_seed(self, msg):
        value=stamp(msg.header)
        if value == 0 and self.last_clock is not None:
            value=self.last_clock
        valid=(msg.header.frame_id == 'map' and self.last_clock is not None
               and 0 <= value <= self.last_clock+.1)
        self.validator.replace_initial_pose(value if valid else math.nan)
        self.emit(Health(False, False, 'initial_pose_replaced', self.last_clock or 0.))

    def evaluate(self):
        now=time.monotonic()
        sim=self.last_clock
        if len(self.get_publishers_info_by_topic('/navigation/health')) != 1:
            self.emit(Health(False, False, 'health_authority_ambiguous', sim or 0.))
            return
        if sim is None or self.clock_advanced_at is None or now-self.clock_advanced_at >= .5:
            self.emit(Health(False, False, 'simulation_clock_stale', sim or 0.))
            return
        transform=None
        if sim > .1:
            requested=Time(nanoseconds=round((sim-.1)*1e9), clock_type=ClockType.ROS_TIME)
            try:
                msg=self.buffer.lookup_transform('map', 'base_footprint', requested)
                transform=TransformSample(stamp(msg.header), msg.header.frame_id, msg.child_frame_id,
                                          xyz(msg.transform.translation), quaternion(msg.transform.rotation))
            except TransformException:
                pass
        self.emit(self.validator.evaluate(now=now, sim_time=sim, transform=transform))

    def destroy_node(self):
        self.listener.unregister()
        return super().destroy_node()
