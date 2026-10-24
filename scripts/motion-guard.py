#!/usr/bin/env python3
"""ROS adapter for the bounded velocity policy; lifecycle is owned by orbital-demo."""
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from orbital.motion import MotionGuard


class GuardNode(Node):
    def __init__(self):
        super().__init__('motion_guard')
        self.guard = MotionGuard()
        self.output_pub = self.create_publisher(Twist, '/motion/guarded_cmd_vel', 1)
        self.state_pub = self.create_publisher(String, '/motion/state', 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.feedback, qos_profile_sensor_data)
        self.input_sub = self.create_subscription(TwistStamped, '/motion/request', self.command, QoSProfile(depth=1))
        self.stop_service = self.create_service(Trigger, '/motion/stop', self.stop)
        self.reset_service = self.create_service(Trigger, '/motion/reset_stop', self.reset)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self.tick, clock=self.steady_clock)

    def feedback(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
            self.guard.stop('invalid_odometry_frame')
            self.guard.feedback_at = None
            self.tick()
            return
        velocity = msg.twist.twist
        speed = math.hypot(velocity.linear.x, velocity.linear.y, velocity.linear.z)
        angular = math.hypot(velocity.angular.x, velocity.angular.y, velocity.angular.z)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.guard.feedback(speed, angular, stamp, time.monotonic())

    def command(self, msg):
        v = msg.twist
        unused = (v.linear.y, v.linear.z, v.angular.x, v.angular.y)
        if msg.header.frame_id != 'base_link' or any(not math.isfinite(x) or x != 0 for x in unused):
            self.guard.stop('invalid_command_frame_or_axis')
        else:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self.guard.command(v.linear.x, v.angular.z, stamp, time.monotonic())
        self.tick()

    def stop(self, request, response):
        self.guard.stop()
        self.tick()
        response.success = True
        response.message = self.guard.reason
        return response

    def reset(self, request, response):
        response.success = self.guard.reset(time.monotonic())
        response.message = self.guard.reason
        self.tick()
        return response

    def tick(self):
        linear, angular = self.guard.output(time.monotonic())
        output = Twist()
        output.linear.x = linear
        output.angular.z = angular
        self.output_pub.publish(output)
        self.state_pub.publish(String(data=json.dumps({
            'latched': self.guard.latched, 'reason': self.guard.reason,
            'linear_mps': linear, 'angular_rps': angular})))


def main():
    rclpy.init()
    node = GuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.guard.stop('shutdown')
            node.tick()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
