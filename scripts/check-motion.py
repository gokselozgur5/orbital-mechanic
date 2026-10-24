#!/usr/bin/env python3
"""Moves the simulated robot in the clear startup area; no physical robot support."""
import json
import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self):
        super().__init__('motion_acceptance', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.odom = None
        self.state = None
        self.states = deque(maxlen=200)
        self.odom_sub = self.create_subscription(Odometry, '/odom', lambda m: setattr(self,'odom',m), qos_profile_sensor_data)
        self.state_sub = self.create_subscription(String, '/motion/state', self.on_state, 10)
        self.pub = self.create_publisher(TwistStamped, '/motion/request', 1)
        self.stop_client = self.create_client(Trigger, '/motion/stop')
        self.reset_client = self.create_client(Trigger, '/motion/reset_stop')
        self.results = []

    def on_state(self, msg):
        self.state = json.loads(msg.data)
        self.states.append((time.monotonic(), self.state))

    def spin(self, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end: rclpy.spin_once(self, timeout_sec=.02)

    def call(self, client):
        assert client.wait_for_service(timeout_sec=3), 'motion service unavailable'
        future = client.call_async(Trigger.Request())
        end = time.monotonic()+3
        while not future.done() and time.monotonic()<end: rclpy.spin_once(self,timeout_sec=.02)
        assert future.done(), 'motion service timeout'
        result = future.result()
        assert result.success, result.message
        self.spin(.15)

    def send(self, linear=0.0, angular=0.0, frame="base_link", lateral=0.0, stale=False):
        msg = TwistStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear)
        msg.twist.linear.y = float(lateral)
        if stale: msg.header.stamp.sec = 0; msg.header.stamp.nanosec = 0
        msg.twist.angular.z = float(angular)
        self.pub.publish(msg)

    def drive(self, linear, angular, duration):
        end = time.monotonic()+duration
        while time.monotonic()<end:
            self.send(linear,angular)
            self.spin(.05)

    def position(self):
        p=self.odom.pose.pose.position
        return p.x,p.y

    def stationary(self):
        v=self.odom.twist.twist
        return abs(v.linear.x)<.02 and abs(v.angular.z)<.05

    def run(self):
        self.spin(2)
        assert self.odom and self.state, 'missing feedback'
        # This harness is valid only near the clear origin, not arbitrary worlds/poses.
        assert math.hypot(*self.position())<.25, 'robot not at clear startup position'
        assert self.state['latched'], 'expected stopped startup'
        self.call(self.reset_client)
        before=self.position()
        self.drive(.12,0,1.5)
        last_sent=time.monotonic()
        self.spin(1.0)
        distance=math.dist(before,self.position())
        stops=[t for t,s in self.states if t>=last_sent and s['reason']=='command_timeout']
        assert .10<distance<.30, ('unexpected forward distance',distance)
        assert stops and stops[0]-last_sent<.55, 'watchdog did not latch promptly'
        assert self.stationary(), 'robot still moving after watchdog'
        self.results.append({'case':'forward_and_timeout','distance_m':distance,'observed_latch_delay_s':stops[0]-last_sent})
        before=self.position()
        self.drive(.12,0,.4)
        self.spin(.2)
        assert math.dist(before,self.position())<.01, 'latched robot restarted'
        self.results.append({'case':'late_commands_blocked','passed':True})
        self.call(self.reset_client)
        before=self.position(); self.drive(-.12,0,1.5)
        self.call(self.stop_client); self.spin(.6)
        assert math.dist(before,self.position())>.10 and self.stationary(), 'reverse/stop failed'
        self.results.append({'case':'reverse_and_explicit_stop','passed':True})
        self.call(self.reset_client); self.drive(0,.3,.8)
        assert self.odom.twist.twist.angular.z>.1, 'rotation not observed'
        self.call(self.stop_client); self.spin(.6)
        assert self.stationary() and self.state['latched'], 'rotation did not stop'
        self.results.append({'case':'rotation_and_stop','passed':True})
        self.call(self.reset_client); self.send(.3,0); self.spin(.2)
        assert self.state['latched'] and self.state['reason']=='invalid_command', 'overspeed accepted'
        self.results.append({'case':'overspeed_rejected','passed':True})
        for name, kwargs, reason in (
            ('wrong_frame', {'frame':'odom'}, 'invalid_command_frame_or_axis'),
            ('unsupported_axis', {'lateral':.1}, 'invalid_command_frame_or_axis'),
            ('stale_stamp', {'stale':True}, 'stale_command'),
        ):
            self.call(self.reset_client)
            self.send(.1, **kwargs); self.spin(.2)
            assert self.state['latched'] and self.state['reason']==reason, name
            assert self.stationary(), name + ' produced motion'
            self.results.append({'case':name,'passed':True})


rclpy.init()
p=Probe()
try:
    p.run()
    print(json.dumps({'passed':True,'checks':p.results},indent=2))
finally:
    try: p.call(p.stop_client)
    finally:
        p.destroy_node()
        rclpy.shutdown()
