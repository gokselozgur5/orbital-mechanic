#!/usr/bin/env python3
"""Bounded CLI controls for the simulated rover; never publishes raw drive output."""
import argparse
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Operator(Node):
    def __init__(self):
        super().__init__('orbital_operator', parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.state = None
        self.odom = None
        self.camera_count = 0
        self.scan_count = 0
        self.state_at = self.odom_at = 0
        self.state_sub = self.create_subscription(String,'/motion/state',self.on_state,1)
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.camera_sub = self.create_subscription(Image,'/camera/image_raw',self.on_image,qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.publisher = self.create_publisher(TwistStamped,'/motion/request',1)
        self.arm_client = self.create_client(Trigger,'/motion/reset_stop')
        self.stop_client = self.create_client(Trigger,'/motion/stop')

    def on_state(self, msg):
        self.state=json.loads(msg.data); self.state_at=time.monotonic()

    def on_odom(self, msg):
        self.odom=msg; self.odom_at=time.monotonic()

    def on_image(self, msg): self.camera_count+=1
    def on_scan(self, msg): self.scan_count+=1

    def spin(self, duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=min(.02,max(0,end-time.monotonic())))

    def call(self, client):
        if not client.wait_for_service(timeout_sec=3): raise RuntimeError('Motion service unavailable')
        future=client.call_async(Trigger.Request())
        end=time.monotonic()+3
        while not future.done() and time.monotonic()<end: self.spin(.02)
        if not future.done(): raise RuntimeError('Motion service timed out')
        response=future.result()
        if not response.success: raise RuntimeError(response.message)
        self.spin(.1)
        return response.message

    def ready(self):
        end=time.monotonic()+5
        while not (self.state and self.odom and self.get_clock().now().nanoseconds>0) and time.monotonic()<end: self.spin(.05)
        if self.get_clock().now().nanoseconds<=0: raise RuntimeError('Simulation clock unavailable')
        self.fresh()

    def fresh(self):
        now=time.monotonic()
        if not self.state or not self.odom or max(now-self.state_at,now-self.odom_at)>.3:
            raise RuntimeError('Fresh motion state and odometry required')

    def report(self):
        v=self.odom.twist.twist if self.odom else None
        return {'guard':self.state, 'measured_linear_mps':v.linear.x if v else None,
                'measured_angular_rps':v.angular.z if v else None,
                'camera_frames':self.camera_count,'lidar_scans':self.scan_count}

    def jog(self, direction, duration):
        self.ready()
        if self.state['latched']: raise RuntimeError('Stopped: run arm explicitly before jog')
        linear,angular={'forward':(.12,0.),'reverse':(-.12,0.),
                        'left':(0.,.3),'right':(0.,-.3)}[direction]
        end=time.monotonic()+duration
        try:
            while time.monotonic()<end:
                self.fresh()
                if self.state['latched']: raise RuntimeError('Guard stopped: '+self.state['reason'])
                msg=TwistStamped();msg.header.frame_id='base_link'
                msg.header.stamp=self.get_clock().now().to_msg()
                msg.twist.linear.x=linear;msg.twist.angular.z=angular
                self.publisher.publish(msg)
                self.spin(min(.05,max(0,end-time.monotonic())))
        finally:
            if rclpy.ok(): self.call(self.stop_client)
        self.spin(.3)
        return self.report()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['arm','stop','status','jog'])
    parser.add_argument('direction',nargs='?',choices=['forward','reverse','left','right'])
    parser.add_argument('--duration',type=float,default=.5,help='Jog seconds, >0 and <=2 (default .5)')
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=2: parser.error('duration must be finite and in (0, 2]')
    if args.command=='jog' and args.direction is None: parser.error('jog requires a direction')
    if os.environ.get('ROS_DOMAIN_ID')!='42': parser.error('Set ROS_DOMAIN_ID=42 for this simulator')
    rclpy.init();node=Operator();code=0
    try:
        if args.command=='stop': result={'result':node.call(node.stop_client)}
        else:
            node.ready()
            if args.command=='arm': result={'result':node.call(node.arm_client)}
            elif args.command=='jog': result=node.jog(args.direction,args.duration)
            else: node.spin(1);result=node.report()
        print(json.dumps(result,indent=2))
    except (KeyboardInterrupt,ExternalShutdownException): code=130
    except RuntimeError as error:
        print(json.dumps({'error':str(error)}));code=1
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    raise SystemExit(code)


if __name__=='__main__': main()
