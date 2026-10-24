#!/usr/bin/env python3
"""Real DDS/TF adapter test using synthetic inputs in an otherwise empty domain.

Refuses the live domain 42 and requires dedicated domain 84. Does not launch
Gazebo, publish velocity, call a motion service or change any existing service.
"""
import json
import math
import os
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from orbital.ros_localization import LocalizationMonitor


def stamp(value):
    ns=round(value*1e9)
    return TimeMessage(sec=ns//10**9,nanosec=ns%10**9)


class Fixture(Node):
    def __init__(self):
        super().__init__('orbital_localization_fixture')
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.scan=self.create_publisher(LaserScan,'/scan',10)
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.imu=self.create_publisher(Imu,'/imu',10)
        self.pose=self.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',retained)
        self.seed=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.tf=TransformBroadcaster(self)
        self.events=[]
        self.health_sub=self.create_subscription(String,'/navigation/health',self.on_health,100)
        self.sim=1.
        self.map_x=0.
        self.map_y=0.
        self.publish_scan=True
        self.publish_tf=True
        self.freeze_clock=False
        self.freeze_scan=False
        self.tilt=0.
        self.last_scan_stamp=1.

    def on_health(self,msg):
        self.events.append((time.monotonic(),json.loads(msg.data)))

    def pose_message(self):
        m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=stamp(self.sim)
        m.pose.pose.position.x=self.map_x;m.pose.pose.position.y=self.map_y
        m.pose.pose.orientation.w=1.
        m.pose.covariance[0]=m.pose.covariance[7]=.01;m.pose.covariance[35]=.02
        return m

    def publish(self):
        if not self.freeze_clock:self.sim+=.02
        self.clock.publish(Clock(clock=stamp(self.sim)))
        if self.publish_scan:
            s=LaserScan();s.header.frame_id='lidar_link'
            if not self.freeze_scan:self.last_scan_stamp=self.sim
            s.header.stamp=stamp(self.last_scan_stamp)
            s.angle_min=-math.pi;s.angle_max=math.pi;s.angle_increment=2*math.pi/359
            s.range_min=.1;s.range_max=12.;s.ranges=[2.]*360
            self.scan.publish(s)
        o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_footprint';o.header.stamp=stamp(self.sim)
        o.pose.pose.orientation.w=1.;self.odom.publish(o)
        i=Imu();i.header.frame_id='base_link';i.header.stamp=stamp(self.sim)
        i.orientation.y=math.sin(self.tilt/2);i.orientation.w=math.cos(self.tilt/2);self.imu.publish(i)
        if self.publish_tf:
            transforms=[]
            for parent,child in [('map','odom'),('odom','base_footprint')]:
                t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp(self.sim)
                if parent=='map':t.transform.translation.x=self.map_x;t.transform.translation.y=self.map_y
                t.transform.rotation.w=1.;transforms.append(t)
            self.tf.sendTransform(transforms)


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='84':raise RuntimeError('Synthetic test requires isolated ROS_DOMAIN_ID=84')
    rclpy.init()
    preflight=Node('orbital_synthetic_preflight')
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(preflight,timeout_sec=.05)
    others=[n for n in preflight.get_node_names() if n!='orbital_synthetic_preflight']
    preflight.destroy_node()
    if others:
        rclpy.shutdown();raise RuntimeError('Domain is not empty; refusing synthetic publishers: '+repr(others))
    monitor=LocalizationMonitor();fixture=Fixture();executor=SingleThreadedExecutor()
    executor.add_node(monitor);executor.add_node(fixture)
    report=[]
    def pump(seconds):
        end=time.monotonic()+seconds
        next_publish=time.monotonic()
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_publish:
                fixture.publish();next_publish=now+.02
            executor.spin_once(timeout_sec=.002)
    def wait_health(healthy, seconds=2, reason=None):
        start=time.monotonic()
        while time.monotonic()-start<seconds:
            pump(.04)
            if fixture.events:
                t,h=fixture.events[-1]
                if t>=start and h['healthy']==healthy and (reason is None or h['reason']==reason):
                    return h,time.monotonic()-start
        raise AssertionError(('health not observed',healthy,reason,fixture.events[-8:]))
    try:
        pump(1)
        fixture.pose.publish(fixture.pose_message())
        baseline,_=wait_health(True)
        report.append({'case':'valid_tf_and_messages','passed':True})
        # Pose is intentionally not republished while independent inputs advance.
        pump(.8);assert fixture.events[-1][1]['healthy']
        report.append({'case':'stationary_retained_pose','passed':True})
        for mode in ('scan_loss','replayed_scan','tilt','tf_loss','clock_pause','duplicate_authority'):
            before=fixture.events[-1][1]
            if mode=='scan_loss':fixture.publish_scan=False
            elif mode=='replayed_scan':fixture.freeze_scan=True
            elif mode=='tilt':fixture.tilt=.522876
            elif mode=='tf_loss':fixture.publish_tf=False
            elif mode=='clock_pause':fixture.freeze_clock=True
            else:duplicate=fixture.create_publisher(String,'/navigation/health',1)
            unhealthy,delay=wait_health(False)
            assert unhealthy['epoch']>before['epoch'],(mode,unhealthy,before)
            assert delay<.8,(mode,delay)
            if mode=='duplicate_authority':fixture.destroy_publisher(duplicate)
            fixture.publish_scan=True;fixture.freeze_scan=False;fixture.tilt=0.;fixture.publish_tf=True;fixture.freeze_clock=False
            recovered,_=wait_health(True)
            assert recovered['epoch']>=unhealthy['epoch'] and recovered['run_id']==baseline['run_id']
            report.append({'case':mode,'invalid_after_wall_s':delay,'reason':unhealthy['reason'],'epoch':recovered['epoch']})
        before=fixture.events[-1][1]
        fixture.sim=.05
        reset,_=wait_health(False,reason='initial_pose_required')
        assert reset['epoch']>before['epoch']
        fixture.pose.publish(fixture.pose_message());pump(.3)
        assert not fixture.events[-1][1]['healthy'],'unseeded new pose bypassed reset'
        fixture.seed.publish(fixture.pose_message());pump(.15)
        fixture.pose.publish(fixture.pose_message())
        recovered,_=wait_health(True)
        report.append({'case':'reset_requires_explicit_seed','epoch':recovered['epoch'],'passed':True})
        # The monitor cannot publish raw velocities, TF, seed or guard commands.
        published={name for name,_ in monitor.get_publisher_names_and_types_by_node(monitor.get_name(),monitor.get_namespace())}
        assert published <= {'/navigation/health','/rosout','/parameter_events'},published
        report.append({'case':'monitor_has_no_control_publishers','publishers':sorted(published)})
        print(json.dumps({'passed':True,'domain':84,'checks':report,'scope':'synthetic ROS transport/TF, not real navigation or stop enforcement'},indent=2))
    except Exception as exc:
        print(json.dumps({'passed':False,'error':str(exc),'checks':report},indent=2))
        raise
    finally:
        executor.remove_node(fixture);executor.remove_node(monitor)
        fixture.destroy_node();monitor.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
