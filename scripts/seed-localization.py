#!/usr/bin/env python3
"""Explicit operator seed for a freshly started world at its known origin.

Stops and verifies stationary feedback before publishing. A returned AMCL pose
acknowledges processing; it is not evidence of convergence or ground-truth accuracy.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fresh-world-origin',action='store_true',required=True,
                        help='Operator confirms the world was freshly started at its known origin')
    parser.add_argument('--seed-offset-x',type=float,default=0.)
    parser.add_argument('--seed-offset-y',type=float,default=0.)
    parser.add_argument('--seed-offset-yaw',type=float,default=0.)
    args=parser.parse_args()
    seed=[args.seed_offset_x,args.seed_offset_y,args.seed_offset_yaw]
    if not all(math.isfinite(x) for x in seed) or math.hypot(*seed[:2])>.5 or abs(seed[2])>.5:
        parser.error('Commissioning seed offsets must be within 0.5 m and 0.5 rad')
    rclpy.init()
    node=rclpy.create_node('orbital_localization_seed',parameter_overrides=[Parameter('use_sim_time',value=True)])
    samples={}
    def receive(name,msg):
        samples[name]=(msg,time.monotonic())
    subscriptions=[node.create_subscription(Odometry,'/odom',lambda m:receive('odom',m),qos_profile_sensor_data),
                   node.create_subscription(String,'/motion/state',lambda m:receive('guard',m),1),
                   node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:receive('pose',m),1)]
    pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    stop=node.create_client(Trigger,'/motion/stop')
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return True
        return False
    try:
        if not stop.wait_for_service(timeout_sec=3):raise RuntimeError('Motion stop unavailable')
        future=stop.call_async(Trigger.Request())
        if not wait(future.done,3) or not future.result().success:raise RuntimeError('Motion stop failed')
        if not wait(lambda:'odom' in samples and 'guard' in samples and pub.get_subscription_count()>0,5):
            raise RuntimeError('Fresh feedback or initial-pose subscriber unavailable')
        if any(time.monotonic()-samples[k][1]>.3 for k in ('odom','guard')):
            raise RuntimeError('Feedback is stale')
        odom=samples['odom'][0];guard=json.loads(samples['guard'][0].data)
        p=odom.pose.pose.position;v=odom.twist.twist
        values=[p.x,p.y,v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]
        if not all(math.isfinite(x) for x in values):raise RuntimeError('Nonfinite feedback')
        if not guard['latched'] or math.hypot(v.linear.x,v.linear.y,v.linear.z)>.01 or math.hypot(v.angular.x,v.angular.y,v.angular.z)>.02:
            raise RuntimeError('Robot must be latched and stationary')
        if math.hypot(p.x,p.y)>.1:raise RuntimeError('Odometry is not near the fresh-world origin')
        if odom.header.frame_id!='odom' or odom.child_frame_id!='base_footprint':
            raise RuntimeError('Unexpected odometry frames')
        if any(name in {'controller_server','bt_navigator'} for name in node.get_node_names()):
            raise RuntimeError('This commissioning seed must not run with an active navigation controller')
        stamp=node.get_clock().now()
        if stamp.nanoseconds<=0:raise RuntimeError('Simulation clock unavailable')
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=stamp.to_msg()
        msg.pose.pose.position.x=seed[0];msg.pose.pose.position.y=seed[1]
        msg.pose.pose.orientation.z=math.sin(seed[2]/2);msg.pose.pose.orientation.w=math.cos(seed[2]/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.25**2
        msg.pose.covariance[35]=math.radians(15)**2
        pub.publish(msg)
        def processed():
            if 'pose' not in samples:return False
            t=samples['pose'][0].header.stamp
            return t.sec*10**9+t.nanosec>=stamp.nanoseconds
        if not wait(processed,8):raise RuntimeError('No AMCL pose after explicit seed')
        pose=samples['pose'][0]
        print(json.dumps({'seed_processed':True,'convergence_verified':False,
                          'seed_xy_yaw':seed, 'seed_std_xy_m':.25,'seed_std_yaw_rad':math.radians(15),
                          'amcl_xy':[pose.pose.pose.position.x,pose.pose.pose.position.y],
                          'amcl_covariance_diagonal':[pose.pose.covariance[i] for i in (0,7,35)]},indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
