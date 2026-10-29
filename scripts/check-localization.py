#!/usr/bin/env python3
"""Read-only known-map localization probe; optional independent Gazebo evaluator.

Ground truth is used only here for acceptance, never supplied to the planner or
AMCL. No motion, initial-pose updates or no-motion-update requests are published.
"""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.mapping import build_grid, FREE, OCCUPIED
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from lifecycle_msgs.srv import GetState
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Imu
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--compare-gazebo',action='store_true')
    mode.add_argument('--expect-unseeded',action='store_true',help='Verify active AMCL has no pose/TF before explicit seed')
    args=parser.parse_args()
    config=json.loads((ROOT/'config/known-map.json').read_text())
    grid=build_grid(ROOT/config['world'],config)
    expected_cells=[0 if x==FREE else 100 if x==OCCUPIED else -1 for x in grid.cells]
    rclpy.init()
    node=rclpy.create_node('orbital_localization_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    samples={};counts={};first_stamp={};regressions={}
    def receive(name,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if name in samples and stamp<samples[name][2]:regressions[name]=regressions.get(name,0)+1
        first_stamp.setdefault(name,stamp);counts[name]=counts.get(name,0)+1
        samples[name]=(msg,time.monotonic(),stamp)
    static=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    subscriptions=[node.create_subscription(OccupancyGrid,'/map',lambda m:receive('map',m),static),
                   node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:receive('pose',m),static),
                   node.create_subscription(LaserScan,'/scan',lambda m:receive('scan',m),qos_profile_sensor_data),
                   node.create_subscription(Odometry,'/odom',lambda m:receive('odom',m),qos_profile_sensor_data),
                   node.create_subscription(Imu,'/imu',lambda m:receive('imu',m),qos_profile_sensor_data)]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    try:
        spin(8)
        if args.expect_unseeded:
            checks={'inputs_received':{'map','scan','odom','imu'}.issubset(samples)}
            client=node.create_client(GetState,'/amcl/get_state')
            active=False
            if client.wait_for_service(timeout_sec=2):
                future=client.call_async(GetState.Request());end=time.monotonic()+2
                while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
                active=future.done() and future.result().current_state.id==3
            checks['amcl_active']=active
            checks['no_automatic_pose']='pose' not in samples
            try:
                buffer.lookup_transform('map','base_footprint',Time())
                checks['no_automatic_map_tf']=False
            except TransformException:
                checks['no_automatic_map_tf']=True
            for name in ('scan','odom','imu'):
                checks[name+'_advancing']=(name in samples and counts[name]>1 and
                    samples[name][2]>first_stamp[name] and time.monotonic()-samples[name][1]<.5)
            passed=all(checks.values())
            print(json.dumps({'passed':passed,'checks':checks,'counts':counts},indent=2))
            return 0 if passed else 1
        checks={'all_topics_received':set(samples)=={'map','pose','scan','odom','imu'}}
        if not checks['all_topics_received']:
            print(json.dumps({'passed':False,'checks':checks,'received':list(samples)},indent=2));return 1
        truth=None
        if args.compare_gazebo:
            v=samples['odom'][0].twist.twist
            if math.hypot(v.linear.x,v.linear.y,v.linear.z)>.01 or math.hypot(v.angular.x,v.angular.y,v.angular.z)>.02:
                raise RuntimeError('Stationary robot required for independent pose comparison')
            output=subprocess.check_output(['gz','model','-m','maintenance_robot','-p'],text=True,stderr=subprocess.DEVNULL,timeout=10)
            output=re.sub(r'\x1b\[[0-9;]*m','',output)
            match=re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]\s*\[([^\]]+)\]',output)
            if not match:raise RuntimeError('Could not parse Gazebo model pose')
            xyz=[float(x) for x in match[1].split()];rpy=[float(x) for x in match[2].split()]
            truth=[xyz[0],xyz[1],rpy[2]]
            checks['physical_body_level']=all(math.isfinite(v) and abs(v)<.05 for v in rpy[:2])
            spin(.5)
        # Request a captured simulation instant; spin until the next odometry TF
        # covers it instead of mistaking ordinary publication lag for TF loss.
        requested=node.get_clock().now();transform=None;end=time.monotonic()+.3
        while time.monotonic()<end:
            try:transform=buffer.lookup_transform('map','base_footprint',requested);break
            except TransformException:rclpy.spin_once(node,timeout_sec=.02)
        now=node.get_clock().now().nanoseconds/1e9;wall=time.monotonic()
        checks['current_tf_available']=transform is not None and 0<=now-requested.nanoseconds/1e9<.5
        for name in ('scan','odom','imu'):
            checks[name+'_fresh_and_advancing']=(wall-samples[name][1]<.5 and 0<=now-samples[name][2]<.5 and
                                               samples[name][2]>first_stamp[name] and not regressions.get(name))
        imu=samples['imu'][0];iq=imu.orientation
        imu_norm=sum(v*v for v in (iq.x,iq.y,iq.z,iq.w))
        roll=math.atan2(2*(iq.w*iq.x+iq.y*iq.z),1-2*(iq.x*iq.x+iq.y*iq.y))
        pitch=math.asin(max(-1.,min(1.,2*(iq.w*iq.y-iq.z*iq.x))))
        checks['imu_body_level']=(imu.header.frame_id=='base_link' and math.isfinite(imu_norm) and
                                  abs(imu_norm-1)<.01 and abs(roll)<.05 and abs(pitch)<.05)
        map_msg=samples['map'][0]
        checks['map_geometry_and_cells']=(map_msg.header.frame_id=='map' and map_msg.info.width==grid.width and
                                          map_msg.info.height==grid.height and abs(map_msg.info.resolution-grid.resolution)<1e-6 and
                                          abs(map_msg.info.origin.position.x-grid.origin[0])<1e-6 and
                                          abs(map_msg.info.origin.position.y-grid.origin[1])<1e-6 and list(map_msg.data)==expected_cells)
        pose=samples['pose'][0];q=pose.pose.pose.orientation;cov=pose.pose.covariance
        values=[pose.pose.pose.position.x,pose.pose.pose.position.y,q.x,q.y,q.z,q.w,*cov]
        checks['finite_pose_and_covariance']=all(math.isfinite(x) for x in values) and pose.header.frame_id=='map'
        checks['normalized_orientation']=abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-3
        # Commissioning thresholds; do not equate covariance with measured accuracy.
        checks['covariance_within_commissioning_bound']=all(0<=cov[i]<=bound for i,bound in [(0,.25**2),(7,.25**2),(35,.35**2)])
        graph={topic:sorted(p.node_name for p in node.get_publishers_info_by_topic(topic)) for topic in ('/map','/amcl_pose','/tf')}
        checks['expected_publishers']=(graph['/map']==['map_server'] and graph['/amcl_pose']==['amcl'] and
                                       graph['/tf']==['amcl','robot_state_publisher','ros_gz_bridge'])
        measured=None;errors=None
        if transform is not None:
            p=transform.transform.translation;measured=[p.x,p.y,yaw(transform.transform.rotation)]
            if truth is not None:
                errors={'xy_m':math.dist(measured[:2],truth[:2]),'yaw_rad':abs(math.remainder(measured[2]-truth[2],2*math.pi))}
                checks['independent_pose_accuracy']=errors['xy_m']<=.1 and errors['yaw_rad']<=.1
                v=samples['odom'][0].twist.twist
                checks['stationary_comparison']=math.hypot(v.linear.x,v.linear.y,v.linear.z)<.01 and math.hypot(v.angular.x,v.angular.y,v.angular.z)<.02
        passed=all(checks.values())
        print(json.dumps({'passed':passed,'checks':checks,'counts':counts,'publisher_nodes':graph,
                          'map_base_xy_yaw':measured,'covariance_diagonal':[cov[i] for i in (0,7,35)],
                          'ground_truth_xy_yaw':truth,'independent_error':errors,
                          'scope':'stationary commissioning sample, not route/obstacle/reset/loss acceptance'},indent=2))
        return 0 if passed else 1
    finally:
        listener.unregister();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
