#!/usr/bin/env python3
"""Prove graph-gate acceptance/refusal over DDS on empty localhost domain84."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.navigation_graph import BASE_TOPICS,NAV_TOPICS,NAV_NODES,PARENTS


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='84' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        raise RuntimeError('Requires empty localhost domain84')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.action import ActionServer
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import NavigateToPose,ComputePathToPose,FollowPath
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    rclpy.init(args=[])
    probe=Node('orbital_graph_fixture_preflight')
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(probe,timeout_sec=.05)
    others=[n for n in probe.get_node_names() if n!=probe.get_name()]
    probe.destroy_node()
    if others:
        rclpy.shutdown();raise RuntimeError('Domain not empty: '+repr(others))
    executor=SingleThreadedExecutor()
    nodes={}
    pubs={}
    children=[]
    resources=[]
    report={'passed':False,'checks':[], 'scope':'isolated DDS fixtures; no Gazebo or drive',
            'sources':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in
                       ('orbital/navigation_graph.py','scripts/check-navigation-graph.py',
                        'scripts/check-navigation-graph-fixtures.py')}}
    def node(name):
        if name not in nodes:
            nodes[name]=Node(name);executor.add_node(nodes[name])
        return nodes[name]
    retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    for topic,owners in BASE_TOPICS.items():
        for name,kind in owners:
            pubs[(topic,name)]=node(name).create_publisher(get_message(kind),topic,retained if topic=='/tf_static' else 10)
    def service(request,response):
        response.success=True;return response
    services=[node('motion_guard').create_service(Trigger,p,service) for p in ('/motion/stop','/motion/reset_stop')]
    started=time.monotonic()
    settings={'guard':True}
    def publish():
        sim=100.+time.monotonic()-started
        stamp=Time(sec=int(sim),nanosec=int((sim%1)*1e9))
        pubs[('/clock','ros_gz_bridge')].publish(Clock(clock=stamp))
        odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_footprint'
        odom.header.stamp=stamp;odom.pose.pose.orientation.w=1.
        pubs[('/odom','ros_gz_bridge')].publish(odom)
        if settings['guard']:
            pubs[('/motion/state','motion_guard')].publish(String(data=json.dumps({'latched':True,'linear_mps':0.,'angular_rps':0.})))
        transforms={('ros_gz_bridge','/tf'):[],('amcl','/tf'):[],
                    ('robot_state_publisher','/tf'):[],('robot_state_publisher','/tf_static'):[]}
        for child,parent in {**PARENTS,'odom':'map'}.items():
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp;t.transform.rotation.w=1.
            key=(('ros_gz_bridge','/tf') if child=='base_footprint' else ('amcl','/tf') if child=='odom'
                 else ('robot_state_publisher','/tf') if child in ('left_wheel','right_wheel')
                 else ('robot_state_publisher','/tf_static'))
            transforms[key].append(t)
        for (owner,topic),values in transforms.items():pubs[(topic,owner)].publish(TFMessage(transforms=values))
    timer=node('ros_gz_bridge').create_timer(.05,publish)
    (ROOT/'runtime').mkdir(exist_ok=True)
    try:
        for case in ('before','configured','duplicate_tf','duplicate_action','bypass','stale_guard'):
            extra_nodes=[]
            extra_actions=[]
            extra_publishers=[]
            if case=='configured':
                for name in NAV_NODES:node(name);extra_nodes.append(name)
                for topic,owners in NAV_TOPICS.items():
                    for name,kind in owners:extra_publishers.append((node(name),node(name).create_publisher(get_message(kind),topic,10)))
                for name,kind,path in (('bt_navigator',NavigateToPose,'/navigate_to_pose'),
                                       ('planner_server',ComputePathToPose,'/compute_path_to_pose'),
                                       ('controller_server',FollowPath,'/follow_path')):
                    extra_actions.append(ActionServer(node(name),kind,path,lambda goal:None))
            elif case=='duplicate_tf':
                extra_publishers.append((node('amcl'),node('amcl').create_publisher(TFMessage,'/tf',10)))
            elif case=='duplicate_action':
                extra_nodes.append('intruder')
                extra_actions.append(ActionServer(node('intruder'),NavigateToPose,'/navigate_to_pose',lambda goal:None))
            elif case=='bypass':
                extra_nodes.append('intruder')
                extra_publishers.append((node('intruder'),node('intruder').create_publisher(Twist,'/cmd_vel',1)))
            elif case=='stale_guard':settings['guard']=False
            output=ROOT/'runtime'/f'graph-{case}.json'
            with output.open('w') as stream, (ROOT/'runtime'/f'graph-{case}.log').open('w') as errors:
                child=subprocess.Popen([sys.executable,str(ROOT/'scripts/check-navigation-graph.py'),
                                        '--phase','configured' if case=='configured' else 'before'],
                                       stdout=stream,stderr=errors)
                children.append(child)
                end=time.monotonic()+15
                while child.poll() is None and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
                if child.poll() is None:raise RuntimeError('Graph fixture probe timed out: '+case)
            result=json.loads(output.read_text())
            expected=case in ('before','configured')
            assert result['passed']==expected and child.returncode==(0 if expected else 1), (case,result)
            if case=='duplicate_tf':assert 'Publisher ownership mismatch: /tf' in result['errors']
            if case=='duplicate_action':assert 'Navigation action ownership mismatch' in result['errors']
            if case=='bypass':assert 'Publisher ownership mismatch: /cmd_vel' in result['errors']
            if case=='stale_guard':assert result['checks']['guard_latched'] is False and not result['errors']
            report['checks'].append({'case':case,'accepted':result['passed'],'errors':result['errors'],'checks':result['checks']})
            for action in extra_actions:action.destroy()
            for owner,publisher in extra_publishers:owner.destroy_publisher(publisher)
            for name in extra_nodes:
                executor.remove_node(nodes[name]);nodes.pop(name).destroy_node()
            settings['guard']=True
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
    finally:
        for child in children:
            if child.poll() is None:child.kill()
            child.wait(timeout=3)
        for owned in nodes.values():executor.remove_node(owned);owned.destroy_node()
        executor.shutdown();rclpy.try_shutdown()
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
