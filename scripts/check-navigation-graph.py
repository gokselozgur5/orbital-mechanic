#!/usr/bin/env python3
"""Read-only ROS ownership/fresh-stationary gate; never arm or activate nodes."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.navigation_graph import BASE_TOPICS, NAV_TOPICS, BYPASS_TOPICS, graph_errors


def collect(node, phase, seconds=5.):
    import rclpy
    from rclpy.action.graph import get_action_server_names_and_types_by_node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    parents, state, subscriptions = {}, {}, []
    def tf(message):
        for transform in message.transforms:
            parents.setdefault(transform.child_frame_id,set()).add(transform.header.frame_id)
    def clock(message):
        value = message.clock.sec+message.clock.nanosec*1e-9
        if value < state.get('clock',value):
            state['clock_regressed'] = True
        state.setdefault('first_clock',value)
        state.update(clock=value,clock_at=time.monotonic())
    def odom(message):
        velocity = message.twist.twist
        values = [velocity.linear.x,velocity.linear.y,velocity.linear.z,
                  velocity.angular.x,velocity.angular.y,velocity.angular.z]
        state.update(odom=values,odom_at=time.monotonic(),
                     odom_stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                     odom_frames=(message.header.frame_id,message.child_frame_id))
    def guard(message):
        try:
            if len(message.data)>4096:raise ValueError('Oversized guard state')
            value = json.loads(message.data)
            if not isinstance(value,dict):raise ValueError('Invalid guard state')
            state.update(guard=value,guard_at=time.monotonic())
        except ValueError:
            state.pop('guard',None)
    retained = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    for kind,topic,callback,qos in (
            (TFMessage,'/tf',tf,qos_profile_sensor_data),(TFMessage,'/tf_static',tf,retained),
            (Clock,'/clock',clock,qos_profile_sensor_data),(Odometry,'/odom',odom,qos_profile_sensor_data),
            (String,'/motion/state',guard,10)):
        subscriptions.append(node.create_subscription(kind,topic,callback,qos))
    snapshots,errors = [],set()
    def snapshot():
        pairs = node.get_node_names_and_namespaces()
        result = {'nodes':[{'name':n,'namespace':ns} for n,ns in pairs],
                  'publishers':{},'actions':[],'services':[],
                  'parents':{child:sorted(values) for child,values in parents.items()}}
        for topic in (*BASE_TOPICS,*NAV_TOPICS,*BYPASS_TOPICS):
            result['publishers'][topic] = [{'node':p.node_name,'namespace':p.node_namespace,'type':p.topic_type,
                                            'gid':list(p.endpoint_gid)} for p in node.get_publishers_info_by_topic(topic)]
        for name,namespace in set(pairs):
            for path,kinds in get_action_server_names_and_types_by_node(node,name,namespace):
                result['actions'].extend({'name':path,'type':kind,'node':name,'namespace':namespace} for kind in kinds)
            for path,kinds in node.get_service_names_and_types_by_node(name,namespace):
                if path in ('/motion/stop','/motion/reset_stop'):
                    result['services'].extend({'name':path,'type':kind,'node':name,'namespace':namespace} for kind in kinds)
        return result
    try:
        started = time.monotonic()
        next_snapshot = started+2.5
        while time.monotonic()-started<seconds:
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()>=next_snapshot:
                item = snapshot();snapshots.append(item);errors.update(graph_errors(item,phase))
                next_snapshot += 1.
        # Drain callbacks after graph reads; freshness uses receipt plus sim stamps.
        for _ in range(20):rclpy.spin_once(node,timeout_sec=0.)
        final = snapshot();snapshots.append(final);errors.update(graph_errors(final,phase))
        now = time.monotonic()
        velocity = state.get('odom',[])
        guard_state = state.get('guard',{})
        checks = {
            'clock_advances':state.get('clock',0)>state.get('first_clock',0)>0 and not state.get('clock_regressed',False)
                             and now-state.get('clock_at',0)<.5,
            'odometry_fresh':now-state.get('odom_at',0)<.5 and 0<=state.get('clock',0)-state.get('odom_stamp',-1)<.5
                             and state.get('odom_frames')==('odom','base_footprint'),
            'stationary':len(velocity)==6 and all(math.isfinite(v) for v in velocity)
                         and math.hypot(*velocity[:3])<=.02 and math.hypot(*velocity[3:])<=.05,
            'guard_latched':guard_state.get('latched') is True and now-state.get('guard_at',0)<.3
                            and guard_state.get('linear_mps')==0 and guard_state.get('angular_rps')==0,
        }
        return {'passed':not errors and all(checks.values()),'phase':phase,'checks':checks,
                'errors':sorted(errors),'snapshot':final,'observations':len(snapshots),
                'per_message_tf_authority_proven':False,
                'scope':'ROS endpoint/parent graph and fresh latched stationarity; trusted local nodes only'}
    finally:
        for sub in subscriptions:node.destroy_subscription(sub)


def main():
    if os.environ.get('ROS_DOMAIN_ID') not in ('42','84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        raise RuntimeError('Requires localhost Orbital domain42 or isolated domain84')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase',choices=('before','configured'),default='before')
    args=parser.parse_args()
    import rclpy
    rclpy.init(args=[])
    node=rclpy.create_node('orbital_navigation_graph_probe')
    try:
        report=collect(node,args.phase)
        print(json.dumps(report,indent=2))
        return 0 if report['passed'] else 1
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':raise SystemExit(main())
