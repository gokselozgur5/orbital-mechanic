#!/usr/bin/env python3
"""Composed control acceptance in empty domain 84, with no Gazebo/actuator bridge."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from orbital.ros_localization import LocalizationMonitor
from orbital.ros_navigation_supervisor import NavigationSupervisor


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module);return module


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='84':raise RuntimeError('Requires isolated domain 84')
    parser=argparse.ArgumentParser();parser.add_argument('--supervisor',action='store_true');args=parser.parse_args()
    rclpy.init()
    if args.supervisor:
        node=NavigationSupervisor({'normal':(0.,0.,0.),'success':(3.,0.,0.)},goal_timeout=3.)
        try:rclpy.spin(node)
        except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
        finally:node.destroy_node();rclpy.try_shutdown()
        return
    preflight=Node('orbital_supervisor_preflight');end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(preflight,timeout_sec=.05)
    others=[n for n in preflight.get_node_names() if n!=preflight.get_name()];preflight.destroy_node()
    if others:rclpy.shutdown();raise RuntimeError('Domain not empty: '+repr(others))
    fixture_module=load('supervisor_fixture',ROOT/'scripts/check-localization-monitor.py')
    guard_module=load('supervisor_guard',ROOT/'scripts/motion-guard.py')
    fixture=fixture_module.Fixture();guard=guard_module.GuardNode();monitor=LocalizationMonitor();controller=Node('controller_server')
    nodes=[fixture,guard,monitor,controller];executor=SingleThreadedExecutor()
    for node in nodes:executor.add_node(node)
    data={};report=[];children=[];logs=[]
    request=fixture.create_publisher(String,'/navigation/command',10)
    raw=controller.create_publisher(TwistStamped,'/navigation/raw_cmd_vel',1)
    probe=ActionClient(controller,NavigateToPose,'/navigate_to_pose')
    subscriptions=[fixture.create_subscription(String,'/navigation/state',lambda m:data.update(nav=json.loads(m.data)),10),
                   fixture.create_subscription(String,'/motion/state',lambda m:data.update(guard=json.loads(m.data)),10),
                   fixture.create_subscription(Twist,'/motion/guarded_cmd_vel',lambda m:data.update(output=m.linear.x),10)]
    raw_stamp=None;raw_count=0;next_publish=time.monotonic()
    def pump(seconds):
        nonlocal raw_count,next_publish
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_publish:
                next_publish=now+.02;fixture.publish()
                m=TwistStamped();m.header.frame_id='base_footprint'
                m.header.stamp=fixture_module.stamp(fixture.sim if raw_stamp is None else raw_stamp)
                m.twist.linear.x=.1;raw.publish(m);raw_count+=1
            executor.spin_once(timeout_sec=.002)
    def wait(predicate,seconds=5):
        start=time.monotonic()
        while time.monotonic()-start<seconds:
            pump(.03)
            if predicate():return time.monotonic()-start
        raise AssertionError(('timeout',data,[c.poll() for c in children]))
    def idle():
        return data.get('nav',{}).get('healthy') and data['nav']['action_state']=='idle' and data.get('guard',{}).get('latched')
    def command(op,target=None,waypoint=None):
        state=data['nav'];seq=state['command_sequence']+1
        value={'run_id':state['run_id'],'sequence':seq,'op':op}
        value.update(waypoint=waypoint) if op=='request' else value.update(target=target)
        request.publish(String(data=json.dumps(value)))
        wait(lambda:(data['nav'].get('last_reply') or {}).get('sequence')==seq)
        return data['nav']['last_reply']['accepted'],value
    def running(name='normal'):
        wait(idle);ok,_=command('request',waypoint=name);assert ok
        target=data['nav']['target'];assert target
        assert data['output']==0,'request moved before arm'
        ok,_=command('arm',target=target);assert ok
        if name=='success':
            wait(lambda:data['nav']['action_state'] in ('running','terminal') or
                 (data['nav'].get('last_completion') or {}).get('target')==target)
        else:wait(lambda:data['nav']['action_state']=='running' and data.get('output',0)>.05)
        return target
    def spawn(args,name):
        path=ROOT/'runtime'/name;path.parent.mkdir(parents=True,exist_ok=True)
        stream=path.open('w');logs.append(stream)
        child=subprocess.Popen([sys.executable,*args],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child
    try:
        spawn([str(ROOT/'scripts/check-navigation-action.py'),'--server'],'composed-action-server.log')
        owner=spawn([__file__,'--supervisor'],'composed-supervisor.log')
        pump(1);fixture.pose.publish(fixture.pose_message())
        wait(lambda:probe.server_is_ready() and request.get_subscription_count()>0 and idle(),8)
        pump(.3);assert data['output']==0 and data['nav']['forwarded_commands']==0
        report.append({'case':'raw_velocity_blocked_without_target_and_arm','passed':True})
        target=running();before=raw_count;fixture.publish_scan=False
        delay=wait(lambda:data['guard']['latched'] and data['output']==0,2)
        assert raw_count>before
        fixture.publish_scan=True;wait(idle);pump(.2);assert data['output']==0
        ok,_=command('arm',target=target);assert not ok
        report.append({'case':'health_loss_cuts_velocity_and_old_target_cannot_rearm','guard_stop_s':delay})

        target=running();raw_stamp=0.
        wait(lambda:data['guard']['latched'] and data['output']==0,2)
        raw_stamp=None;wait(idle)
        report.append({'case':'stale_controller_stamp_cancels_target','passed':True})

        target=running();guard.destroy_service(guard.stop_service);fixture.publish_scan=False
        delay=wait(lambda:data['guard']['latched'] and data['output']==0,2)
        assert data['guard']['reason']=='command_timeout',data['guard']
        guard.stop_service=guard.create_service(Trigger,'/motion/stop',guard.stop)
        fixture.publish_scan=True;wait(idle)
        report.append({'case':'stop_service_loss_still_stops_owned_velocity_stream','guard_stop_s':delay,'reason':'command_timeout'})

        target=running('success');wait(idle)
        assert data['nav']['last_completion']['target']==target
        assert data['nav']['last_completion']['outcome']=='arrival_not_verified'
        report.append({'case':'nav2_success_without_measured_arrival_rejected','passed':True})
        fixture.map_x=3.;fixture.pose.publish(fixture.pose_message());pump(.3)
        target=running('success');wait(idle)
        assert data['nav']['last_completion']['outcome']=='arrived',data['nav']['last_completion']
        report.append({'case':'arrival_requires_matching_measured_transform','passed':True})

        target=running();ok,_=command('cancel',target=target);assert ok;wait(idle)
        report.append({'case':'explicit_token_cancel_stops_and_releases_after_terminal','passed':True})
        target=running();before=raw_count
        os.killpg(owner.pid,signal.SIGKILL);owner.wait(timeout=3)
        delay=wait(lambda:data['guard']['latched'] and data['output']==0,2)
        assert data['guard']['reason']=='command_timeout' and raw_count>before
        report.append({'case':'supervisor_process_death_stops_while_controller_keeps_publishing','guard_stop_s':delay})
        print(json.dumps({'passed':True,'domain':84,'checks':report,'scope':'composed ROS control with fake Nav2/sensors; no real routes or physical stopping distance'},indent=2))
    except Exception as exc:
        print(json.dumps({'passed':False,'error':str(exc),'checks':report},indent=2));raise
    finally:
        guard.guard.stop('test_shutdown');guard.tick()
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=1)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=1)
        for stream in logs:stream.close()
        probe.destroy()
        for node in nodes:executor.remove_node(node)
        for node in reversed(nodes):node.destroy_node()
        executor.shutdown();rclpy.try_shutdown()


if __name__=='__main__':main()
