#!/usr/bin/env python3
"""Isolated ROS test: actual guard service stops despite continuing valid requests.

No Gazebo, physical device or actuator bridge exists in domain 84. Guarded output
is measured as a ROS message; this is not physical stopping-distance evidence.
"""
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from orbital.ros_localization import LocalizationMonitor
from orbital.ros_health_interlock import HealthInterlock


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


fixture_module=load('synthetic_fixture',ROOT/'scripts/check-localization-monitor.py')
guard_module=load('guard_adapter',ROOT/'scripts/motion-guard.py')


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='84':raise RuntimeError('Requires isolated ROS domain 84')
    rclpy.init();preflight=Node('orbital_interlock_preflight')
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(preflight,timeout_sec=.05)
    others=[n for n in preflight.get_node_names() if n!='orbital_interlock_preflight']
    preflight.destroy_node()
    if others:
        rclpy.shutdown();raise RuntimeError('Domain is not empty: '+repr(others))
    fixture=fixture_module.Fixture();monitor=LocalizationMonitor()
    guard=guard_module.GuardNode();interlock=HealthInterlock()
    nodes=[fixture,monitor,guard,interlock];executor=SingleThreadedExecutor()
    for node in nodes:executor.add_node(node)
    data={};report=[];send=False;sent=0
    state_sub=fixture.create_subscription(String,'/motion/state',lambda m:data.update(state=json.loads(m.data)),10)
    output_sub=fixture.create_subscription(Twist,'/motion/guarded_cmd_vel',lambda m:data.update(output=m.linear.x),10)
    publisher=fixture.create_publisher(TwistStamped,'/motion/request',1)
    arm=fixture.create_client(Trigger,'/motion/reset_stop')
    next_publish=time.monotonic()
    def pump(seconds):
        nonlocal sent,next_publish
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            now=time.monotonic()
            if now>=next_publish:
                fixture.publish();next_publish=now+.02
                if send:
                    m=TwistStamped();m.header.frame_id='base_link';m.header.stamp=fixture_module.stamp(fixture.sim)
                    m.twist.linear.x=.1;publisher.publish(m);sent+=1
            executor.spin_once(timeout_sec=.002)
    def wait(predicate,seconds=3):
        start=time.monotonic()
        while time.monotonic()-start<seconds:
            pump(.03)
            if predicate():return time.monotonic()-start
        raise AssertionError(('condition timed out',data,interlock.lease.reason,fixture.events[-5:]))
    def ready():
        return interlock.lease.healthy and not interlock.lease.stop_required and interlock.stop_acks>0
    try:
        pump(1);fixture.pose.publish(fixture.pose_message());wait(ready)
        assert data['state']['latched'] and data['output']==0
        report.append({'case':'startup_stops_before_ready','passed':True})
        for case in ('scan_loss','tilt','tf_loss','clock_pause','health_silence'):
            send=False
            wait(ready)
            future=arm.call_async(Trigger.Request());wait(future.done)
            assert future.result().success,future.result().message
            send=True;wait(lambda:data.get('output',0)>.05 and data['state']['reason']=='command_active')
            before=sent
            if case=='scan_loss':fixture.publish_scan=False
            elif case=='tilt':fixture.tilt=.522876
            elif case=='tf_loss':fixture.publish_tf=False
            elif case=='clock_pause':fixture.freeze_clock=True
            else:monitor.timer.cancel()
            delay=wait(lambda:data['state']['latched'] and data['output']==0,seconds=1.2)
            assert data['state']['reason']=='operator_stop',(case,data)
            assert sent>before,(case,'no continuing requests')
            assert delay<1.,(case,delay)
            report.append({'case':case,'guard_stop_s':delay,'requests_while_faulted':sent-before,
                           'guard_reason':data['state']['reason']})
            fixture.publish_scan=True;fixture.tilt=0.;fixture.publish_tf=True;fixture.freeze_clock=False
            if case=='health_silence':monitor.timer.reset()
            wait(ready);pump(.3)
            assert data['state']['latched'] and data['output']==0,'recovery restarted old commands'
        send=False
        publishers={name for name,_ in interlock.get_publisher_names_and_types_by_node(interlock.get_name(),interlock.get_namespace())}
        assert publishers <= {'/navigation/interlock','/rosout','/parameter_events'},publishers
        report.append({'case':'interlock_has_no_velocity_or_arm_publishers','publishers':sorted(publishers)})
        print(json.dumps({'passed':True,'domain':84,'checks':report,'stop_requests':interlock.stop_requests,
                          'acknowledged_stops':interlock.stop_acks,'scope':'actual ROS guard output, not physical motion or Nav2 cancellation'},indent=2))
    except Exception as exc:
        print(json.dumps({'passed':False,'error':str(exc),'checks':report},indent=2));raise
    finally:
        guard.guard.stop('test_shutdown');guard.tick()
        for node in nodes:executor.remove_node(node)
        for node in reversed(nodes):node.destroy_node()
        executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
