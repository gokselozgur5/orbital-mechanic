#!/usr/bin/env python3
"""Exercise real NavigateToPose action transport against an isolated fake server.

No Nav2 controller, world or motion publisher is launched. Domain 84 must be empty.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMessage
from builtin_interfaces.msg import Time as TimeMessage
from nav2_msgs.action import NavigateToPose
from orbital.navigation import NavigationPolicy
from orbital.ros_navigation_action import NavigationActionSession


class FakeNavigator(Node):
    def __init__(self):
        super().__init__('orbital_fake_navigator')
        self.server=ActionServer(self,NavigateToPose,'/navigate_to_pose',self.execute,
            callback_group=ReentrantCallbackGroup(),goal_callback=self.goal,cancel_callback=self.cancel)

    def goal(self,request):
        mode=round(request.pose.pose.position.x)
        if mode==1:time.sleep(.3)
        return GoalResponse.REJECT if mode==2 else GoalResponse.ACCEPT

    def cancel(self,handle):
        return CancelResponse.REJECT if round(handle.request.pose.pose.position.x)==4 else CancelResponse.ACCEPT

    def execute(self,handle):
        start=time.monotonic();mode=round(handle.request.pose.pose.position.x)
        while rclpy.ok() and time.monotonic()-start<5:
            if handle.is_cancel_requested:
                time.sleep(.25)  # An accepted cancellation is deliberately not terminal yet.
                handle.canceled();return NavigateToPose.Result()
            if mode==3 and time.monotonic()-start>.15:
                handle.succeed();return NavigateToPose.Result()
            if mode==4 and time.monotonic()-start>.7:
                handle.abort();return NavigateToPose.Result()
            time.sleep(.01)
        if handle.is_active:handle.abort()
        return NavigateToPose.Result()


def server():
    rclpy.init();node=FakeNavigator();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='84':raise RuntimeError('Requires isolated domain 84')
    parser=argparse.ArgumentParser();parser.add_argument('--server',action='store_true');args=parser.parse_args()
    if args.server:return server()
    rclpy.init();node=Node('orbital_action_lifecycle_test',parameter_overrides=[Parameter('use_sim_time',value=True)])
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    others=[n for n in node.get_node_names() if n!=node.get_name()]
    if others:
        node.destroy_node();rclpy.shutdown();raise RuntimeError('Domain not empty: '+repr(others))
    log=Path(__file__).resolve().parents[1]/'runtime/action-server.log'
    log.parent.mkdir(parents=True,exist_ok=True)
    stream=log.open('w')
    child=subprocess.Popen([sys.executable,__file__,'--server'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    poses={name:(float(i),0.,0.) for i,name in enumerate(['normal','slow_accept','reject','success','reject_cancel'])}
    policy=NavigationPolicy(poses);stops=[]
    session=NavigationActionSession(node,policy,poses,lambda:stops.append(time.monotonic()))
    healthy=True
    started=time.monotonic()
    clock_pub=node.create_publisher(ClockMessage,'/clock',10)
    def health():
        now=time.monotonic();sim=now-started+1.;ns=round(sim*1e9)
        clock_pub.publish(ClockMessage(clock=TimeMessage(sec=ns//10**9,nanosec=ns%10**9)))
        policy.observe(sim_time=sim,valid=healthy,now=now)
    steady=Clock(clock_type=ClockType.STEADY_TIME)
    timer=node.create_timer(.02,health,clock=steady)
    report=[]
    def wait(predicate,seconds=3):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
        raise AssertionError(('timeout',session.state,session.outcome,policy.reason,child.poll()))
    def request(name,timeout=2.):
        wait(lambda:policy.tick(time.monotonic()))
        target=session.request(name);assert target is not None
        assert not session.start(target),'dispatch happened before explicit arm'
        assert policy.arm(target,stationary=True,now=time.monotonic())
        assert session.start(target,timeout=timeout)
        return target
    def release(target):
        assert not session.release(target,stationary=False,stop_latched=True)
        assert not session.release(target,stationary=True,stop_latched=False)
        assert session.release(target,stationary=True,stop_latched=True)
    try:
        wait(session.client.server_is_ready,5)
        t=request('normal');wait(lambda:session.state=='running')
        before=len(stops)
        assert session.cancel(t)
        assert len(stops)>before,'cancel did not request immediate stop'
        wait(lambda:session.cancel_acknowledged)
        assert session.state=='cancelling' and session.current==t
        assert session.request('success') is None
        assert not session.release(t,stationary=True,stop_latched=True)
        wait(lambda:session.state=='terminal');assert session.outcome=='cancelled'
        release(t);report.append({'case':'cancel_ack_is_not_terminal','passed':True})

        t=request('slow_accept');assert session.state=='dispatching'
        assert session.cancel(t);assert not session.release(t,stationary=True,stop_latched=True)
        wait(lambda:session.state=='terminal');assert session.outcome=='cancelled'
        release(t);report.append({'case':'cancel_before_goal_acceptance','passed':True})

        t=request('reject');wait(lambda:session.state=='terminal');assert session.outcome=='rejected'
        release(t);report.append({'case':'rejected_goal_is_terminal','passed':True})

        t=request('normal',timeout=.25);wait(lambda:session.state=='terminal')
        assert session.cancel_requested and session.outcome=='cancelled'
        release(t);report.append({'case':'deadline_cancels_exact_goal','passed':True})

        t=request('normal',timeout=.2);wait(lambda:session.state=='running')
        assert session.motion_permitted(t)
        time.sleep(.22)  # Deliberately prevent the client timer from running.
        assert not session.motion_permitted(t) and session.cancel_requested
        wait(lambda:session.state=='terminal');release(t)
        report.append({'case':'command_path_checks_deadline_before_late_timer','passed':True})

        t=request('reject_cancel');wait(lambda:session.state=='running');session.cancel(t)
        wait(lambda:session.cancel_future is None and time.monotonic()>session.next_cancel-.4)
        assert session.state=='cancelling' and not session.cancel_acknowledged
        assert session.request('success') is None
        wait(lambda:session.state=='terminal');assert session.outcome=='aborted'
        release(t);report.append({'case':'cancel_rejection_does_not_release_goal','passed':True})

        t=request('normal');wait(lambda:session.state=='running');healthy=False
        wait(lambda:session.state=='terminal');assert session.outcome=='cancelled'
        healthy=True;release(t);report.append({'case':'health_loss_cancels_running_goal','passed':True})

        t=request('success');wait(lambda:session.state=='terminal')
        assert session.outcome=='nav2_succeeded_unverified'
        assert session.request('normal') is None
        old=t;release(t)
        t=request('normal');wait(lambda:session.state=='running')
        assert not session.cancel(old) and not session.release(old,stationary=True,stop_latched=True)
        session.cancel(t);wait(lambda:session.state=='terminal');release(t)
        report.append({'case':'success_requires_feedback_release_and_old_token_cannot_cancel_next','passed':True})
        wait(lambda:policy.tick(time.monotonic()))
        t=session.request('normal');assert policy.arm(t,stationary=True,now=time.monotonic())
        try:session.start(t,timeout=float('nan'))
        except ValueError:pass
        else:raise AssertionError('nonfinite deadline accepted')
        assert session.state=='terminal' and session.outcome=='not_dispatched' and policy.stop_required
        release(t);report.append({'case':'invalid_deadline_stops_without_dispatch','passed':True})

        t=request('success');wait(lambda:session.state=='running')
        time.sleep(.2)  # Server finishes while the client's result callback is delayed.
        session.cancel(t)
        wait(lambda:session.state=='terminal')
        assert session.outcome=='completed_after_cancel'
        release(t);report.append({'case':'late_success_after_cancel_is_not_arrival','passed':True})

        t=request('normal',timeout=.25);wait(lambda:session.state=='running')
        os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
        wait(lambda:session.cancel_requested)
        end=time.monotonic()+.4
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        assert session.state=='cancelling' and session.current==t
        assert session.request('success') is None
        assert not session.release(t,stationary=True,stop_latched=True)
        assert not session.motion_permitted(t)
        report.append({'case':'server_death_does_not_fabricate_terminal_result','passed':True})
        assert stops
        print(json.dumps({'passed':True,'domain':84,'checks':report,'stop_callback_calls':len(stops),
                          'scope':'real action transport against fake server; no routes, velocity ownership or measured arrival'},indent=2))
    except Exception as exc:
        print(json.dumps({'passed':False,'error':str(exc),'checks':report},indent=2));raise
    finally:
        session.destroy();node.destroy_timer(timer);node.destroy_node();rclpy.try_shutdown()
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=2)
        stream.close()


if __name__=='__main__':main()
