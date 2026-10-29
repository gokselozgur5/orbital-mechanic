#!/usr/bin/env python3
"""Real ROS arm-ordering proof with production supervisor/guard, synthetic health.

Private domain84 only. Nav2 stand-in records then rejects goals; no drive/world.
The guard applies reset normally; its response is held to control the race.
"""
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='84' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        raise RuntimeError('Requires empty localhost domain84')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.task import Future
    from rclpy.action import ActionServer, GoalResponse
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from orbital.ros_navigation_supervisor import NavigationSupervisor

    rclpy.init(args=[])
    fixture=Node('orbital_arm_fixture')
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(fixture,timeout_sec=.05)
    others=[n for n in fixture.get_node_names() if n!=fixture.get_name()]
    if others:
        fixture.destroy_node();rclpy.shutdown()
        raise RuntimeError('Domain84 occupied: '+repr(others))
    spec=importlib.util.spec_from_file_location('arm_proof_guard',ROOT/'scripts/motion-guard.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    guard=module.GuardNode()
    supervisor=NavigationSupervisor({'pump':(2.,1.8,0.)},goal_timeout=3.)
    executor=SingleThreadedExecutor()
    for node in (fixture,guard,supervisor):executor.add_node(node)
    state={'stationary':True,'publish_health':True,'sequence':0,'run':str(uuid.uuid4()),'release':None,'ack_at':None}
    resets=[];goals=[];observed={};checks=[]
    health=fixture.create_publisher(String,'/navigation/health',10)
    clock=fixture.create_publisher(Clock,'/clock',10)
    odom=fixture.create_publisher(Odometry,'/odom',10)
    commands=fixture.create_publisher(String,'/navigation/command',10)
    sub=fixture.create_subscription(String,'/navigation/state',lambda m:observed.update(json.loads(m.data)),10)
    started=time.monotonic()
    def publish():
        sim=100.+time.monotonic()-started;ns=round(sim*1e9)
        stamp=Time(sec=ns//10**9,nanosec=ns%10**9)
        clock.publish(Clock(clock=stamp))
        message=Odometry();message.header.frame_id='odom';message.child_frame_id='base_footprint';message.header.stamp=stamp
        message.pose.pose.orientation.w=1.;message.twist.twist.linear.x=0. if state['stationary'] else .1
        odom.publish(message)
        state['sequence']+=1
        if state['publish_health']:
            health.publish(String(data=json.dumps({'schema_version':1,'run_id':state['run'],'epoch':0,
                'sequence':state['sequence'],'sim_time':sim,'healthy':True,'stationary':state['stationary'],'reason':'valid'})))
    timer=fixture.create_timer(.02,publish)
    guard.destroy_service(guard.reset_service)
    async def reset(request,response):
        resets.append(time.monotonic())
        response=guard.reset(request,response)
        pending=Future(executor=executor);state['release']=pending
        await pending
        state['ack_at']=time.monotonic()
        return response
    guard.reset_service=guard.create_service(Trigger,'/motion/reset_stop',reset,callback_group=ReentrantCallbackGroup())
    def goal(request):
        goals.append({'at':time.monotonic(),'stationary':state['stationary'],'ack_at':state['ack_at']})
        return GoalResponse.REJECT
    server=ActionServer(fixture,NavigateToPose,'/navigate_to_pose',lambda handle:NavigateToPose.Result(),goal_callback=goal)
    def wait(predicate,seconds=4.):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.005)
            if predicate():return
        raise AssertionError(('timeout',observed,supervisor.lease.reason))
    def pump(seconds):
        until=time.monotonic()+seconds
        wait(lambda:time.monotonic()>=until,seconds+1)
    def idle():
        return observed.get('healthy') and observed.get('action_state')=='idle' and guard.guard.latched
    def command(op,**extra):
        seq=observed['command_sequence']+1
        commands.publish(String(data=json.dumps({'run_id':observed['run_id'],'sequence':seq,'op':op,**extra})))
        wait(lambda:(observed.get('last_reply') or {}).get('sequence')==seq)
        return observed['last_reply']['accepted']
    def reserve():
        wait(idle)
        assert command('request',waypoint='pump')
        return observed['target']
    def arm(target):
        state['release']=None;state['ack_at']=None
        assert command('arm',target=target)
        wait(lambda:state['release'] is not None)
    result={'passed':False,'checks':checks,'scope':'DDS protocol with production guard/supervisor and synthetic health; not physical motion'}
    try:
        wait(lambda:idle() and commands.get_subscription_count()==1 and supervisor.session.client.server_is_ready(),8)
        # Delay only the supervisor's periodic tick, leaving actual DDS callbacks
        # and fresh guard feedback running. The command path must enforce expiry.
        target=reserve();supervisor.timer.cancel();state['publish_health']=False
        pump(.3)
        seq=observed['command_sequence']+1;count=len(resets)
        commands.publish(String(data=json.dumps({'run_id':observed['run_id'],'sequence':seq,'op':'arm','target':target})))
        wait(lambda:(supervisor.last_reply or {}).get('sequence')==seq)
        assert supervisor.last_reply['accepted'] is False, 'Expired health accepted while periodic tick was delayed'
        assert len(resets)==count and not goals
        state['publish_health']=True;supervisor.timer.reset()
        wait(lambda:observed.get('command_sequence')==seq)
        command('cancel',target=target)  # It may already be cancelled by expiry.
        wait(idle)
        assert observed['last_completion']['target']==target and observed['last_completion']['outcome']=='not_dispatched'
        checks.append({'case':'command_path_rejects_expired_health_without_periodic_tick','passed':True})
        target=reserve();state['stationary']=False
        wait(lambda:not supervisor.lease.stationary and guard.guard.speed[0]>.05)
        count=len(resets)
        assert not command('arm',target=target)
        pump(.1);assert len(resets)==count and not goals
        assert command('cancel',target=target)
        state['stationary']=True;wait(idle)
        checks.append({'case':'moving_feedback_refuses_arm_before_guard_rpc','passed':True})

        target=reserve();arm(target)
        pump(.12);assert not goals and not guard.guard.latched
        checks.append({'case':'unlatched_guard_without_response_cannot_dispatch','passed':True})
        state['stationary']=False
        wait(lambda:not supervisor.lease.stationary and guard.guard.speed[0]>.05)
        state['release'].set_result(True)
        pump(.2)
        assert not goals, ('Dispatched after stationarity was lost',goals)
        state['stationary']=True;wait(idle)
        assert not command('arm',target=target)
        checks.append({'case':'lost_stationarity_during_pending_arm_invalidates_target','passed':True})

        target=reserve();arm(target)
        supervisor.timer.cancel()
        state['stationary']=False
        wait(lambda:not supervisor.lease.stationary and guard.guard.speed[0]>.05)
        state['stationary']=True
        wait(lambda:supervisor.lease.stationary and guard.guard.speed[0]<.01)
        # Both health callbacks ran while the periodic supervisor tick was held.
        # Recovery must not erase the earlier moving observation for this target.
        state['release'].set_result(True)
        supervisor.timer.reset()
        wait(idle)
        assert not goals and not command('arm',target=target)
        checks.append({'case':'motion_then_recovery_before_tick_cannot_dispatch_old_target','passed':True})

        target=reserve();arm(target)
        pump(.12);assert not goals
        state['release'].set_result(True)
        wait(lambda:len(goals)==1)
        assert goals[0]['stationary'] and goals[0]['ack_at'] is not None and goals[0]['at']>=goals[0]['ack_at']
        wait(idle)
        checks.append({'case':'stationary_target_dispatches_only_after_guard_response','passed':True})

        count=len(goals);target=reserve();arm(target)
        pump(.6)
        assert len(goals)==count and supervisor.arm_future is not None
        assert observed['target']==target, 'Released ownership before unresolved arm response'
        state['release'].set_result(True)
        wait(idle)
        assert len(goals)==count and not command('arm',target=target)
        checks.append({'case':'late_guard_response_cannot_dispatch_or_rearm_old_target','passed':True})
        result.update(passed=True,goals=goals,reset_calls=len(resets),last_state=observed)
    except Exception as error:
        result.update(error=str(error),goals=goals,last_state=observed)
    finally:
        if state['release'] is not None and not state['release'].done():state['release'].set_result(True)
        guard.guard.stop('arm_test_cleanup');guard.tick()
        server.destroy()
        for node in (supervisor,guard,fixture):executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.try_shutdown()
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
