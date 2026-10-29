#!/usr/bin/env python3
"""Commission real navigation CLI children on empty localhost domain 84.

Production supervisor/guard, synthetic feedback, and a goal-rejecting Nav2 peer.
No world or drive connection; receipts prove command delivery, not physical motion.
"""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '84'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('Requires empty localhost domain 84')
    import rclpy
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.task import Future
    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import NavigateToPose
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from orbital.navigation_configuration import load_waypoint_contract
    from orbital.ros_navigation_supervisor import NavigationSupervisor

    output = ROOT/'runtime/navigation-cli-proof'
    output.mkdir(parents=True)  # Fresh directory required: preserve earlier trials.
    rclpy.init(args=[])
    fixture = Node('orbital_navigation_cli_fixture')
    until = time.monotonic()+2
    while time.monotonic() < until:
        rclpy.spin_once(fixture, timeout_sec=.05)
    others = [n for n in fixture.get_node_names() if n != fixture.get_name()]
    if others:
        fixture.destroy_node(); rclpy.shutdown()
        raise RuntimeError('Domain 84 occupied: '+repr(others))
    inputs, calls, goals, checks = [], [], [], []

    class RecordedSupervisor(NavigationSupervisor):
        def on_command(self, message):
            inputs.append(json.loads(message.data))
            super().on_command(message)

    spec = importlib.util.spec_from_file_location('cli_proof_guard', ROOT/'scripts/motion-guard.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    guard = module.GuardNode()
    waypoints, _ = load_waypoint_contract(ROOT/'config/known-map.json', root=ROOT, require_geometry=True)
    supervisor = RecordedSupervisor(waypoints, goal_timeout=3.)
    executor = SingleThreadedExecutor()
    for node in (fixture, guard, supervisor):
        executor.add_node(node)
    state = {'stationary':True, 'sequence':0, 'release':None, 'drop_from':None}
    run, started = str(uuid.uuid4()), time.monotonic()
    health = fixture.create_publisher(String, '/navigation/health', 10)
    clock = fixture.create_publisher(Clock, '/clock', 10)
    odom = fixture.create_publisher(Odometry, '/odom', 10)

    def publish_feedback():
        sim = 100.+time.monotonic()-started; ns = round(sim*1e9)
        stamp = Time(sec=ns//10**9, nanosec=ns%10**9)
        clock.publish(Clock(clock=stamp))
        message = Odometry(); message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'; message.header.stamp = stamp
        message.pose.pose.orientation.w = 1.
        message.twist.twist.linear.x = 0. if state['stationary'] else .1
        odom.publish(message)
        state['sequence'] += 1
        health.publish(String(data=json.dumps({'schema_version':1, 'run_id':run,
            'epoch':0, 'sequence':state['sequence'], 'sim_time':sim, 'healthy':True,
            'stationary':state['stationary'], 'reason':'valid'})))

    fixture.create_timer(.02, publish_feedback)
    guard.destroy_service(guard.reset_service)

    async def held_reset(request, response):
        response = guard.reset(request, response)
        state['release'] = Future(executor=executor)
        await state['release']
        return response

    guard.reset_service = guard.create_service(Trigger, '/motion/reset_stop', held_reset,
                                               callback_group=ReentrantCallbackGroup())

    def reject_goal(request):
        goals.append(time.monotonic())
        return GoalResponse.REJECT

    server = ActionServer(fixture, NavigateToPose, '/navigate_to_pose',
                          lambda handle:NavigateToPose.Result(), goal_callback=reject_goal)
    state_publisher = supervisor.nav_state_pub

    class StateFilter:
        def publish(self, message):
            if state['drop_from'] is None or len(inputs) < state['drop_from']:
                state_publisher.publish(message)

    supervisor.nav_state_pub = StateFilter()  # Deliberate lost-reply control only.

    def wait(predicate, seconds=5.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
            if predicate():
                return
        raise AssertionError(('timeout', supervisor.session.state, supervisor.lease.reason))

    def pump(seconds):
        end = time.monotonic()+seconds
        wait(lambda:time.monotonic() >= end, seconds+1.)

    def idle():
        return (supervisor.lease.healthy and supervisor.session.current is None
                and supervisor.session.state == 'idle' and guard.guard.latched)

    def invoke(arguments, code, delivery, publications, timeout=5.):
        before = len(inputs); stem = output/f'{len(calls)+1:02d}-{arguments[0]}'
        with stem.with_suffix('.stdout.json').open('w') as out, stem.with_suffix('.stderr.log').open('w') as err:
            child = subprocess.Popen([sys.executable, '-B', str(ROOT/'scripts/orbital-navigate.py'),
                '--timeout', str(timeout), *arguments], cwd=ROOT, stdout=out, stderr=err)
            try:
                wait(lambda:child.poll() is not None, timeout+5.)
            finally:
                if child.poll() is None:
                    child.kill()
                child.wait(timeout=3.)
        pump(.1)
        receipt = json.loads(stem.with_suffix('.stdout.json').read_text())
        calls.append({'arguments':arguments, 'exit':child.returncode, 'receipt':receipt,
                      'received_commands':inputs[before:]})
        assert (child.returncode, receipt['delivery']) == (code, delivery), calls[-1]
        assert len(inputs)-before == publications, calls[-1]
        return receipt

    result = {'passed':False, 'checks':checks, 'calls':calls,
              'scope':'native CLI/DDS protocol with synthetic feedback; not physical motion'}
    try:
        wait(lambda:idle() and supervisor.session.client.server_is_ready(), 8.)
        invoke(['status'], 0, 'observed', 0)
        checks.append('status_sends_no_command')
        receipt = invoke(['request','pump'], 0, 'acknowledged', 1)
        target_file = output/'request.json'; target_file.write_text(json.dumps(receipt))
        invoke(['cancel','--target-file',str(target_file)], 0, 'acknowledged', 1)
        wait(idle)
        assert supervisor.last_completion['outcome'] == 'not_dispatched' and not goals
        checks.append('request_receipt_selects_exact_cancel_target')
        invoke(['arm','--target-file',str(target_file)], 2, 'not_sent', 0)
        checks.append('released_token_cannot_publish_arm')

        receipt = invoke(['request','pump'], 0, 'acknowledged', 1)
        token = json.dumps(receipt['target']); state['stationary'] = False
        wait(lambda:not supervisor.lease.stationary and guard.guard.speed[0] > .05)
        invoke(['arm','--target',token], 1, 'rejected', 1)
        assert state['release'] is None and not goals
        invoke(['cancel','--target',token], 0, 'acknowledged', 1)
        state['stationary'] = True; wait(idle)
        checks.append('moving_arm_returns_explicit_rejection_without_reset')

        receipt = invoke(['request','pump'], 0, 'acknowledged', 1)
        token = json.dumps(receipt['target'])
        invoke(['arm','--target',token], 0, 'acknowledged', 1)
        wait(lambda:state['release'] is not None)
        assert supervisor.arm_future is not None and supervisor.session.current is not None and not goals
        pump(.6); state['release'].set_result(True); wait(idle)
        assert not goals
        checks.append('arm_ack_does_not_imply_dispatch_or_release_pending_rpc')

        peer = fixture.create_publisher(String, '/navigation/command', 1)
        try:
            pump(.5)
            response = invoke(['request','pump'], 2, 'not_sent', 0)
            assert response['error'] == 'Ambiguous command publisher', response
        finally:
            fixture.destroy_publisher(peer)
        wait(lambda:not supervisor.get_publishers_info_by_topic('/navigation/command'))
        checks.append('competing_publisher_refused_before_publication')
        with (ROOT/'runtime/navigation-client-84.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            response = invoke(['request','pump'], 2, 'not_sent', 0)
            assert 'Another navigation command client' in response['error'], response
        checks.append('checkout_lock_refuses_second_client')

        state['drop_from'] = len(inputs)+1
        invoke(['request','pump'], 3, 'uncertain', 1, timeout=1.5)
        assert supervisor.session.current is not None and supervisor.session.state == 'waiting_for_arm'
        state['drop_from'] = None
        status = invoke(['status'], 0, 'observed', 0)
        token = json.dumps(status['state']['target'])
        invoke(['cancel','--target',token], 0, 'acknowledged', 1); wait(idle)
        checks.append('lost_reply_preserves_owned_target_without_retransmission')
        assert not goals and supervisor.forwarded == 0 and guard.guard.latched
        result.update(passed=True, received_commands=inputs, goals=goals,
                      final_state={'idle':idle(), 'forwarded_commands':supervisor.forwarded})
    except Exception as error:
        result.update(error=str(error), received_commands=inputs, goals=goals)
    finally:
        if state['release'] is not None and not state['release'].done():
            state['release'].set_result(True)
        guard.guard.stop('cli_test_cleanup'); guard.tick(); server.destroy()
        for node in (supervisor, guard, fixture):
            executor.remove_node(node); node.destroy_node()
        executor.shutdown(); rclpy.try_shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
