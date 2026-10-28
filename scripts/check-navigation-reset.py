#!/usr/bin/env python3
"""Reset the live world with an unarmed target and verify explicit seed recovery.

Only the invalidated old target receives an arm attempt, which must be rejected.
No fresh target is armed. The reset establishes the known origin for the seed;
measured simulator truth is never supplied to AMCL.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    if ROOT != Path('/home/jensen/.goske/codex/orbital-mechanic') or not os.environ.get('TMUX'):
        raise RuntimeError('Run inside Thor tmux orbital-work')
    if (os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'orbital-maintenance'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('Requires the live Orbital ROS/Gazebo environment')
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from lifecycle_msgs.srv import GetState

    units = ('orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service')
    def prop(unit, name):
        return subprocess.check_output(['systemctl', '--user', 'show', unit, '-p', name, '--value'], text=True).strip()
    invocations = {u: prop(u, 'InvocationID') for u in units}
    if any(not value for value in invocations.values()) or any(prop(u, 'ActiveState') != 'active' for u in units):
        raise RuntimeError('All owned services must be active')
    rclpy.init(args=[])
    node = rclpy.create_node('orbital_navigation_reset_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    samples, runs, reasons, violations = {}, set(), set(), set()
    clocks, regressions, poses = [], [], []
    report = {'passed': False, 'service_invocations': invocations, 'replies': [],
              'scope': 'unarmed live target invalidation/reset/reseed; moving-goal reset remains separate'}
    command_pub = node.create_publisher(String, '/navigation/command', 10)
    seed_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    stop = node.create_client(Trigger, '/motion/stop')

    def state(name, msg):
        value = json.loads(msg.data)
        samples[name] = (value, time.monotonic())
        if name == 'nav':
            runs.add(value['run_id'])
            if value['forwarded_commands'] != 0 or value['action_state'] not in ('idle', 'waiting_for_arm', 'terminal'):
                violations.add('Navigation dispatched or forwarded velocity')
        elif name == 'guard' and value['latched'] is not True:
            violations.add('Guard unlatched')
        elif name == 'health':
            reasons.add(value['reason'])

    def clock(msg):
        value = msg.clock.sec*10**9+msg.clock.nanosec
        if clocks and value < clocks[-1]:
            regressions.append((clocks[-1], value))
        clocks.append(value)
        samples['clock'] = (value, time.monotonic())

    def odom(msg):
        samples['odom'] = (msg, time.monotonic())
        v = msg.twist.twist
        values = (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)
        if not all(math.isfinite(x) for x in values) or math.hypot(*values[:3]) > .01 or math.hypot(*values[3:]) > .02:
            violations.add('Nonstationary odometry')

    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(String, topic, lambda m, n=name: state(n, m), 10)
                     for topic, name in (('/navigation/state', 'nav'), ('/navigation/health', 'health'), ('/motion/state', 'guard'))]
    subscriptions += [node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
                      node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
                      node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: poses.append(m), retained),
                      node.create_subscription(TwistStamped, '/motion/request', lambda m: violations.add('Velocity request'), 10)]

    def wait(predicate, seconds=8.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if violations or len(runs) > 1:
                raise RuntimeError('Ownership/motion violation: '+repr(sorted(violations)))
            if predicate():
                return
        raise RuntimeError('Timed out waiting for reset evidence')

    def fresh():
        return all(n in samples and time.monotonic()-samples[n][1] < .3 for n in ('nav', 'health', 'guard', 'clock', 'odom'))

    def command(op, expected, **payload):
        value = samples['nav'][0]
        sequence = value['command_sequence']+1
        command_pub.publish(String(data=json.dumps(dict(run_id=value['run_id'], sequence=sequence, op=op, **payload))))
        wait(lambda: (samples['nav'][0].get('last_reply') or {}).get('sequence') == sequence, 2.)
        reply = samples['nav'][0]['last_reply']
        report['replies'].append(reply)
        if reply != {'sequence': sequence, 'accepted': expected, 'op': op}:
            raise RuntimeError('Unexpected command reply: '+repr(reply))

    clients = []
    control = None
    try:
        wait(lambda: fresh() and samples['nav'][0]['healthy'] and samples['nav'][0]['action_state'] == 'idle'
             and command_pub.get_subscription_count() == 1 and seed_pub.get_subscription_count() >= 2)
        for name in ('amcl', 'controller_server', 'planner_server', 'bt_navigator'):
            client = node.create_client(GetState, '/'+name+'/get_state'); clients.append(client)
            wait(client.service_is_ready)
            future = client.call_async(GetState.Request()); wait(future.done)
            if future.result().current_state.id != 3:
                raise RuntimeError(name+' is not active')
        command('request', True, waypoint='pump')
        old = samples['nav'][0]['target']
        if old is None or samples['nav'][0]['action_state'] != 'waiting_for_arm':
            raise RuntimeError('No unarmed target was reserved')
        report['old_target'] = old
        old_epoch = samples['health'][0]['epoch']
        report['old_sim_ns'] = clocks[-1]
        control = subprocess.Popen(['gz', 'service', '-s', '/world/maintenance/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', 'reset: {all: true}'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        wait(lambda: control.poll() is not None, 6.)
        stdout, stderr = control.communicate()
        if control.returncode or 'data: true' not in stdout:
            raise RuntimeError('World reset was not acknowledged: '+stderr[-300:])
        wait(lambda: fresh() and regressions and samples['health'][0]['reason'] == 'initial_pose_required'
             and samples['health'][0]['epoch'] > old_epoch and not samples['nav'][0]['healthy']
             and samples['nav'][0]['action_state'] == 'terminal')
        report['clock_regressions'] = regressions
        report['reset_health_epoch'] = samples['health'][0]['epoch']
        command('arm', False, target=old)
        command('request', False, waypoint='cooler')
        # Deliberately wait with real post-reset sensor input. Time alone must
        # never bypass the explicit seed requirement.
        until = time.monotonic()+1.
        wait(lambda: time.monotonic() >= until, 2.)
        if not fresh() or samples['nav'][0]['healthy'] or samples['health'][0]['reason'] != 'initial_pose_required':
            raise RuntimeError('Localization recovered without an explicit seed')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'; msg.header.stamp = node.get_clock().now().to_msg()
        seed_ns = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if not 0 < seed_ns <= clocks[-1]+100000000:
            raise RuntimeError('Seed does not belong to the reset world clock')
        msg.pose.pose.orientation.w = 1.
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25**2
        msg.pose.covariance[35] = math.radians(15)**2
        poses.clear()
        seed_pub.publish(msg)
        wait(lambda: fresh() and samples['nav'][0]['healthy'] and samples['nav'][0]['action_state'] == 'idle'
             and any(p.header.stamp.sec*10**9+p.header.stamp.nanosec >= seed_ns for p in poses), 12.)
        report['seed_ns'] = seed_ns
        report['completion_after_recovery'] = samples['nav'][0]['last_completion']
        if ((report['completion_after_recovery'] or {}).get('target') != old
                or report['completion_after_recovery'].get('outcome') != 'not_dispatched'):
            raise RuntimeError('Old target terminal evidence was lost')
        command('arm', False, target=old)
        command('request', True, waypoint='cooler')
        new = samples['nav'][0]['target']
        if new is None or new == old or new['epoch'] <= old['epoch']:
            raise RuntimeError('Recovered target identity did not advance')
        report['new_target'] = new
        command('cancel', True, target=new)
        wait(lambda: fresh() and samples['nav'][0]['action_state'] == 'idle' and samples['nav'][0]['target'] is None)
        if any(prop(u, 'InvocationID') != invocations[u] or prop(u, 'ActiveState') != 'active' for u in units):
            raise RuntimeError('Service restart substituted for in-process reset recovery')
        report.update(passed=True, reasons=sorted(reasons), supervisor_run_id=next(iter(runs)))
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if control is not None and control.poll() is None:
            control.terminate(); control.wait(timeout=2)
        if stop.wait_for_service(timeout_sec=3.):
            future = stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            report['stop_ack'] = future.done() and future.result() is not None and future.result().success
        else:
            report['stop_ack'] = False
        report['passed'] = report['passed'] and report['stop_ack'] and not violations
        report['violations'] = sorted(violations)
        report['last_navigation'] = samples.get('nav', (None,))[0]
        report['last_health'] = samples.get('health', (None,))[0]
        node.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
