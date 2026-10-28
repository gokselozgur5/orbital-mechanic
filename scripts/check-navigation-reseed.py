#!/usr/bin/env python3
"""Briefly drive a native goal, then reseed and require terminal old-goal ownership.

Requires the freshly reset origin. A separately pinned trial guard bounds the
navigation owner's lifetime if this probe dies or stalls. No second goal is armed.
"""
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    if ROOT != Path('/home/jensen/.goske/codex/orbital-mechanic') or not os.environ.get('TMUX'):
        raise RuntimeError('Run inside Thor tmux orbital-work')
    if (os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'orbital-maintenance'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('Requires live Orbital environment')
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    invocation = subprocess.check_output(['systemctl', '--user', 'show', 'orbital-navigation.service',
                                         '-p', 'InvocationID', '--value'], text=True).strip()
    rclpy.init(args=[])
    node = rclpy.create_node('orbital_reseed_trial', parameter_overrides=[Parameter('use_sim_time', value=True)])
    samples, runs, reasons, violations = {}, set(), set(), set()
    requests = []
    report = {'passed': False, 'replies': [], 'navigation_invocation': invocation,
              'scope': 'native moving-goal reseed ownership; odometry stopping, not independent physical braking distance'}
    pub = node.create_publisher(String, '/navigation/command', 10)
    seed = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    stop = node.create_client(Trigger, '/motion/stop')
    guardian = None
    old = None

    def state(name, msg):
        value = json.loads(msg.data)
        samples[name] = (value, time.monotonic())
        if name == 'nav':
            runs.add(value['run_id'])
        if name == 'health':
            reasons.add(value['reason'])

    def odom(msg):
        samples['odom'] = (msg, time.monotonic())
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not all(math.isfinite(v) for v in (p.x, p.y, yaw)) or math.hypot(p.x, p.y) > .35 or abs(yaw) > .8:
            violations.add('Left the bounded origin commissioning region')

    def imu(msg):
        q = msg.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        if not all(math.isfinite(x) for x in (roll, pitch)) or max(abs(roll), abs(pitch)) > .05:
            violations.add('Chassis tilt exceeded the commissioning bound')
        samples['imu'] = (msg, time.monotonic())

    subscriptions = [node.create_subscription(String, topic, lambda m, n=name: state(n, m), 10)
                     for topic, name in (('/navigation/state', 'nav'), ('/navigation/health', 'health'), ('/motion/state', 'guard'))]
    subscriptions += [node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
                      node.create_subscription(Imu, '/imu', imu, qos_profile_sensor_data),
                      node.create_subscription(TwistStamped, '/motion/request', lambda m: requests.append(m), 10)]

    def wait(predicate, seconds=5.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if violations or len(runs) > 1:
                raise RuntimeError('Trial bounds/ownership violation: '+repr(sorted(violations)))
            if predicate():
                return
        raise RuntimeError('Trial observation timeout')

    def fresh():
        return all(n in samples and time.monotonic()-samples[n][1] < .3 for n in ('nav', 'health', 'guard', 'odom', 'imu'))

    def stationary():
        if not fresh():
            return False
        v = samples['odom'][0].twist.twist
        return math.hypot(v.linear.x, v.linear.y, v.linear.z) < .01 and math.hypot(v.angular.x, v.angular.y, v.angular.z) < .02

    def heading():
        q = samples['odom'][0].pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def command(op, expected, **payload):
        state = samples['nav'][0]
        sequence = state['command_sequence']+1
        pub.publish(String(data=json.dumps(dict(run_id=state['run_id'], sequence=sequence, op=op, **payload))))
        wait(lambda: (samples['nav'][0].get('last_reply') or {}).get('sequence') == sequence, 1.)
        reply = samples['nav'][0]['last_reply']; report['replies'].append(reply)
        if reply != {'sequence': sequence, 'accepted': expected, 'op': op}:
            raise RuntimeError('Unexpected reply: '+repr(reply))

    try:
        wait(lambda: stationary() and samples['nav'][0]['healthy'] and samples['nav'][0]['action_state'] == 'idle'
             and samples['guard'][0]['latched'] and pub.get_subscription_count() == 1
             and seed.get_subscription_count() >= 2 and stop.service_is_ready(), 10.)
        p = samples['odom'][0].pose.pose.position
        if math.hypot(p.x, p.y) > .02 or abs(heading()) > .05:
            raise RuntimeError('A freshly reset origin is required for this trial seed')
        guardian = subprocess.Popen([sys.executable, str(ROOT/'scripts/navigation-trial-guard.py'),
            '--invocation', invocation, '--seconds', '12'], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        wait(lambda: bool(select.select([guardian.stdout], [], [], 0)[0]), 3.)
        guard_ready = json.loads(guardian.stdout.readline())
        if guard_ready.get('ready') is not True or guard_ready.get('invocation') != invocation:
            raise RuntimeError('Independent trial guard did not pin navigation')
        report['trial_guard'] = guard_ready
        command('request', True, waypoint='pump')
        old = samples['nav'][0]['target']; report['old_target'] = old
        command('arm', True, target=old)
        def moving():
            if not fresh() or samples['nav'][0]['action_state'] != 'running':
                return False
            v = samples['odom'][0].twist.twist
            return (abs(v.linear.x) > .03 or abs(v.angular.z) > .05) and bool(requests)
        wait(moving, 4.)
        p = samples['odom'][0].pose.pose.position
        if math.hypot(p.x, p.y) > .15 or abs(heading()) > .3:
            raise RuntimeError('No longer within the uncertain origin seed region')
        report['requests_before_seed'] = len(requests)
        report['motion_before_seed'] = {'linear': samples['odom'][0].twist.twist.linear.x,
                                        'angular': samples['odom'][0].twist.twist.angular.z}
        msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg(); msg.pose.pose.orientation.w = 1.
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25**2
        msg.pose.covariance[35] = math.radians(15)**2
        reasons.clear()
        started = time.monotonic(); seed.publish(msg)
        wait(lambda: 'initial_pose_replaced' in reasons and stationary() and samples['guard'][0]['latched']
             and samples['nav'][0]['action_state'] == 'idle'
             and (samples['nav'][0].get('last_completion') or {}).get('target') == old, 4.)
        report['seed_to_stationary_terminal_seconds'] = time.monotonic()-started
        completion = samples['nav'][0]['last_completion']; report['completion'] = completion
        if completion['outcome'] not in ('cancelled', 'aborted'):
            raise RuntimeError('Dispatched target did not end cancelled/aborted after reseed')
        command('arm', False, target=old)
        baseline = samples['nav'][0]['forwarded_commands']
        until = time.monotonic()+1.
        wait(lambda: time.monotonic() >= until, 2.)
        if not stationary() or not samples['guard'][0]['latched'] or samples['nav'][0]['forwarded_commands'] != baseline:
            raise RuntimeError('Old command stream resumed after localization recovered')
        command('request', True, waypoint='cooler')
        new = samples['nav'][0]['target']; report['new_target'] = new
        if new is None or new == old or new['epoch'] <= old['epoch']:
            raise RuntimeError('New target did not use a fresh identity')
        command('cancel', True, target=new)
        wait(lambda: stationary() and samples['guard'][0]['latched'] and samples['nav'][0]['action_state'] == 'idle')
        report.update(passed=True, reasons=sorted(reasons), supervisor_run_id=next(iter(runs)))
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if stop.wait_for_service(timeout_sec=3.):
            future = stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            report['stop_ack'] = future.done() and future.result() is not None and future.result().success
        else:
            report['stop_ack'] = False
        report['passed'] = report['passed'] and report['stop_ack'] and not violations
        if guardian is not None:
            if guardian.poll() is None:
                if report['passed'] and stationary() and samples['nav'][0]['action_state'] == 'idle' and samples['guard'][0]['latched']:
                    guardian.stdin.write(b'safe\n'); guardian.stdin.flush()
                guardian.stdin.close()
            guardian.wait(timeout=4)
            remaining = guardian.stdout.read()
            report['guardian_result'] = json.loads(remaining) if remaining else {'error': guardian.stderr.read().decode()}
            report['passed'] = report['passed'] and report['guardian_result'].get('action') == 'disarmed'
        report['last_navigation'] = samples.get('nav', (None,))[0]
        report['last_guard'] = samples.get('guard', (None,))[0]
        report['last_health'] = samples.get('health', (None,))[0]
        report['violations'] = sorted(violations)
        node.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
