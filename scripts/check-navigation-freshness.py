#!/usr/bin/env python3
"""Real scan/TF expiry must inhibit the live supervisor; optional --scan-gap.

No arm or velocity command is sent. A separate pidfd-bound helper requests AMCL
resume after a two-second timeout even if the probe dies. Requires seeded, idle, guarded
native navigation. Leaves the same services running and the guard latched.
The scan case lowers only lidar rate and restores the versioned 10 Hz rate.
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def pause_helper(fd):
    try:
        signal.pidfd_send_signal(fd, signal.SIGSTOP)
        print('paused', flush=True)
        select.select([sys.stdin], [], [], 2.)
    finally:
        signal.pidfd_send_signal(fd, signal.SIGCONT)
        os.close(fd)


def set_scan_rate(rate):
    subprocess.run(['gz', 'service', '-s', '/scan/set_rate', '--reqtype', 'gz.msgs.Double',
                    '--reptype', 'gz.msgs.Empty', '--timeout', '1500', '--req', 'data: '+str(rate)],
                   check=True, timeout=5, stdout=subprocess.DEVNULL)


def scan_helper():
    try:
        set_scan_rate(.1)
        print('paused', flush=True)
        select.select([sys.stdin], [], [], 3.)
    finally:
        set_scan_rate(10.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-gap', action='store_true')
    args = parser.parse_args()
    if ROOT != Path('/home/jensen/.goske/codex/orbital-mechanic') or not os.environ.get('TMUX'):
        raise RuntimeError('Run inside Thor tmux orbital-work')
    if (os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
            or os.environ.get('GZ_PARTITION') != 'orbital-maintenance'):
        raise RuntimeError('Requires live localhost Orbital domain42')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from lifecycle_msgs.srv import GetState

    def prop(unit, name):
        return subprocess.check_output(['systemctl', '--user', 'show', unit, '-p', name, '--value'], text=True).strip()

    units = ('orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service')
    invocations = {u: prop(u, 'InvocationID') for u in units}
    if any(not value for value in invocations.values()) or any(prop(u, 'ActiveState') != 'active' for u in units):
        raise RuntimeError('All owned services must already be active')
    group = prop('orbital-localization.service', 'ControlGroup')
    if not group.endswith('/orbital-localization.service'):
        raise RuntimeError('Unexpected localization cgroup')
    pids = []
    for value in (Path('/sys/fs/cgroup')/group.lstrip('/')/'cgroup.procs').read_text().split():
        try:
            if os.readlink('/proc/'+value+'/exe') == '/opt/ros/jazzy/lib/nav2_amcl/amcl':
                pids.append(int(value))
        except FileNotFoundError:
            pass
    if len(pids) != 1:
        raise RuntimeError('Expected exactly one AMCL executable in the owned cgroup')
    pid = pids[0]
    fd = os.pidfd_open(pid)
    # Pinning prevents PID reuse from targeting another process after discovery.
    if (str(pid) not in (Path('/sys/fs/cgroup')/group.lstrip('/')/'cgroup.procs').read_text().split()
            or os.readlink(f'/proc/{pid}/exe') != '/opt/ros/jazzy/lib/nav2_amcl/amcl'):
        os.close(fd)
        raise RuntimeError('AMCL left the owned cgroup')
    rclpy.init(args=[])
    node = rclpy.create_node('orbital_live_tf_loss_probe')
    samples, stamps, counts, runs, violations, reasons = {}, {}, {}, set(), set(), set()
    helper = None
    report = {'passed': False, 'case': 'scan_gap' if args.scan_gap else 'tf_gap',
              'amcl_pid': pid, 'service_invocations': invocations,
              'scope': 'stationary live input expiry and recovery; no moving-goal braking proof'}
    stop = node.create_client(Trigger, '/motion/stop')
    pub = node.create_publisher(String, '/navigation/command', 10)

    def receive(name, msg):
        value = json.loads(msg.data)
        samples[name] = (value, time.monotonic())
        if name == 'nav':
            runs.add(value['run_id'])
            if value['forwarded_commands'] != 0 or value['action_state'] not in ('idle', 'waiting_for_arm', 'terminal'):
                violations.add('Navigation dispatched or forwarded commands')
        elif name == 'guard' and value.get('latched') is not True:
            violations.add('Guard became unlatched')
        elif name == 'health':
            reasons.add(value['reason'])

    def sensor(name, msg):
        counts[name] = counts.get(name, 0)+1
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        stamps.setdefault(name, [stamp, stamp])[1] = stamp
        samples[name] = (msg, time.monotonic())
        if name == 'odom':
            v = msg.twist.twist
            values = (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)
            if not all(math.isfinite(x) for x in values) or math.hypot(*values[:3]) > .01 or math.hypot(*values[3:]) > .02:
                violations.add('Odometry was not stationary')

    subscriptions = [node.create_subscription(String, topic, lambda m, n=name: receive(n, m), 10)
                     for topic, name in (('/navigation/state', 'nav'), ('/navigation/health', 'health'),
                                         ('/navigation/interlock', 'interlock'), ('/motion/state', 'guard'))]
    subscriptions += [node.create_subscription(LaserScan, '/scan', lambda m: sensor('scan', m), qos_profile_sensor_data),
                      node.create_subscription(Odometry, '/odom', lambda m: sensor('odom', m), qos_profile_sensor_data),
                      node.create_subscription(TwistStamped, '/motion/request', lambda m: violations.add('Velocity request observed'), 10)]

    def wait(predicate, seconds=5.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if violations or len(runs) > 1:
                raise RuntimeError('Live ownership/stationarity violation: '+repr(sorted(violations)))
            if predicate():
                return
        raise RuntimeError('Timed out waiting for live fault evidence')

    def fresh(include_scan=True):
        names = ('nav', 'guard', 'health', 'odom', 'interlock')+ (('scan',) if include_scan else ())
        return all(n in samples and time.monotonic()-samples[n][1] < .3 for n in names)

    def command(op, **payload):
        value = samples['nav'][0]
        seq = value['command_sequence']+1
        pub.publish(String(data=json.dumps(dict(run_id=value['run_id'], sequence=seq, op=op, **payload))))
        wait(lambda: (samples['nav'][0].get('last_reply') or {}).get('sequence') == seq, .5)
        return samples['nav'][0]['last_reply']

    def resume():
        nonlocal helper
        if helper is not None:
            if helper.poll() is None:
                helper.stdin.close()
            helper.wait(timeout=6)
            failed = helper.returncode
            helper = None
            if failed:
                raise RuntimeError('Input restoration helper failed')

    try:
        wait(lambda: fresh() and samples['nav'][0]['healthy'] and samples['nav'][0]['action_state'] == 'idle'
             and samples['nav'][0]['target'] is None and pub.get_subscription_count() == 1, 10.)
        clients = []
        for name in ('amcl', 'controller_server', 'planner_server', 'bt_navigator'):
            client = node.create_client(GetState, '/'+name+'/get_state'); clients.append(client)
            wait(client.service_is_ready)
            future = client.call_async(GetState.Request()); wait(future.done)
            if future.result().current_state.id != 3:
                raise RuntimeError(name+' must be active before fault injection')
        base_counts = dict(counts)
        acknowledgments = samples['interlock'][0]['stop_acknowledgments']
        helper_args = ['--scan-helper'] if args.scan_gap else ['--pause-helper', str(fd)]
        started = time.monotonic()
        helper = subprocess.Popen([sys.executable, __file__, *helper_args],
                                  pass_fds=(fd,), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  text=True, start_new_session=True)
        # Keep consuming sensor callbacks while the Gazebo service discovers its
        # peer. Blocking here would relabel old queued scans with new receipt times.
        wait(lambda: bool(select.select([helper.stdout], [], [], 0.)[0]), 6.)
        if helper.stdout.readline().strip() != 'paused':
            raise RuntimeError('Input fault was not acknowledged')
        report['helper_ack_seconds'] = time.monotonic()-started
        expected_reason = 'scan_stale' if args.scan_gap else 'transform_invalid'
        wait(lambda: fresh(include_scan=not args.scan_gap) and samples['health'][0]['reason'] == expected_reason
             and not samples['nav'][0]['healthy'], 1.5)
        report['invalidation_observed_seconds'] = time.monotonic()-started
        report['timing_scope'] = 'wall time from helper launch; includes process/service discovery and acknowledgment'
        if not args.scan_gap and Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'T':
            raise RuntimeError('AMCL resumed before the rejection check')
        reply = command('request', waypoint='pump')
        if reply.get('accepted') is not False or reply.get('op') != 'request':
            raise RuntimeError('Stale-TF request was not explicitly rejected')
        report['fault_reply'] = reply
        if helper.poll() is not None:
            raise RuntimeError('Fault ended before command rejection was observed')
        advancing = ('odom',) if args.scan_gap else ('scan', 'odom')
        if any(counts[n] <= base_counts[n]+2 or stamps[n][1] <= stamps[n][0] for n in advancing):
            raise RuntimeError('Unaffected real inputs did not keep advancing')
        if args.scan_gap and time.monotonic()-samples['scan'][1] < .5:
            raise RuntimeError('Scan gap did not exceed the configured input deadline')
        resume()
        wait(lambda: fresh() and samples['nav'][0]['healthy']
             and samples['interlock'][0]['stop_acknowledgments'] > acknowledgments, 12.)
        if args.scan_gap:
            # Confirm the restored stream continues, not just one delayed scan.
            initial_stamp = stamps['scan'][1]
            initial_count = counts['scan']
            until = time.monotonic()+1.5
            wait(lambda: time.monotonic() >= until, 2.)
            report['recovered_scan_samples'] = counts['scan']-initial_count
            if not fresh() or report['recovered_scan_samples'] < 5 or stamps['scan'][1] <= initial_stamp:
                raise RuntimeError('Restored lidar stream did not remain fresh and advancing')
        if samples['nav'][0]['target'] is not None or samples['nav'][0]['action_state'] != 'idle':
            raise RuntimeError('Recovery created a target without a new request')
        # Positive control: a new request can reserve a target after recovery,
        # but it still cannot move without a separate arm. Cancel it explicitly.
        reply = command('request', waypoint='pump')
        target = samples['nav'][0]['target']
        if reply.get('accepted') is not True or target is None:
            raise RuntimeError('Recovered localization did not permit a fresh request')
        report['recovery_reply'] = reply
        if command('cancel', target=target).get('accepted') is not True:
            raise RuntimeError('Unarmed positive-control target could not be cancelled')
        wait(lambda: fresh() and samples['nav'][0]['action_state'] == 'idle' and samples['nav'][0]['target'] is None)
        if any(prop(u, 'InvocationID') != invocations[u] or prop(u, 'ActiveState') != 'active' for u in units):
            raise RuntimeError('A service restarted or stopped during the bounded suspension')
        report.update(passed=True, reasons=sorted(reasons), counts=counts,
                      stop_acknowledgments=samples['interlock'][0]['stop_acknowledgments']-acknowledgments,
                      supervisor_run_id=next(iter(runs)), final_guard_latched=samples['guard'][0]['latched'])
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        try:
            resume()
        except Exception as exc:
            report.update(passed=False, restoration_error=str(exc))
        finally:
            if not args.scan_gap:
                try:
                    signal.pidfd_send_signal(fd, signal.SIGCONT)
                except ProcessLookupError:
                    report.update(passed=False, restoration_error='Pinned AMCL process exited')
            os.close(fd)
        # Retry the versioned rate on failed scan runs; restoration errors must
        # never skip the independent guard stop below.
        if args.scan_gap and not report['passed']:
            try:
                set_scan_rate(10.)
            except Exception as exc:
                report['scan_restore_error'] = str(exc)
        if stop.wait_for_service(timeout_sec=3.):
            future = stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            report['stop_ack'] = future.done() and future.result() is not None and future.result().success
        else:
            report['stop_ack'] = False
        report['passed'] = report['passed'] and report['stop_ack'] and not violations
        report['violations'] = sorted(violations)
        node.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--pause-helper':
        pause_helper(int(sys.argv[2]))
    elif sys.argv[1:] == ['--scan-helper']:
        scan_helper()
    else:
        raise SystemExit(main())
