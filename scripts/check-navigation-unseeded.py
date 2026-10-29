#!/usr/bin/env python3
"""Attempt valid navigation commands against an explicitly unseeded live stack.

Requires configured native Nav2 servers and advancing real sensors. Never publishes
initial pose or raw velocity. A surprising acceptance aborts and requests stop.
Leave the world unseeded and guard latched for explicit subsequent commissioning.
"""
import json
import math
import os
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires the live Orbital localhost domain42')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Imu
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener, TransformException
    from rclpy.time import Time

    rclpy.init(args=[])
    node = rclpy.create_node('orbital_unseeded_navigation_probe')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    samples, stamps, counts, runs, violations = {}, {}, {}, set(), set()
    report = {'passed': False, 'replies': [], 'scope': 'live unseeded admission refusal; no route or recovery acceptance'}
    stop = node.create_client(Trigger, '/motion/stop')
    command = node.create_publisher(String, '/navigation/command', 10)

    def receive(name, msg):
        counts[name] = counts.get(name, 0)+1
        if name in ('nav', 'guard'):
            value = json.loads(msg.data)
            if name == 'nav':
                runs.add(value['run_id'])
                if value['healthy'] or value['target'] is not None or value['action_state'] != 'idle' or value['forwarded_commands'] != 0:
                    violations.add('Navigation became healthy, owned a target or forwarded velocity')
            elif value.get('latched') is not True:
                violations.add('Guard became unlatched')
        else:
            value = msg
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            stamps.setdefault(name, [stamp, stamp])[1] = stamp
            if name in ('pose', 'request'):
                violations.add('Unexpected '+name+' sample')
            if name == 'odom':
                v = msg.twist.twist
                values = (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)
                if not all(math.isfinite(x) for x in values) or math.hypot(*values[:3]) > .01 or math.hypot(*values[3:]) > .02:
                    violations.add('Robot odometry is not stationary')
        samples[name] = (value, time.monotonic())

    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(kind, topic, lambda m, n=name: receive(n, m), qos)
                     for kind, topic, name, qos in (
                         (String, '/navigation/state', 'nav', 10), (String, '/motion/state', 'guard', 10),
                         (Odometry, '/odom', 'odom', qos_profile_sensor_data),
                         (LaserScan, '/scan', 'scan', qos_profile_sensor_data),
                         (Imu, '/imu', 'imu', qos_profile_sensor_data),
                         (PoseWithCovarianceStamped, '/amcl_pose', 'pose', retained),
                         (TwistStamped, '/motion/request', 'request', 10))]
    clients = []

    def wait(predicate, seconds=5.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if violations:
                raise RuntimeError('; '.join(sorted(violations)))
            if predicate():
                return
        raise RuntimeError('Timed out waiting for live evidence')

    def checkpoint():
        if len(runs) != 1:
            raise RuntimeError('Supervisor run changed')
        for name in ('nav', 'guard', 'odom', 'scan', 'imu'):
            if name not in samples or time.monotonic()-samples[name][1] > .5:
                raise RuntimeError('Missing or stale '+name)
        for name in ('odom', 'scan', 'imu'):
            if stamps[name][1] <= stamps[name][0]:
                raise RuntimeError('Nonadvancing '+name)
        try:
            buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return
        raise RuntimeError('Map pose exists before explicit seed')

    try:
        wait(lambda: all(counts.get(n, 0) > 2 for n in ('nav', 'guard', 'odom', 'scan', 'imu'))
             and command.get_subscription_count() == 1 and stop.service_is_ready(), 10.)
        checkpoint()
        report['lifecycle_states'] = {}
        for name in ('amcl', 'controller_server', 'planner_server', 'bt_navigator'):
            client = node.create_client(GetState, '/'+name+'/get_state'); clients.append(client)
            wait(client.service_is_ready)
            future = client.call_async(GetState.Request()); wait(future.done)
            state = future.result().current_state.id
            report['lifecycle_states'][name] = state
            expected = (3,) if name == 'amcl' else (2, 3)
            if state not in expected:
                raise RuntimeError(name+' is not configured; absence is not admission refusal')
        for waypoint in ('pump', 'cooler', 'power_unit'):
            for op in ('request', 'arm'):
                checkpoint()
                nav = samples['nav'][0]
                seq = nav['command_sequence']+1
                value = {'run_id': nav['run_id'], 'sequence': seq, 'op': op}
                if op == 'request':
                    value['waypoint'] = waypoint
                else:
                    value['target'] = {'run_id': nav['run_id'], 'epoch': 0, 'sequence': 1, 'waypoint': waypoint}
                command.publish(String(data=json.dumps(value)))
                wait(lambda: (samples['nav'][0].get('last_reply') or {}).get('sequence') == seq)
                reply = samples['nav'][0]['last_reply']
                if reply != {'sequence': seq, 'accepted': False, 'op': op}:
                    raise RuntimeError('Command was accepted or failed schema validation: '+repr(reply))
                report['replies'].append({'waypoint': waypoint, **reply})
        until = time.monotonic()+3.
        wait(lambda: time.monotonic() >= until, 4.)
        checkpoint()
        report.update(passed=True, supervisor_run_id=next(iter(runs)), counts=counts,
                      no_map_tf=True, guard_latched=True, supervisor_velocity_requests=counts.get('request', 0))
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        # Do not let a failed assertion or a surprising acceptance leave motion enabled.
        if stop.wait_for_service(timeout_sec=3.):
            future = stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            report['stop_ack'] = future.done() and future.result() is not None and future.result().success
        else:
            report['stop_ack'] = False
        report['passed'] = report['passed'] and report['stop_ack'] and not violations
        report['violations'] = sorted(violations)
        listener.unregister()
        node.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
