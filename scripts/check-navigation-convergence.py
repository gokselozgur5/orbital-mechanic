#!/usr/bin/env python3
"""Reset the Orbital world, commission an offset seed, then check Nav2 readiness.

Runs only inside the owned Thor tmux workspace with explicit acknowledgment.
Known origin is established by restarting the world, never by feeding measured
Gazebo ground truth into localization. Actual motion uses the bounded operator.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--restart-world-and-jog', action='store_true', required=True)
    parser.parse_args()
    if ROOT != Path('/home/jensen/.goske/codex/orbital-mechanic') or not os.environ.get('TMUX'):
        raise RuntimeError('Run inside Thor tmux orbital-work')
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'orbital-maintenance':
        raise RuntimeError('Source scripts/ros-env.sh first')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires localhost discovery')
    result = ROOT/'runtime'/time.strftime('navigation-convergence-%Y%m%dT%H%M%SZ', time.gmtime())
    result.mkdir()
    print(result, flush=True)
    report = {'passed': False, 'steps': [], 'scope': 'offset-seed convergence and guarded Nav2 readiness, not route acceptance'}

    def run(args, name=None, timeout=35, check=True):
        process = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, timeout=timeout)
        if name:
            (result/(name+'.stdout')).write_text(process.stdout)
            (result/(name+'.stderr')).write_text(process.stderr)
            report['steps'].append({'name': name, 'returncode': process.returncode})
        if check and process.returncode:
            raise RuntimeError(f'{name or args} failed ({process.returncode}): {process.stderr[-500:]}')
        return process

    def python(script, name, *args, timeout=35):
        process = run([sys.executable, 'scripts/'+script, *args], name, timeout)
        return json.loads(process.stdout)

    def prop(unit, field):
        return run(['systemctl', '--user', 'show', unit, '-p', field, '--value']).stdout.strip()

    def stop(unit):
        if prop(unit, 'LoadState') != 'not-found':
            run(['systemctl', '--user', 'stop', unit])
        end = time.monotonic()+8
        while prop(unit, 'LoadState') != 'not-found':
            if time.monotonic() >= end:
                raise RuntimeError('Unit did not finish cleanup: '+unit)
            time.sleep(.1)

    def wait_service(path, unit):
        # Discovery is readiness, not a fixed startup delay. The actual probes
        # subsequently verify lifecycle state and independently fresh inputs.
        end = time.monotonic()+30
        while time.monotonic() < end:
            if prop(unit, 'ActiveState') not in ('active', 'activating'):
                raise RuntimeError(unit+' ended during startup')
            found = run(['ros2', 'service', 'list'], timeout=5).stdout.splitlines()
            if path in found:
                return
            time.sleep(.2)
        raise RuntimeError('Service discovery timed out: '+path)

    try:
        before = python('navigation-manifest.py', 'source-before')
        report['manifest_identity'] = before['identity']
        report['old_demo_invocation'] = prop('orbital-demo.service', 'InvocationID')
        stop('orbital-navigation.service')
        python('orbital-operator.py', 'initial-stop', 'stop')
        stop('orbital-localization.service')
        stop('orbital-demo.service')
        run(['bash', 'scripts/launch-thor.sh'], 'launch-demo')
        wait_service('/motion/stop', 'orbital-demo.service')
        python('orbital-operator.py', 'fresh-world-status', 'status')
        report['new_demo_invocation'] = prop('orbital-demo.service', 'InvocationID')
        if not report['new_demo_invocation'] or report['new_demo_invocation'] == report['old_demo_invocation']:
            raise RuntimeError('A fresh world invocation was not established')
        run(['bash', 'scripts/launch-localization.sh'], 'launch-localization')
        wait_service('/amcl/get_state', 'orbital-localization.service')
        python('check-localization.py', 'unseeded', '--expect-unseeded')
        seed = python('seed-localization.py', 'offset-seed', '--fresh-world-origin',
                      '--seed-offset-x', '.2', '--seed-offset-y', '-.15', '--seed-offset-yaw', '.15')
        if seed.get('seed_xy_yaw') != [.2, -.15, .15] or seed.get('seed_processed') is not True:
            raise RuntimeError('The requested offset seed was not processed')
        for direction, duration in (('forward', '1'), ('left', '.7')):
            python('orbital-operator.py', 'arm-'+direction, 'arm')
            jog = python('orbital-operator.py', direction, 'jog', direction, '--duration', duration)
            if not jog.get('command_acknowledged') or jog.get('guard', {}).get('latched') is not True:
                raise RuntimeError('Jog acknowledgment or final stop missing')
        convergence = python('check-localization.py', 'convergence', '--compare-gazebo')
        if convergence.get('passed') is not True:
            raise RuntimeError('Independent convergence checks failed')
        report['independent_error'] = convergence['independent_error']
        run([sys.executable, 'scripts/launch-navigation.py'], 'launch-navigation')
        wait_service('/controller_server/get_state', 'orbital-navigation.service')
        ready = python('check-navigation-ready.py', 'navigation-ready', '--activate', timeout=75)
        if (ready.get('passed') is not True or ready.get('localization_healthy') is not True
                or ready.get('supervisor_velocity_requests') != 0
                or set(ready.get('final_states', {}).values()) != {3}
                or ready.get('manifest_identity') != before['identity']):
            raise RuntimeError('Healthy latched navigation readiness not established')
        report['supervisor_run_id'] = ready['supervisor_run_id']
        # Check accuracy again after activation; startup must not move the robot.
        after = python('check-localization.py', 'post-navigation-convergence', '--compare-gazebo')
        report['post_navigation_error'] = after['independent_error']
        start_pose, end_pose = convergence['ground_truth_xy_yaw'], after['ground_truth_xy_yaw']
        report['navigation_start_displacement_m'] = math.dist(start_pose[:2], end_pose[:2])
        report['navigation_start_yaw_change_rad'] = abs(math.remainder(end_pose[2]-start_pose[2], 2*math.pi))
        if report['navigation_start_displacement_m'] > .001 or report['navigation_start_yaw_change_rad'] > .001:
            raise RuntimeError('Navigation startup changed the independently measured pose')
        final = python('navigation-manifest.py', 'source-after')
        if final['identity'] != before['identity']:
            raise RuntimeError('Source/artifact identity changed during commissioning')
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
        try:
            stop('orbital-navigation.service')
            python('orbital-operator.py', 'failure-stop', 'stop', timeout=12)
            report['failure_stop_ack'] = True
        except Exception as stop_error:
            report['failure_stop_error'] = str(stop_error)
            run(['systemctl', '--user', 'stop', 'orbital-demo.service'], 'failure-world-stop', check=False)
    finally:
        (result/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
