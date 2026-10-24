#!/usr/bin/env python3
"""Destructive fault injection limited to the orbital simulated demo cgroup.

Run from Thor's orbital-work tmux session. Restarts the demo on completion.
Never use for hardware. Evidence is emitted as JSON, including failures.
"""
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import rclpy

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('motion_probe', ROOT/'scripts/check-motion.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def control(request):
    result = subprocess.run([
        'gz', 'service', '-s', '/world/maintenance/control',
        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000', '--req', request,
    ], capture_output=True, text=True, timeout=6, check=True)
    assert 'data: true' in result.stdout, result.stdout + result.stderr


def service_state():
    return subprocess.run(['systemctl','--user','is-active','orbital-demo.service'],
                          capture_output=True,text=True).stdout.strip()


def cgroup_members():
    result = subprocess.run(['systemctl','--user','show','orbital-demo.service',
                             '--property=ControlGroup','--value'], capture_output=True,text=True)
    group = result.stdout.strip()
    assert group.startswith('/user.slice/'), 'unexpected demo cgroup'
    base = Path('/sys/fs/cgroup')/group.lstrip('/')
    members = set()
    for p in base.rglob('cgroup.procs'):
        members.update(int(pid) for pid in p.read_text().split())
    return members


def cmdline(pid):
    try: return (Path('/proc')/str(pid)/'cmdline').read_bytes().replace(b'\0', b' ').decode()
    except FileNotFoundError: return ''


def wait_stopped(p, start, origin):
    end = start + 1.2
    while time.monotonic() < end:
        p.spin(.02)
        if p.state['latched'] and p.stationary():
            displacement = math.dist(origin, p.position())
            elapsed = time.monotonic()-start
            assert displacement < .12, ('stopping displacement too large', displacement)
            p.spin(.25)
            assert p.stationary() and p.state['latched']
            return {'stop_distance_m':displacement,'observed_stop_s':elapsed}
    raise AssertionError('odometry did not stop within 1.2 wall seconds')


def run(p, report):
    deadline=time.monotonic()+10
    while time.monotonic()<deadline and not (p.odom and p.state): p.spin(.1)
    assert p.odom and p.state, 'missing odometry or motion state after 10 seconds'
    assert p.state['latched'], 'startup is not latched'
    assert math.hypot(*p.position()) < .25, 'robot outside clear startup area'
    for speed in (.15, -.15):
        p.call(p.reset_client)
        p.drive(speed, 0, .8)
        assert abs(p.odom.twist.twist.linear.x) > .1, 'no measured motion'
        origin = p.position(); start = time.monotonic()
        result = wait_stopped(p, start, origin)
        assert p.state['reason'] == 'command_timeout'
        report.append({'case':'timeout_forward' if speed>0 else 'timeout_reverse', **result})

    p.call(p.reset_client); p.drive(.12,0,.6)
    control('pause: true')
    p.spin(.8)
    assert p.state['latched'], 'steady watchdog did not latch during pause'
    assert p.state['linear_mps']==0 and p.state['angular_rps']==0
    assert p.state['reason'] in ('feedback_timeout','command_timeout')
    origin=p.position()
    control('pause: false'); p.spin(.8)
    assert p.stationary() and p.state['latched'], 'resume restarted rover'
    displacement=math.dist(origin,p.position())
    assert displacement<.03, ('resume travel',displacement)
    report.append({'case':'pause_resume','resume_displacement_m':displacement})

    p.call(p.reset_client); p.drive(.1,0,.5)
    old_stamp=p.odom.header.stamp.sec+p.odom.header.stamp.nanosec/1e9
    control('reset: {all: true}'); p.spin(1.0)
    new_stamp=p.odom.header.stamp.sec+p.odom.header.stamp.nanosec/1e9
    assert new_stamp<old_stamp, 'world clock did not reset'
    assert p.state['latched'] and p.stationary(), 'world reset restarted rover'
    assert p.state['reason']=='simulation_reset', p.state
    report.append({'case':'world_reset','old_sim_s':old_stamp,'new_sim_s':new_stamp})

    p.call(p.reset_client); p.drive(.1,0,.6)
    assert p.odom.twist.twist.linear.x>.05
    members=cgroup_members()
    guards=[pid for pid in members if str(ROOT/'scripts/motion-guard.py') in cmdline(pid)]
    assert len(guards)==1, ('unexpected guards in demo cgroup',guards)
    start=time.monotonic(); os.kill(guards[0],signal.SIGKILL)
    deadline=start+5
    while time.monotonic()<deadline:
        alive=[pid for pid in members if cmdline(pid)]
        if not alive and service_state() not in ('active','activating','deactivating'): break
        p.spin(.1)
    else: raise AssertionError('demo descendants survived guard death: '+str({pid:cmdline(pid) for pid in alive})+' service='+service_state())
    report.append({'case':'guard_sigkill','cgroup_processes_before':len(members),
                   'teardown_wall_s':time.monotonic()-start,'remaining_processes':0})


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='42'
    assert os.environ.get('GZ_PARTITION')=='orbital-maintenance'
    rclpy.init(); p=module.Probe()
    report={'passed':False,'checks':[], 'source_sha256':{
        name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in (
            'orbital/motion.py','scripts/motion-guard.py','scripts/start-demo.sh',
            'scripts/launch-thor.sh','scripts/check-motion.py','scripts/check-motion-failures.py')},
        'limits':{'stop_distance_m':.12,'odometry_stop_s':1.2,'cgroup_teardown_s':5}}

    try:
        run(p,report['checks'])
    except Exception as error:
        report['error']=repr(error)
        report['traceback']=traceback.format_exc()
    finally:
        # Full cgroup restart is necessary after world reset or killed guard.
        p.destroy_node()
        subprocess.run(['bash',str(ROOT/'scripts/stop-thor.sh')],capture_output=True)
        subprocess.run(['bash',str(ROOT/'scripts/launch-thor.sh')],check=True)
        time.sleep(12)
        p=module.Probe();p.spin(2)
        report['clean_restart_latched']=bool(p.state and p.state['latched'] and p.odom and p.stationary())
        report['passed']='error' not in report and report['clean_restart_latched']
        p.destroy_node();rclpy.shutdown()
        print(json.dumps(report,indent=2),flush=True)
    if not report['passed']: raise SystemExit(1)


if __name__=='__main__': main()
