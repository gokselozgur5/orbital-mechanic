#!/usr/bin/env python3
"""Thor-only commissioning: stops/restarts the simulation and localization.

Checks cgroup ownership and explicit startup, then leaves a fresh latched world
with a newly seeded localization service. No navigation target is dispatched.
"""
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def run(args, *, check=True, timeout=30):
    result=subprocess.run(args,cwd=ROOT,capture_output=True,text=True,check=False,timeout=timeout)
    if check and result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}): {args}\n{result.stdout[-2000:]}\n{result.stderr[-2000:]}')
    return result


def active(unit):
    return run(['systemctl','--user','is-active','--quiet',unit],check=False).returncode==0


def wait_collected(unit):
    end=time.monotonic()+5
    while time.monotonic()<end:
        state=run(['systemctl','--user','show',unit,'--property=LoadState','--value']).stdout.strip()
        if state=='not-found':return
        if not active(unit):run(['systemctl','--user','reset-failed',unit],check=False)
        time.sleep(.1)
    raise RuntimeError('Unit did not finish collection: '+unit)


def main():
    if ROOT!=Path('/home/jensen/.goske/codex/orbital-mechanic') or not os.environ.get('TMUX'):
        raise RuntimeError('Run only in Thor tmux orbital-work')
    if not active('orbital-demo.service') or not active('orbital-localization.service'):
        raise RuntimeError('Both demo and localization must be active before this test')
    stop=run(['/usr/bin/python3','scripts/orbital-operator.py','stop'])
    group=run(['systemctl','--user','show','orbital-localization.service','--property=ControlGroup','--value']).stdout.strip()
    if not group.endswith('/orbital-localization.service'):raise RuntimeError('Unexpected localization cgroup')
    cgroup=Path('/sys/fs/cgroup')/group.lstrip('/')
    pids=[int(x) for x in (cgroup/'cgroup.procs').read_text().split()]
    if not pids:raise RuntimeError('No localization processes sampled')
    checks={}
    try:
        started=time.monotonic()
        run(['bash','scripts/stop-thor.sh'])
        deadline=started+5
        while any(Path('/proc',str(pid)).exists() for pid in pids) and time.monotonic()<deadline:
            time.sleep(.05)
        remaining=[pid for pid in pids if Path('/proc',str(pid)).exists()]
        checks['demo_stop_cleans_localization']={'sampled_processes':len(pids),'remaining':remaining,'seconds':time.monotonic()-started}
        if remaining or active('orbital-localization.service'):raise RuntimeError('Localization survived demo stop')
        wait_collected('orbital-localization.service');wait_collected('orbital-demo.service')
        rejected=run(['bash','scripts/launch-localization.sh'],check=False)
        checks['no_implicit_world_start']=rejected.returncode!=0 and not active('orbital-demo.service')
        if not checks['no_implicit_world_start']:raise RuntimeError('Localization implicitly started a world')
        run(['bash','scripts/launch-thor.sh']);time.sleep(12)
        checks['demo_restart_does_not_start_localization']=not active('orbital-localization.service')
        if not checks['demo_restart_does_not_start_localization']:raise RuntimeError('Localization restarted implicitly')
        status=json.loads(run(['/usr/bin/python3','scripts/orbital-operator.py','status']).stdout)
        checks['fresh_demo_latched']=status['guard']['latched']
        if not checks['fresh_demo_latched']:raise RuntimeError('Fresh demo is not latched')
        run(['bash','scripts/launch-localization.sh']);time.sleep(15)
        if not active('orbital-localization.service'):raise RuntimeError('Localization did not start')
        unseeded=json.loads(run(['/usr/bin/python3','scripts/check-localization.py','--expect-unseeded'],timeout=20).stdout)
        checks['new_localizer_requires_seed']=unseeded['passed']
        if not checks['new_localizer_requires_seed']:raise RuntimeError('Old localization pose survived restart')
        seed=json.loads(run(['/usr/bin/python3','scripts/seed-localization.py','--fresh-world-origin'],timeout=25).stdout)
        probe=json.loads(run(['/usr/bin/python3','scripts/check-localization.py','--compare-gazebo'],check=False,timeout=25).stdout)
        print(json.dumps({'passed':probe['passed'],'checks':checks,'new_seed':seed,'new_localization':probe},indent=2))
        return 0 if probe['passed'] else 1
    finally:
        # Restore a fresh safe viewer if a stop/start assertion interrupted this run.
        if not active('orbital-demo.service'):
            try:
                wait_collected('orbital-demo.service')
                run(['bash','scripts/launch-thor.sh'])
            except Exception as exc:
                print('Could not restore demo: '+str(exc),file=__import__('sys').stderr)


if __name__=='__main__':raise SystemExit(main())
