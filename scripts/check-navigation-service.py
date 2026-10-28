#!/usr/bin/env python3
"""Exercise the real service ownership contract with disposable fixture units.

No ROS nodes or existing Orbital units are changed. Only uniquely named proof
units are created/stopped; all artifacts stay beneath this checkout's runtime.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.service_owner import dependency_identity, run_owned, transient_command


def identity(pid):
    try:
        fields = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()
        return {'pid':pid, 'start':fields[19], 'state':fields[0]}
    except FileNotFoundError:
        return None


def alive(saved):
    current = identity(saved['pid'])
    return current is not None and current['start']==saved['start'] and current['state']!='Z'


def wait(predicate, timeout=6):
    started = time.monotonic()
    while time.monotonic()-started<timeout:
        if predicate():
            return time.monotonic()-started
        time.sleep(.05)
    raise RuntimeError('Service acceptance timed out')


def ctl(*args, check=True):
    return subprocess.run(['systemctl','--user',*args], capture_output=True, text=True, timeout=8, check=check)


def active(unit):
    return ctl('is-active','--quiet',unit,check=False).returncode==0


def fixture(args):
    if args.owner:
        run_owned([[sys.executable, __file__, '--worker', str(args.directory)], ['/bin/sleep','120']], args.owner)
        return
    if args.worker:
        directory = args.worker
        subprocess.Popen([sys.executable, __file__, '--detached', str(directory)], start_new_session=True)
        name = 'worker.json'
    else:
        directory = args.detached
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        name = 'detached.json'
    temporary = directory/(name+'.tmp')
    temporary.write_text(json.dumps(identity(os.getpid())))
    temporary.replace(directory/name)
    while True:
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--owner', nargs=2)
    parser.add_argument('--directory', type=Path)
    parser.add_argument('--worker', type=Path)
    parser.add_argument('--detached', type=Path)
    args = parser.parse_args()
    if args.owner or args.worker or args.detached:
        fixture(args)
        return 0
    if not os.environ.get('TMUX') or not str(ROOT).startswith('/home/jensen/.goske/codex/orbital-mechanic/'):
        raise RuntimeError('Run from an isolated proof copy inside Thor tmux orbital-work')
    prefix = 'orbital-nav-proof-'+uuid.uuid4().hex[:10]
    directory = ROOT/'runtime'/prefix
    directory.mkdir(parents=True)
    created, processes = [], []
    report = {'passed':False, 'checks':[], 'scope':'real systemd contract with fixture processes; no ROS/world',
              'sources':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest()
                         for n in ('orbital/service_owner.py','scripts/check-navigation-service.py')}}

    def start_dependencies(index):
        units = [f'{prefix}-{index}-{role}.service' for role in ('demo','localization')]
        for unit in units:
            created.append(unit)
            subprocess.run(['systemd-run','--user','--collect','--unit='+unit,'/bin/sleep','120'],
                           check=True, capture_output=True, text=True, timeout=5)
            dependency_identity(unit)
        return units

    def start_owner(index, deps, require_start=True):
        unit = f'{prefix}-{index}-navigation.service'
        created.append(unit)
        case_dir = directory/str(index)
        case_dir.mkdir()
        command = [sys.executable,__file__,'--owner',*deps,'--directory',str(case_dir)]
        subprocess.run(transient_command(ROOT,unit,deps,command), check=require_start,
                       capture_output=True, text=True, timeout=6)
        if not require_start:
            return unit,case_dir,[]
        wait(lambda:(case_dir/'worker.json').exists() and (case_dir/'detached.json').exists())
        group = ctl('show',unit,'--property=ControlGroup','--value').stdout.strip()
        cgroup = (Path('/sys/fs/cgroup')/group.lstrip('/')).resolve()
        assert cgroup.is_relative_to('/sys/fs/cgroup'), 'Invalid cgroup path'
        identities = [identity(int(pid)) for pid in (cgroup/'cgroup.procs').read_text().split()]
        assert len(identities)>=4 and all(identities), 'Incomplete owner/worker/detached/sibling fixture'
        processes.extend(identities)
        assert all(alive(p) for p in identities), 'Fixture failed to start'
        return unit,case_dir,identities

    def stop_units(units):
        for unit in reversed(units):
            ctl('stop',unit,check=False)
            ctl('reset-failed',unit,check=False)

    try:
        missing = [prefix+'-missing-demo.service',prefix+'-missing-localization.service']
        unit,path,_ = start_owner('missing',missing,False)
        wait(lambda:not active(unit))
        assert not (path/'worker.json').exists() and not any(active(d) for d in missing)
        report['checks'].append({'case':'missing_dependencies_not_started','passed':True})

        for index, case in enumerate(('demo_stop','localization_failure','dependency_restart','child_failure','explicit_stop')):
            deps = start_dependencies(index)
            pinned = {d:dependency_identity(d) for d in deps}
            unit,case_dir,identities = start_owner(index,deps)
            # Both properties come from the same builder as production.
            properties = ctl('show',unit,'--property=KillMode','--property=Restart','--property=After').stdout
            assert 'KillMode=control-group' in properties and 'Restart=no' in properties
            assert all(d in properties for d in deps)
            if case=='demo_stop':
                ctl('stop',deps[0])
            elif case=='localization_failure':
                ctl('kill','--kill-whom=main','--signal=KILL',deps[1])
            elif case=='dependency_restart':
                ctl('restart',deps[1])
                assert dependency_identity(deps[1])!=pinned[deps[1]]
            elif case=='child_failure':
                worker = json.loads((case_dir/'worker.json').read_text())
                assert alive(worker)
                os.kill(worker['pid'],signal.SIGKILL)
            else:
                ctl('stop',unit)
            delay = wait(lambda:not active(unit) and not any(alive(p) for p in identities))
            if case in ('child_failure','explicit_stop'):
                assert all(dependency_identity(d)==pinned[d] for d in deps), 'Owner changed a dependency'
            if case=='dependency_restart':
                time.sleep(.5)
                assert not active(unit), 'Navigation automatically restarted'
            report['checks'].append({'case':case,'passed':True,'post_command_cleanup_s':delay,
                                     'detached_sigterm_ignoring_child_gone':True})
            stop_units([*deps,unit])
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        stop_units(created)
        survivors = [p for p in processes if alive(p)]
        if survivors:
            report.update(passed=False, surviving_fixture_processes=survivors)
            for saved in survivors:
                if alive(saved):
                    os.kill(saved['pid'],signal.SIGKILL)
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
