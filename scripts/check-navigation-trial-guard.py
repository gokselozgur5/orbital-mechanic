#!/usr/bin/env python3
"""Native pidfd guardian acceptance using only disposable owned user services."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def main():
    if ROOT != Path('/home/jensen/.goske/codex/orbital-mechanic') or not os.environ.get('TMUX'):
        raise RuntimeError('Run inside Thor tmux orbital-work')
    unit = 'orbital-trial-proof-'+uuid.uuid4().hex[:12]+'.service'
    reports = []
    helper = None

    def prop(name):
        return subprocess.check_output(['systemctl', '--user', 'show', unit, '-p', name, '--value'], text=True).strip()

    def wait(predicate, seconds=6.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.03)
        raise RuntimeError('Fixture state timeout')

    def stop():
        if prop('LoadState') != 'not-found':
            subprocess.run(['systemctl', '--user', 'stop', unit], check=True, timeout=5)
        wait(lambda: prop('LoadState') == 'not-found')

    def start():
        # Both owner and descendant ignore SIGTERM, exercising pidfd SIGKILL
        # escalation and the unit's control-group cleanup of surviving children.
        subprocess.run(['systemd-run', '--user', '--collect', '--unit='+unit,
                        '--property=KillMode=control-group', '--property=TimeoutStopSec=1',
                        '/bin/bash', '-c', "trap '' TERM; sleep 300 & wait"],
                       check=True, capture_output=True, timeout=5)
        wait(lambda: prop('ActiveState') == 'active')
        return prop('InvocationID')

    def guard(invocation, seconds):
        process = subprocess.Popen([sys.executable, str(ROOT/'scripts/navigation-trial-guard.py'),
            '--unit', unit, '--invocation', invocation, '--seconds', str(seconds)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        if not select.select([process.stdout], [], [], 3.)[0]:
            raise RuntimeError('Guard did not acknowledge pinning')
        ready = json.loads(process.stdout.readline())
        if ready.get('ready') is not True:
            raise RuntimeError('Guard not ready')
        return process

    def finish(process):
        process.wait(timeout=5)
        if process.returncode:
            raise RuntimeError(process.stderr.read().decode())
        return json.loads(process.stdout.read())

    try:
        identity = start()
        wrong = subprocess.run([sys.executable, str(ROOT/'scripts/navigation-trial-guard.py'),
            '--unit', unit, '--invocation', '0'*32], input=b'', capture_output=True, timeout=5)
        assert wrong.returncode != 0 and prop('InvocationID') == identity and prop('ActiveState') == 'active'
        reports.append({'case': 'wrong_generation_refused', 'passed': True})
        helper = guard(identity, 5.)
        helper.stdin.write(b'safe\n'); helper.stdin.flush(); helper.stdin.close()
        result = finish(helper); helper = None
        assert result['action'] == 'disarmed' and prop('ActiveState') == 'active'
        reports.append({'case': 'explicit_safe_disarms', **result}); stop()

        for case in ('pipe_close', 'deadline'):
            identity = start()
            group = Path('/sys/fs/cgroup')/prop('ControlGroup').lstrip('/')
            pids = [int(x) for x in (group/'cgroup.procs').read_text().split()]
            helper = guard(identity, .3 if case == 'deadline' else 5.)
            if case == 'pipe_close':
                helper.stdin.close()
            result = finish(helper); helper = None
            assert result['action'] == 'stopped_owner' and result['sigkill']
            wait(lambda: prop('LoadState') == 'not-found')
            assert all(not Path(f'/proc/{pid}').exists() or Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[0]=='Z' for pid in pids)
            reports.append({'case': case, 'tracked_processes': len(pids), **result})

        old = start(); helper = guard(old, 15.)
        stop(); new = start()
        helper.stdin.close(); result = finish(helper); helper = None
        assert new != old and result['action'] == 'owner_already_exited'
        assert prop('InvocationID') == new and prop('ActiveState') == 'active'
        reports.append({'case': 'replacement_owner_untouched', **result})
        print(json.dumps({'passed': True, 'unit': unit, 'cases': reports}, indent=2))
    finally:
        if helper is not None and helper.poll() is None:
            helper.stdin.close(); helper.wait(timeout=5)
        stop()


if __name__ == '__main__':
    main()
