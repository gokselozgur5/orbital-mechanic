"""Own a transient service's children and pin its already-running dependencies.

No dependency is started or restarted here. The surrounding systemd cgroup is
the final containment boundary for descendants that detach from process groups.
"""
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def dependency_identity(unit):
    result = subprocess.run(['systemctl', '--user', 'show', unit,
                             '--property=ActiveState', '--property=SubState', '--property=InvocationID'],
                            text=True, capture_output=True, timeout=1, check=True)
    fields = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    identity = fields.get('InvocationID', '')
    if fields.get('ActiveState') != 'active' or fields.get('SubState') != 'running' or not re.fullmatch('[0-9a-f]{32}', identity):
        raise RuntimeError('Required service is not running: '+unit)
    return identity


def transient_command(root, unit, dependencies, command):
    """Shared production/proof unit contract, with caller-owned names and paths."""
    if not dependencies or any(not re.fullmatch(r'[A-Za-z0-9_-]+\.service', n) for n in [unit, *dependencies]):
        raise ValueError('Expected explicit service unit names')
    root = Path(root).resolve()
    return ['systemd-run', '--user', '--collect', '--unit='+unit,
            '--property=WorkingDirectory='+str(root),
            # Requisite propagates explicit restart jobs, even with Restart=no.
            # Pin running dependencies in run_owned instead of auto-pulling them.
            '--property=After='+' '.join(dependencies),
            '--property=Restart=no', '--property=KillMode=control-group',
            '--property=TimeoutStopSec=2',
            '--property=StandardOutput=append:'+str(root/'runtime'/f'{unit}.log'),
            '--property=StandardError=inherit', *command]


def run_owned(commands, dependencies):
    """Exit if any child ends or a prerequisite changes invocation identity."""
    pinned = {unit: dependency_identity(unit) for unit in dependencies}
    children = []
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for command in commands:
            if stopping:
                return
            children.append(subprocess.Popen(command, start_new_session=True))
        while not stopping:
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f'Owned child {child.pid} exited with {child.returncode}')
            for unit, identity in pinned.items():
                if dependency_identity(unit) != identity:
                    raise RuntimeError('Required service restarted: '+unit)
            time.sleep(.1)
    finally:
        for child in children:
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic()+.7
        while time.monotonic()<deadline and any(c.poll() is None for c in children):
            time.sleep(.02)
        for child in children:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        for child in children:
            child.wait(timeout=1)
        for sig, handler in previous.items():
            signal.signal(sig, handler)
