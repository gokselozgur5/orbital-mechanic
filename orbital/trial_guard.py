"""Bound a commissioning trial to the exact navigation-owner process, not a name.

Linux pidfds prevent a later service invocation/PID reuse from being signalled.
The service's existing KillMode=control-group contains its descendants.
"""
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import time


def pin(unit, invocation):
    if unit != 'orbital-navigation.service' and not re.fullmatch(r'orbital-trial-proof-[a-z0-9-]+\.service', unit):
        raise ValueError('Only the navigation owner or an explicitly named trial fixture may be pinned')
    if not re.fullmatch('[0-9a-f]{32}', invocation):
        raise ValueError('Exact service InvocationID required')

    def properties():
        out = subprocess.check_output(['systemctl', '--user', 'show', unit, '-p', 'InvocationID',
            '-p', 'MainPID', '-p', 'ActiveState', '-p', 'KillMode'], text=True, timeout=2)
        return dict(line.split('=', 1) for line in out.splitlines() if '=' in line)

    before = properties()
    pid = int(before.get('MainPID', '0'))
    if (before.get('InvocationID') != invocation or before.get('ActiveState') != 'active'
            or before.get('KillMode') != 'control-group' or pid <= 1):
        raise RuntimeError('Service identity/state/containment does not match')
    fd = os.pidfd_open(pid)
    try:
        if properties() != before or not any(line.endswith('/'+unit) for line in Path(f'/proc/{pid}/cgroup').read_text().splitlines()):
            raise RuntimeError('Service changed while pinning its owner')
        return fd, pid
    except BaseException:
        os.close(fd)
        raise


def enforce(fd, stream, seconds):
    if not .1 <= seconds <= 15:
        raise ValueError('Trial deadline must be between 0.1 and 15 seconds')
    started = time.monotonic()
    reason = 'deadline'
    if select.select([stream], [], [], seconds)[0]:
        # Only the exact success acknowledgment disarms the guard. EOF after a
        # probe crash, partial input and any other text all stop the pinned owner.
        value = os.read(stream.fileno(), 16)
        if value == b'safe\n':
            return {'action': 'disarmed', 'seconds': time.monotonic()-started}
        reason = 'probe_lost'
    if select.select([fd], [], [], 0)[0]:
        return {'action': 'owner_already_exited', 'reason': reason}
    try:
        signal.pidfd_send_signal(fd, signal.SIGTERM)
        killed = not bool(select.select([fd], [], [], 1.)[0])
        if killed:
            signal.pidfd_send_signal(fd, signal.SIGKILL)
        return {'action': 'stopped_owner', 'reason': reason, 'sigkill': killed,
                'seconds': time.monotonic()-started}
    except ProcessLookupError:
        return {'action': 'owner_already_exited', 'reason': reason}
