#!/usr/bin/env python3
"""Start only the inspection owner, with existing Orbital service generations pinned."""
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.service_owner import dependency_identity, transient_command


def main():
    if not os.environ.get('TMUX') or str(ROOT) != '/home/jensen/.goske/codex/orbital-mechanic':
        raise RuntimeError('Run inside Thor tmux orbital-work from the Orbital workspace')
    dependencies = ['orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service']
    for unit in dependencies: dependency_identity(unit)
    state = subprocess.check_output(['systemctl', '--user', 'show', 'orbital-inspection.service',
                                     '--property=LoadState', '--value'], text=True).strip()
    if state != 'not-found':
        raise RuntimeError('Previous inspection service must finish cleanup before launch')
    (ROOT/'runtime').mkdir(exist_ok=True)
    subprocess.run(transient_command(ROOT, 'orbital-inspection.service', dependencies,
        ['/bin/bash', str(ROOT/'scripts/start-inspection.sh')]), check=True)


if __name__ == '__main__': main()
