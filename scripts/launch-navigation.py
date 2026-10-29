#!/usr/bin/env python3
"""Start only Orbital navigation; never start the demo or localization."""
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
    dependencies = ['orbital-demo.service', 'orbital-localization.service']
    for unit in dependencies:
        dependency_identity(unit)
    state = subprocess.check_output(['systemctl', '--user', 'show', 'orbital-navigation.service',
                                     '--property=LoadState', '--value'], text=True).strip()
    if state != 'not-found':
        raise RuntimeError('Previous navigation service must finish cleanup before launch')
    (ROOT/'runtime').mkdir(exist_ok=True)
    subprocess.run(transient_command(ROOT, 'orbital-navigation.service', dependencies,
                                     ['/bin/bash', str(ROOT/'scripts/start-navigation.sh')]), check=True)


if __name__ == '__main__':
    main()
