#!/usr/bin/env python3
"""Own one read-only inspection at a time on Orbital's local ROS graph."""
import os
from pathlib import Path
import signal
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.ros_inspection import serve


def main():
    if os.environ.get('ROS_DOMAIN_ID') not in ('42', '84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires Orbital localhost domain 42 or an empty isolated domain 84')
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        return serve(ROOT, stop_requested=lambda: stopping)
    finally:
        for sig, handler in previous.items(): signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
