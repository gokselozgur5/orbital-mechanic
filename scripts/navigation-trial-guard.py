#!/usr/bin/env python3
"""Stop a pinned navigation owner on trial deadline or lost parent pipe.

Prints readiness after pinning. Only an exact safe newline on stdin disarms it;
the trial must first verify latched stationary idle state before sending that.
"""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.trial_guard import enforce, pin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unit', default='orbital-navigation.service')
    parser.add_argument('--invocation', required=True)
    parser.add_argument('--seconds', type=float, default=8.)
    args = parser.parse_args()
    if ROOT != Path('/home/jensen/.goske/codex/orbital-mechanic') or not os.environ.get('TMUX'):
        raise RuntimeError('Run only inside Thor tmux orbital-work')
    if not .1 <= args.seconds <= 15:
        parser.error('seconds must be between 0.1 and 15')
    fd, pid = pin(args.unit, args.invocation)
    try:
        print(json.dumps({'ready': True, 'pid': pid, 'invocation': args.invocation}), flush=True)
        print(json.dumps(enforce(fd, sys.stdin, args.seconds)), flush=True)
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
