#!/usr/bin/env python3
"""Record independent simulator truth, exact-time localization and command identities."""
import argparse
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.trajectory import TraceWriter, verify_trace


def bounded_seconds(value):
    seconds = float(value)
    if not math.isfinite(seconds) or not .1 <= seconds <= 300.:
        raise ValueError('Duration must be between 0.1 and 300 seconds')
    return seconds


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument('--name', help='New run name under runtime/evaluation')
    operation.add_argument('--verify', type=Path, help='Check a completed run without ROS/Gazebo')
    parser.add_argument('--duration', type=bounded_seconds, default=10.)
    parser.add_argument('--warmup', type=bounded_seconds, default=10.)
    args = parser.parse_args(argv)
    try:
        if args.verify:
            result, code = verify_trace(args.verify), 0
        else:
            if (os.environ.get('ROS_DOMAIN_ID') != '42'
                    or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
                    or os.environ.get('GZ_PARTITION') != 'orbital-maintenance'):
                raise ValueError('Requires localhost Orbital domain 42 and orbital-maintenance Gazebo partition')
            if args.warmup > 30.:
                raise ValueError('Warmup must not exceed 30 seconds')
            from orbital.ros_trajectory import run
            writer = TraceWriter(ROOT, args.name)
            result, code = run(ROOT, writer, duration=args.duration, warmup=args.warmup)
            result = {'output': str(writer.path), **result}
    except (ValueError, OSError) as error:
        result, code = {'complete': False, 'error': str(error)}, 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
