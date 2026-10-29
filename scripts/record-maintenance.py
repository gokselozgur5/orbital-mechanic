#!/usr/bin/env python3
"""Record one maintenance mission through a named terminal action; replay its public sequence."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.inspection_protocol import finite, identifier
from orbital.maintenance_trace import MaintenanceTrace, target, verify_maintenance
from orbital.machines import MACHINE_IDS
from orbital.trajectory import TraceWriter


def seconds(value):
    return finite(float(value), .1, 120.)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument('--name', help='New directory under runtime/evaluation; wait for ready.json before sending')
    operation.add_argument('--verify', type=Path, help='Offline check with the same configured machine registry')
    parser.add_argument('--machine-id', choices=MACHINE_IDS)
    for name in ('run-id', 'mission-id', 'action-id'):
        parser.add_argument('--'+name, type=identifier)
    parser.add_argument('--duration', type=seconds, default=60., help='Observation deadline after readiness (seconds)')
    parser.add_argument('--warmup', type=seconds, default=10.)
    parser.add_argument('--require-recovery', action='store_true', help='Require the full public recovery window after observed restoration')
    args = parser.parse_args(argv)
    identity = {name:getattr(args, name) for name in ('run_id', 'mission_id', 'action_id', 'machine_id')}
    if args.verify and (any(identity.values()) or args.require_recovery):
        parser.error('verification reads the stored identity; do not provide target arguments')
    try:
        if args.verify:
            value, code = verify_maintenance(ROOT, args.verify), 0
        else:
            identity = target({**identity, 'require_recovery':args.require_recovery})
            finite(args.warmup, .1, 30.)
            if (os.environ.get('ROS_DOMAIN_ID') not in ('42', '84')
                    or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
                    or os.environ.get('GZ_PARTITION') != 'orbital-maintenance'):
                raise ValueError('Requires Orbital localhost domain 42 or isolated 84 and orbital-maintenance partition')
            from orbital.ros_isolation_trace import run
            writer = TraceWriter(ROOT, args.name)
            value, code = run(ROOT, writer, identity, duration=args.duration, warmup=args.warmup,
                              trace_factory=MaintenanceTrace, node_prefix='orbital_maintenance_observer')
            value = {'output': str(writer.path), **value}
    except (ValueError, OSError) as error:
        value, code = {'complete': False, 'error': str(error)}, 2
    print(json.dumps(value, indent=2, allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
