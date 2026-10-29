#!/usr/bin/env python3
"""Submit, query or cancel an exact inspection; transport ACK is not success."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.inspection_client import run_ros
from orbital.inspection_protocol import identifier
from orbital.machines import MACHINE_IDS
from orbital.navigation_client import command_timeout


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('op', choices=('status', 'inspect', 'cancel', 'query'))
    parser.add_argument('machine_id', nargs='?', choices=MACHINE_IDS)
    parser.add_argument('--run-id', type=identifier)
    parser.add_argument('--mission-id', type=identifier)
    parser.add_argument('--action-id', type=identifier)
    parser.add_argument('--timeout', type=command_timeout, default=10.)
    parser.add_argument('--no-wait', action='store_true')
    args = parser.parse_args(argv)
    if args.op == 'inspect' and args.machine_id is None:
        parser.error('inspect requires a machine ID')
    if args.op != 'inspect' and args.machine_id is not None:
        parser.error('only inspect accepts a machine ID')
    if args.op in ('cancel', 'query') and not all((args.run_id, args.mission_id, args.action_id)):
        parser.error('cancel/query require exact --run-id, --mission-id and --action-id')
    if os.environ.get('ROS_DOMAIN_ID') not in ('42', '84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('requires Orbital localhost domain 42 or an empty isolated domain 84')
    try:
        value = run_ros(args.op, machine_id=args.machine_id, timeout_s=args.timeout,
                        run_id=args.run_id, mission_id=args.mission_id, action_id=args.action_id, wait=not args.no_wait)
        code = 0 if args.op == 'status' or value.get('transport') in ('terminal', 'acknowledged') else 2
        if value.get('result') is not None and value['result']['status'] != 'succeeded': code = 2
    except KeyboardInterrupt:
        value, code = {'transport': 'not_sent', 'error_code': 'interrupted'}, 130
    except (ValueError, RuntimeError, OSError, ImportError):
        value, code = {'transport': 'not_sent', 'error_code': 'inspection_server_unavailable'}, 2
    print(json.dumps(value, indent=2, allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
