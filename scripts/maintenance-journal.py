#!/usr/bin/env python3
"""Prepare one explicit new-world generation or inspect retained maintenance history."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.inspection_protocol import identifier
from orbital.maintenance_journal import JournalUnavailable, prepare_configured_scenario, read_history
from orbital.scenario_spec import PROFILES
from orbital.scenario_input import check_prepared


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='operation', required=True)
    prepare = commands.add_parser('prepare', help='Reserve a generation for a new full-world start, never station-only recovery.')
    prepare.add_argument('--new-world', action='store_true', required=True)
    prepare.add_argument('--profile', choices=PROFILES, default='recoverable', help='Private operator input; not a ROS parameter.')
    prepare.add_argument('--seed', type=int, default=0)
    check = commands.add_parser('check-prepared', help='Check an unclaimed input for a new full-world launch.')
    check.add_argument('--trial-id', required=True)
    commands.add_parser('status', help='Read retained records; this does not establish live process or machine state.')
    query = commands.add_parser('query', help='Read one historical action, without renewing permission or freshness.')
    query.add_argument('--action-id', required=True)
    args = parser.parse_args(argv)
    try:
        if args.operation == 'prepare': value = prepare_configured_scenario(ROOT, args.profile, args.seed)
        elif args.operation == 'check-prepared': value = check_prepared(ROOT, args.trial_id)
        else:
            history = read_history(ROOT)
            if args.operation == 'status': value = history.status()
            else:
                action = history.actions.get(identifier(args.action_id))
                if action is None: raise ValueError('unknown_historical_action')
                value = {'schema_version':1,'historical_only':True,'record':action}
        print(json.dumps(value, sort_keys=True, allow_nan=False))
        return 0
    except (ValueError, OSError, JournalUnavailable, KeyError, TypeError, UnicodeError) as error:
        print(json.dumps({'error':'maintenance_history_unavailable','detail':str(error)}), file=sys.stderr)
        return 2


if __name__ == '__main__': raise SystemExit(main())
