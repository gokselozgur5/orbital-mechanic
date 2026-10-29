#!/usr/bin/env python3
"""Evaluate closed mission/station evidence and an optional covering collision trace."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.maintenance_journal import JournalUnavailable
from orbital.mission_evaluation import evaluate


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-id',required=True)
    parser.add_argument('--mission-id',required=True)
    parser.add_argument('--collision-trace',type=Path)
    args=parser.parse_args(argv)
    try:
        report=evaluate(ROOT,args.trial_id,args.mission_id,collision_trace=args.collision_trace)
        print(json.dumps(report,sort_keys=True,allow_nan=False))
        return 0  # Valid evidence, not overall mission acceptance.
    except (ValueError,OSError,RuntimeError,JournalUnavailable,KeyError,TypeError,UnicodeError,RecursionError):
        print(json.dumps({'error':'mission_evaluation_unavailable'}),file=sys.stderr)
        return 2


if __name__=='__main__':raise SystemExit(main())
