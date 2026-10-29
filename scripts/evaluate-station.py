#!/usr/bin/env python3
"""Score a stopped station's retained actual model observations; never act on it."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.maintenance_journal import JournalUnavailable
from orbital.station_evaluation import evaluate


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-id', required=True)
    args = parser.parse_args(argv)
    try:
        report = evaluate(ROOT, args.trial_id)
        print(json.dumps(report, sort_keys=True, allow_nan=False))
        return 0  # Valid evidence/report, not an accepted mission or successful repair.
    except (ValueError, OSError, RuntimeError, JournalUnavailable, KeyError, TypeError, UnicodeError, RecursionError):
        print(json.dumps({'error':'station_evaluation_unavailable'}), file=sys.stderr)
        return 2


if __name__ == '__main__': raise SystemExit(main())
