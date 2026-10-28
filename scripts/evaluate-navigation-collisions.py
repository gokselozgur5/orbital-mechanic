#!/usr/bin/env python3
"""Evaluate captured robot/world geometry without ROS, simulator access or controls."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.collision_evaluation import evaluate_trace


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path, help='Completed runtime/evaluation/<run> directory')
    args = parser.parse_args(argv)
    try:
        result = evaluate_trace(args.trace)
        code = 0 if result['sampled_clear'] else 1
    except (ValueError, TypeError, KeyError, AttributeError, OSError, OverflowError, RecursionError) as error:
        result, code = {'evaluation_complete': False, 'sampled_clear': False,
                        'error': type(error).__name__+': '+str(error)}, 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
