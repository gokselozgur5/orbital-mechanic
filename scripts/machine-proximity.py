#!/usr/bin/env python3
"""Measure one machine's planar proximity from trusted ROS state, without actions."""
import argparse
import json
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.machines import MACHINE_IDS
from orbital.ros_proximity import run


def duration(value):
    result = float(value)
    if not math.isfinite(result) or not .1 <= result <= 10.:
        raise argparse.ArgumentTypeError('timeout must be between 0.1 and 10 seconds')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('machine_id',choices=MACHINE_IDS)
    parser.add_argument('--timeout',type=duration,default=10.)
    parser.add_argument('--require-inspection-pose',action='store_true',
                        help='require measured inspection position and heading for exit 0; no action permission')
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') not in ('42','84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('requires Orbital localhost domain42 or an empty isolated domain84')
    try:
        result = run(ROOT,args.machine_id,timeout=args.timeout)
        accepted = result['within_inspection_pose'] if args.require_inspection_pose else result['within_distance']
        code = 0 if accepted else 1
    except KeyboardInterrupt:
        result,code = {'valid':False,'within_distance':False,'error':'interrupted'},130
    except (ValueError,RuntimeError,OSError,KeyError,TypeError,AttributeError,ImportError) as error:
        result,code = {'valid':False,'within_distance':False,'error':str(error)},2
    print(json.dumps(result,indent=2,allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
