#!/usr/bin/env python3
"""Run the bounded public-evidence mission through the existing ROS owners."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.inspection_protocol import finite,identifier


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=['run'])
    parser.add_argument('--timeout',type=float,default=600.)
    parser.add_argument('--mission-id')
    args=parser.parse_args(argv)
    try:
        finite(args.timeout,1.,600.)
        if args.mission_id is not None:identifier(args.mission_id)
    except ValueError as error:parser.error(str(error))
    return args


def main(argv=None):
    args=parse_args(argv)
    try:
        domain=os.environ.get('ROS_DOMAIN_ID')
        if domain not in ('42','84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
            raise ValueError('requires_localhost_Orbital_domain_42_or_isolated_fixture_84')
        if domain=='42' and (not os.environ.get('TMUX') or str(ROOT)!='/home/jensen/.goske/codex/orbital-mechanic'):
            raise ValueError('run_in_Thor_orbital_work_tmux_from_Orbital_workspace')
        from orbital.ros_mission import run
        report=run(ROOT,mission_id=args.mission_id,timeout_s=args.timeout)
    except (ValueError,RuntimeError,OSError,KeyboardInterrupt) as error:
        report={'schema_version':1,'complete':False,'error':str(error),'error_type':type(error).__name__}
    print(json.dumps(report,indent=2,allow_nan=False))
    return 0 if report['complete'] and report.get('mission',{}).get('outcome') in ('healthy','recovered','escalated') else 1


if __name__=='__main__':raise SystemExit(main())
