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
    parser.add_argument('operation',choices=['run','reset-tool-stop'])
    parser.add_argument('--timeout',type=float,default=600.)
    parser.add_argument('--mission-id')
    parser.add_argument('--tool-contract',action='store_true',help='Execute the scripted policy through the durable planner tools')
    parser.add_argument('--provider-replay',type=Path,help='Replay one operator-selected synthetic Messages fixture; no API calls')
    parser.add_argument('--mission-text',help='Mission text for the recorded provider (at most 1024 characters)')
    parser.add_argument('--provider-max-turns',type=int)
    parser.add_argument('--provider-max-output-tokens',type=int)
    parser.add_argument('--provider-max-reported-tokens',type=int)
    parser.add_argument('--provider-max-rejections',type=int)
    parser.add_argument('--acknowledge-stop',action='store_true',help='Explicit operator acknowledgement for reset-tool-stop only')
    args=parser.parse_args(argv)
    try:
        finite(args.timeout,1.,600.)
        if args.mission_id is not None:identifier(args.mission_id)
        if args.operation=='reset-tool-stop' and (not args.acknowledge_stop or args.mission_id or args.tool_contract):
            raise ValueError('reset_requires_explicit_operator_acknowledgement_without_run_options')
        if args.operation=='run' and args.acknowledge_stop:raise ValueError('acknowledgement_requires_reset_operation')
        from orbital.planner_messages import bounded_integer
        args.provider_options={}
        for name,minimum,maximum in [('max_turns',1,32),('max_output_tokens',16,4096),
                ('max_reported_tokens',1,1000000),('max_rejections',0,4)]:
            value=getattr(args,'provider_'+name)
            if value is not None:args.provider_options[name]=bounded_integer(value,minimum,maximum)
        if args.mission_text is not None:
            if not 1<=len(args.mission_text)<=1024 or not args.mission_text.strip():raise ValueError('invalid_mission_text')
            args.provider_options['mission_text']=args.mission_text
        if args.provider_replay is None and args.provider_options:raise ValueError('provider_options_require_recording')
        if args.provider_replay is not None and (args.operation!='run' or args.tool_contract):
            raise ValueError('recording_requires_run_without_tool_contract')
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
        if args.operation=='reset-tool-stop':
            from orbital.inspection_worker import OwnerLock
            from orbital.mission_journal import MissionJournal
            lock=OwnerLock(ROOT,service='mission-service')
            journal=None
            try:
                journal=MissionJournal(lock)
                report={'schema_version':1,'complete':True,'operator_reset':journal.reset_planner_stop()}
            finally:
                if journal is not None:journal.close()
                lock.close()
        else:
            provider=None
            if args.provider_replay is not None:
                from orbital.planner_recording import load
                provider=load(args.provider_replay)
            from orbital.ros_mission import run
            try:
                report=run(ROOT,mission_id=args.mission_id,timeout_s=args.timeout,tool_contract=args.tool_contract,
                    provider=provider,provider_options=args.provider_options)
            finally:
                if provider is not None:provider.close()
    except (ValueError,RuntimeError,OSError,KeyboardInterrupt) as error:
        report={'schema_version':1,'complete':False,'error':str(error),'error_type':type(error).__name__}
    print(json.dumps(report,indent=2,allow_nan=False))
    return 0 if report['complete'] and (report.get('operator_reset',{}).get('cleared') is True or
        report.get('mission',{}).get('outcome') in ('healthy','recovered','escalated')) else 1


if __name__=='__main__':raise SystemExit(main())
