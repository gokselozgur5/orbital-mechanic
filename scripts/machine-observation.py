#!/usr/bin/env python3
"""Read one same-time machine/robot inspection record; no camera or maintenance action."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.machines import MACHINE_IDS
from orbital.navigation_client import command_timeout
from orbital.ros_proximity import read_machine_observation


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('machine_id',choices=MACHINE_IDS)
    parser.add_argument('--timeout',type=command_timeout,default=10.)
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') not in ('42','84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        parser.error('requires Orbital localhost domain42 or an empty isolated domain84')
    try:
        result,code = read_machine_observation(ROOT,args.machine_id,timeout=args.timeout),0
    except KeyboardInterrupt:
        result,code = {'valid':False,'error':'interrupted'},130
    except (ValueError,RuntimeError,OSError,KeyError,TypeError,AttributeError,ImportError) as error:
        result,code = {'valid':False,'error':str(error)},2
    print(json.dumps(result,indent=2,allow_nan=False))
    return code


if __name__=='__main__':
    raise SystemExit(main())
