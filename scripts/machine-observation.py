#!/usr/bin/env python3
"""Read one machine/robot record, optionally retaining local timestamped camera evidence."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.machines import MACHINE_IDS
from orbital.navigation_client import command_timeout
from orbital.ros_proximity import read_machine_observation,capture_machine_observation


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('machine_id',choices=MACHINE_IDS)
    parser.add_argument('--timeout',type=command_timeout,default=10.)
    parser.add_argument('--with-camera',action='store_true',help='require a camera pair and save a bounded local capture bundle')
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') not in ('42','84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        parser.error('requires Orbital localhost domain42 or an empty isolated domain84')
    try:
        reader = capture_machine_observation if args.with_camera else read_machine_observation
        result,code = reader(ROOT,args.machine_id,timeout=args.timeout),0
    except KeyboardInterrupt:
        result,code = {'valid':False,'error':'interrupted'},130
    except (ValueError,RuntimeError,OSError,KeyError,TypeError,AttributeError,ImportError) as error:
        result,code = {'valid':False,'error':str(error)},2
    print(json.dumps(result,indent=2,allow_nan=False))
    return code


if __name__=='__main__':
    raise SystemExit(main())
