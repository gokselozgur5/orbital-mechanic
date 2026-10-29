#!/usr/bin/env python3
"""Run health observer, guarded supervisor and inactive Nav2 as one owned unit."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.service_owner import run_owned


def main():
    if str(ROOT) != '/home/jensen/.goske/codex/orbital-mechanic' or not os.environ.get('INVOCATION_ID'):
        raise RuntimeError('Navigation must run inside its Thor systemd service')
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Navigation requires the Orbital localhost ROS environment')
    run_owned([
        ['/usr/bin/python3', str(ROOT/'scripts/localization-monitor.py')],
        ['/usr/bin/python3', str(ROOT/'scripts/navigation-supervisor.py')],
        ['/opt/ros/jazzy/bin/ros2', 'launch', str(ROOT/'scripts/navigation.launch.py')],
    ], ['orbital-demo.service', 'orbital-localization.service'])


if __name__ == '__main__':
    main()
