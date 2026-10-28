#!/usr/bin/env python3
"""Print the public configured scene registry without starting ROS or controls."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.machine_registry import MachineRegistry

if __name__ == '__main__':
    print(json.dumps(MachineRegistry.load(ROOT).public(), indent=2, allow_nan=False))
