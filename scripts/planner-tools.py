#!/usr/bin/env python3
"""Print the provider-neutral planner catalog; no ROS/network/filesystem writes."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from orbital.planner_schema import catalog

if __name__ == '__main__':
    print(json.dumps(catalog(), indent=2, allow_nan=False))
