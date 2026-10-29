#!/usr/bin/env python3
"""Expose the source-bound simplified machine model as read-only ROS telemetry."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.ros_machine_telemetry import run


if __name__ == '__main__':
    run(ROOT)
