#!/usr/bin/env python3
"""Contain inspection children and stop when a prerequisite service changes."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.service_owner import run_owned


if __name__ == '__main__':
    run_owned([[sys.executable, str(ROOT/'scripts/inspection-server.py')]],
              ['orbital-demo.service', 'orbital-localization.service', 'orbital-navigation.service'])
