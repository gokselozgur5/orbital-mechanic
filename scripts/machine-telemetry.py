#!/usr/bin/env python3
"""Run the source-bound station telemetry and owned power-isolation endpoint."""
import signal
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.ros_machine_telemetry import run


if __name__ == '__main__':
    stopped = [False]
    def stop(*_): stopped[0] = True
    previous = {sig:signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        run(ROOT, stop_requested=lambda:stopped[0])
    finally:
        for sig, handler in previous.items(): signal.signal(sig, handler)
