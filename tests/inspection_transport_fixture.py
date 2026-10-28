"""Real-clock controlled ROS transport; production worker entry and storage run unchanged."""
import json
import math
import os
from pathlib import Path
import runpy
import sys
import time
import traceback
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'tests'))
from orbital.machine_registry import MachineRegistry
from orbital.map_snapshot import MapSnapshot
from proximity_fixture import Harness


def transport(root, origin):
    prepared = MapSnapshot.prepare(MachineRegistry.load(root))
    case = NS(root=root, directory=prepared.yaml_path.parent, payload=prepared.bundle()['grid'])
    harness = Harness(case)
    original_spin = harness.modules['rclpy'].spin_once
    harness.wall = time.monotonic()
    harness.sim = 10**9 + round((harness.wall-origin)*1e9)
    # Give this positive-control provider a known post-settling sample pair.
    # Otherwise the very first selected sample can land exactly on the settling
    # boundary, with its nearest camera frame legitimately preceding settling.
    harness.next_telemetry = (harness.sim//500000000+3)*500000000
    harness.next_camera = ((harness.sim-50000000)//200000000+1)*200000000+50000000
    def spin(node, timeout_sec):
        time.sleep(timeout_sec)
        wall = time.monotonic()
        # A 50 Hz simulated clock is independent of executor wakeup jitter.
        # Sensors publish only on a new clock tick, including 1 ms storage polls.
        stamp = 10**9 + (round((wall-origin)*1e9)//20000000)*20000000
        if stamp <= harness.sim:
            return
        harness.wall = wall-timeout_sec
        harness.sim = stamp-round(timeout_sec*1e9)
        previous_sample, previous_camera = harness.next_telemetry, harness.next_camera
        original_spin(node, timeout_sec)
        if harness.next_telemetry != previous_sample:
            harness.next_telemetry = (harness.sim//500000000+1)*500000000
        if harness.next_camera != previous_camera:
            harness.next_camera = ((harness.sim-50000000)//200000000+1)*200000000+50000000
    harness.modules['rclpy'].spin_once = spin
    harness.modules['rclpy'].ok = lambda: True
    signals = ModuleType('rclpy.signals'); signals.SignalHandlerOptions = NS(NO='none')
    harness.modules['rclpy.signals'] = signals
    return harness


if __name__ == '__main__':
    # Test-selected executable installs transport stand-ins, then executes the
    # actual private worker including input, lifeline, context and result logic.
    harness = transport(Path.cwd(), float(os.environ['ORBITAL_TEST_ORIGIN']))
    from orbital import ros_proximity
    trace = []
    def telemetry(packet):
        trace.append(['telemetry', harness.sim, packet['stamp_ns']])
        return packet
    def camera(kind, message):
        if kind == 'image': trace.append(['image', harness.sim, message.header.stamp.sec*10**9+message.header.stamp.nanosec])
    harness.telemetry_hook, harness.camera_hook = telemetry, camera
    actual = ros_proximity.capture_machine_observation
    def capture(*args, **kwargs):
        try: return actual(*args, **kwargs)
        except BaseException:
            (Path.cwd()/'runtime/test-worker-error.txt').write_text(traceback.format_exc()+'\n'+json.dumps(trace[-40:]))
            raise
    with patch.dict(sys.modules, harness.modules), patch.object(ros_proximity, 'capture_machine_observation', capture):
        runpy.run_path(str(ROOT/'scripts/inspection-worker.py'), run_name='__main__')
