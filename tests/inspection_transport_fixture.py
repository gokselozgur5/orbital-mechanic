"""Controlled ROS/clocks; production worker entry and actual storage run unchanged."""
import json
import math
import mmap
import os
from pathlib import Path
import runpy
import sys
import struct
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


class SharedClock:
    """Test-only clock: parent advances ticks; child freezes completion scheduling."""
    def __init__(self, root, create=False):
        file = root/'runtime/endpoint-test-clock'
        if create: file.write_bytes(struct.pack('dB', 1000., 0))
        self.file = file.open('r+b')
        self.memory = mmap.mmap(self.file.fileno(), 9)
    def now(self): return struct.unpack('d', self.memory[:8])[0]
    def advance(self, seconds, force=False):
        if force or not self.memory[8]: self.memory[:8] = struct.pack('d', self.now()+seconds)
    def freeze(self): self.memory[8] = 1
    def close(self): self.memory.close(); self.file.close()


def transport(root, origin, clock=None, writer=False):
    prepared = MapSnapshot.prepare(MachineRegistry.load(root))
    case = NS(root=root, directory=prepared.yaml_path.parent, payload=prepared.bundle()['grid'])
    harness = Harness(case)
    original_spin = harness.modules['rclpy'].spin_once
    now = time.monotonic if clock is None else clock.now
    harness.wall = now()
    harness.sim = 10**9 + round((harness.wall-origin)*1e9)
    # Give this positive-control provider a known post-settling sample pair.
    # Otherwise the very first selected sample can land exactly on the settling
    # boundary, with its nearest camera frame legitimately preceding settling.
    harness.next_telemetry = (harness.sim//500000000+3)*500000000
    harness.next_camera = ((harness.sim-50000000)//200000000+1)*200000000+50000000
    def spin(node, timeout_sec):
        if clock is None:
            time.sleep(timeout_sec)
        else:
            if writer: clock.advance(timeout_sec)
            time.sleep(.005)  # Yield CPU to the actual child, without advancing its clock.
        wall = now()
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
    create = harness.modules['rclpy'].create_node
    def node(*args, **kwargs):
        value = create(*args, **kwargs)
        subscribe = value.create_subscription
        def subscription(kind, topic, callback, qos):
            if topic == '/station/telemetry':
                # Bind provider startup to subscription, not map-preparation CPU time.
                stamp = 10**9+round((now()-origin)*1e9)
                harness.next_telemetry = (stamp//500000000+3)*500000000
            return subscribe(kind, topic, callback, qos)
        value.create_subscription = subscription
        return value
    harness.modules['rclpy'].create_node = node
    harness.modules['rclpy'].ok = lambda: True
    signals = ModuleType('rclpy.signals'); signals.SignalHandlerOptions = NS(NO='none')
    harness.modules['rclpy.signals'] = signals
    return harness


if __name__ == '__main__':
    # Test-selected executable installs transport stand-ins, then executes the
    # actual private worker including input, lifeline, context and result logic.
    clock = SharedClock(Path.cwd()) if os.environ.get('ORBITAL_TEST_SHARED_CLOCK') == '1' else None
    harness = transport(Path.cwd(), float(os.environ['ORBITAL_TEST_ORIGIN']), clock)
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
        try:
            if clock is not None: kwargs['now'] = clock.now
            result = actual(*args, **kwargs)
            if clock is not None: clock.freeze()
            return result
        except BaseException:
            (Path.cwd()/'runtime/test-worker-error.txt').write_text(traceback.format_exc()+'\n'+json.dumps(trace[-40:]))
            raise
    with patch.dict(sys.modules, harness.modules), patch.object(ros_proximity, 'capture_machine_observation', capture):
        try:
            runpy.run_path(str(ROOT/'scripts/inspection-worker.py'), run_name='__main__')
        except SystemExit:
            # Delay exit AFTER the actual worker has emitted its success reply.
            # Logical time stays frozen so this tests reap ownership alone.
            time.sleep(float(os.environ.get('ORBITAL_TEST_EXIT_DELAY', '0')))
            raise
