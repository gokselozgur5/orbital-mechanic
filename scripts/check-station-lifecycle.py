#!/usr/bin/env python3
"""Native station startup/claim/clock proof on empty localhost domain 84.

Real telemetry and observer children use a synthetic clock. Each explicit
preparation represents a new isolated model fixture, not a running Gazebo world.
No navigation, inspection or maintenance command is published.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '84'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('Requires empty localhost domain 84')
    import rclpy
    from rclpy.node import Node
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from orbital.machine_registry import MachineRegistry
    from orbital.maintenance_journal import read_history

    output = ROOT/'runtime/station-lifecycle-proof'
    output.mkdir(parents=True)  # Preserve earlier trials; require a fresh checkout.
    assert not (ROOT/'runtime/station-service/events.jsonl').exists()
    rclpy.init(args=[])
    fixture = Node('ros_gz_bridge')  # Expected clock owner, synthetic in this domain only.
    deadline = time.monotonic()+2.
    while time.monotonic() < deadline:
        rclpy.spin_once(fixture, timeout_sec=.05)
    others = [name for name in fixture.get_node_names() if name != fixture.get_name()]
    if others:
        fixture.destroy_node(); rclpy.shutdown()
        raise RuntimeError('Domain 84 occupied: '+repr(others))
    publisher = fixture.create_publisher(Clock, '/clock', 10)
    started = time.monotonic(); offset = [1000.]
    packets, children, checks, calls = [], [], [], []
    stream = (output/'telemetry.jsonl').open('w')

    def receive(message):
        assert len(packets) < 128 and len(message.data) < 65536, 'Unexpected telemetry volume'
        packets.append(json.loads(message.data)); stream.write(message.data+'\n'); stream.flush()

    fixture.create_subscription(String, '/station/telemetry', receive, 10)

    def tick():
        ns = round((offset[0]+time.monotonic()-started)*1e9)
        publisher.publish(Clock(clock=Time(sec=ns//10**9, nanosec=ns%10**9)))

    fixture.create_timer(.02, tick)

    def wait(predicate, seconds=6.):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(fixture, timeout_sec=.005)
            if predicate():
                return
        raise AssertionError('Timed out waiting for native station lifecycle')

    def pump(seconds):
        end = time.monotonic()+seconds
        wait(lambda:time.monotonic() >= end, seconds+1.)

    def spawn(label, script, *arguments):
        with (output/(label+'.stdout')).open('w') as out, (output/(label+'.stderr')).open('w') as err:
            child = subprocess.Popen([sys.executable,'-B',str(ROOT/'scripts'/script),*arguments],
                                     cwd=ROOT, stdout=out, stderr=err)
        children.append(child)
        return child

    def finish(child, label, expected, *, error=None, seconds=8.):
        wait(lambda:child.poll() is not None, seconds)
        calls.append({'label':label,'pid':child.pid,'exit':child.returncode})
        assert child.returncode == expected, (label, child.returncode, (output/(label+'.stderr')).read_text())
        if error is not None:
            assert error in (output/(label+'.stderr')).read_text(), label
        return (output/(label+'.stdout')).read_text()

    def prepare(label):
        child = spawn(label, 'maintenance-journal.py', 'prepare', '--new-world')
        result = json.loads(finish(child, label, 0))
        assert result['new_world_required'] is True
        return result

    def start_station(label):
        return spawn(label, 'machine-telemetry.py', '--ros-args', '-p', 'use_sim_time:=true')

    def observe(label, expected=0, timeout=5.):
        child = spawn(label, 'observe-station.py', '--timeout', str(timeout))
        return json.loads(finish(child, label, expected, seconds=timeout+4.))

    def public(value):
        assert value['valid'] is True and value['freshness']['consecutive_samples'] >= 2
        packet = value['telemetry']
        assert packet['schema_version'] == 2 and packet['control'] == {'revision':0,'last_applied':None}
        assert packet['registry']['identity'] == MachineRegistry.load(ROOT).identity
        def keys(item):
            if isinstance(item, dict):
                for key, nested in item.items():
                    yield key
                    yield from keys(nested)
            elif isinstance(item, list):
                for nested in item:
                    yield from keys(nested)
        assert not {'fault','fault_label','scenario','reset_count'} & set(keys(packet))
        return packet['run_id']

    result = {'passed':False,'checks':checks,'calls':calls,
              'scope':'native station/observer lifecycle with synthetic clock; no Gazebo or maintenance acceptance'}
    try:
        absent = start_station('unprepared')
        finish(absent, 'unprepared', 1, error='events.jsonl')
        pump(.2); assert not packets
        checks.append('unprepared_generation_refuses_publication')

        first = prepare('prepare-first')
        station = start_station('station-first')
        observed = observe('observe-first'); first_run = public(observed)
        checks.append('prepared_station_publishes_fresh_public_schema')
        station.terminate(); finish(station, 'station-first', 0)
        pump(.3); count = len(packets)
        history = ROOT/'runtime/station-service/events.jsonl'
        before = history.read_bytes()
        restart = start_station('restart-claimed')
        finish(restart, 'restart-claimed', 1, error='scenario_already_claimed')
        pump(.2); assert len(packets) == count and history.read_bytes() == before
        checks.append('clean_station_restart_cannot_reclaim_generation')

        second = prepare('prepare-second')
        assert first['scenario_id'] != second['scenario_id']
        station = start_station('station-second')
        observed = observe('observe-second'); second_run = public(observed)
        assert second_run != first_run
        status = read_history(ROOT).status()
        assert status['scenario_count'] == 2 and status['action_count'] == 0
        assert status['restart_requires_new_scenario'] is True
        checks.append('explicit_new_fixture_retains_history_and_uses_new_identity')

        offset[0] -= 5.
        finish(station, 'station-second', 1, error='station_reset_requires_new_scenario', seconds=4.)
        checks.append('backward_clock_terminates_durable_station')
        pump(.3)
        unavailable = observe('observe-stopped', expected=2, timeout=1.5)
        assert unavailable['valid'] is False
        checks.append('stopped_source_cannot_supply_fresh_observation')
        assert read_history(ROOT).status()['restart_requires_new_scenario'] is True
        result.update(passed=True, packets=len(packets), first_run=first_run, second_run=second_run,
                      retained_history=read_history(ROOT).status())
    except Exception as error:
        result.update(error=str(error), packets=len(packets))
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
                try: child.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    child.kill(); child.wait(timeout=3.)
        stream.close(); fixture.destroy_node(); rclpy.try_shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
