#!/usr/bin/env python3
"""Run real DDS publisher-attribution controls on an initially empty domain 84.

No simulator, actuator bridge or command publisher is created. The wrong-owner
case preserves the correct graph inventory and TF parents, but sends map->odom
from the bridge endpoint. Endpoint-only validation cannot distinguish it.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
EDGES = {
    'odom': ('map', 'amcl', False),
    'base_footprint': ('odom', 'ros_gz_bridge', False),
    'base_link': ('base_footprint', 'robot_state_publisher', True),
    'lidar_link': ('base_link', 'robot_state_publisher', True),
    'camera_link': ('base_link', 'robot_state_publisher', True),
    'camera_optical_frame': ('camera_link', 'robot_state_publisher', True),
    'left_wheel': ('base_link', 'robot_state_publisher', False),
    'right_wheel': ('base_link', 'robot_state_publisher', False),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_case(case, executable, output):
    nodes = {name: Node(name) for name in ('amcl', 'ros_gz_bridge', 'robot_state_publisher')}
    executor = SingleThreadedExecutor()
    for node in nodes.values():
        executor.add_node(node)
    publishers = {name: node.create_publisher(TFMessage, '/tf', 20) for name, node in nodes.items()}
    duplicate = None
    if case == 'duplicate_owner':
        nodes['duplicate'] = Node('amcl')
        executor.add_node(nodes['duplicate'])
        duplicate = nodes['duplicate'].create_publisher(TFMessage, '/tf', 20)
    fixed = nodes['robot_state_publisher'].create_publisher(
        TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def message(static, owner):
        transforms = []
        for child, (parent, source, is_static) in EDGES.items():
            if case == 'wrong_owner' and child == 'odom':
                source = 'ros_gz_bridge'
            if is_static != static or source != owner or (case == 'missing_edge' and child == 'lidar_link'):
                continue
            t = TransformStamped()
            t.header.stamp = nodes[owner].get_clock().now().to_msg()
            if case == 'frozen_stamp' and child == 'odom':
                t.header.stamp.sec, t.header.stamp.nanosec = 1, 0
            t.header.frame_id = 'wrong_parent' if case == 'wrong_parent' and child == 'odom' else parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            if case == 'invalid_geometry' and child == 'odom':
                t.transform.translation.x = float('nan')
            transforms.append(t)
        return TFMessage(transforms=transforms)

    # Publish once BEFORE the auditor starts; a volatile reader misses these.
    fixed.publish(message(True, 'robot_state_publisher'))
    def publish_dynamic():
        for owner, publisher in publishers.items():
            publisher.publish(message(False, owner))
        if duplicate is not None:
            duplicate.publish(message(False, 'amcl'))
    timer = nodes['amcl'].create_timer(0.05, publish_dynamic)
    process = None
    try:
        with (output/f'{case}.json').open('w') as stdout, (output/f'{case}.stderr').open('w') as stderr:
            process = subprocess.Popen([str(executable), '--seconds', '4'], stdout=stdout, stderr=stderr)
            deadline = time.monotonic()+12
            while process.poll() is None and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            if process.poll() is None:
                raise TimeoutError('Auditor did not finish: '+case)
        report = json.loads((output/f'{case}.json').read_text())
        expected_error = {
            'wrong_owner': 'Wrong message publisher: odom from ros_gz_bridge',
            'wrong_parent': 'Unexpected edge parent/topic: wrong_parent -> odom',
            'invalid_geometry': 'Invalid transform geometry: odom',
            'missing_edge': 'Missing or multiple message owners: lidar_link',
            'duplicate_owner': 'Missing or multiple message owners: odom',
            'frozen_stamp': 'Nonadvancing dynamic edge: odom',
        }.get(case)
        if expected_error is None:
            assert process.returncode == 0 and report['passed'], report
            assert len(report['observations']) == 8, report
            assert report['per_message_tf_authority_proven'], report
        else:
            assert process.returncode == 1 and not report['passed'], report
            assert expected_error in report['errors'], report
        return {'case': case, 'passed': True, 'exit_code': process.returncode,
                'errors': report['errors'], 'report_sha256': sha(output/f'{case}.json')}
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        nodes['amcl'].destroy_timer(timer)
        for node in nodes.values():
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '84' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('Requires isolated localhost domain84')
    executable = ROOT/'runtime/tf-authority-build/orbital_tf_authority'
    paths = [ROOT/'native/tf_authority/audit.cpp', ROOT/'native/tf_authority/CMakeLists.txt',
             ROOT/'scripts/build-tf-authority.sh', Path(__file__).resolve(), executable]
    before = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    preflight = Node('orbital_tf_authority_preflight')
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            rclpy.spin_once(preflight, timeout_sec=0.05)
        foreign = [(name, ns) for name, ns in preflight.get_node_names_and_namespaces()
                   if name != preflight.get_name() or ns != '/']
        assert not foreign, ('Domain84 occupied', foreign)
    finally:
        preflight.destroy_node()
    try:
        cases = [run_case(case, executable, args.output) for case in
                 ('good', 'wrong_owner', 'wrong_parent', 'invalid_geometry', 'missing_edge',
                  'duplicate_owner', 'frozen_stamp')]
        after = {str(p.relative_to(ROOT)): sha(p) for p in paths}
        assert before == after, 'Source or executable changed during proof'
        result = {'passed': True, 'cases': cases, 'sha256': before,
                  'scope': 'real DDS synthetic TF; no simulator, motion or authentication proof'}
        (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
