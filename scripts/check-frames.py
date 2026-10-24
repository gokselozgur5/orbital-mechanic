#!/usr/bin/env python3
"""Read-only live acceptance of the robot's flat-floor TF geometry.

Does not publish commands, reset the world or provide localization ground truth.
Publisher graph counts and observed parents detect common duplicate-tree errors;
this Jazzy build does not expose publisher GIDs in subscription callbacks, so
these checks do not prove per-message publisher ownership.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=8)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 3 <= args.seconds <= 30:
        parser.error('--seconds must be finite and between 3 and 30')
    model = ET.parse(ROOT/'worlds/maintenance.sdf').getroot().find(
        "world/model[@name='maintenance_robot']")
    body = model.find("link[@name='base_link']")
    body_xyz = [float(v) for v in body.findtext('pose').split()[:3]]
    expected = {'base_link': body_xyz}
    for sensor, frame in [('camera', 'camera_link'), ('lidar', 'lidar_link')]:
        offset = [float(v) for v in body.find(f"sensor[@name='{sensor}']").findtext('pose').split()[:3]]
        expected[frame] = [a+b for a, b in zip(body_xyz, offset)]
    for frame in ('left_wheel', 'right_wheel'):
        expected[frame] = [float(v) for v in model.find(f"link[@name='{frame}']").findtext('pose').split()[:3]]
    rclpy.init()
    node = rclpy.create_node('orbital_frame_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    parents = {}
    feedback = {}
    regressions = 0

    def on_tf(msg):
        for t in msg.transforms:
            parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id)

    def on_odom(msg):
        nonlocal regressions
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if stamp < feedback.get('stamp_s', stamp):
            regressions += 1
        feedback.update(parent=msg.header.frame_id, child=msg.child_frame_id,
                        stamp_s=stamp, received_at=time.monotonic(),
                        z_m=msg.pose.pose.position.z)

    static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [node.create_subscription(TFMessage, '/tf', on_tf, qos_profile_sensor_data),
                     node.create_subscription(TFMessage, '/tf_static', on_tf, static_qos),
                     node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)]
    try:
        end = time.monotonic()+args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        now = node.get_clock().now().nanoseconds/1e9
        rows = {}
        checks = {}
        for frame, target in expected.items():
            try:
                transform = buffer.lookup_transform('base_footprint', frame, Time())
                p = transform.transform.translation
                actual = [p.x, p.y, p.z]
                error = math.dist(actual, target)
                rows[frame] = {'actual_xyz_m': actual, 'expected_xyz_m': target, 'error_m': error}
                checks['position_'+frame] = math.isfinite(error) and error < 1e-5
            except TransformException as exc:
                rows[frame] = {'error': str(exc)}
                checks['position_'+frame] = False
        try:
            t = buffer.lookup_transform('odom', 'base_footprint', Time())
            stamp = t.header.stamp.sec+t.header.stamp.nanosec/1e9
            checks['dynamic_tf_fresh'] = now > 0 and 0 <= now-stamp < .5
            checks['planar_tf'] = abs(t.transform.translation.z) < 1e-6
        except TransformException as exc:
            rows['odom_to_base_footprint'] = {'error': str(exc)}
            checks['dynamic_tf_fresh'] = checks['planar_tf'] = False
        expected_parents = {'base_footprint': 'odom', 'base_link': 'base_footprint',
                            'camera_link': 'base_link', 'camera_optical_frame': 'camera_link',
                            'lidar_link': 'base_link', 'left_wheel': 'base_link', 'right_wheel': 'base_link'}
        checks['observed_parent_tree'] = all(parents.get(c) == {p} for c, p in expected_parents.items())
        checks['odometry_frames'] = feedback.get('parent') == 'odom' and feedback.get('child') == 'base_footprint'
        checks['odometry_fresh'] = (bool(feedback) and now > 0 and
                                  time.monotonic()-feedback['received_at'] < .5 and
                                  0 <= now-feedback['stamp_s'] < .5)
        checks['odometry_no_regressions'] = regressions == 0 and bool(feedback)
        graph = {topic: sorted(p.node_name for p in node.get_publishers_info_by_topic(topic))
                 for topic in ('/tf', '/tf_static', '/odom')}
        checks['expected_publisher_graph'] = (graph['/tf'] == ['robot_state_publisher', 'ros_gz_bridge'] and
                                            graph['/tf_static'] == ['robot_state_publisher'] and
                                            graph['/odom'] == ['ros_gz_bridge'])
        # Metadata only: no image payloads or private inputs.
        feedback.pop('received_at', None)
        passed = all(checks.values())
        print(json.dumps({'passed': passed, 'checks': checks, 'positions': rows,
                          'observed_parents': {k: sorted(v) for k,v in parents.items()},
                          'publisher_nodes': graph, 'odometry': feedback,
                          'source_sha256': {f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest()
                                            for f in ('config/robot.urdf', 'worlds/maintenance.sdf')},
                          'scope': 'flat-floor TF geometry/freshness; not localization or per-message publisher authentication'}, indent=2))
        return 0 if passed else 1
    finally:
        listener.unregister()
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
