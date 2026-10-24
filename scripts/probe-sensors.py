#!/usr/bin/env python3
"""Bounded, read-only ROS probe; reports measurements without image payloads."""
import json
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

rclpy.init()
node = rclpy.create_node('orbital_sensor_probe')
results = {}
subs = []
def callback(topic, msg):
    row = results.setdefault(topic, {'count': 0})
    row['count'] += 1
    row['_last_wall'] = time.monotonic()
    if hasattr(msg, 'header'):
        row['frame_id'] = msg.header.frame_id
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp < row.get('stamp_s', stamp): row['stamp_regressions'] = row.get('stamp_regressions', 0) + 1
        row['stamp_s'] = stamp
        row.setdefault('first_stamp_s', row['stamp_s'])
    if isinstance(msg, Image):
        row.update(width=msg.width, height=msg.height, bytes=len(msg.data), encoding=msg.encoding)
    if isinstance(msg, LaserScan):
        values = [r for r in msg.ranges if math.isfinite(r)]
        row.update(samples=len(msg.ranges), finite_samples=len(values), min_m=min(values) if values else None)
    if isinstance(msg, TFMessage):
        row['frames'] = sorted(set(row.get('frames', [])) | {t.child_frame_id for t in msg.transforms})
    if isinstance(msg, Clock):
        row['sim_s'] = msg.clock.sec + msg.clock.nanosec / 1e9
        row.setdefault('first_sim_s', row['sim_s'])

spec = {'/camera/image_raw': Image, '/camera/camera_info': CameraInfo, '/scan': LaserScan, '/imu': Imu, '/odom': Odometry, '/joint_states': JointState, '/clock': Clock, '/tf': TFMessage, '/tf_static': TFMessage}
for topic, kind in spec.items():
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE) if topic == '/tf_static' else qos_profile_sensor_data
    # Large raw images need reliable delivery on this local DDS transport.
    if topic == '/camera/image_raw': qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    subs.append(node.create_subscription(kind, topic, lambda msg, t=topic: callback(t,msg), qos))
start = time.monotonic()
while time.monotonic() - start < 20:
    rclpy.spin_once(node, timeout_sec=.2)
end = time.monotonic()
for row in results.values(): row['last_receive_age_s'] = end - row.pop('_last_wall')
missing = sorted(set(spec) - set(results))
image = results.get('/camera/image_raw', {})
scan = results.get('/scan', {})
valid = (not missing and all(row['count'] > 1 for topic, row in results.items() if topic != '/tf_static')
         and all(row.get('stamp_regressions', 0) == 0 for row in results.values())
         and all(row['last_receive_age_s'] < 1.0 for topic, row in results.items() if topic != '/tf_static')
         and 0 <= results.get('/clock', {}).get('sim_s', 0) - image.get('stamp_s', 0) < 1.0
         and image.get('bytes', 0) > 0 and image.get('width') == 640
         and image.get('height') == 480 and scan.get('finite_samples', 0) > 0
         and image.get('stamp_s', 0) > image.get('first_stamp_s', 0)
         and scan.get('stamp_s', 0) > scan.get('first_stamp_s', 0)
         and image.get('frame_id') == 'camera_optical_frame'
         and scan.get('frame_id') == 'lidar_link'
         and 'base_link' in results.get('/tf', {}).get('frames', [])
         and {'camera_optical_frame','lidar_link'}.issubset(results.get('/tf_static', {}).get('frames', [])))
print(json.dumps({'wall_seconds': time.monotonic()-start, 'topics': results, 'missing': missing, 'passed': valid}, indent=2))
node.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if valid else 1)
