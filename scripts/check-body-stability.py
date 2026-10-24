#!/usr/bin/env python3
"""Simulator regression: planar wheel odometry cannot prove body stability."""
import importlib.util
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('motion_probe', ROOT/'scripts/check-motion.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def attitude(q):
    values = (q.x, q.y, q.z, q.w)
    assert all(math.isfinite(v) for v in values), 'nonfinite IMU orientation'
    assert abs(sum(v*v for v in values)-1) < .01, 'invalid IMU quaternion'
    return (math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))))


class StabilityProbe(module.Probe):
    def __init__(self):
        super().__init__()
        self.imu = None
        self.imu_received = 0.
        self.stamps = []
        self.max_tilt = [0., 0.]
        self.imu_sub = self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        self.imu = msg
        self.imu_received = time.monotonic()
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        for i, value in enumerate(attitude(msg.orientation)):
            self.max_tilt[i] = max(self.max_tilt[i], abs(value))

    def check(self):
        assert self.imu is not None and time.monotonic()-self.imu_received < .25, 'stale IMU'
        assert self.imu.header.frame_id == 'base_link', 'unexpected IMU frame'
        assert max(self.max_tilt) < .05, ('body tilt violates flat-floor model', self.max_tilt)
        assert self.stamps[-1] >= self.stamps[0], 'simulation clock regressed'


def main():
    rclpy.init()
    p = StabilityProbe()
    results = []
    try:
        p.spin(2)
        p.call(p.stop_client)
        p.check()  # Reject an already tipped robot without commanding more motion.
        assert p.odom and math.hypot(*p.position()) < .25, 'requires clear startup area'
        assert p.pub.get_subscription_count() > 0, 'missing motion request subscriber'
        for name, linear, angular in [('forward', .12, 0), ('reverse', -.12, 0),
                                      ('left', 0, .3), ('right', 0, -.3),
                                      ('max_forward', .2, 0), ('max_reverse', -.2, 0),
                                      ('max_left', 0, .5), ('max_right', 0, -.5)]:
            p.call(p.reset_client)
            before = p.position()
            q = p.odom.pose.pose.orientation
            before_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            end = time.monotonic()+1.5
            while time.monotonic() < end:
                p.check()
                p.send(linear, angular)
                p.spin(.04)
            p.call(p.stop_client)
            p.spin(.8)
            p.check()
            assert p.state['latched'] and p.stationary(), 'not safely stopped'
            if linear:
                assert abs(linear)*.8 < math.dist(before, p.position()) < abs(linear)*2, 'motion not exercised'
            else:
                q = p.odom.pose.pose.orientation
                after_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                delta = math.remainder(after_yaw-before_yaw, 2*math.pi)
                assert abs(angular)*.8 < abs(delta) < abs(angular)*2, 'rotation not exercised'
                assert (delta > 0) == (angular > 0), 'wrong rotation direction'
            results.append({'case': name, 'max_abs_roll_pitch_rad': list(p.max_tilt)})
        assert len(p.stamps) > 100 and p.stamps[-1]-p.stamps[0] > 5, 'insufficient advancing IMU'
        print(json.dumps({'passed': True, 'samples': len(p.stamps), 'checks': results}, indent=2))
    except Exception as exc:
        print(json.dumps({'passed': False, 'error': str(exc), 'samples': len(p.stamps),
                          'max_abs_roll_pitch_rad': p.max_tilt}, indent=2))
        raise
    finally:
        try:
            p.call(p.stop_client)
        finally:
            p.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
