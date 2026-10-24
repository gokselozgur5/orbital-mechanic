#!/usr/bin/env python3
"""Expose the simplified machine model as read-only ROS telemetry."""
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from orbital.machines import Fault, Station


class Telemetry(Node):
    def __init__(self):
        super().__init__('station_telemetry')
        self.declare_parameter('scenario', 'latched_controller')
        self.fault = Fault(self.get_parameter('scenario').value)
        self.station = Station(self.fault)
        self.previous = None
        self.telemetry = self.create_publisher(String, '/station/telemetry', 5)
        self.voltage = self.create_publisher(Float32, '/station/bus_voltage', 5)
        self.timer = self.create_timer(.5, self.update)

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.previous is not None:
            dt = now - self.previous
            if dt < 0 or dt > 10:
                self.station = Station(self.fault)
            else:
                while dt > 0:
                    step = min(dt, 1.0)
                    self.station.tick(step)
                    dt -= step
        self.previous = now
        self.telemetry.publish(String(data=json.dumps(self.station.observe())))
        self.voltage.publish(Float32(data=self.station.voltage_v))

rclpy.init()
node = Telemetry()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
