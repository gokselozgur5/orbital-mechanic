#!/usr/bin/env python3
"""Observe localization and publish /navigation/health; never command the robot."""
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.executors import ExternalShutdownException
from orbital.ros_localization import LocalizationMonitor


def main():
    rclpy.init()
    probe=rclpy.create_node('orbital_monitor_preflight')
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(probe,timeout_sec=.05)
    existing=probe.get_publishers_info_by_topic('/navigation/health')
    probe.destroy_node()
    if existing:
        rclpy.shutdown()
        raise RuntimeError('A navigation health publisher already exists; refusing a duplicate observer')
    node=LocalizationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
