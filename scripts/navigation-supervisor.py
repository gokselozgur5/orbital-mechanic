#!/usr/bin/env python3
"""Own guarded navigation requests, refusing competing command publishers."""
import argparse
import fcntl
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.navigation_configuration import bounded_timeout, load_waypoint_contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-config', type=Path, default=ROOT/'config/known-map.json')
    parser.add_argument('--goal-timeout', type=bounded_timeout, default=120.)
    args = parser.parse_args()
    waypoints, config_sha256 = load_waypoint_contract(args.map_config, root=ROOT, require_geometry=True)
    domain = os.environ.get('ROS_DOMAIN_ID')
    if domain not in ('42', '84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires localhost-only Orbital domain 42 or isolated test domain 84')
    (ROOT/'runtime').mkdir(exist_ok=True)
    with (ROOT/'runtime'/f'navigation-supervisor-{domain}.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Navigation supervisor is already running in this checkout/domain') from None
        return run(waypoints, args.goal_timeout, config_sha256)


def run(waypoints, timeout, config_sha256):
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from orbital.ros_navigation_supervisor import NavigationSupervisor
    rclpy.init(args=[])
    probe, node, executor = None, None, None
    try:
        probe = rclpy.create_node('orbital_navigation_owner_preflight')
        end = time.monotonic()+2
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=.05)
        for topic in ('/motion/request', '/navigation/state'):
            if probe.get_publishers_info_by_topic(topic):
                raise RuntimeError('Competing publisher on '+topic)
        raw = probe.get_publishers_info_by_topic('/navigation/raw_cmd_vel')
        if len(raw) > 1 or any(p.node_name != 'controller_server' or p.node_namespace != '/'
                              or p.topic_type != 'geometry_msgs/msg/TwistStamped' for p in raw):
            raise RuntimeError('Competing or incompatible controller publisher')
        probe.destroy_node()
        probe = None
        node = NavigationSupervisor(waypoints, goal_timeout=timeout, map_config_sha256=config_sha256)
        # All action responses, commands and timers share serialized ownership.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
