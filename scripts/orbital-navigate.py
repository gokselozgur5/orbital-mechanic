#!/usr/bin/env python3
"""Observe navigation or send one bounded request, explicit arm or exact cancel."""
import argparse
from contextlib import nullcontext
import fcntl
import json
import os
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.machines import MACHINE_IDS
from orbital.navigation_client import NavigationClient, command_timeout, execute, read_target


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=command_timeout, default=5., help='Total ROS discovery/ack deadline, 0.1–10 seconds')
    commands = parser.add_subparsers(dest='operation', required=True)
    commands.add_parser('status', help='Read fresh supervisor state; sends no command')
    request = commands.add_parser('request', help='Reserve a named target without arming')
    request.add_argument('waypoint', choices=MACHINE_IDS)
    for name in ('arm', 'cancel'):
        command = commands.add_parser(name, help='Send '+name+' for an explicit exact target')
        target = command.add_mutually_exclusive_group(required=True)
        target.add_argument('--target', help='Exact target token as JSON')
        target.add_argument('--target-file', type=Path, help='Saved successful request receipt or exact target JSON')
    args = parser.parse_args(argv)
    args.target_value = None
    if args.operation in ('arm', 'cancel'):
        try:
            if args.target_file:
                with args.target_file.open() as stream:
                    raw = stream.read(16385)
            else:
                raw = args.target
            args.target_value = read_target(raw)
        except (ValueError, OSError, OverflowError, RecursionError) as error:
            parser.error(str(error))
    return args


def ros_command(args, client):
    import rclpy
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('orbital_navigate_'+uuid.uuid4().hex[:16])
        node.create_subscription(String, '/navigation/state', lambda message: client.observe(message.data), 1)
        publisher = node.create_publisher(String, '/navigation/command', 1) if args.operation != 'status' else None

        class Channel:
            def poll(self, duration):
                rclpy.spin_once(node, timeout_sec=duration)

            def authority_clear(self):
                def owner(items, expected, role):
                    if not items:
                        return False
                    if len(items) != 1:
                        raise ValueError('Ambiguous '+role)
                    info = items[0]
                    if info.node_name in ('', '_NODE_NAME_UNKNOWN_') or info.node_namespace == '_NODE_NAMESPACE_UNKNOWN_':
                        return False
                    if (info.node_name != expected or info.node_namespace != '/'
                            or info.topic_type != 'std_msgs/msg/String'):
                        raise ValueError('Unexpected '+role)
                    return True
                ready = owner(node.get_publishers_info_by_topic('/navigation/state'),
                              'orbital_navigation_supervisor', 'state publisher')
                if publisher is not None:
                    ready = owner(node.get_publishers_info_by_topic('/navigation/command'),
                                  node.get_name(), 'command publisher') and ready
                    ready = owner(node.get_subscriptions_info_by_topic('/navigation/command'),
                                  'orbital_navigation_supervisor', 'command subscriber') and ready
                    ready = publisher.get_subscription_count() == 1 and ready
                return ready

            def publish(self, text):
                publisher.publish(String(data=text))

        return execute(client, Channel(), args.operation, timeout=args.timeout,
                       waypoint=getattr(args, 'waypoint', None), target=args.target_value)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    args = parse_args(argv)
    client = NavigationClient(time.monotonic)
    try:
        domain = os.environ.get('ROS_DOMAIN_ID')
        if domain not in ('42', '84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
            raise ValueError('Requires localhost-only Orbital domain 42 or isolated fixture domain 84')
        if args.operation == 'request':
            from orbital.navigation_configuration import load_waypoint_contract
            load_waypoint_contract(ROOT/'config/known-map.json', root=ROOT, require_geometry=True)
        if args.operation != 'status':
            (ROOT/'runtime').mkdir(exist_ok=True)
            context = (ROOT/'runtime'/f'navigation-client-{domain}.lock').open('a')
        else:
            context = nullcontext(None)
        with context as lock:
            if lock is not None:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise ValueError('Another navigation command client owns this checkout/domain') from None
            result, code = ros_command(args, client)
    except KeyboardInterrupt:
        result, code = client.receipt(args.operation, error='Interrupted; inspect state before another command'), 130
    except Exception as error:
        result, code = client.receipt(args.operation, error=str(error)), 3 if client.published else 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
