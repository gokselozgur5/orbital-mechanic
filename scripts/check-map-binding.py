#!/usr/bin/env python3
"""Read-only map revision gate. Fails before navigation startup/activation."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--localization-only', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') not in ('42','84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('Requires Orbital localhost domain42 or isolated domain84')
    import rclpy
    from orbital.ros_map_binding import collect
    rclpy.init(args=[])
    node = rclpy.create_node('orbital_map_binding_probe')
    try:
        report = collect(node, ROOT, configured=not args.localization_only)
    except Exception as error:
        report = {'passed':False, 'error':str(error)}
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
