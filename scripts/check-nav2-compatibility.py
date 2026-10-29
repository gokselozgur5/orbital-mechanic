#!/usr/bin/env python3
"""Inspect installed artifacts before a Nav2 configuration trial; never start ROS.

Passing is not proof that plugins load or that any route is safe. The subsequent
inactive lifecycle/graph trial must verify the actual stamped publisher type.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import xml.etree.ElementTree as ET

EXECUTABLES = {
    'nav2_controller': 'controller_server', 'nav2_planner': 'planner_server',
    'nav2_bt_navigator': 'bt_navigator', 'nav2_lifecycle_manager': 'lifecycle_manager',
}
PLUGINS = {
    'nav2_regulated_pure_pursuit_controller': ('nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController', 'nav2_core::Controller'),
    'nav2_navfn_planner': ('nav2_navfn_planner::NavfnPlanner', 'nav2_core::GlobalPlanner'),
}


def inspect_install(prefix):
    prefix = Path(prefix)
    errors, artifacts, versions = [], {}, {}

    def read(path):
        data = path.read_bytes()
        artifacts[str(path.relative_to(prefix))] = hashlib.sha256(data).hexdigest()
        return data

    packages = set(EXECUTABLES) | set(PLUGINS) | {'nav2_util', 'nav2_msgs', 'nav2_costmap_2d'}
    for package in sorted(packages):
        try:
            metadata = ET.fromstring(read(prefix / 'share' / package / 'package.xml'))
            version = metadata.findtext('version')
            if metadata.findtext('name') != package:
                raise ValueError('package identity mismatch')
            versions[package] = version
            if not version or not version.startswith('1.3.'):
                raise ValueError('unreviewed Nav2 series; expected Jazzy 1.3.x')
            if package in EXECUTABLES:
                executable = prefix / 'lib' / package / EXECUTABLES[package]
                if not read(executable).startswith(b'\x7fELF'):
                    raise ValueError('server executable is not an ELF binary')
                if not os.access(executable, os.X_OK):
                    raise ValueError('server executable lacks execution permission')
            if package in PLUGINS:
                wanted, base = PLUGINS[package]
                found = False
                for xml in sorted((prefix / 'share' / package).glob('*.xml')):
                    document = ET.fromstring(read(xml))
                    for library in document.iter('library'):
                        if any(c.get('type') == wanted and c.get('base_class_type') == base
                               for c in library.iter('class')):
                            library_name = Path(library.attrib['path']).name
                            candidates = [prefix / 'lib' / (library_name + '.so'),
                                          prefix / 'lib' / ('lib' + library_name + '.so')]
                            binary = next((p for p in candidates if p.is_file()), None)
                            if binary is None or not read(binary).startswith(b'\x7fELF'):
                                raise ValueError('plugin library missing or not ELF')
                            found = True
                if not found:
                    raise ValueError('required plugin class/base not exported')
        except (OSError, ET.ParseError, ValueError, KeyError) as exc:
            errors.append(f'{package}: {exc}')
    if len(set(versions.values())) > 1:
        errors.append('Nav2 package versions differ; review this mixed installation')
    try:
        header = read(prefix / 'include/nav2_util/nav2_util/twist_publisher.hpp').decode()
        if not all(s in header for s in ('enable_stamped_cmd_vel', 'geometry_msgs::msg::TwistStamped')):
            raise ValueError('stamped publisher support absent from installed header')
        action = read(prefix / 'share/nav2_msgs/action/NavigateToPose.action').decode()
        sections = action.split('---')
        if len(sections) != 3:
            raise ValueError('unexpected NavigateToPose section count')
        fields = [{line.split('#')[0].strip() for line in s.splitlines()} for s in sections]
        if not {'geometry_msgs/PoseStamped pose', 'string behavior_tree'} <= fields[0] or 'uint16 error_code' not in fields[1]:
            raise ValueError('NavigateToPose contract incompatible with action client')
    except (OSError, UnicodeError, ValueError) as exc:
        errors.append(f'interface: {exc}')
    return {'schema_version': 1, 'artifact_preflight_passed': not errors,
            'runtime_compatibility_proven': False, 'prefix': str(prefix),
            'host_architecture': platform.machine(), 'versions': versions,
            'required_controller_parameter': {'enable_stamped_cmd_vel': True},
            'required_velocity_route': 'controller_server -> /navigation/raw_cmd_vel -> supervisor -> /motion/request -> guard',
            'errors': errors, 'artifact_sha256': artifacts}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, default=Path('/opt/ros/jazzy'))
    args = parser.parse_args()
    result = inspect_install(args.prefix)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['artifact_preflight_passed'] else 1)
