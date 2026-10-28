"""Validate named navigation inputs before constructing a ROS command owner."""
import json
import hashlib
import math
from pathlib import Path
import re


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate configuration key: '+key)
        result[key] = value
    return result


def load_waypoints(path):
    return load_waypoint_contract(path)[0]


def load_waypoint_contract(path):
    raw = Path(path).read_bytes()
    data = json.loads(raw, object_pairs_hook=_unique_pairs)
    if not isinstance(data, dict) or data.get('frame') != 'map':
        raise ValueError('Navigation poses must use the map frame')
    entries = data.get('inspection_poses')
    if not isinstance(entries, dict) or not 1 <= len(entries) <= 100:
        raise ValueError('Expected 1 to 100 named inspection poses')
    poses = {}
    for name, entry in entries.items():
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', name):
            raise ValueError('Invalid waypoint name: '+name)
        values = entry.get('goal_xy_yaw') if isinstance(entry, dict) else None
        if (not isinstance(values, list) or len(values) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise ValueError('Expected finite numeric XY/yaw for '+name)
        if abs(values[2]) > math.pi:
            raise ValueError('Waypoint yaw must be in [-pi, pi]')
        poses[name] = tuple(float(v) for v in values)
    return poses, hashlib.sha256(raw).hexdigest()


def bounded_timeout(value):
    timeout = float(value)
    if not math.isfinite(timeout) or not .1 <= timeout <= 300:
        raise ValueError('Goal timeout must be in [0.1, 300] seconds')
    return timeout
