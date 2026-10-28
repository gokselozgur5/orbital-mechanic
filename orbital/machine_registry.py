"""Public configured scene identities, independent of scenario and robot truth.

This binds reviewed source bytes, not the simulator's loaded entities or physics.
The registry does not establish robot proximity or authorize an intervention.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from orbital.collision_geometry import Scene, Transform, IDENTITY, same_transform, sdf_pose
from orbital.machines import MACHINE_IDS
from orbital.navigation_configuration import parse_waypoint_contract

CONFIG_PATH = 'config/known-map.json'
SOURCE_LIMIT = 32768


def read_source(root, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError('Registry source must be repository-relative')
    path = root/relative
    if not path.resolve().is_relative_to(root):
        raise ValueError('Registry source escapes root')
    if path.is_symlink() or any(p.is_symlink() for p in path.parents if p != root and p.is_relative_to(root)):
        raise ValueError('Registry source cannot be a symlink')
    if not path.is_file():
        raise ValueError('Registry source must be a regular file')
    with path.open('rb') as stream:
        data = stream.read(SOURCE_LIMIT+1)
    if not data or len(data) > SOURCE_LIMIT:
        raise ValueError('Registry source exceeds its bound or is empty')
    return data


def digest(data):
    return hashlib.sha256(data).hexdigest()


@dataclass(frozen=True)
class MachineBinding:
    machine_id: str
    model_pose: tuple
    collision_center: tuple
    collision_size: tuple
    control: tuple
    inspection: tuple

    def public(self):
        return {'model_name': self.machine_id, 'model_pose_xyz_xyzw': list(self.model_pose),
                'collision_center_xyz_m': list(self.collision_center),
                'collision_size_xyz_m': list(self.collision_size),
                'control_xy_m': list(self.control), 'inspection_xy_yaw': list(self.inspection)}


@dataclass(frozen=True)
class MachineRegistry:
    root: Path
    world_path: str
    config_sha256: str
    world_sha256: str
    bindings: tuple

    @classmethod
    def load(cls, root):
        root = Path(root).resolve()
        config_raw = read_source(root, CONFIG_PATH)
        config, poses = parse_waypoint_contract(config_raw)
        if set(poses) != set(MACHINE_IDS):
            raise ValueError('Registry requires the exact station machine IDs')
        world_raw = read_source(root, config.get('world'))
        scene = Scene(world_raw)
        world = ET.fromstring(world_raw).find('world')
        bindings = []
        for name in MACHINE_IDS:
            model = scene.models[name]
            if len(model['parts']) != 1 or model['parts'][0].kind != 'box':
                raise ValueError('Registry machine must have one box collision: '+name)
            part = model['parts'][0]
            transform = model['pose'].compose(model['links'][part.link]).compose(part.local)
            if not same_transform(Transform((0., 0., 0.), transform.axes), Transform.pose(IDENTITY)):
                raise ValueError('Registry controls require axis-aligned machine geometry')
            control = config['inspection_poses'][name].get('control_xy_m')
            expected = (transform.center[0]-part.size[0]/2, transform.center[1])
            if (not isinstance(control, list) or len(control) != 2
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in control)
                    or math.dist(control, expected) > 1e-8):
                raise ValueError('Registry control does not match the machine west face: '+name)
            bindings.append(MachineBinding(name, sdf_pose(world.find("model[@name='"+name+"']")),
                                           transform.center, part.size, tuple(control), poses[name]))
        registry = cls(root, config['world'], digest(config_raw), digest(world_raw), tuple(bindings))
        registry.verify_current()
        return registry

    def content(self):
        return {'schema_version': 1, 'frame': 'map', 'world_name': 'maintenance',
                'geometry_source': 'configured_world',
                'source_sha256': {'map_config': self.config_sha256, 'world': self.world_sha256},
                'machines': {binding.machine_id: binding.public() for binding in self.bindings}}

    @property
    def identity(self):
        return digest(json.dumps(self.content(), sort_keys=True, separators=(',', ':'), allow_nan=False).encode())

    def public(self):
        return {'identity': self.identity, **self.content()}

    def verify_current(self):
        if (digest(read_source(self.root, CONFIG_PATH)) != self.config_sha256
                or digest(read_source(self.root, self.world_path)) != self.world_sha256):
            raise ValueError('Machine registry source changed; restart with a reviewed world/map')
