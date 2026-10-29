"""Independent supported-world geometry, using observed model and link poses.

Cylinders use inner/outer convex prisms: uncertain boundary cases never become
clearance or measured collision. This evaluates samples, not physics contacts.
"""
from collections import Counter, defaultdict
from dataclasses import dataclass
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET

EPS = 1e-9
CYLINDER_SIDES = 32
SUPPORT_PENETRATION_M = .002
WORLD_LIMIT = 32768
IDENTITY = (0., 0., 0., 0., 0., 0., 1.)
SUPPORT_PARTS = {'caster/collision': 'sphere', 'left_wheel/collision': 'cylinder',
                 'right_wheel/collision': 'cylinder'}


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def scale(a, value):
    return tuple(x*value for x in a)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def finite(values, count, bound=10000.):
    if (len(values) != count or any(type(v) not in (int, float) or not math.isfinite(v)
                                   or abs(v) > bound for v in values)):
        raise ValueError('Unsupported/nonfinite geometry values')
    return tuple(values)


@dataclass(frozen=True)
class Transform:
    center: tuple
    axes: tuple

    @classmethod
    def pose(cls, values):
        finite(values, 7)
        x, y, z, w = values[3:]
        norm = math.sqrt(x*x+y*y+z*z+w*w)
        if abs(norm*norm-1.) > .001:
            raise ValueError('Non-unit geometry quaternion')
        x, y, z, w = (v/norm for v in (x, y, z, w))
        # Columns of the local-to-parent rotation matrix.
        return cls(tuple(values[:3]), (
            (1-2*(y*y+z*z), 2*(x*y+z*w), 2*(x*z-y*w)),
            (2*(x*y-z*w), 1-2*(x*x+z*z), 2*(y*z+x*w)),
            (2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y))))

    def rotate(self, vector):
        return tuple(sum(vector[j]*self.axes[j][i] for j in range(3)) for i in range(3))

    def compose(self, other):
        return Transform(add(self.center, self.rotate(other.center)), tuple(self.rotate(a) for a in other.axes))


def same_transform(a, b):
    return all(abs(x-y) <= 1e-7 for x, y in zip((*a.center, *sum(a.axes, ())),
                                               (*b.center, *sum(b.axes, ()))))


def sdf_pose(element):
    if len(element.findall('pose')) > 1:
        raise ValueError('Duplicate SDF pose')
    pose = element.find('pose')
    if pose is None:
        return IDENTITY
    if pose.attrib or len(pose):
        raise ValueError('Only default parent-relative SDF poses are supported')
    x, y, z, roll, pitch, yaw = finite(tuple(float(v) for v in (pose.text or '').split()), 6)
    sr, cr = math.sin(roll/2), math.cos(roll/2)
    sp, cp = math.sin(pitch/2), math.cos(pitch/2)
    sy, cy = math.sin(yaw/2), math.cos(yaw/2)
    return (x, y, z, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


@dataclass(frozen=True)
class Part:
    link: str
    name: str
    kind: str
    size: tuple
    local: Transform


@dataclass(frozen=True)
class Shape:
    name: str
    kind: str
    size: tuple
    transform: Transform

    def radius(self, axis, polygon=None):
        local = tuple(dot(axis, a) for a in self.transform.axes)
        if self.kind == 'box':
            return sum(abs(local[i])*self.size[i]/2 for i in range(3))
        if self.kind == 'sphere':
            return self.size[0]  # axis is unit length
        radius, length = self.size
        if polygon is None:
            radial = radius*math.hypot(local[0], local[1])
        else:
            radial = radius*polygon*max(abs(local[0]*math.cos(i*2*math.pi/CYLINDER_SIDES)
                                           + local[1]*math.sin(i*2*math.pi/CYLINDER_SIDES))
                                       for i in range(CYLINDER_SIDES//2))
        return radial+length/2*abs(local[2])

    def sphere_radius(self):
        if self.kind == 'box':
            return math.sqrt(sum((v/2)**2 for v in self.size))
        if self.kind == 'sphere':
            return self.size[0]
        return math.hypot(self.size[0], self.size[1]/2)


def separated(a, b, axes, polygon=None):
    delta = tuple(x-y for x, y in zip(a.transform.center, b.transform.center))
    for axis in axes:
        length = math.sqrt(dot(axis, axis))
        if length < 1e-12:
            continue
        axis = scale(axis, 1/length)
        if abs(dot(delta, axis))-a.radius(axis, polygon)-b.radius(axis) > EPS:
            return True
    return False


def relation(part, obstacle):
    """Return clear, overlap_or_touch or indeterminate against an oriented box."""
    if obstacle.kind != 'box':
        raise ValueError('World obstacles must be boxes')
    delta = tuple(x-y for x, y in zip(part.transform.center, obstacle.transform.center))
    if math.sqrt(dot(delta, delta)) > part.sphere_radius()+obstacle.sphere_radius()+EPS:
        return 'clear'
    axes = obstacle.transform.axes
    if part.kind == 'sphere':
        squared = sum(max(abs(dot(delta, axes[i]))-obstacle.size[i]/2, 0.)**2 for i in range(3))
        return 'clear' if math.sqrt(squared) > part.size[0]+EPS else 'overlap_or_touch'
    if part.kind == 'box':
        candidates = (*axes, *part.transform.axes,
                      *(cross(a, b) for a in axes for b in part.transform.axes))
        return 'clear' if separated(part, obstacle, candidates) else 'overlap_or_touch'
    if part.kind != 'cylinder':
        raise ValueError('Unsupported robot collision shape')
    # Exact cylinder projection can cheaply prove separation on box/cap axes.
    if separated(part, obstacle, (*axes, part.transform.axes[2])):
        return 'clear'
    u, v, w = part.transform.axes
    normals = [add(scale(u, math.cos((i+.5)*2*math.pi/CYLINDER_SIDES)),
                   scale(v, math.sin((i+.5)*2*math.pi/CYLINDER_SIDES)))
               for i in range(CYLINDER_SIDES//2)]
    edges = [cross(w, n) for n in normals]+[w]
    candidates = (*axes, w, *normals, *(cross(a, b) for a in axes for b in edges))
    if separated(part, obstacle, candidates, polygon=1/math.cos(math.pi/CYLINDER_SIDES)):
        return 'clear'
    if not separated(part, obstacle, candidates, polygon=1.):
        return 'overlap_or_touch'
    return 'indeterminate'


class Scene:
    """Bounded explicit SDF subset; unsupported geometry is never silently skipped."""
    def __init__(self, data):
        if not isinstance(data, bytes) or not data or len(data) > WORLD_LIMIT:
            raise ValueError('World source exceeds its bound or is empty')
        if b'<!DOCTYPE' in data.upper() or b'<!ENTITY' in data.upper():
            raise ValueError('World XML declarations are unsupported')
        try:
            root = ET.fromstring(data)
        except ET.ParseError as error:
            raise ValueError('Malformed world XML') from error
        worlds = root.findall('world')
        if root.tag != 'sdf' or root.get('version') != '1.9' or len(worlds) != 1 or worlds[0].get('name') != 'maintenance':
            raise ValueError('Expected the explicit maintenance SDF 1.9 world')
        world = worlds[0]
        if world.findall('.//include') or world.findall('.//frame') or world.findall('.//model/model') or world.findall('state'):
            raise ValueError('Includes, nested models and named frames are unsupported')
        self.data, self.sha256 = data, hashlib.sha256(data).hexdigest()
        self.models, self.entries, self.static_links = {}, Counter(), defaultdict(list)
        self.robot_links = set()
        for model in world.findall('model'):
            if set(model.attrib) != {'name'}:
                raise ValueError('Model frame/placement attributes are unsupported')
            name = self.name(model)
            if name in self.models or len(self.models) >= 32:
                raise ValueError('Duplicate or too many models')
            if len(model.findall('static')) > 1:
                raise ValueError('Duplicate model static flag')
            flag = model.findtext('static', 'false').strip()
            if flag not in ('true', 'false', '1', '0') or (flag in ('false', '0')) != (name == 'maintenance_robot'):
                raise ValueError('Only maintenance_robot may be dynamic')
            parts, links = [], {}
            self.entries[name] += 1
            for link in model.findall('link'):
                link_name = self.name(link)
                if link_name in links or len(links) >= 16:
                    raise ValueError('Duplicate or too many links')
                links[link_name] = Transform.pose(sdf_pose(link))
                self.entries[link_name] += 1
                if name == 'maintenance_robot':
                    self.robot_links.add(link_name)
                else:
                    self.static_links[link_name].append(links[link_name])
                collision_names = set()
                for collision in link.findall('collision'):
                    collision_name = self.name(collision)
                    if collision_name in collision_names or len(parts) >= 32:
                        raise ValueError('Duplicate or too many collision parts')
                    collision_names.add(collision_name)
                    geometry = collision.find('geometry')
                    if len(collision.findall('geometry')) != 1 or len(geometry) != 1:
                        raise ValueError('Expected one collision primitive')
                    kind = geometry[0].tag
                    fields = {'box': {'size'}, 'sphere': {'radius'}, 'cylinder': {'radius', 'length'}}.get(kind)
                    if (fields is None or geometry.attrib or geometry[0].attrib
                            or len(geometry[0]) != len(fields) or {c.tag for c in geometry[0]} != fields
                            or any(c.attrib for c in geometry[0])):
                        raise ValueError('Unsupported or ambiguous primitive dimensions')
                    if kind == 'box':
                        size = finite(tuple(float(v) for v in geometry.findtext('box/size', '').split()), 3, 100.)
                    elif name == 'maintenance_robot' and kind == 'sphere':
                        size = finite((float(geometry.findtext('sphere/radius', 'nan')),), 1, 100.)
                    elif name == 'maintenance_robot' and kind == 'cylinder':
                        size = finite(tuple(float(geometry.findtext('cylinder/'+k, 'nan')) for k in ('radius', 'length')), 2, 100.)
                    else:
                        raise ValueError('Unsupported collision primitive: '+kind)
                    if any(v <= 0 for v in size):
                        raise ValueError('Collision dimensions must be positive')
                    parts.append(Part(link_name, collision_name, kind, size, Transform.pose(sdf_pose(collision))))
                for visual in link.findall('visual'):
                    self.entries[self.name(visual)] += 1
            if not parts:
                raise ValueError('Every model must have declared collision geometry')
            self.models[name] = {'pose': Transform.pose(sdf_pose(model)), 'links': links, 'parts': parts}
        for light in world.findall('.//light'):
            self.entries[self.name(light)] += 1
            # Gazebo creates this child visual even when it is absent from SDF.
            self.entries[self.name(light)+'Visual'] += 1
        required = {'maintenance_robot', 'floor', 'north', 'south', 'east', 'west', 'pump', 'cooler', 'power_unit'}
        if not required <= self.models.keys():
            raise ValueError('Maintenance world model missing')
        if sum(len(m['parts']) for m in self.models.values()) > 128:
            raise ValueError('Too many world collision parts')
        robot_parts = {p.link+'/'+p.name: p.kind for p in self.models['maintenance_robot']['parts']}
        if len(robot_parts) > 16 or any(robot_parts.get(n) != k for n, k in
                                       {**SUPPORT_PARTS, 'base_link/collision': 'box'}.items()):
            raise ValueError('Required robot collision parts missing or changed type')
        if any(self.entries[n] != 1 for n in (*self.models, *self.robot_links)):
            raise ValueError('Model/robot link names must be unique in the pose stream')
        if any(self.entries[n] != len(poses) for n, poses in self.static_links.items()):
            raise ValueError('Ambiguous static-link pose name')
        # Unscoped Pose_V names carry no parent mapping. A shared name is safe
        # only when every possible parent expects the same relative transform.
        if any(not same_transform(poses[0], pose) for poses in self.static_links.values() for pose in poses[1:]):
            raise ValueError('Ambiguous static-link parent: use unique names for different relative poses')
        floor = self.models['floor']
        if len(floor['parts']) != 1 or floor['parts'][0].kind != 'box':
            raise ValueError('Expected one floor box')
        shape = self._shapes('floor', floor['pose'], floor['links'])[0]
        if not same_transform(Transform((0., 0., 0.), shape.transform.axes), Transform.pose(IDENTITY)):
            raise ValueError('Floor must be horizontal and axis aligned')

    @staticmethod
    def name(element):
        name = element.get('name', '')
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,63}', name):
            raise ValueError('Unsupported or missing entity name')
        return name

    @staticmethod
    def observed_pose(pose):
        if not pose.HasField('position') or not pose.HasField('orientation'):
            raise ValueError('Scene pose fields missing')
        p, q = pose.position, pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        Transform.pose(values)
        return values

    def observe(self, message):
        if Counter(p.name for p in message.pose) != self.entries:
            raise ValueError('Loaded scene pose topology differs from declared world')
        by_name = defaultdict(list)
        identifiers = []
        for pose in message.pose:
            if type(pose.id) is not int or not 0 < pose.id < 2**32:
                raise ValueError('Invalid scene entity ID')
            identifiers.append((pose.name, pose.id))
            by_name[pose.name].append(pose)
        if len({p.id for p in message.pose}) != len(message.pose):
            raise ValueError('Duplicate scene entity ID')
        static_links = {name: [{'entity_id': pose.id, 'pose': self.observed_pose(pose)}
                              for pose in by_name[name]] for name in self.static_links}
        self.check_static_links(static_links)
        def observation(name):
            value = by_name[name][0]
            return {'entity_id': value.id, 'pose': self.observed_pose(value)}
        links = {name: observation(name) for name in sorted(self.robot_links)}
        obstacles = {name: observation(name) for name in sorted(self.models) if name != 'maintenance_robot'}
        if not same_transform(Transform.pose(obstacles['floor']['pose']), self.models['floor']['pose']):
            raise ValueError('Floor moved; support policy no longer applies')
        return {'links': links, 'obstacles': obstacles, 'static_links': static_links, 'world_sha256': self.sha256,
                'entity_signature': hashlib.sha256(json.dumps(sorted(identifiers)).encode()).hexdigest()}

    def check_static_links(self, values):
        if not isinstance(values, dict) or set(values) != set(self.static_links):
            raise ValueError('Missing static link observations')
        for name, expected in self.static_links.items():
            if not isinstance(values[name], list) or len(values[name]) != len(expected):
                raise ValueError('Static link count changed')
            remaining = list(expected)
            for value in values[name]:
                actual = Transform.pose(value['pose'])
                index = next((i for i, value in enumerate(remaining) if same_transform(value, actual)), None)
                if index is None:
                    raise ValueError('Static link moved relative to its model')
                remaining.pop(index)

    def _shapes(self, name, model, links):
        return [Shape(p.link+'/'+p.name if name == 'maintenance_robot' else name+'/'+p.link+'/'+p.name,
                      p.kind, p.size, model.compose(links[p.link]).compose(p.local))
                for p in self.models[name]['parts']]

    def evaluate(self, truth):
        if (truth.get('world_sha256') != self.sha256 or set(truth.get('links', {})) != self.robot_links
                or set(truth.get('obstacles', {})) != set(self.models)-{'maintenance_robot'}):
            raise ValueError('Truth lacks exact world/link geometry coverage')
        self.check_static_links(truth.get('static_links'))
        identifiers = [truth['entity_id'], *(v['entity_id'] for v in truth['links'].values()),
                       *(v['entity_id'] for v in truth['obstacles'].values()),
                       *(v['entity_id'] for rows in truth['static_links'].values() for v in rows)]
        if (any(type(i) is not int or not 0 < i < 2**32 for i in identifiers)
                or len(set(identifiers)) != len(identifiers)
                or not isinstance(truth.get('entity_signature'), str)
                or not re.fullmatch('[0-9a-f]{64}', truth['entity_signature'])):
            raise ValueError('Invalid geometry entity identities')
        robot = self._shapes('maintenance_robot', Transform.pose(truth['pose']),
                             {n: Transform.pose(v['pose']) for n, v in truth['links'].items()})
        obstacles = []
        for name, value in truth['obstacles'].items():
            pose = Transform.pose(value['pose'])
            if name == 'floor' and not same_transform(pose, self.models['floor']['pose']):
                raise ValueError('Floor moved')
            obstacles.extend(self._shapes(name, pose, self.models[name]['links']))
        floor = next(s for s in obstacles if s.name.startswith('floor/'))
        others = [s for s in obstacles if s is not floor]
        events, penetration = [], {}
        top = floor.transform.center[2]+floor.size[2]/2
        basis = Transform.pose(IDENTITY).axes
        for part in robot:
            for obstacle in others:
                outcome = relation(part, obstacle)
                if outcome != 'clear':
                    events.append({'part': part.name, 'obstacle': obstacle.name, 'result': outcome})
            bottom = part.transform.center[2]-part.radius(basis[2])
            if SUPPORT_PARTS.get(part.name) == part.kind:
                penetration[part.name] = max(0., top-bottom)
                if top-bottom > SUPPORT_PENETRATION_M+EPS:
                    events.append({'part': part.name, 'obstacle': floor.name, 'result': 'support_penetration'})
            elif bottom <= top+EPS:
                events.append({'part': part.name, 'obstacle': floor.name, 'result': 'body_at_or_below_floor'})
            if any(abs(part.transform.center[i]-floor.transform.center[i])+part.radius(basis[i])
                   > floor.size[i]/2+EPS for i in (0, 1)):
                events.append({'part': part.name, 'obstacle': floor.name, 'result': 'outside_floor'})
        return {'events': events, 'support_penetration_m': penetration,
                'checked_pairs': len(robot)*len(others), 'robot_parts': len(robot)}
