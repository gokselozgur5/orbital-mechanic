"""Offline acceptance of the single-room inspection poses and control points.

This checks public geometry only. It cannot authorize maintenance or establish
executed clearance, localization accuracy or live arrival at a machine.
"""
from dataclasses import dataclass
import math
from pathlib import Path

from orbital.machines import MACHINE_IDS, MAX_CONTROL_DISTANCE_M
from orbital.mapping import Grid, build_grid, static_boxes
from orbital.path_clearance import require_clear_path

# Reviewed MVP bounds, shared with the supervisor's arrival check. Changing the
# map JSON must not silently reduce the footprint or the accepted arrival area.
ROBOT_RADIUS_M = .37
ARRIVAL_XY_TOLERANCE_M = .10
ARRIVAL_YAW_TOLERANCE_RAD = .15


@dataclass(frozen=True)
class InspectionMap:
    world: Path
    grid: Grid
    checks: dict


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_control_heading(goal, control, name):
    """A nominal inspection pose faces its west-face control, not another target."""
    if goal[0] >= control[0]:
        raise ValueError('Inspection heading must approach the west control face: '+name)
    bearing = math.atan2(control[1]-goal[1],control[0]-goal[0])
    # This is nominal source consistency, not the robot's measured yaw tolerance.
    if abs(math.remainder(goal[2]-bearing,2*math.pi)) > 1e-8:
        raise ValueError('Inspection heading does not face its control: '+name)
    return bearing


def validate_inspection_map(config, poses, root):
    """Require the reviewed bounds, actual west-face controls and clear goals."""
    root = Path(root).resolve()
    relative = config.get('world')
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError('Expected a repository-relative world')
    world = root/relative
    if not world.resolve().is_relative_to(root):
        raise ValueError('World escapes repository root')
    if world.is_symlink() or any(p.is_symlink() for p in world.parents if p != root and p.is_relative_to(root)):
        raise ValueError('World symlinks are not permitted')
    if set(poses) != set(MACHINE_IDS):
        raise ValueError('Inspection poses must match the three station machines')
    bounds = {'robot_radius_m': ROBOT_RADIUS_M,
              'arrival_xy_tolerance_m': ARRIVAL_XY_TOLERANCE_M,
              'arrival_yaw_tolerance_rad': ARRIVAL_YAW_TOLERANCE_RAD}
    for name, expected in bounds.items():
        if not _number(config.get(name)) or config[name] != expected:
            raise ValueError('Inspection bound differs from reviewed MVP contract: '+name)
    origin = config.get('origin_xy_m')
    if (not _number(config.get('resolution_m')) or config['resolution_m'] <= 0
            or not _number(config.get('slice_height_m'))
            or not isinstance(origin, list) or len(origin) != 2 or not all(map(_number, origin))
            or any(type(config.get(k)) is not int or config[k] <= 0 for k in ('width_cells', 'height_cells'))):
        raise ValueError('Invalid map dimensions')
    boxes = static_boxes(world)
    checks = {}
    for name, goal in poses.items():
        matches = [b for b in boxes if b.name == name]
        if len(matches) != 1:
            raise ValueError('Expected one machine collision box: '+name)
        box = matches[0]
        if abs(config['slice_height_m']-box.z) > box.sz/2:
            raise ValueError('Map slice misses machine collision: '+name)
        control = config['inspection_poses'][name].get('control_xy_m')
        if not isinstance(control, list) or len(control) != 2 or not all(map(_number, control)):
            raise ValueError('Expected finite numeric control XY for '+name)
        if math.dist(control, (box.x-box.sx/2, box.y)) > 1e-8:
            raise ValueError('Control must match the machine west-face center: '+name)
        bearing = validate_control_heading(goal,control,name)
        distance = math.dist(goal[:2], control)
        margin = MAX_CONTROL_DISTANCE_M-distance-ARRIVAL_XY_TOLERANCE_M
        if margin < -1e-12:
            raise ValueError('Arrival tolerance exceeds control proximity bound: '+name)
        checks[name] = {'goal_xy_yaw': list(goal), 'control_xy_m': control,
                        'nominal_control_bearing_rad': bearing,
                        'goal_control_distance_m': distance, 'control_margin_m': max(0., margin),
                        'arrival_footprint_radius_m': ROBOT_RADIUS_M+ARRIVAL_XY_TOLERANCE_M}
    grid = build_grid(world, config)
    # Check the complete allowed arrival area, including its boundary, rather
    # than only the nominal center. Unknown and outside cells are not free.
    require_clear_path(grid, [(0., 0.)], ROBOT_RADIUS_M)
    for name, goal in poses.items():
        try:
            require_clear_path(grid, [goal[:2]], ROBOT_RADIUS_M+ARRIVAL_XY_TOLERANCE_M)
        except ValueError as error:
            raise ValueError('Inspection arrival area is not clear: '+name+': '+str(error)) from error
    if not grid.reachable_all((0., 0.), (goal[:2] for goal in poses.values()), ROBOT_RADIUS_M):
        raise ValueError('Inspection poses are unreachable in inflated grid')
    for name in poses:
        checks[name]['inflated_reachable_from_origin'] = True
    return InspectionMap(world, grid, checks)
