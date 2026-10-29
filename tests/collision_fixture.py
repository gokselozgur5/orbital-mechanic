"""Explicit test-only SceneBroadcaster packets from the supported world source."""
import xml.etree.ElementTree as ET

from orbital.collision_geometry import IDENTITY, sdf_pose
from test_trajectory import Proto


def scene_message(data, stamp, *, robot_pose=None, link_poses=None, model_poses=None):
    world = ET.fromstring(data).find('world')
    rows = []
    def append(name, pose):
        x, y, z, qx, qy, qz, qw = pose
        rows.append(Proto(name=name, id=len(rows)+1, position=Proto(x=x, y=y, z=z),
                          orientation=Proto(x=qx, y=qy, z=qz, w=qw)))
    for model in world.findall('model'):
        name = model.get('name')
        pose = (model_poses or {}).get(name, sdf_pose(model))
        if name == 'maintenance_robot' and robot_pose is not None:
            pose = robot_pose
        append(name, pose)
    for model in world.findall('model'):
        for link in model.findall('link'):
            pose = sdf_pose(link)
            if model.get('name') == 'maintenance_robot':
                pose = (link_poses or {}).get(link.get('name'), pose)
            append(link.get('name'), pose)
    for visual in world.findall('.//visual'):
        append(visual.get('name'), sdf_pose(visual))
    for light in world.findall('.//light'):
        append(light.get('name'), sdf_pose(light))
    return Proto(header=Proto(stamp=Proto(sec=stamp//10**9, nsec=stamp % 10**9)), pose=rows)
