"""Cross-model frame regressions for the flat-floor robot.

Wheel odometry is planar. Its child must be the ground projection, while
URDF body and sensor heights must match SDF geometry relative to that plane.
The SDF model's initial 2 cm drop is a spawn clearance, not a TF offset.
"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def xyz(text):
    return tuple(float(x) for x in text.split()[:3])


class FrameGeometryTests(unittest.TestCase):
    def setUp(self):
        self.robot = ET.parse(ROOT/'config/robot.urdf').getroot()
        self.model = ET.parse(ROOT/'worlds/maintenance.sdf').getroot().find(
            "world/model[@name='maintenance_robot']")
        self.parents = {}
        for joint in self.robot.findall('joint'):
            origin = joint.find('origin')
            self.parents[joint.find('child').get('link')] = (
                joint.find('parent').get('link'),
                xyz(origin.get('xyz', '0 0 0')) if origin is not None else (0, 0, 0))

    def position(self, child):
        total = [0., 0., 0.]
        visited = set()
        while child in self.parents:
            self.assertNotIn(child, visited, 'TF cycle')
            visited.add(child)
            child, offset = self.parents[child]
            total = [a+b for a, b in zip(total, offset)]
        self.assertEqual(child, 'base_footprint')
        return total

    def assertPosition(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=6)

    def test_planar_odometry_targets_ground_projection(self):
        drive = self.model.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(drive.findtext('child_frame_id'), 'base_footprint')
        links = {link.get('name') for link in self.robot.findall('link')}
        self.assertIn('base_footprint', links)
        self.assertNotIn('base_footprint', self.parents)
        body_joint = next(j for j in self.robot.findall('joint')
                          if j.find('child').get('link') == 'base_link')
        self.assertEqual(body_joint.get('type'), 'fixed')
        self.assertEqual(body_joint.find('origin').get('rpy', '0 0 0'), '0 0 0')

    def test_body_and_sensor_heights_match_collision_model(self):
        body = self.model.find("link[@name='base_link']")
        body_xyz = xyz(body.findtext('pose'))
        self.assertPosition(self.position('base_link'), body_xyz)
        for sensor_name, frame in [('lidar', 'lidar_link'), ('camera', 'camera_link')]:
            sensor = body.find(f"sensor[@name='{sensor_name}']")
            offset = xyz(sensor.findtext('pose'))
            self.assertPosition(self.position(frame), [a+b for a,b in zip(body_xyz,offset)])
        self.assertAlmostEqual(self.position('lidar_link')[2], .38)
        self.assertAlmostEqual(self.position('camera_link')[2], .34)

    def test_wheel_centers_are_above_ground_by_their_radius(self):
        for name in ('left_wheel', 'right_wheel'):
            link = self.model.find(f"link[@name='{name}']")
            radius = float(link.findtext('collision/geometry/cylinder/radius'))
            self.assertPosition(self.position(name), xyz(link.findtext('pose')))
            self.assertAlmostEqual(self.position(name)[2], radius)


if __name__ == '__main__':
    unittest.main()
