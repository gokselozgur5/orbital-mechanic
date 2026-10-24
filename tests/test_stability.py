"""Mass distribution must leave a conservative flat-floor support margin."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class StabilityTests(unittest.TestCase):
    def test_combined_center_of_mass_inside_support_triangle(self):
        model = ET.parse(ROOT/'worlds/maintenance.sdf').find("world/model[@name='maintenance_robot']")
        total = 0.
        weighted = [0., 0., 0.]
        for link in model.findall('link'):
            mass = float(link.findtext('inertial/mass'))
            origin = [float(x) for x in link.findtext('pose').split()[:3]]
            offset = [float(x) for x in link.findtext('inertial/pose', '0 0 0 0 0 0').split()[:3]]
            # Wheel inertial origins are centered; only the unrotated body has an offset.
            if any(offset):
                self.assertEqual([float(x) for x in link.findtext('pose').split()[3:]], [0., 0., 0.])
            total += mass
            weighted = [a+mass*(b+c) for a,b,c in zip(weighted, origin, offset)]
        x,y,z = [v/total for v in weighted]
        support = [tuple(float(x) for x in model.findtext(f"link[@name='{name}']/pose").split()[:2])
                   for name in ('right_wheel', 'left_wheel', 'caster')]
        margins = []
        for a,b in zip(support, support[1:]+support[:1]):
            cross = (b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])
            margins.append(cross/math.dist(a,b))
        self.assertGreater(min(margins), .04, (x,y,z,margins))
        drive = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
        acceleration = float(drive.findtext('max_linear_acceleration', 'inf'))
        deceleration = float(drive.findtext('min_linear_acceleration', '-inf'))
        self.assertGreater(acceleration, 0)
        self.assertLess(deceleration, 0)
        # Inertial load shift with a 2x margin must stay inside the static polygon.
        self.assertLess(2*max(acceleration, -deceleration)*z/9.81, min(margins))


if __name__ == '__main__':
    unittest.main()
