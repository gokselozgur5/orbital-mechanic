import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from orbital.mapping import build_grid,static_boxes,Grid,FREE,OCCUPIED,UNKNOWN

ROOT=Path(__file__).resolve().parents[1]


class MappingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=json.loads((ROOT/'config/known-map.json').read_text())
        cls.grid=build_grid(ROOT/cls.config['world'],cls.config)

    def test_world_geometry_and_reference_pixels(self):
        g=self.grid
        self.assertEqual((g.width,g.height),(210,170))
        self.assertEqual(g.at(*g.cell(0,0)),FREE)
        for x,y in ((3,1.8),(3,0),(3,-1.8),(5,0),(-5,0),(0,4),(0,-4)):
            self.assertEqual(g.at(*g.cell(x,y)),OCCUPIED,(x,y))
        self.assertEqual(g.at(*g.cell(-5.2,-4.2)),UNKNOWN)
        for c,r in ((0,0),(20,50),(209,169)):
            self.assertEqual(g.cell(*g.center(c,r)),(c,r))

    def test_pgm_rows_are_top_to_bottom(self):
        g=Grid(1,(0,0),2,2,[FREE,UNKNOWN,OCCUPIED,FREE])
        self.assertEqual(g.pgm(),b'P2\n2 2\n255\n0 254\n254 128\n')

    def test_goal_reachability_and_control_margin(self):
        for pose in self.config['inspection_poses'].values():
            goal=pose['goal_xy_yaw'][:2]
            self.assertTrue(self.grid.reachable((0,0),goal,.37))
            self.assertLessEqual(math.dist(goal,pose['control_xy_m'])+self.config['arrival_xy_tolerance_m'],.75)
        self.assertFalse(self.grid.reachable((0,0),(3,0),.37))
        self.assertFalse(self.grid.reachable((0,0),(5.2,4.2),.37))

    def test_inflation_rejects_narrow_corridor(self):
        cells=[OCCUPIED]*49
        for row in range(7):cells[row*7+3]=FREE
        g=Grid(1,(0,0),7,7,cells)
        self.assertTrue(g.reachable((3.5,1.5),(3.5,5.5),0))
        self.assertFalse(g.reachable((3.5,1.5),(3.5,5.5),.6))

    def test_all_goals_require_one_connected_inflated_region(self):
        cells = [FREE]*63
        for row in range(7):
            cells[row*9+4] = OCCUPIED
        grid = Grid(1, (0,0), 9, 7, cells)
        self.assertTrue(grid.reachable_all((1.5,1.5), [(1.5,1.5),(2.5,4.5)], .1))
        self.assertFalse(grid.reachable_all((1.5,1.5), [(2.5,4.5),(6.5,4.5)], .1))
        self.assertFalse(grid.reachable_all((1.5,1.5), [(4.5,4.5)], .1))
        self.assertFalse(grid.reachable_all((1.5,1.5), [(-1.,4.5)], .1))

    def test_rotated_static_collision_is_not_silently_ignored(self):
        tree=ET.parse(ROOT/self.config['world'])
        tree.getroot().find("world/model[@name='pump']/pose").text='3 1.8 .6 0 0 .3'
        # Test artifacts stay in the ignored repository runtime root.
        runtime=ROOT/'runtime';runtime.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=runtime) as d:
            path=Path(d)/'world.sdf';tree.write(path)
            with self.assertRaisesRegex(ValueError,'Rotated'):static_boxes(path)

    def test_unsupported_geometry_is_rejected(self):
        tree=ET.parse(ROOT/self.config['world'])
        shape=tree.getroot().find("world/model[@name='pump']/link/collision/geometry")
        shape.clear();ET.SubElement(shape,'sphere')
        runtime=ROOT/'runtime';runtime.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=runtime) as d:
            path=Path(d)/'world.sdf';tree.write(path)
            with self.assertRaisesRegex(ValueError,'box'):static_boxes(path)


if __name__=='__main__':unittest.main()
