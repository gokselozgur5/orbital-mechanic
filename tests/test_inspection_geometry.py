"""Full-config regressions through the existing waypoint-loading API."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

from orbital.mapping import build_grid
from orbital.navigation_configuration import load_waypoint_contract
from orbital.path_clearance import require_clear_path

ROOT = Path(__file__).resolve().parents[1]


class InspectionGeometryTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'runtime').mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT/'runtime')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path = self.root/'config/known-map.json'
        self.path.parent.mkdir()
        self.world = self.root/'worlds/maintenance.sdf'
        self.world.parent.mkdir()
        shutil.copyfile(ROOT/'worlds/maintenance.sdf', self.world)
        self.config = json.loads((ROOT/'config/known-map.json').read_bytes())

    def load(self, config=None):
        self.path.write_text(json.dumps(self.config if config is None else config))
        return load_waypoint_contract(self.path)

    def test_valid_contract_preserves_exact_byte_identity(self):
        poses, digest = self.load()
        self.assertEqual(poses['pump'], (2., 1.8, 0.))
        self.assertEqual(digest, hashlib.sha256(self.path.read_bytes()).hexdigest())

    def test_nominal_clearance_does_not_cover_arrival_tolerance(self):
        self.config['inspection_poses']['pump']['goal_xy_yaw'][0] = 2.1
        grid = build_grid(self.world, self.config)
        require_clear_path(grid, [(2.1, 1.8)], .37)  # Nominal body is clear.
        with self.assertRaises(ValueError):
            self.load()  # The allowed arrival envelope reaches occupied cells.

    def test_free_reachable_goal_can_be_too_far_from_control(self):
        self.config['inspection_poses']['pump']['goal_xy_yaw'][0] = 1.9
        grid = build_grid(self.world, self.config)
        self.assertTrue(grid.reachable((0., 0.), (1.9, 1.8), .37))
        with self.assertRaises(ValueError):
            self.load()

    def test_occupied_goal_rejected(self):
        self.config['inspection_poses']['pump']['goal_xy_yaw'][0] = 3.
        with self.assertRaises(ValueError):
            self.load()

    def test_cropped_map_cannot_exclude_inspection_poses(self):
        self.config['width_cells'] = 140  # Right edge is x=1.75; origin still fits.
        with self.assertRaises(ValueError):
            self.load()

    def test_control_must_be_the_matching_machine_west_face(self):
        for control in ([3., 1.8], [2.6, 0.], [2.6, 2.]):
            self.config['inspection_poses']['pump']['control_xy_m'] = control
            with self.subTest(control=control), self.assertRaises(ValueError):
                self.load()

    def test_control_requires_finite_coordinates(self):
        for control in (None, [], [True, 1.8], [2.6, float('nan')], ['2.6', 1.8]):
            self.config['inspection_poses']['pump']['control_xy_m'] = control
            with self.subTest(control=control), self.assertRaises(ValueError):
                self.load()

    def test_missing_or_unknown_machine_rejected(self):
        missing = copy.deepcopy(self.config)
        del missing['inspection_poses']['pump']
        extra = copy.deepcopy(self.config)
        extra['inspection_poses']['unknown'] = {'goal_xy_yaw': [0, 0, 0]}
        for config in (missing, extra):
            with self.subTest(names=list(config['inspection_poses'])), self.assertRaises(ValueError):
                self.load(config)

    def test_json_cannot_weaken_reviewed_bounds(self):
        for name, value in (('robot_radius_m', .01), ('arrival_xy_tolerance_m', .01),
                            ('arrival_yaw_tolerance_rad', 1.), ('robot_radius_m', True)):
            config = copy.deepcopy(self.config)
            config[name] = value
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                self.load(config)

    def test_slice_cannot_hide_machine_collisions(self):
        self.config['slice_height_m'] = 5.
        with self.assertRaises(ValueError):
            self.load()

    def test_actual_world_pose_must_match_configured_control(self):
        tree = ET.parse(self.world)
        tree.getroot().find("world/model[@name='pump']/pose").text = '3 1.9 .6 0 0 0'
        tree.write(self.world)
        with self.assertRaises(ValueError):
            self.load()

    def test_disconnected_clear_goal_rejected(self):
        tree = ET.parse(self.world)
        wall = ET.fromstring('''<model name="partition"><static>true</static>
          <pose>1 0 1 0 0 0</pose><link name="wall"><collision name="body">
          <geometry><box><size>.1 8 2</size></box></geometry></collision></link></model>''')
        tree.getroot().find('world').append(wall)
        tree.write(self.world)
        grid = build_grid(self.world, self.config)
        require_clear_path(grid, [(2., 1.8)], .47)
        self.assertFalse(grid.reachable((0., 0.), (2., 1.8), .37))
        with self.assertRaises(ValueError):
            self.load()

    def test_matching_machine_translation_remains_valid(self):
        tree = ET.parse(self.world)
        tree.getroot().find("world/model[@name='pump']/pose").text = '3.1 1.8 .6 0 0 0'
        tree.write(self.world)
        self.config['inspection_poses']['pump']['control_xy_m'][0] = 2.7
        self.config['inspection_poses']['pump']['goal_xy_yaw'][0] = 2.1
        self.assertEqual(self.load()[0]['pump'], (2.1, 1.8, 0.))

    def test_exact_control_proximity_boundary_remains_valid(self):
        self.config['inspection_poses']['pump']['goal_xy_yaw'][0] = 1.95
        self.assertEqual(self.load()[0]['pump'], (1.95, 1.8, 0.))
