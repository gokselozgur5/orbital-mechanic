import json
from pathlib import Path
import tempfile
import unittest

from orbital.navigation_configuration import bounded_timeout, load_waypoints, load_waypoint_contract


class NavigationConfigurationTests(unittest.TestCase):
    def load(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'map.json'
            path.write_text(text)
            return load_waypoints(path)

    def test_versioned_poses(self):
        poses = load_waypoints(Path(__file__).resolve().parents[1]/'config/known-map.json')
        self.assertEqual(set(poses), {'pump', 'cooler', 'power_unit'})
        self.assertEqual(poses['pump'], (2., 1.8, 0.))

    def test_invalid_frame_or_shape(self):
        for data in ([], {}, {'frame':'odom', 'inspection_poses':{'pump':{'goal_xy_yaw':[0,0,0]}}},
                     {'frame':'map', 'inspection_poses':{}}, {'frame':'map','inspection_poses':[]}):
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.load(json.dumps(data))

    def test_invalid_pose_and_name(self):
        for value in ([0,0], [True,0,0], [0,float('nan'),0], [0,0,4], ['1',0,0], None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load(json.dumps({'frame':'map','inspection_poses':{'pump':{'goal_xy_yaw':value}}}))
        with self.assertRaises(ValueError):
            self.load('{"frame":"map","inspection_poses":{"../pump":{"goal_xy_yaw":[0,0,0]}}}')

    def test_duplicate_poses_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.load('{"frame":"map","inspection_poses":{"pump":{},"pump":{"goal_xy_yaw":[0,0,0]}}}')

    def test_loaded_contract_retains_original_revision_after_file_edit(self):
        root = Path(__file__).resolve().parents[1]/'runtime'
        root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root) as directory:
            path = Path(directory)/'map.json'
            path.write_text('{"frame":"map","inspection_poses":{"pump":{"goal_xy_yaw":[0,0,0]}}}')
            first, identity = load_waypoint_contract(path)
            path.write_text('{"frame":"map","inspection_poses":{"pump":{"goal_xy_yaw":[1,0,0]}}}')
            changed, new_identity = load_waypoint_contract(path)
            self.assertEqual(first['pump'], (0.,0.,0.))
            self.assertEqual(changed['pump'], (1.,0.,0.))
            self.assertNotEqual(identity, new_identity)

    def test_timeout_validated_before_ros(self):
        self.assertEqual(bounded_timeout('120'),120.)
        for value in ('nan','inf','0','301','bad'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                bounded_timeout(value)
