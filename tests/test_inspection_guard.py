import shutil
from types import SimpleNamespace as NS
import unittest

from orbital.ros_inspection import LiveGuard, OWNER, TOPICS
from proximity_fixture import Harness
import test_map_binding as map_fixture

ROOT = map_fixture.ROOT


class GuardTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)
        for name in ('config/robot.urdf', 'config/bridge.yaml'):
            shutil.copyfile(ROOT/name, self.root/name)
        self.h = Harness(self)
        node = self.h.modules['rclpy'].create_node()
        actual = node.get_publishers_info_by_topic
        self.foreign = False
        def publishers(topic):
            if topic in TOPICS:
                peers = [NS(node_name=OWNER, node_namespace='/', topic_type='std_msgs/msg/String', endpoint_gid=[9]*24)]
                return peers*2 if self.foreign else peers
            return actual(topic)
        node.get_publishers_info_by_topic = publishers
        self.node = node
        self.guard = LiveGuard(self.root, node, self.h.modules['tf2_ros'].Buffer(), NS(verify=lambda: None),
            now=lambda: self.h.wall, time_type=lambda stamp: NS(nanoseconds=stamp), tf_error=LookupError)
        self.assertTrue(self.guard.ready()); self.guard.begin('pump')
        node.create_subscription(None, '/navigation/health', self.guard.health, None)
        node.create_subscription(None, '/odom', self.guard.odometry, None)

    def settle(self):
        measured = None
        for _ in range(32):
            self.h.modules['rclpy'].spin_once(self.node, .02)
            measured = self.guard.sample()
        self.assertIsNotNone(measured)
        return measured

    def test_live_guard_requires_its_own_settling_and_exact_pose(self):
        self.assertIsNone(self.guard.sample())
        measured = self.settle()
        self.assertTrue(measured['within_settled_inspection_pose'])
        self.assertEqual(measured['pose_stamp_ns'], self.h.sim-100000000)
        self.h.pose = (4., 1.8, 0.)
        with self.assertRaisesRegex(ValueError, 'inspection_pose_required'): self.guard.sample()

    def test_motion_invalidates_even_if_the_next_odometry_is_stopped(self):
        self.settle(); self.h.moving = True
        self.h.modules['rclpy'].spin_once(self.node, .02)
        self.h.moving = False
        for _ in range(30): self.h.modules['rclpy'].spin_once(self.node, .02)
        with self.assertRaises(ValueError): self.guard.sample()

    def test_changed_sources_clock_and_competing_server_remain_unavailable(self):
        for change in ('changed_gid', 'clock_mode_changed', 'foreign'):
            with self.subTest(change=change):
                self.setUp(); self.settle()
                if change == 'foreign': self.foreign = True
                else: setattr(self.h, change, True)
                self.assertFalse(self.guard.ready())
                if change == 'foreign': self.foreign = False
                else: setattr(self.h, change, False)
                self.assertFalse(self.guard.ready())

    def test_registry_map_and_camera_file_changes_stop_current_inspection(self):
        for name in ('config/known-map.json', 'config/robot.urdf', 'config/bridge.yaml',
                     str(self.directory.relative_to(self.root)/'map.pgm')):
            file = self.root/name
            original = file.read_bytes()
            try:
                file.write_bytes(original+b'\n')
                self.assertFalse(self.guard.ready())
            finally:
                file.write_bytes(original)
            self.assertTrue(self.guard.broken)
            self.guard.broken = False  # New test case only; production requires restart.

    def test_missing_transform_is_pending_only_before_a_completed_pose(self):
        self.h.no_tf = True
        for _ in range(32):
            self.h.modules['rclpy'].spin_once(self.node, .02)
            self.assertIsNone(self.guard.sample())
        self.h.no_tf = False; self.assertIsNotNone(self.guard.sample())
        self.h.no_tf = True
        with self.assertRaises(ValueError): self.guard.sample()


if __name__ == '__main__': unittest.main()
