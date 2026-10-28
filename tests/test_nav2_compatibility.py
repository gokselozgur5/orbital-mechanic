"""Reject plausible but incompatible installation artifacts before ROS startup."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('compatibility', Path(__file__).resolve().parents[1] / 'scripts/check-nav2-compatibility.py')
compat = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(compat)


class CompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in set(compat.EXECUTABLES) | set(compat.PLUGINS) | {'nav2_util', 'nav2_msgs', 'nav2_costmap_2d'}:
            self.write(f'share/{name}/package.xml', f'<package><name>{name}</name><version>1.3.13</version></package>')
        for name, executable in compat.EXECUTABLES.items():
            self.write(f'lib/{name}/{executable}', b'\x7fELFfixture')
            (self.root / 'lib' / name / executable).chmod(0o755)
        for name, (plugin, base) in compat.PLUGINS.items():
            self.write(f'share/{name}/plugins.xml', f'<library path="{name}"><class type="{plugin}" base_class_type="{base}"/></library>')
            self.write(f'lib/lib{name}.so', b'\x7fELFfixture')
        self.write('include/nav2_util/nav2_util/twist_publisher.hpp', 'enable_stamped_cmd_vel geometry_msgs::msg::TwistStamped')
        self.write('share/nav2_msgs/action/NavigateToPose.action', 'geometry_msgs/PoseStamped pose\nstring behavior_tree\n---\nuint16 error_code\n---\n')

    def write(self, name, value):
        p = self.root / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(value if isinstance(value, bytes) else value.encode())

    def test_complete_artifacts_do_not_claim_runtime_acceptance(self):
        result = compat.inspect_install(self.root)
        self.assertTrue(result['artifact_preflight_passed'], result['errors'])
        self.assertFalse(result['runtime_compatibility_proven'])

    def test_plugin_metadata_without_binary_is_rejected(self):
        (self.root / 'lib/libnav2_navfn_planner.so').unlink()
        self.assertFalse(compat.inspect_install(self.root)['artifact_preflight_passed'])

    def test_mixed_package_versions_are_rejected(self):
        self.write('share/nav2_msgs/package.xml', '<package><name>nav2_msgs</name><version>1.3.12</version></package>')
        self.assertFalse(compat.inspect_install(self.root)['artifact_preflight_passed'])

    def test_missing_stamped_support_is_rejected(self):
        self.write('include/nav2_util/nav2_util/twist_publisher.hpp', 'geometry_msgs::msg::Twist')
        self.assertFalse(compat.inspect_install(self.root)['artifact_preflight_passed'])

    def test_incompatible_goal_is_rejected(self):
        self.write('share/nav2_msgs/action/NavigateToPose.action', 'geometry_msgs/Pose pose\nstring behavior_tree\n---\nuint16 error_code\n---\n')
        self.assertFalse(compat.inspect_install(self.root)['artifact_preflight_passed'])

    def test_empty_installation_is_rejected(self):
        with tempfile.TemporaryDirectory() as empty:
            self.assertFalse(compat.inspect_install(empty)['artifact_preflight_passed'])

    def test_nonexecutable_server_is_rejected(self):
        (self.root / 'lib/nav2_controller/controller_server').chmod(0o644)
        self.assertFalse(compat.inspect_install(self.root)['artifact_preflight_passed'])
