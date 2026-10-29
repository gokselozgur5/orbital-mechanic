import copy
from pathlib import Path
import tempfile
import unittest

from orbital.navigation_manifest import REQUIRED,capture,digest,verify


class NavigationManifestTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)/'source';self.prefix=Path(self.temporary.name)/'ros'
        for name in REQUIRED:
            p=self.root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('fixture source '+name)
        p=self.prefix/'lib/libnav2_fixture.so';p.parent.mkdir(parents=True);p.write_bytes(b'ELF fixture')
        self.compat={'artifact_preflight_passed':True,'errors':[],'versions':{'fixture':'1.3.13'},
                     'artifact_sha256':{'lib/libnav2_fixture.so':digest(p.read_bytes())}}

    def manifest(self):return capture(self.root,self.prefix,self.compat)

    def test_content_identity_ignores_runtime_outputs(self):
        before=self.manifest()
        (self.root/'runtime').mkdir();(self.root/'runtime/private.log').write_text('not source')
        self.assertEqual(verify(before,self.manifest()),before['identity'])

    def test_source_change_is_detected(self):
        before=self.manifest();(self.root/'scripts/navigation-service.py').write_text('changed')
        with self.assertRaisesRegex(ValueError,'changed'):verify(before,self.manifest())

    def test_native_auditor_changes_identity(self):
        for name in ('native/tf_authority/audit.cpp', 'native/tf_authority/CMakeLists.txt'):
            p = self.root/name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text('native fixture')
        for name in ('native/tf_authority/audit.cpp', 'native/tf_authority/CMakeLists.txt'):
            with self.subTest(source=name):
                before = self.manifest()
                (self.root/name).write_text('different auditor or build definition')
                with self.assertRaisesRegex(ValueError, 'changed'):
                    verify(before, self.manifest())

    def test_missing_native_auditor_fails(self):
        (self.root/'native/tf_authority/audit.cpp').unlink()
        with self.assertRaises(FileNotFoundError):
            self.manifest()

    def test_installed_change_after_preflight_is_rejected(self):
        (self.prefix/'lib/libnav2_fixture.so').write_bytes(b'changed binary')
        with self.assertRaisesRegex(ValueError,'during capture'):self.manifest()

    def test_additional_bt_library_changes_identity(self):
        before=self.manifest();(self.prefix/'lib/libnav2_extra_bt_node.so').write_bytes(b'extra plugin')
        with self.assertRaisesRegex(ValueError,'changed'):verify(before,self.manifest())

    def test_missing_required_source_fails(self):
        (self.root/'config/robot.urdf').unlink()
        with self.assertRaises(FileNotFoundError):self.manifest()

    def test_source_symlink_escape_fails(self):
        p=self.root/'config/robot.urdf';p.unlink();p.symlink_to(self.prefix/'lib/libnav2_fixture.so')
        with self.assertRaisesRegex(ValueError,'escapes'):self.manifest()

    def test_manifest_tamper_or_failed_preflight_fails(self):
        current=self.manifest();tampered=copy.deepcopy(current)
        tampered['content']['nav2_versions']['fixture']='different'
        with self.assertRaisesRegex(ValueError,'invalid'):verify(tampered,current)
        self.compat['artifact_preflight_passed']=False
        with self.assertRaisesRegex(ValueError,'preflight'):self.manifest()
