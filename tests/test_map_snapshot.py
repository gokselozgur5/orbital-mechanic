import unittest
from unittest.mock import patch

from orbital.machine_registry import MachineRegistry
from orbital.map_snapshot import MapSnapshot
import orbital.map_snapshot as snapshot_module
import test_map_binding as map_fixture


class MapSnapshotTests(unittest.TestCase):
    def setUp(self):
        map_fixture.MapBindingTests.setUp(self)

    def prepare(self):
        return MapSnapshot.prepare(MachineRegistry.load(self.root))

    def test_prepared_map_matches_full_validator_and_returns_independent_grids(self):
        value=self.prepare();self.assertEqual(value.bundle(),self.bundle)
        copy=value.bundle();copy['grid']['data'][0]=12345
        self.assertEqual(value.bundle(),self.bundle)
        value.verify_current(self.root,self.directory/'map.yaml')

    def test_every_source_and_artifact_fingerprint_is_checked(self):
        value=self.prepare()
        for name,_,_ in value.fingerprints:
            path=self.root/name;before=path.read_bytes()
            try:
                path.write_bytes(before+b'\n')
                with self.subTest(name=name),self.assertRaises(ValueError):value.verify_current(self.root,value.yaml_path)
            finally:path.write_bytes(before)
        value.verify_current(self.root,value.yaml_path)

    def test_source_change_during_preparation_is_not_adopted_as_new_identity(self):
        original=snapshot_module.validate_bundle
        def changed(*args):
            result=original(*args);path=self.root/'orbital/mapping.py'
            path.write_bytes(path.read_bytes()+b'\n');return result
        with patch.object(snapshot_module,'validate_bundle',changed),self.assertRaises(ValueError):self.prepare()

    def test_linked_artifact_wrong_root_and_wrong_loaded_revision_are_refused(self):
        value=self.prepare()
        with self.assertRaises(ValueError):value.verify_current(self.root,self.root/'wrong.yaml')
        with self.assertRaises(ValueError):value.verify_current(self.root/'other',value.yaml_path)
        path=self.directory/'map.pgm';data=path.read_bytes();target=self.root/'alternate.pgm';target.write_bytes(data)
        path.unlink();path.symlink_to(target)
        with self.assertRaises(ValueError):value.verify_current(self.root,value.yaml_path)

    def test_registry_source_change_before_preparation_is_refused(self):
        registry=MachineRegistry.load(self.root)
        path=self.root/registry.world_path;path.write_bytes(path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError,'registry source changed'):MapSnapshot.prepare(registry)

    def test_tampered_cells_cannot_be_prepared_from_an_existing_manifest(self):
        path=self.directory/'map.pgm';data=path.read_bytes();path.write_bytes(data[:-1]+bytes([255-data[-1]]))
        with self.assertRaises(ValueError):self.prepare()
