import copy
from dataclasses import FrozenInstanceError
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from orbital.machine_registry import MachineRegistry, SOURCE_LIMIT

ROOT = Path(__file__).resolve().parents[1]


class RegistryFixture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        (self.root/'config').mkdir()
        (self.root/'worlds').mkdir()
        self.config_path = self.root/'config/known-map.json'
        self.world_path = self.root/'worlds/maintenance.sdf'
        self.config_path.write_bytes((ROOT/'config/known-map.json').read_bytes())
        self.world_path.write_bytes((ROOT/'worlds/maintenance.sdf').read_bytes())
        self.config = json.loads(self.config_path.read_bytes())

    def save_config(self):
        self.config_path.write_text(json.dumps(self.config))


class MachineRegistryTests(RegistryFixture):
    def test_actual_scene_models_controls_and_source_identity(self):
        registry = MachineRegistry.load(self.root)
        catalog = registry.public()
        self.assertEqual(catalog['frame'], 'map')
        self.assertEqual(catalog['world_name'], 'maintenance')
        self.assertEqual(catalog['geometry_source'], 'configured_world')
        self.assertEqual(set(catalog['machines']), {'pump', 'cooler', 'power_unit'})
        for name, y in (('pump', 1.8), ('cooler', 0.), ('power_unit', -1.8)):
            entry = catalog['machines'][name]
            self.assertEqual(entry['model_name'], name)
            self.assertEqual(entry['model_pose_xyz_xyzw'], [3., y, .6, 0., 0., 0., 1.])
            self.assertEqual(entry['control_xy_m'], [2.6, y])
            self.assertEqual(entry['inspection_xy_yaw'], [2., y, 0.])
        content = {k:v for k,v in catalog.items() if k != 'identity'}
        self.assertEqual(catalog['identity'], hashlib.sha256(json.dumps(content,sort_keys=True,separators=(',',':')).encode()).hexdigest())
        self.assertEqual(catalog['source_sha256']['world'], hashlib.sha256(self.world_path.read_bytes()).hexdigest())

    def test_translated_model_and_link_offset_bind_actual_control(self):
        old = MachineRegistry.load(self.root).identity
        world = ET.fromstring(self.world_path.read_bytes())
        model = world.find("world/model[@name='pump']")
        model.find('pose').text = '3.1 1.8 .6 0 0 0'
        link = model.find('link')
        link.set('name', 'pump_body')
        ET.SubElement(link, 'pose').text = '.2 0 0 0 0 0'
        self.world_path.write_bytes(ET.tostring(world))
        self.config['inspection_poses']['pump']['control_xy_m'][0] = 2.9
        self.save_config()
        registry = MachineRegistry.load(self.root)
        self.assertNotEqual(registry.identity, old)
        value = registry.public()['machines']['pump']
        self.assertAlmostEqual(value['collision_center_xyz_m'][0], 3.3)
        self.assertEqual(value['model_pose_xyz_xyzw'][0], 3.1)
        self.assertEqual(value['control_xy_m'][0], 2.9)

    def test_missing_duplicate_dynamic_or_multiple_machine_geometry_is_refused(self):
        source = self.world_path.read_bytes()
        for kind in ('missing', 'duplicate', 'dynamic', 'multiple'):
            world = ET.fromstring(source)
            owner = world.find('world')
            model = owner.find("model[@name='pump']")
            if kind == 'missing':
                owner.remove(model)
            elif kind == 'duplicate':
                owner.append(copy.deepcopy(model))
            elif kind == 'dynamic':
                model.find('static').text = 'false'
            else:
                collision = copy.deepcopy(model.find('link/collision'))
                collision.set('name', 'second')
                model.find('link').append(collision)
            self.world_path.write_bytes(ET.tostring(world))
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                MachineRegistry.load(self.root)

    def test_missing_unknown_and_wrong_control_machine_bindings_are_refused(self):
        original = copy.deepcopy(self.config)
        for kind in ('missing', 'unknown', 'control', 'boolean', 'nan'):
            self.config = copy.deepcopy(original)
            entries = self.config['inspection_poses']
            if kind == 'missing':
                del entries['pump']
            elif kind == 'unknown':
                entries['unknown'] = entries.pop('pump')
            else:
                entries['pump']['control_xy_m'] = {'control':[2.6,0.], 'boolean':[True,1.8], 'nan':[float('nan'),1.8]}[kind]
            self.save_config()
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                MachineRegistry.load(self.root)

    def test_duplicate_config_frame_and_rotated_control_contract_are_refused(self):
        raw = self.config_path.read_bytes()
        for changed in (raw.replace(b'"frame": "map"', b'"frame": "odom"'),
                        raw.replace(b'"frame": "map"', b'"frame": "map", "frame": "map"')):
            self.config_path.write_bytes(changed)
            with self.assertRaises(ValueError):
                MachineRegistry.load(self.root)
        self.config_path.write_bytes(raw)
        self.world_path.write_bytes(self.world_path.read_bytes().replace(b'3 1.8 .6 0 0 0', b'3 1.8 .6 0 0 .2'))
        with self.assertRaisesRegex(ValueError, 'axis-aligned'):
            MachineRegistry.load(self.root)

    def test_sources_are_bounded_and_world_cannot_escape_root(self):
        original = self.config_path.read_bytes()
        self.config_path.write_bytes(b' '*(SOURCE_LIMIT+1))
        with self.assertRaisesRegex(ValueError, 'bound'):
            MachineRegistry.load(self.root)
        self.config_path.write_bytes(original)
        for world in ('../world.sdf', str(ROOT/'worlds/maintenance.sdf')):
            self.config['world'] = world
            self.save_config()
            with self.subTest(world=world), self.assertRaises(ValueError):
                MachineRegistry.load(self.root)

    def test_source_symlink_cannot_bypass_binding(self):
        link = self.root/'worlds/linked.sdf'
        link.symlink_to(self.world_path)
        self.config['world'] = 'worlds/linked.sdf'
        self.save_config()
        with self.assertRaisesRegex(ValueError, 'symlink'):
            MachineRegistry.load(self.root)

    def test_returned_catalog_mutation_cannot_rebind_registry(self):
        registry = MachineRegistry.load(self.root)
        before = registry.public()
        changed = registry.public()
        changed['machines']['pump']['control_xy_m'][0] = 0
        changed['source_sha256']['world'] = '0'*64
        self.assertEqual(registry.public(), before)
        with self.assertRaises(FrozenInstanceError):
            registry.bindings[0].machine_id = 'other'

    def test_changed_source_is_refused_until_explicit_reload(self):
        registry = MachineRegistry.load(self.root)
        registry.verify_current()
        self.config_path.write_bytes(self.config_path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError, 'source changed'):
            registry.verify_current()
        replacement = MachineRegistry.load(self.root)
        self.assertNotEqual(replacement.identity, registry.identity)
        self.world_path.write_bytes(self.world_path.read_bytes()+b'\n')
        with self.assertRaisesRegex(ValueError, 'source changed'):
            replacement.verify_current()

    def test_cli_prints_the_same_public_registry_without_ros(self):
        result = subprocess.run([sys.executable,str(ROOT/'scripts/machine-registry.py')], capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), MachineRegistry.load(ROOT).public())
