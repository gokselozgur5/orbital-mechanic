import copy
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO

from orbital.map_binding import GENERATOR_SOURCES, validate_bundle, validate_grid

ROOT = Path(__file__).resolve().parents[1]


class MapBindingTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'runtime').mkdir(exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT/'runtime')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        for name in ('config/known-map.json', 'worlds/maintenance.sdf', *GENERATOR_SOURCES):
            target = self.root/name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/name, target)
        spec = importlib.util.spec_from_file_location('map_generator_fixture', self.root/'scripts/generate-known-map.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.generator = module
        output = StringIO()
        with redirect_stdout(output):
            module.generate()
        self.directory = Path(json.loads(output.getvalue())['directory'])
        self.bundle = validate_bundle(self.root, self.directory/'map.yaml')
        self.payload = copy.deepcopy(self.bundle['grid'])

    def test_current_artifacts_and_live_grid_match(self):
        validate_grid(self.bundle, self.payload)
        self.payload['resolution'] = 0.05000000074505806  # ROS float32
        validate_grid(self.bundle, self.payload)

    def test_changed_config_rejects_old_map_even_with_identical_pixels(self):
        path = self.root/'config/known-map.json'
        config = json.loads(path.read_text())
        config['inspection_poses']['pump']['goal_xy_yaw'][0] = 1.9
        path.write_text(json.dumps(config))
        with self.assertRaisesRegex(ValueError, 'revision'):
            validate_bundle(self.root, self.directory/'map.yaml')

    def test_generator_refuses_invalid_control_before_writing_artifacts(self):
        path = self.root/'config/known-map.json'
        config = json.loads(path.read_text())
        config['inspection_poses']['pump']['goal_xy_yaw'][0] = 1.9
        path.write_text(json.dumps(config))
        before = sorted((self.root/'runtime/maps').iterdir())
        with self.assertRaisesRegex(ValueError, 'proximity'):
            self.generator.generate()
        self.assertEqual(sorted((self.root/'runtime/maps').iterdir()), before)

    def test_geometry_validator_change_rejects_old_map(self):
        path = self.root/'orbital/inspection_geometry.py'
        path.write_bytes(path.read_bytes()+b'\n# changed validator identity\n')
        with self.assertRaisesRegex(ValueError, 'revision'):
            validate_bundle(self.root, self.directory/'map.yaml')

    def test_tampered_geometry_report_rejected(self):
        path = self.directory/'manifest.json'
        manifest = json.loads(path.read_bytes())
        manifest['inspection_geometry']['pump']['control_margin_m'] = 1.
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, 'manifest'):
            validate_bundle(self.root, self.directory/'map.yaml')

    def test_tampered_image_yaml_and_manifest_rejected(self):
        for name in ('map.pgm', 'map.yaml', 'manifest.json'):
            path = self.directory/name
            original = path.read_bytes()
            try:
                path.write_bytes(original+b'\nchanged')
                with self.subTest(artifact=name), self.assertRaises(ValueError):
                    validate_bundle(self.root, self.directory/'map.yaml')
            finally:
                path.write_bytes(original)

    def test_wrong_live_cells_origin_frame_or_dimensions_rejected(self):
        variants = [('data', [100]+self.payload['data'][1:]), ('origin', [0.,0.,0.,0.,0.,0.,1.]),
                    ('frame', 'odom'), ('width', self.payload['width']-1), ('resolution', float('nan'))]
        for field, value in variants:
            payload = copy.deepcopy(self.payload)
            payload[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_grid(self.bundle, payload)

    def test_symlinked_artifact_rejected(self):
        path = self.directory/'map.pgm'
        data = path.read_bytes()
        path.unlink()
        other = self.root/'other.pgm'
        other.write_bytes(data)
        path.symlink_to(other)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            validate_bundle(self.root, self.directory/'map.yaml')
