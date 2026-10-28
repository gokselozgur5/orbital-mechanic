"""Bind generated map artifacts and observed occupancy cells to versioned source.

A map snapshot is not proof of AMCL's private memory or continuous enforcement.
"""
import hashlib
import json
import math
from pathlib import Path

from orbital.mapping import FREE, OCCUPIED
from orbital.navigation_configuration import parse_waypoint_contract
from orbital.inspection_geometry import validate_inspection_map

# Every repository module used to generate/validate these map artifacts is part
# of their revision. Include this file because it owns the identity calculation.
GENERATOR_SOURCES = ('orbital/mapping.py', 'orbital/path_clearance.py',
                     'orbital/machines.py', 'orbital/navigation_configuration.py',
                     'orbital/inspection_geometry.py', 'orbital/map_binding.py',
                     'scripts/generate-known-map.py')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_source(root, relative):
    path = root/relative
    if not path.resolve().is_relative_to(root):
        raise ValueError('Map source escapes root')
    if path.is_symlink() or any(p.is_symlink() for p in path.parents if p != root and p.is_relative_to(root)):
        raise ValueError('Map symlink is not permitted')
    return path.read_bytes()


def generator_digest(root):
    return sha(b''.join(read_source(root, name) for name in GENERATOR_SOURCES))


def validate_bundle(root, map_yaml):
    root = Path(root).resolve()
    config_bytes = read_source(root, 'config/known-map.json')
    config, poses = parse_waypoint_contract(config_bytes)
    world = read_source(root, config['world'])
    source_sha, config_sha = sha(world), sha(config_bytes)
    generator_sha = generator_digest(root)
    revision = sha((source_sha+config_sha+generator_sha).encode())[:16]
    expected = root/'runtime/maps'/revision/'map.yaml'
    if Path(map_yaml).absolute() != expected:
        raise ValueError('Loaded map revision differs from versioned source')
    inspection = validate_inspection_map(config, poses, root)
    grid = inspection.grid
    image = grid.pgm()
    yaml = (f'image: map.pgm\nresolution: {grid.resolution}\n'
            f'origin: [{grid.origin[0]}, {grid.origin[1]}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n').encode()
    directory = expected.parent
    for name, wanted in (('map.pgm', image), ('map.yaml', yaml)):
        if read_source(root, str((directory/name).relative_to(root))) != wanted:
            raise ValueError('Map artifact differs: '+name)
    manifest_bytes = read_source(root, str((directory/'manifest.json').relative_to(root)))
    manifest = json.loads(manifest_bytes)
    expected_fields = {'source_sha256':source_sha, 'config_sha256':config_sha, 'generator_sha256':generator_sha,
                       'image_sha256':sha(image), 'map_yaml_sha256':sha(yaml),
                       'width':grid.width, 'height':grid.height, 'frame':'map',
                       'reachable_poses':list(poses), 'inspection_geometry':inspection.checks}
    if any(manifest.get(k) != v for k, v in expected_fields.items()):
        raise ValueError('Map manifest differs from source')
    return {'revision':revision, 'config_sha256':config_sha, 'image_sha256':sha(image),
            'yaml_sha256':sha(yaml), 'manifest_sha256':sha(manifest_bytes),
            'grid':{'frame':'map', 'width':grid.width, 'height':grid.height,
                    'resolution':grid.resolution, 'origin':[*grid.origin,0.,0.,0.,0.,1.],
                    'data':[0 if c == FREE else 100 if c == OCCUPIED else -1 for c in grid.cells]}}


def validate_grid(bundle, observed):
    wanted = bundle['grid']
    if any(observed.get(k) != wanted[k] for k in ('frame','width','height','data')):
        raise ValueError('Observed map frame, dimensions or cells differ')
    values = [observed.get('resolution'), *observed.get('origin', [])]
    expected = [wanted['resolution'], *wanted['origin']]
    if (len(values) != len(expected) or any(type(v) not in (int,float) or not math.isfinite(v)
            or abs(v-e) > 1e-8 for v, e in zip(values, expected))):
        raise ValueError('Observed map geometry differs')
