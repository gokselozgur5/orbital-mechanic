#!/usr/bin/env python3
"""Generate ignored map artifacts from the versioned world and map contract."""
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.navigation_configuration import parse_waypoint_contract
from orbital.inspection_geometry import validate_inspection_map
from orbital.map_binding import generator_digest, read_source


def generate():
    config_bytes=read_source(ROOT,'config/known-map.json')
    config,poses=parse_waypoint_contract(config_bytes)
    inspection=validate_inspection_map(config,poses,ROOT)
    grid=inspection.grid
    source_sha=hashlib.sha256(read_source(ROOT,config['world'])).hexdigest()
    config_sha=hashlib.sha256(config_bytes).hexdigest()
    generator_sha=generator_digest(ROOT)
    identity=hashlib.sha256((source_sha+config_sha+generator_sha).encode()).hexdigest()[:16]
    output=ROOT/'runtime/maps'/identity
    image=grid.pgm()
    yaml=(f'image: map.pgm\nresolution: {grid.resolution}\n'
          f'origin: [{grid.origin[0]}, {grid.origin[1]}, 0.0]\n'
          'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n').encode()
    manifest={'source_sha256':source_sha,'config_sha256':config_sha,'generator_sha256':generator_sha,
              'image_sha256':hashlib.sha256(image).hexdigest(),
              'map_yaml_sha256':hashlib.sha256(yaml).hexdigest(),
              'width':grid.width,'height':grid.height,'frame':config['frame'],
              'reachable_poses':list(poses), 'inspection_geometry':inspection.checks,
              'validation_scope':'offline arrival clearance, control geometry and inflated reachability; not live navigation'}
    artifacts={'map.pgm':image,'map.yaml':yaml,'manifest.json':(json.dumps(manifest,indent=2)+'\n').encode()}
    output.mkdir(parents=True,exist_ok=True)
    for name,data in artifacts.items():
        target=output/name
        if target.exists() and target.read_bytes()!=data:raise ValueError(f'Existing artifact differs: {target}')
        if not target.exists():target.write_bytes(data)
    print(json.dumps({'directory':str(output),**manifest},indent=2))


if __name__=='__main__':generate()
