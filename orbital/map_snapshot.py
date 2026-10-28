"""Prepare expensive map geometry before starting time-sensitive ROS observation."""
from dataclasses import dataclass
import json
from pathlib import Path

from orbital.map_binding import GENERATOR_SOURCES, sha, validate_bundle

MAX_FILE_BYTES = 2*1024*1024


def _read(root, relative, limit=MAX_FILE_BYTES):
    path = root/relative
    if (not path.resolve().is_relative_to(root) or path.is_symlink()
            or any(p.is_symlink() for p in path.parents if p != root and p.is_relative_to(root))
            or not path.is_file()):
        raise ValueError('Map snapshot requires regular files under its root')
    with path.open('rb') as stream:
        data = stream.read(limit+1)
    if len(data)>limit:
        raise ValueError('Map snapshot file exceeds its bound')
    return data


@dataclass(frozen=True)
class MapSnapshot:
    root: Path
    yaml_path: Path
    fingerprints: tuple
    _bundle: bytes

    @classmethod
    def prepare(cls, registry):
        root = registry.root
        names = tuple(dict.fromkeys((*GENERATOR_SOURCES,'config/known-map.json',registry.world_path)))
        content = {name:_read(root,name) for name in names}
        config_hash,world_hash = sha(content['config/known-map.json']),sha(content[registry.world_path])
        if (config_hash,world_hash)!=(registry.config_sha256,registry.world_sha256):
            raise ValueError('Map snapshot registry source changed')
        generator_hash = sha(b''.join(content[name] for name in GENERATOR_SOURCES))
        # Only derive the candidate path here. The existing full validator proves
        # its actual revision, source, geometry, manifest and every artifact byte.
        revision = sha((world_hash+config_hash+generator_hash).encode())[:16]
        directory = root/'runtime/maps'/revision
        for name in ('map.pgm','map.yaml','manifest.json'):
            relative = str((directory/name).relative_to(root))
            content[relative] = _read(root,relative)
        bundle = validate_bundle(root,directory/'map.yaml')
        value = cls(root,directory/'map.yaml',
                    tuple((name,len(data),sha(data)) for name,data in content.items()),
                    json.dumps(bundle,sort_keys=True,separators=(',',':'),allow_nan=False).encode())
        value.verify_current(root,value.yaml_path)
        return value

    def verify_current(self, root, yaml_path):
        if Path(root).resolve()!=self.root or Path(yaml_path).absolute()!=self.yaml_path:
            raise ValueError('Map snapshot root or loaded revision mismatch')
        for name,size,digest in self.fingerprints:
            data = _read(self.root,name,size)
            if len(data)!=size or sha(data)!=digest:
                raise ValueError('Map snapshot source or artifact changed')

    def bundle(self):
        return json.loads(self._bundle)  # Independent copy; no mutable cached grid escapes.
