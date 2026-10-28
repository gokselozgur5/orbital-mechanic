"""Content identity for navigation source/config and installed Nav2 artifacts.

This records deployment bytes, not a signed release, transitive OS lockfile or
proof of which libraries a running process has actually loaded.
"""
import hashlib
import json
from pathlib import Path

REQUIRED = (
    'config/known-map.json','config/bridge.yaml','config/robot.urdf','config/localization.yaml',
    'config/fastdds-local.xml','config/nav2-controller.yaml','config/nav2-planner.yaml',
    'config/nav2-navigator.yaml','config/navigate-guarded.xml','worlds/maintenance.sdf',
    'scripts/start-navigation.sh','scripts/navigation-service.py','scripts/navigation-supervisor.py',
    'scripts/navigation.launch.py','scripts/localization-monitor.py','scripts/ros-env.sh',
    'scripts/check-navigation-graph.py','scripts/check-nav2-compatibility.py',
    'scripts/navigation-manifest.py','orbital/navigation_manifest.py',
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def content_id(payload):
    return digest(json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False).encode())


def file_hash(root, relative, *, symlinks=False):
    path = root/relative
    if Path(relative).is_absolute() or not path.resolve().is_relative_to(root):
        raise ValueError('Artifact escapes its root: '+relative)
    if not symlinks and (path.is_symlink() or any(p.is_symlink() for p in path.parents if p!=root and p.is_relative_to(root))):
        raise ValueError('Source symlinks are not permitted: '+relative)
    return digest(path.read_bytes())


def capture(root, prefix, compatibility):
    root, prefix = Path(root).resolve(), Path(prefix).resolve()
    if compatibility.get('artifact_preflight_passed') is not True or compatibility.get('errors'):
        raise ValueError('Installed artifact preflight must pass')
    paths = set(REQUIRED)
    for directory,patterns in (('orbital',('*.py',)),('scripts',('*.py','*.sh'))):
        for pattern in patterns:
            paths.update(str(p.relative_to(root)) for p in (root/directory).glob(pattern))
    sources = {p:file_hash(root,p) for p in sorted(paths)}
    artifacts = {}
    for name,expected in compatibility['artifact_sha256'].items():
        actual = file_hash(prefix,name,symlinks=True)
        if actual != expected:
            raise ValueError('Installed artifact changed during capture: '+name)
        artifacts[name] = actual
    # Includes costmap/checker/navigator/BT plugins beyond the initial artifact
    # gate's two exported controller/planner plugin classes.
    for path in sorted((prefix/'lib').glob('libnav2*.so*')):
        name = str(path.relative_to(prefix))
        artifacts[name] = file_hash(prefix,name,symlinks=True)
    if not artifacts:
        raise ValueError('No installed artifacts captured')
    payload = {'schema_version':1,'source_sha256':sources,'installed_sha256':artifacts,
               'nav2_versions':compatibility['versions']}
    return {'identity':content_id(payload),'content':payload,
            'source_root':str(root),'install_prefix':str(prefix),
            'scope':'source/config and installed Nav2 bytes; not runtime loading or complete OS dependency closure'}


def verify(expected, actual):
    for value in (expected,actual):
        if value.get('identity') != content_id(value['content']):
            raise ValueError('Manifest content identity is invalid')
        if value['content'].get('schema_version') != 1:
            raise ValueError('Unsupported manifest schema')
    if expected['identity'] != actual['identity']:
        raise ValueError('Navigation source or installation changed')
    return expected['identity']
