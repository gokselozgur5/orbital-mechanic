#!/usr/bin/env python3
"""Read-only native TF audit bound to unchanged source and executable bytes.

Build with build-tf-authority.sh first. This is a bounded sampled audit, not a
continuous interlock or a substitute for current localization health.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from orbital.navigation_manifest import capture, digest, verify


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=6.0)
    args = parser.parse_args()
    if not 2 <= args.seconds <= 20:
        parser.error('Observation must be between 2 and 20 seconds')
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('Requires live Orbital localhost domain42; fixtures use check-tf-authority.py')
    spec = importlib.util.spec_from_file_location('nav2_compatibility', ROOT/'scripts/check-nav2-compatibility.py')
    compatibility = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compatibility)
    prefix = Path('/opt/ros/jazzy')
    before = capture(ROOT, prefix, compatibility.inspect_install(prefix))
    binary = ROOT/'runtime/tf-authority-build/orbital_tf_authority'
    binary_hash = digest(binary.read_bytes())
    run = subprocess.run([str(binary), '--seconds', str(args.seconds)], text=True,
                         capture_output=True, timeout=args.seconds+10, check=False)
    report = json.loads(run.stdout)
    after = capture(ROOT, prefix, compatibility.inspect_install(prefix))
    identity = verify(before, after)
    if binary_hash != digest(binary.read_bytes()):
        raise RuntimeError('Auditor executable changed during observation')
    report['binding'] = {'manifest': before, 'manifest_identity': identity,
                         'executable_sha256': binary_hash,
                         'scope': 'before/after byte identity; not reproducible compilation or complete dependency closure'}
    report['native_exit_code'] = run.returncode
    report['native_stderr'] = run.stderr
    report['passed'] = run.returncode == 0 and report.get('passed') is True
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
