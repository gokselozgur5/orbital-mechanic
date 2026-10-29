#!/usr/bin/env python3
"""Capture deployment identity or bind a fresh gate report to unchanged bytes."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from orbital.navigation_manifest import capture,digest,verify


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,default=Path('/opt/ros/jazzy'))
    parser.add_argument('--verify',type=Path)
    parser.add_argument('--evidence',type=Path,help='Fresh JSON gate output to bind after source revalidation')
    args=parser.parse_args()
    if args.evidence and not args.verify:parser.error('--evidence requires --verify')
    spec=importlib.util.spec_from_file_location('nav2_compatibility',ROOT/'scripts/check-nav2-compatibility.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    current=capture(ROOT,args.prefix,module.inspect_install(args.prefix))
    if args.verify:
        identity=verify(json.loads(args.verify.read_text()),current)
        result={'passed':True,'manifest_identity':identity}
        if args.evidence:
            data=args.evidence.read_bytes()
            evidence=json.loads(data)
            if evidence.get('passed') is not True:raise ValueError('Cannot bind a failed or incomplete gate')
            result.update(evidence_sha256=digest(data),evidence_file=str(args.evidence),
                          scope='gate report bound to before/after source identity; report semantics remain the gate responsibility')
        print(json.dumps(result,indent=2))
    else:
        print(json.dumps(current,indent=2))


if __name__=='__main__':main()
