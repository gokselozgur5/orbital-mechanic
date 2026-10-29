#!/usr/bin/env python3
"""Verify source binding on a disposable Thor copy, with one read-only graph gate."""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    if not os.environ.get('TMUX') or not ROOT.is_relative_to('/home/jensen/.goske/codex/orbital-mechanic/runtime'):
        raise RuntimeError('This mutation probe requires a disposable copy under Thor Orbital runtime')
    output=ROOT/'runtime/manifest-proof'
    output.mkdir(parents=True)
    manifest=output/'source.json'
    graph=output/'graph.json'
    script=str(ROOT/'scripts/navigation-manifest.py')
    def run(arguments, filename, *, failure=None):
        result=subprocess.run([sys.executable,*arguments],capture_output=True,timeout=30)
        (output/filename).write_bytes(result.stdout)
        (output/(filename+'.stderr')).write_bytes(result.stderr)
        if failure:
            assert result.returncode!=0 and failure in result.stderr.decode(), result.stderr.decode()
        else:
            assert result.returncode==0, result.stderr.decode()
        return result
    run([script],'source.json')
    run([str(ROOT/'scripts/check-navigation-graph.py'),'--phase','before'],'graph.json')
    result=run([script,'--verify',str(manifest),'--evidence',str(graph)],'binding.json')
    binding=json.loads(result.stdout)
    config=ROOT/'config/nav2-controller.yaml'
    original=config.read_bytes()
    try:
        assert b'desired_linear_vel: 0.15' in original
        config.write_bytes(original.replace(b'desired_linear_vel: 0.15',b'desired_linear_vel: 0.19'))
        run([script,'--verify',str(manifest)],'changed-config.json',failure='source or installation changed')
    finally:
        config.write_bytes(original)
    run([script,'--verify',str(manifest)],'restored.json')
    failed=output/'failed-gate.json'
    failed.write_text(json.dumps({'passed':False,'scope':'deliberately failed fixture'}))
    run([script,'--verify',str(manifest),'--evidence',str(failed)],'rejected-gate.json',failure='failed or incomplete gate')
    captured=json.loads(manifest.read_text())
    report={'passed':True,'manifest_identity':binding['manifest_identity'],
            'source_files':len(captured['content']['source_sha256']),
            'installed_artifacts':len(captured['content']['installed_sha256']),
            'checks':['live_readonly_graph_bound','changed_controller_config_rejected',
                      'restored_bytes_match','failed_gate_binding_rejected'],
            'scope':'source/installed artifact identity; no live deployment or library loading claim'}
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
