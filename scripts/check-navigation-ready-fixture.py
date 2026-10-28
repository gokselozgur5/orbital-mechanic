#!/usr/bin/env python3
"""Run real readiness/Navigator/supervisor against isolated base-system fixtures.

Requires its own systemd cgroup so detached test descendants cannot survive.
Only the disposable copy's controller configuration is changed and restored.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    if (os.environ.get('ROS_DOMAIN_ID')!='84' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST'
            or not os.environ.get('INVOCATION_ID')
            or not ROOT.is_relative_to('/home/jensen/.goske/codex/orbital-mechanic/runtime')):
        raise RuntimeError('Requires a disposable Thor copy and dedicated systemd proof cgroup on domain84')
    output=ROOT/'runtime/readiness-proof';output.mkdir(parents=True)
    children=[];streams=[];report={'passed':False,'checks':[]}
    def spawn(command,name):
        stream=(output/(name+'.log')).open('w');streams.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child);return child
    def run(command,name,*,success=True):
        result=subprocess.run(command,capture_output=True,timeout=50)
        (output/(name+'.json')).write_bytes(result.stdout)
        (output/(name+'.stderr')).write_bytes(result.stderr)
        assert (result.returncode==0)==success, (name,result.stderr.decode(),result.stdout.decode()[-1500:])
        data=json.loads(result.stdout)
        assert data['passed']==success
        report['checks'].append({'case':name,'passed':data['passed'],
                                 'lifecycle_commands':data['lifecycle_commands'],
                                 'states':data.get('final_states'),'error':data.get('error')})
        return data
    source=ROOT/'config/nav2-controller.yaml';original=source.read_bytes()
    try:
        fixture=spawn([sys.executable,str(ROOT/'scripts/check-navigation-graph-fixtures.py'),'--hold'],'base')
        end=time.monotonic()+12
        while not (ROOT/'runtime/graph-fixture-ready').exists():
            assert fixture.poll() is None, 'Base fixture exited'
            if time.monotonic()>end:raise RuntimeError('Base fixture did not initialize')
            time.sleep(.05)
        spawn([sys.executable,str(ROOT/'scripts/localization-monitor.py')],'monitor')
        spawn([sys.executable,str(ROOT/'scripts/navigation-supervisor.py')],'supervisor')
        spawn(['/opt/ros/jazzy/bin/ros2','launch',str(ROOT/'scripts/navigation.launch.py')],'nav2')
        manifest=output/'source.json'
        result=subprocess.run([sys.executable,str(ROOT/'scripts/navigation-manifest.py')],capture_output=True,timeout=30,check=True)
        manifest.write_bytes(result.stdout)
        command=[sys.executable,str(ROOT/'scripts/check-navigation-ready.py'),'--manifest',str(manifest)]
        configured=run([*command,'--configure'],'configure')
        assert set(configured['final_states'].values())=={2}
        active=run([*command,'--activate'],'activate')
        assert set(active['final_states'].values())=={3} and active['supervisor_velocity_requests']==0
        observed=run(command,'repeat_readonly')
        assert not observed['lifecycle_commands'] and observed['supervisor_velocity_requests']==0
        source.write_bytes(original.replace(b'desired_linear_vel: 0.15',b'desired_linear_vel: 0.19'))
        rejected=run([*command,'--activate'],'changed_source',success=False)
        assert 'Source manifest verification failed' in rejected['error']
        assert rejected['guard_stop_ack'] and rejected['lifecycle_pause_ack']
        source.write_bytes(original)
        recovered=run([*command,'--activate'],'explicit_recovery')
        assert set(recovered['initial_states'].values())=={2}
        assert set(recovered['final_states'].values())=={3} and recovered['supervisor_velocity_requests']==0
        report.update(passed=True,manifest_identity=active['manifest_identity'],
                      scope='real Nav2 and supervisor lifecycle on synthetic base graph; no Gazebo/drive')
    except Exception as exc:
        report['error']=str(exc)
    finally:
        source.write_bytes(original)
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
        for child in reversed(children):
            try:child.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=2)
        for stream in streams:stream.close()
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
