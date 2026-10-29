#!/usr/bin/env python3
"""Read-only native recorder controls against the installed Orbital workspace.

Stage this driver in a bounded disposable cgroup. Negative controls alter only
this observer's TF query time; no live publisher, service or controller changes.
"""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path('/home/jensen/.goske/codex/orbital-mechanic')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True, help='Unused simple prefix for three retained traces')
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID')!='42'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST'
            or os.environ.get('GZ_PARTITION')!='orbital-maintenance'):
        parser.error('Requires the installed localhost Orbital simulation')
    sys.path.insert(0,str(ROOT))
    from orbital.ros_trajectory import run,runtime_snapshot
    from orbital.trajectory import TraceWriter,verify_trace
    from orbital.collision_evaluation import evaluate_trace
    from rclpy.time import Time
    from tf2_ros import Buffer

    before = runtime_snapshot(ROOT)
    original = Buffer.lookup_transform
    results = []

    class ControlWriter(TraceWriter):
        def __init__(self,mode,offset):
            super().__init__(ROOT,args.prefix+'-'+mode)
            self.mode,self.offset = mode,offset

        def finish(self,complete,details):
            return super().finish(complete,{**details,'native_timing_control':{
                'mode':self.mode,'query_offset_ns':self.offset,
                'scope':'observer query fault only; live TF and control publishers unchanged'}})

    try:
        for mode,offset,expected in (
            ('normal',0,None),
            ('wrong-time',-10000000,'Localization transform is not at the requested truth time/frame'),
            ('unavailable',60000000000,'Recorder warmup timed out')):
            def lookup(buffer,target,source,stamp,*a,**kw):
                shifted = Time(nanoseconds=stamp.nanoseconds+offset,clock_type=stamp.clock_type)
                return original(buffer,target,source,shifted,*a,**kw)
            Buffer.lookup_transform = lookup if offset else original
            writer = ControlWriter(mode,offset)
            result,code = run(ROOT,writer,duration=2.,warmup=5. if not offset else 3.)
            row = {'mode':mode,'exit':code,'path':str(writer.path),'complete':result['complete'],
                   'error':result.get('error'),'records':result['records']}
            results.append(row)
            if expected is None:
                assert code==0 and result['complete'],row
                verified = verify_trace(writer.path)
                observation = verified['observation']
                assert observation['capture_samples']>=2
                assert observation['baseline']['command_sequence']==observation['final_command_sequence']
                assert observation['baseline']['forwarded_commands']==observation['final_forwarded_commands']
                evaluation = evaluate_trace(writer.path)
                assert evaluation['sampled_clear'],evaluation
                row.update(observation=observation,sampled_clear=True)
            else:
                assert code==1 and not result['complete'] and expected in result.get('error',''),row
                assert result['records']>0 and not (writer.path/'ready.json').exists(),row
                try:
                    verify_trace(writer.path)
                except ValueError as error:
                    assert 'incomplete' in str(error),error
                else:
                    raise AssertionError('Negative trace passed the integrity gate')
        after = runtime_snapshot(ROOT)
        assert before==after,'Installed source or service identity changed'
        value = {'passed':True,'controls':results,'runtime':after,
                 'scope':'native idle capture and observer query refusals; no physical loss-stop or route acceptance'}
    except Exception as error:
        value = {'passed':False,'controls':results,'error':str(error)}
    finally:
        Buffer.lookup_transform = original
    print(json.dumps(value,indent=2,allow_nan=False))
    return 0 if value['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
