#!/usr/bin/env python3
"""Exercise the real operator CLI against the simulated rover, including disconnect."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import rclpy

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('probe',ROOT/'scripts/check-motion.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def launch(*args):
    return subprocess.Popen([sys.executable,str(ROOT/'scripts/orbital-operator.py'),*args],
                            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)


def finish(p, child, expected=0):
    end=time.monotonic()+12
    while child.poll() is None and time.monotonic()<end:p.spin(.05)
    if child.poll() is None: child.kill();child.wait();raise AssertionError('operator timed out')
    out,err=child.communicate()
    assert child.returncode==expected,(child.returncode,out,err)
    return out


def main():
    rclpy.init();p=module.Probe();report=[]
    try:
        p.spin(2)
        assert p.odom and math.hypot(*p.position())<.25, 'requires a fresh world near the clear origin'
        finish(p,launch('stop'))
        finish(p,launch('jog','forward'),expected=1)
        report.append({'case':'latched_jog_rejected','passed':True})
        for direction in ['forward','reverse','left','right']:
            finish(p,launch('arm'));before=p.position()
            q=p.odom.pose.pose.orientation;before_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            result=finish(p,launch('jog',direction,'--duration','.7'))
            p.spin(.3)
            assert p.state['latched'] and p.stationary(),(direction,p.state)
            if direction in ('forward','reverse'):
                distance=math.dist(before,p.position());assert .04<distance<.13,(direction,distance)
            else:
                distance=None
                q=p.odom.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                delta=math.atan2(math.sin(yaw-before_yaw),math.cos(yaw-before_yaw))
                assert .1<abs(delta)<.3,(direction,delta)
                assert (delta>0)==(direction=='left'),(direction,delta)
            report.append({'case':direction,'displacement_m':distance,'result':json.loads(result)})
        finish(p,launch('arm'));child=launch('jog','forward','--duration','2')
        end=time.monotonic()+10
        while time.monotonic()<end:
            p.spin(.05)
            if child.poll() is not None:
                out,err=child.communicate();raise AssertionError(('jog exited before disconnect',child.returncode,out,err))
            if p.odom.twist.twist.linear.x>.05: break
        else:
            child.kill();out,err=child.communicate();raise AssertionError(('no movement before disconnect',out,err,p.state))
        start=time.monotonic();before=p.position();child.kill()
        # Keep servicing ROS while the killed client is being reaped. Waiting on
        # its pipes here can consume the observation window without callbacks.
        while time.monotonic()-start<1.2:
            p.spin(.03)
            if p.state['latched'] and p.stationary():break
        assert p.state['latched'] and p.stationary(),('disconnect left movement',p.state,
            p.odom.twist.twist.linear.x,p.odom.twist.twist.angular.z,child.returncode,
            list(p.states)[-15:])
        observed_stop=time.monotonic()-start
        child.communicate(timeout=5)
        report.append({'case':'operator_sigkill','observed_stop_s':observed_stop,
                       'stop_distance_m':math.dist(before,p.position()),'reason':p.state['reason']})
        print(json.dumps({'passed':True,'checks':report},indent=2))
    finally:
        try:p.call(p.stop_client)
        finally:p.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
