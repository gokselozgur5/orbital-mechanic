#!/usr/bin/env python3
"""Check guarded native readiness; --configure/--activate are explicit mutations.

Never requests a target or calls reset_stop. Failure requests guard stop and
pauses lifecycle nodes. A passing result is infrastructure readiness, not a route.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    if os.environ.get('ROS_DOMAIN_ID') not in ('42','84') or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        raise RuntimeError('Requires localhost Orbital domain42 or isolated domain84')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=ROOT/'runtime/navigation/source-manifest.json')
    parser.add_argument('--configure',action='store_true')
    parser.add_argument('--activate',action='store_true',help='Configure if necessary, then activate with guard still latched')
    args=parser.parse_args()
    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    from geometry_msgs.msg import TwistStamped
    spec=importlib.util.spec_from_file_location('navigation_graph_probe',ROOT/'scripts/check-navigation-graph.py')
    graph_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(graph_module)
    rclpy.init(args=[])
    node=rclpy.create_node('orbital_navigation_readiness_probe')
    names=('controller_server','planner_server','bt_navigator')
    nav={};runs=set();violations=set();requests=[]
    report={'passed':False,'lifecycle_commands':[],'graph_checks':[],
            'scope':'native lifecycle/graph with guard latched; not physical navigation acceptance'}
    clients={'/motion/stop':node.create_client(Trigger,'/motion/stop')}

    def on_state(message):
        try:
            value=json.loads(message.data)
            if not isinstance(value,dict) or not value.get('run_id'):raise ValueError('Missing navigation run identity')
            if value.get('target') is not None or value.get('action_state')!='idle':
                violations.add('Navigation target is already owned')
            if value.get('guard_latched') is False:
                violations.add('Supervisor does not observe a latched guard')
            runs.add(value['run_id']);nav.update(value);nav['received_at']=time.monotonic()
        except (ValueError,TypeError):violations.add('Malformed navigation state')
    def on_request(message):
        requests.append(message)
        violations.add('Supervisor emitted a velocity request during readiness')
    subscriptions=[node.create_subscription(String,'/navigation/state',on_state,10),
                   node.create_subscription(TwistStamped,'/motion/request',on_request,10)]

    def wait(predicate, seconds=15., label='readiness evidence'):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
        raise RuntimeError('Timed out waiting for '+label)
    def call(kind,path,request,timeout=15.):
        if path not in clients:clients[path]=node.create_client(kind,path)
        client=clients[path]
        wait(client.service_is_ready,timeout,path+' discovery')
        future=client.call_async(request);wait(future.done,timeout,path+' response')
        return future.result()
    def states():
        return {name:call(GetState,'/'+name+'/get_state',GetState.Request()).current_state.id for name in names}
    def manage(command,timeout=15.):
        report['lifecycle_commands'].append(command)
        return call(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes',
                    ManageLifecycleNodes.Request(command=command),timeout).success
    def source_identity():
        result=subprocess.run([sys.executable,str(ROOT/'scripts/navigation-manifest.py'),'--verify',str(args.manifest)],
                              capture_output=True,text=True,timeout=30)
        if result.returncode:
            raise RuntimeError('Source manifest verification failed: '+result.stderr[-500:])
        return json.loads(result.stdout)['manifest_identity']
    def graph(phase):
        value=graph_module.collect(node,phase)
        report['graph_checks'].append(value)
        if not value['passed']:raise RuntimeError('Graph/guard readiness gate failed: '+phase)
        if len(runs)!=1 or time.monotonic()-nav.get('received_at',0)>.3 or nav.get('guard_latched') is not True or violations:
            raise RuntimeError('Idle ownership check failed: '+repr(sorted(violations)))

    try:
        report['manifest_identity']=source_identity()
        wait(lambda:bool(nav))
        before=states();report['initial_states']=before
        if set(before.values())=={1}:
            graph('unconfigured')
            if not (args.configure or args.activate):raise RuntimeError('Stack is unconfigured; explicit --configure required')
            if not manage(ManageLifecycleNodes.Request.CONFIGURE):raise RuntimeError('Lifecycle configure failed')
            if set(states().values())!={2}:raise RuntimeError('Configure did not leave all servers inactive')
        elif set(before.values()) not in ({2},{3}):
            raise RuntimeError('Mixed or invalid lifecycle states require explicit service recovery')
        graph('configured')
        if args.activate and set(states().values())=={2}:
            if source_identity()!=report['manifest_identity']:raise RuntimeError('Source identity changed before activation')
            # Drain state callbacks after hashing and re-establish a fresh gate.
            graph('configured')
            if not manage(ManageLifecycleNodes.Request.RESUME):raise RuntimeError('Lifecycle activation failed')
        final=states();report['final_states']=final
        if args.activate and set(final.values())!={3}:raise RuntimeError('Activation did not reach active state')
        graph('configured')
        if source_identity()!=report['manifest_identity']:raise RuntimeError('Source identity changed during readiness')
        report.update(passed=True,supervisor_run_id=next(iter(runs)),
                      supervisor_velocity_requests=len(requests),localization_healthy=nav.get('healthy'))
    except Exception as exc:
        report['error']=str(exc)
        try:report['guard_stop_ack']=call(Trigger,'/motion/stop',Trigger.Request(),3.).success
        except Exception as stop_error:report['guard_stop_error']=str(stop_error)
        try:report['lifecycle_pause_ack']=manage(ManageLifecycleNodes.Request.PAUSE,3.)
        except Exception as pause_error:report['lifecycle_pause_error']=str(pause_error)
    finally:
        for client in clients.values():node.destroy_client(client)
        for subscription in subscriptions:node.destroy_subscription(subscription)
        node.destroy_node();rclpy.try_shutdown()
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
