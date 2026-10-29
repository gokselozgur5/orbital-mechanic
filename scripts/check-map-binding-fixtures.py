#!/usr/bin/env python3
"""Exercise map-gate refusal over ROS on an empty private domain; no motion."""
import json
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'runtime/map-binding-fixtures')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '84' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Requires isolated localhost domain84')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.srv import GetMap
    from orbital.map_binding import validate_bundle
    generated = json.loads(subprocess.check_output([sys.executable,str(ROOT/'scripts/generate-known-map.py')],text=True))
    yaml_path = generated['directory']+'/map.yaml'
    bundle = validate_bundle(ROOT,yaml_path)
    rclpy.init(args=[])
    probe = Node('orbital_map_binding_fixture_preflight')
    end = time.monotonic()+2
    while time.monotonic()<end:
        rclpy.spin_once(probe,timeout_sec=.05)
    others = [n for n in probe.get_node_names() if n != probe.get_name()]
    probe.destroy_node()
    if others:
        rclpy.shutdown()
        raise RuntimeError('Domain84 occupied: '+repr(others))
    nodes = {'/map_server':Node('map_server'), '/amcl':Node('amcl'),
             '/global_costmap/global_costmap':Node('global_costmap',namespace='/global_costmap'),
             '/bt_navigator':Node('bt_navigator'), '/orbital_navigation_supervisor':Node('orbital_navigation_supervisor')}
    params = {
        '/map_server':{'yaml_filename':yaml_path,'frame_id':'map','topic_name':'map','use_sim_time':True},
        '/amcl':{'global_frame_id':'map','base_frame_id':'base_footprint','odom_frame_id':'odom',
                 'map_topic':'map','first_map_only':False,'use_sim_time':True},
        '/global_costmap/global_costmap':{'global_frame':'map','robot_base_frame':'base_footprint',
            'static_layer.map_topic':'/map','static_layer.map_subscribe_transient_local':True,
            'static_layer.plugin':'nav2_costmap_2d::StaticLayer','rolling_window':False,'use_sim_time':True},
        '/bt_navigator':{'global_frame':'map','robot_base_frame':'base_footprint','use_sim_time':True},
        '/orbital_navigation_supervisor':{'map_config_sha256':bundle['config_sha256']}}
    executor = SingleThreadedExecutor()
    for path,node in nodes.items():
        executor.add_node(node)
        for name,value in params[path].items():
            if node.has_parameter(name):
                node.set_parameters([Parameter(name,value=value)])
            else:
                node.declare_parameter(name,value)
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = nodes['/map_server'].create_publisher(OccupancyGrid,'/map',qos)
    subs = [nodes[path].create_subscription(OccupancyGrid,'/map',lambda m:None,qos)
            for path in ('/amcl','/global_costmap/global_costmap')]
    grid = bundle['grid']
    msg = OccupancyGrid()
    msg.header.frame_id = grid['frame']
    msg.info.width,msg.info.height,msg.info.resolution = grid['width'],grid['height'],grid['resolution']
    msg.info.origin.position.x,msg.info.origin.position.y = grid['origin'][:2]
    msg.info.origin.orientation.w = 1.
    msg.data = grid['data']
    def serve(request,response):
        response.map = msg
        return response
    service = nodes['/map_server'].create_service(GetMap,'/map_server/map',serve)
    output = args.output
    output.mkdir(exist_ok=False)
    report = {'passed':False,'cases':[]}
    process = None
    try:
        cases = (('good',None), ('wrong_loaded_revision','Loaded map revision'),
                 ('wrong_cells','Observed map'), ('wrong_amcl_topic','consumer configuration'),
                 ('wrong_planner_topic','consumer configuration'), ('wrong_supervisor_config','consumer configuration'),
                 ('restored',None))
        for case,error in cases:
            for path,node in nodes.items():
                node.set_parameters([Parameter(k,value=v) for k,v in params[path].items()])
            msg.data = grid['data']
            if case == 'wrong_loaded_revision':
                nodes['/map_server'].set_parameters([Parameter('yaml_filename',value='/wrong/revision/map.yaml')])
            elif case == 'wrong_cells':
                msg.data[0] = 100
            elif case == 'wrong_amcl_topic':
                nodes['/amcl'].set_parameters([Parameter('map_topic',value='other_map')])
            elif case == 'wrong_planner_topic':
                nodes['/global_costmap/global_costmap'].set_parameters([Parameter('static_layer.map_topic',value='/other_map')])
            elif case == 'wrong_supervisor_config':
                nodes['/orbital_navigation_supervisor'].set_parameters([Parameter('map_config_sha256',value='0'*64)])
            pub.publish(msg)
            with (output/(case+'.json')).open('w') as stdout, (output/(case+'.stderr')).open('w') as stderr:
                process = subprocess.Popen([sys.executable,str(ROOT/'scripts/check-map-binding.py')],stdout=stdout,stderr=stderr)
                deadline = time.monotonic()+20
                while process.poll() is None and time.monotonic()<deadline:
                    executor.spin_once(timeout_sec=.02)
                if process.poll() is None:
                    raise RuntimeError('Gate timed out: '+case)
            data = json.loads((output/(case+'.json')).read_text())
            assert (process.returncode == 0) == (error is None), (case,data)
            assert data['passed'] == (error is None), (case,data)
            if error:
                assert error in data['error'], (case,data)
            report['cases'].append({'case':case,'passed':True,'error':data.get('error')})
        report['passed'] = True
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=2)
        for node in nodes.values():
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()
        (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
