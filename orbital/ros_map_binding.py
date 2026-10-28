"""Read-only map gate for startup/readiness; no lifecycle or motion side effects."""
import time
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

from orbital.map_binding import validate_bundle, validate_grid


def grid_payload(msg):
    p, q = msg.info.origin.position, msg.info.origin.orientation
    return {'frame':msg.header.frame_id, 'width':msg.info.width, 'height':msg.info.height,
            'resolution':msg.info.resolution, 'origin':[p.x,p.y,p.z,q.x,q.y,q.z,q.w], 'data':list(msg.data)}


def collect(node, root, *, configured=True, timeout=12.):
    root = Path(root).resolve()
    deadline = time.monotonic()+timeout
    clients = []
    received = []
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(OccupancyGrid, '/map', lambda m: received.append(grid_payload(m)), retained)
    def wait(predicate):
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError('Map binding discovery/response timeout')
            rclpy.spin_once(node, timeout_sec=.02)
    def call(kind, path, request):
        client = node.create_client(kind, path)
        clients.append(client)
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done)
        return future.result()
    def params(name, expected):
        response = call(GetParameters, name+'/get_parameters', GetParameters.Request(names=list(expected)))
        actual = {k:parameter_value_to_python(v) for k,v in zip(expected,response.values)}
        if actual != expected:
            raise ValueError('Map consumer configuration mismatch: '+name+' '+repr(actual))
        return actual
    try:
        response = call(GetParameters, '/map_server/get_parameters', GetParameters.Request(names=['yaml_filename']))
        yaml_path = parameter_value_to_python(response.values[0])
        bundle = validate_bundle(root, yaml_path)
        params('/map_server', {'frame_id':'map','topic_name':'map','use_sim_time':True})
        params('/amcl', {'global_frame_id':'map','base_frame_id':'base_footprint',
                        'odom_frame_id':'odom','map_topic':'map','first_map_only':False,'use_sim_time':True})
        if configured:
            params('/global_costmap/global_costmap', {'global_frame':'map','robot_base_frame':'base_footprint',
                   'static_layer.map_topic':'/map','static_layer.map_subscribe_transient_local':True,
                   'static_layer.plugin':'nav2_costmap_2d::StaticLayer','rolling_window':False,'use_sim_time':True})
            params('/bt_navigator', {'global_frame':'map','robot_base_frame':'base_footprint','use_sim_time':True})
            params('/orbital_navigation_supervisor', {'map_config_sha256':bundle['config_sha256']})
        service_map = call(GetMap, '/map_server/map', GetMap.Request()).map
        validate_grid(bundle, grid_payload(service_map))
        wait(lambda: bool(received))
        for message in received:
            validate_grid(bundle, message)
        # Endpoint discovery can precede its ROS node-name metadata. Wait for
        # that relation, but never accept a complete wrong/duplicate publisher.
        def resolved_publishers():
            items = node.get_publishers_info_by_topic('/map')
            return bool(items) and all(p.node_name and p.node_name != '_NODE_NAME_UNKNOWN_'
                                      and p.node_namespace != '_NODE_NAMESPACE_UNKNOWN_' for p in items)
        wait(resolved_publishers)
        publishers = node.get_publishers_info_by_topic('/map')
        if (len(publishers) != 1 or publishers[0].node_name != 'map_server' or publishers[0].node_namespace != '/'
                or publishers[0].topic_type != 'nav_msgs/msg/OccupancyGrid'
                or publishers[0].qos_profile.durability != DurabilityPolicy.TRANSIENT_LOCAL):
            raise ValueError('Map publisher authority mismatch')
        expected = {('amcl','/')}
        if configured:
            expected.add(('global_costmap','/global_costmap'))
        wait(lambda: expected.issubset({(p.node_name,p.node_namespace)
                                      for p in node.get_subscriptions_info_by_topic('/map')}))
        consumers = node.get_subscriptions_info_by_topic('/map')
        for name, ns in expected:
            matches = [p for p in consumers if p.node_name == name and p.node_namespace == ns]
            if (len(matches) != 1 or matches[0].topic_type != 'nav_msgs/msg/OccupancyGrid'
                    or matches[0].qos_profile.durability != DurabilityPolicy.TRANSIENT_LOCAL
                    or matches[0].qos_profile.reliability != ReliabilityPolicy.RELIABLE):
                raise ValueError('Map consumer subscription mismatch: '+name)
        for message in received:
            validate_grid(bundle, message)
        after = validate_bundle(root, yaml_path)
        if after != bundle:
            raise ValueError('Map source/artifacts changed during binding')
        return {'passed':True, 'configured':configured, 'map_yaml':yaml_path,
                'binding':{k:v for k,v in bundle.items() if k != 'grid'},
                'received_maps':len(received), 'consumers':sorted(expected),
                'scope':'source/artifact/server cells and configured subscriptions; not private AMCL/costmap memory or continuous enforcement'}
    finally:
        node.destroy_subscription(sub)
        for client in clients:
            node.destroy_client(client)
