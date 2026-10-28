"""Controlled DDS-shaped transport executing the real map/proximity adapters."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

from orbital.machine_registry import MachineRegistry
from orbital.navigation_graph import BASE_TOPICS,NAV_TOPICS
from orbital import ros_proximity
from test_proximity import health,transform

ROOT = Path(__file__).resolve().parents[1]


class Harness:
    def __init__(self, case):
        self.root,self.map_path,self.payload = case.root,case.directory/'map.yaml',case.payload
        self.config_hash = MachineRegistry.load(self.root).config_sha256
        self.wall,self.sim,self.sequence = 10.,10**9,0
        self.callbacks,self.clients,self.reads = {},[],[]
        self.destroyed = self.unregistered = self.shutdown = False
        self.changed_gid = self.wrong_map = self.wrong_params = False
        self.no_tf = self.wrong_tf = self.paused = self.unhealthy = False
        self.changed_epoch = self.drop_health = self.clock_mode_changed = False
        self.lookup_hook = None
        self.lookups = []
        self.pose = (2.,1.8,0.)
        owner = self
        parameters = {
            '/map_server':{'yaml_filename':str(self.map_path),'frame_id':'map','topic_name':'map','use_sim_time':True},
            '/amcl':{'global_frame_id':'map','base_frame_id':'base_footprint','odom_frame_id':'odom','map_topic':'map','first_map_only':False,'use_sim_time':True},
            '/global_costmap/global_costmap':{'global_frame':'map','robot_base_frame':'base_footprint','static_layer.map_topic':'/map','static_layer.map_subscribe_transient_local':True,'static_layer.plugin':'nav2_costmap_2d::StaticLayer','rolling_window':False,'use_sim_time':True},
            '/bt_navigator':{'global_frame':'map','robot_base_frame':'base_footprint','use_sim_time':True},
            '/orbital_navigation_supervisor':{'map_config_sha256':self.config_hash},
        }
        def grid():
            p = copy.deepcopy(owner.payload)
            if owner.wrong_map: p['data'][0] = 100
            return NS(header=NS(frame_id=p['frame']),data=p['data'],
                      info=NS(width=p['width'],height=p['height'],resolution=p['resolution'],
                              origin=NS(position=NS(x=p['origin'][0],y=p['origin'][1],z=p['origin'][2]),
                                        orientation=NS(x=p['origin'][3],y=p['origin'][4],z=p['origin'][5],w=p['origin'][6]))))
        class Node:
            def create_subscription(self, kind,topic,callback,qos):
                assert topic in ('/navigation/health','/map')
                owner.callbacks[topic] = callback
                if topic == '/map': callback(grid())
                return NS(topic=topic)
            def destroy_subscription(self, sub):
                owner.callbacks.pop(sub.topic,None)
            def create_publisher(self,*args,**kwargs):
                raise AssertionError('Proximity must never publish a control or measurement topic')
            def create_client(self,kind,path):
                assert path.endswith('/get_parameters') or path == '/map_server/map'
                def call(request):
                    owner.reads.append((path,getattr(request,'names',None)))
                    if path == '/map_server/map': value = NS(map=grid())
                    else:
                        values = parameters[path.removesuffix('/get_parameters')].copy()
                        if owner.wrong_params and path == '/amcl/get_parameters': values['map_topic'] = 'wrong_map'
                        value = NS(values=[values[name] for name in request.names])
                    return NS(done=lambda:True,result=lambda:value)
                client = NS(service_is_ready=lambda:True,call_async=call)
                owner.clients.append(client)
                return client
            def destroy_client(self,client):
                owner.clients.remove(client)
            def get_publishers_info_by_topic(self,topic):
                owners = NAV_TOPICS.get(topic,BASE_TOPICS.get(topic,[]))
                return [NS(node_name=name,node_namespace='/',topic_type=kind,
                           endpoint_gid=[i+1]*(23)+[2 if owner.changed_gid else 1],
                           qos_profile=NS(durability='transient_local')) for i,(name,kind) in enumerate(owners)]
            def get_subscriptions_info_by_topic(self,topic):
                assert topic == '/map'
                return [NS(node_name=name,node_namespace=namespace,topic_type='nav_msgs/msg/OccupancyGrid',
                           qos_profile=NS(durability='transient_local',reliability='reliable'))
                        for name,namespace in (('amcl','/'),('global_costmap','/global_costmap'))]
            def get_clock(self):
                return NS(now=lambda:NS(nanoseconds=owner.sim))
            def get_parameter(self,name):
                assert name == 'use_sim_time'
                return NS(value=not owner.clock_mode_changed)
            def destroy_node(self): owner.destroyed = True
        class Buffer:
            def __init__(self,**kwargs): pass
            def lookup_transform(self,parent,child,stamp):
                assert (parent,child) == ('map','base_footprint')
                owner.lookups.append(stamp.nanoseconds)
                if owner.no_tf: raise LookupError('controlled missing transform')
                if owner.lookup_hook: owner.lookup_hook()
                return transform(stamp.nanoseconds+int(owner.wrong_tf),*owner.pose)
        def spin(node,timeout_sec):
            owner.wall += timeout_sec
            if not owner.paused: owner.sim += round(timeout_sec*1e9)
            owner.sequence += 1
            if not owner.drop_health and '/navigation/health' in owner.callbacks:
                owner.callbacks['/navigation/health'](NS(data=health(owner.sequence,owner.sim/1e9,
                    healthy=not owner.unhealthy,epoch=int(owner.changed_epoch))))
        def unregister(): owner.unregistered = True
        def shutdown(): owner.shutdown = True
        exports = {
            'rclpy':{'init':lambda **kw:None,'create_node':lambda *a,**kw:Node(),'spin_once':spin,'try_shutdown':shutdown},
            'rclpy.clock':{'ClockType':NS(ROS_TIME=1)},
            'rclpy.time':{'Time':lambda **kw:NS(**kw)},
            'rclpy.parameter':{'Parameter':lambda *a,**kw:NS(),'parameter_value_to_python':lambda v:v},
            'rclpy.qos':{'QoSProfile':NS,'DurabilityPolicy':NS(TRANSIENT_LOCAL='transient_local'),'ReliabilityPolicy':NS(RELIABLE='reliable')},
            'std_msgs.msg':{'String':NS},'nav_msgs.msg':{'OccupancyGrid':NS},
            'nav_msgs.srv':{'GetMap':NS(Request=NS)},'rcl_interfaces.srv':{'GetParameters':NS(Request=NS)},
            'tf2_ros':{'Buffer':Buffer,'TransformException':LookupError,'TransformListener':lambda *a:NS(unregister=unregister)},
        }
        self.modules = {name:ModuleType(name) for name in ('std_msgs','nav_msgs','rcl_interfaces')}
        for name,values in exports.items():
            self.modules[name] = ModuleType(name)
            self.modules[name].__dict__.update(values)

    def run(self,timeout=2.,invoke=None):
        with patch.dict(sys.modules,self.modules):
            spec = importlib.util.spec_from_file_location('orbital.ros_map_binding',ROOT/'orbital/ros_map_binding.py')
            mapping = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mapping)
            mapping.time = NS(monotonic=lambda:self.wall)
            with patch.dict(sys.modules,{'orbital.ros_map_binding':mapping}):
                if invoke is not None:
                    return invoke()
                return ros_proximity.run(self.root,'pump',timeout=timeout,now=lambda:self.wall)
