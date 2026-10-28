"""Controlled ROS transport replays packets emitted by the shipped telemetry adapter."""
import json
import sys
from types import ModuleType,SimpleNamespace as NS
from unittest.mock import patch

from orbital import ros_station_observation
from test_machine_telemetry_adapter import AdapterHarness


class Harness:
    def __init__(self, case):
        self.root=case.root
        producer=AdapterHarness(times=(1100000000,1600000000,2100000000,2600000000))
        producer.run(self.root)
        self.packets=producer.messages['/station/telemetry']
        self.wall,self.sim,self.spins,self.delivered=10.,10**9,0,0
        self.initialized=self.destroyed=self.shutdown=False
        self.callback=None
        self.before_spin=self.before_packet=None
        self.mode=True;self.gid=1
        self.missing=self.duplicate=self.wrong_owner=self.bad_gid=False
        self.paused=self.drop=False
        owner=self
        class Node:
            def get_clock(self):return NS(now=lambda:NS(nanoseconds=owner.sim))
            def get_parameter(self,name):
                assert name=='use_sim_time';return NS(value=owner.mode)
            def get_publishers_info_by_topic(self,topic):
                assert topic in ros_station_observation.SOURCES
                if owner.missing:return []
                name,kind=ros_station_observation.SOURCES[topic][0]
                peer=NS(node_name='wrong' if owner.wrong_owner else name,node_namespace='/',topic_type=kind,
                        endpoint_gid=([0]*24 if owner.bad_gid else [1]*23+[owner.gid]))
                return [peer,peer] if owner.duplicate else [peer]
            def create_subscription(self,kind,topic,callback,qos):
                assert topic=='/station/telemetry' and qos==5
                owner.callback=callback;return NS()
            def create_publisher(self,*args,**kwargs):raise AssertionError('Observer must not publish')
            def create_client(self,*args,**kwargs):raise AssertionError('Observer must not create service clients')
            def destroy_node(self):owner.destroyed=True
        def spin(node,timeout_sec):
            owner.spins+=1;owner.wall+=.05
            if not owner.paused:owner.sim+=50000000
            if owner.before_spin:owner.before_spin()
            if owner.callback is not None and owner.spins in (2,12,22,32) and not owner.drop:
                index=(owner.spins-2)//10
                text=owner.packets[index]
                if owner.before_packet:text=owner.before_packet(index,text)
                owner.delivered+=1
                owner.callback(NS(data=text))
        def init(**kwargs):owner.initialized=True
        def shutdown():owner.shutdown=True
        exports={'rclpy':{'init':init,'create_node':lambda *args,**kwargs:Node(),'spin_once':spin,'try_shutdown':shutdown},
                 'rclpy.parameter':{'Parameter':lambda *args,**kwargs:NS()},'std_msgs.msg':{'String':NS}}
        self.modules={'std_msgs':ModuleType('std_msgs')}
        for name,values in exports.items():
            self.modules[name]=ModuleType(name);self.modules[name].__dict__.update(values)

    def run(self,timeout=2.,invoke=None):
        with patch.dict(sys.modules,self.modules):
            if invoke:return invoke()
            return ros_station_observation.run(self.root,timeout=timeout,now=lambda:self.wall)
