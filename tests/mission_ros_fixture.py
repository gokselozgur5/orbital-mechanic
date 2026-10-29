"""Controlled DDS graph/clock around the real persistent ROS mission adapter."""
import hashlib
from types import SimpleNamespace as NS

from orbital.ros_mission import OWNERS, SOURCES


class MissionRos:
    def __init__(self, bus):
        self.bus=bus
        self.active=self.destroyed=self.shutdown=False
        self.mode=True
        self.mutations={}
        self.subscribers={}
        self.node=None
        self.cleanup_error=False
        self.startup_delay=0.
        self.String=lambda data='':NS(data=data)
        self.modules={
            'rclpy':NS(init=self.init,create_node=self.create_node,ok=lambda:self.active,
                       spin_once=lambda node,timeout_sec:bus.spin(timeout_sec),try_shutdown=self.close),
            'rclpy.parameter':NS(Parameter=lambda name,value:NS(name=name,value=value)),
            'rclpy.signals':NS(SignalHandlerOptions=NS(NO=0)),
            'std_msgs':NS(), 'std_msgs.msg':NS(String=self.String)}

    def init(self, **kwargs):
        self.active=True
        if self.startup_delay:self.bus.spin(self.startup_delay)

    def create_node(self, name, **kwargs):
        def publisher(kind, topic, depth):
            family=topic.split('/')[1]
            return NS(publish=lambda msg:self.bus.publish(family,msg.data),
                      get_subscription_count=lambda:len(self.peers(topic,subscriber=True)))
        def subscribe(kind, topic, callback, depth):
            if topic in self.subscribers:raise ValueError('duplicate_subscription')
            self.subscribers[topic]=callback
            self.bus.callbacks[topic]=lambda text:callback(self.String(data=text))
        self.node=NS(get_name=lambda:name,create_publisher=publisher,create_subscription=subscribe,
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=self.bus.sim_ns())),
            get_parameter=lambda key:NS(value=self.mode),
            get_publishers_info_by_topic=self.peers,
            get_subscriptions_info_by_topic=lambda topic:self.peers(topic,subscriber=True),
            destroy_node=self.destroy)
        return self.node

    def peers(self, topic, subscriber=False):
        family=topic.split('/')[1]
        if topic.endswith('/command'):
            entries=[(OWNERS[family] if subscriber else self.node.get_name(),'std_msgs/msg/String')]
        else:entries={name:peers for group in SOURCES.values() for name,peers in group.items()}[topic]
        result=[NS(node_name=name,node_namespace='/',topic_type=kind,
                   endpoint_gid=list(hashlib.sha256((topic+name+str(subscriber)).encode()).digest()[:16]))
                for name,kind in entries]
        change=self.mutations.get((topic,subscriber))
        return change(result) if change else result

    def destroy(self):
        self.destroyed=True
        self.bus.callbacks={}
        if self.cleanup_error:raise RuntimeError('controlled_cleanup_failure')

    def close(self):
        self.active=False
        self.shutdown=True
