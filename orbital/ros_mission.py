"""One persistent ROS session for scripted and future planner gateway calls."""
from collections import Counter
import signal
import time
import uuid

from orbital.inspection_worker import OwnerLock
from orbital.inspection_protocol import finite, identifier
from orbital.machine_registry import MachineRegistry
from orbital.mission_journal import MissionJournal
from orbital.mission_runtime import MissionRuntime
from orbital.navigation_graph import BASE_TOPICS
from orbital.proximity import Unavailable
from orbital.ros_inspection import OWNER as INSPECTION_OWNER, TOPICS as INSPECTION_TOPICS
from orbital.ros_maintenance import OWNER as MAINTENANCE_OWNER, TOPICS as MAINTENANCE_TOPICS
from orbital.ros_proximity import authority

OWNERS={'navigation':'orbital_navigation_supervisor',
        'inspection':INSPECTION_OWNER,'maintenance':MAINTENANCE_OWNER}
SOURCES={
    'navigation':{'/navigation/state':[(OWNERS['navigation'],'std_msgs/msg/String')]},
    'inspection':{name:[(INSPECTION_OWNER,'std_msgs/msg/String')] for name in INSPECTION_TOPICS},
    'maintenance':{name:[(MAINTENANCE_OWNER,'std_msgs/msg/String')] for name in MAINTENANCE_TOPICS},
    'public':{'/clock':BASE_TOPICS['/clock'],
              '/station/telemetry':[(MAINTENANCE_OWNER,'std_msgs/msg/String')]}}


class RosMissionTransport:
    def __init__(self, *, now=time.monotonic, startup_timeout=10.):
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        self.ros,self.String,self.now=rclpy,String,now
        self.node=None
        self.context_owned=False
        self.publishers={}
        self.pinned=None
        self.installed=False
        try:
            rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
            self.context_owned=True
            self.node=rclpy.create_node('orbital_mission_'+uuid.uuid4().hex[:16],
                parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.publishers={family:self.node.create_publisher(String,'/'+family+'/command',16) for family in OWNERS}
            deadline=now()+startup_timeout
            while now()<deadline:
                self.spin(min(.02,deadline-now()))
                try:
                    self._mode()
                    self.pinned=self._inventory()
                    break
                except Unavailable:pass
            if self.pinned is None:raise ValueError('mission_sources_unavailable')
        except BaseException:
            self.close()
            raise

    def sim_ns(self):return self.node.get_clock().now().nanoseconds

    def _mode(self):
        if self.node.get_parameter('use_sim_time').value is not True or self.sim_ns()<=0:
            raise Unavailable('mission_simulation_clock_unavailable')

    def _inventory(self, family=None):
        groups=SOURCES if family is None else {family:SOURCES[family]}
        result={}
        for name,sources in groups.items():
            expected=dict(sources)
            if name in OWNERS:
                expected['/'+name+'/command']=[(self.node.get_name(),'std_msgs/msg/String')]
            observed=authority(self.node,expected)
            if name in OWNERS:
                topic='/'+name+'/command'
                peers=self.node.get_subscriptions_info_by_topic(topic)
                if (Counter((p.node_name,p.node_namespace,p.topic_type) for p in peers)
                        != Counter([(OWNERS[name],'/','std_msgs/msg/String')])
                        or self.publishers[name].get_subscription_count()!=1):
                    raise Unavailable('mission_command_subscriber_unavailable')
                for peer in peers:
                    gid=list(peer.endpoint_gid)
                    if not 1<=len(gid)<=128 or not any(gid) or any(type(v) is not int or not 0<=v<=255 for v in gid):
                        raise Unavailable('mission_subscriber_identity_unavailable')
                    observed.append((topic,peer.node_name,peer.node_namespace,peer.topic_type,bytes(gid).hex(),'subscriber'))
            result[name]=observed
        return result

    def fence(self, family=None):
        if self.pinned is None:raise Unavailable('mission_sources_not_pinned')
        if family is None:self._mode()
        observed=self._inventory(family)
        if any(value!=self.pinned[name] for name,value in observed.items()):
            raise Unavailable('mission_source_generation_changed')
        if family is None:self._mode()
        return observed

    def install(self, callbacks):
        if self.installed:raise ValueError('mission_subscriptions_already_installed')
        expected={topic for group in SOURCES.values() for topic in group if topic!='/clock'}
        if set(callbacks)!=expected:raise ValueError('mission_subscription_contract_mismatch')
        self.fence()
        for topic,callback in callbacks.items():
            self.node.create_subscription(self.String,topic,lambda msg,fn=callback:fn(msg.data),64)
        self.installed=True

    def spin(self, duration):
        if not self.ros.ok():raise ValueError('mission_ros_context_unavailable')
        self.ros.spin_once(self.node,timeout_sec=duration)

    def publish(self, family, text):
        self.publishers[family].publish(self.String(data=text))

    def close(self):
        failed=False
        if self.node is not None:
            try:self.node.destroy_node()
            except (ValueError,RuntimeError,OSError,KeyboardInterrupt):failed=True
            self.node=None
        if self.context_owned:
            self.context_owned=False
            try:self.ros.try_shutdown()
            except (ValueError,RuntimeError,OSError,KeyboardInterrupt):failed=True
        return not failed


def run(root, *, mission_id=None, timeout_s=600., now=time.monotonic, transport_factory=RosMissionTransport):
    """Run one explicit bounded mission; never start/restart a prerequisite."""
    deadline=finite(now(),0.,float('inf'))+finite(timeout_s,1.,600.)
    mission_id=identifier(uuid.uuid4().hex if mission_id is None else mission_id)
    registry=MachineRegistry.load(root)
    lock=OwnerLock(root,service='mission-service')
    journal=transport=None
    stopping=False
    def stop(signum,frame):
        nonlocal stopping
        stopping=True
    previous={}
    report=None
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):previous[sig]=signal.signal(sig,stop)
        journal=MissionJournal(lock)
        if journal.active is not None:
            raise ValueError('unfinished_mission_requires_reconciliation')
        remaining=deadline-now()
        if remaining<1.:raise ValueError('mission_startup_deadline')
        transport=transport_factory(now=now,startup_timeout=min(10.,remaining))
        remaining=deadline-now()
        if remaining<1.:raise ValueError('mission_startup_deadline')
        driver=MissionRuntime(registry,transport,journal,mission_id=mission_id,timeout_s=remaining,
            stop_requested=lambda:stopping)
        report=driver.run()
        return report
    finally:
        try:
            if transport is not None and not transport.close() and report is not None:
                report['complete']=False
                report['client_cleanup_failed']=True
        finally:
            if journal is not None:journal.close()
            lock.close()
            for sig,handler in previous.items():signal.signal(sig,handler)
