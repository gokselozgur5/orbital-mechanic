"""Experimental composition owning Nav2-to-guard command publication.

The controller's stamped output must be remapped to /navigation/raw_cmd_vel.
No direct controller-to-drive bridge may exist. Trusted local ROS processes only.
"""
from dataclasses import asdict
import json
import math
import time
from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from orbital.navigation import NavigationPolicy, Target
from orbital.localization import planar_pose_valid
from orbital.navigation_command import CommandGate
from orbital.ros_health_interlock import HealthInterlock
from orbital.ros_navigation_action import NavigationActionSession
from orbital.inspection_geometry import ARRIVAL_XY_TOLERANCE_M, ARRIVAL_YAW_TOLERANCE_RAD


class NavigationSupervisor(HealthInterlock):
    def __init__(self, waypoints, *, goal_timeout=120., map_config_sha256=''):
        super().__init__(name='orbital_navigation_supervisor',use_sim_time=True)
        self.declare_parameter('map_config_sha256', map_config_sha256,
                               ParameterDescriptor(read_only=True), ignore_override=True)
        self.policy=NavigationPolicy(waypoints)
        self.commands=CommandGate(self.policy.run_id,waypoints)
        self.session=NavigationActionSession(self,self.policy,waypoints,lambda:self.lease.invalidate('navigation_stop'))
        self.goal_timeout=goal_timeout
        self.guard_state=None;self.guard_at=0.
        self.arm_future=None;self.arm_target=None;self.arm_at=0.
        self.dispatch_stamp=None;self.last_velocity_stamp=None
        self.last_reply=None;self.last_completion=None
        self.forwarded=0
        self.audit_sequence=0;self.audit_error=None
        self.arm_client=self.create_client(Trigger,'/motion/reset_stop')
        self.request_pub=self.create_publisher(TwistStamped,'/motion/request',1)
        self.nav_state_pub=self.create_publisher(String,'/navigation/state',10)
        self.audit_pub=self.create_publisher(String,'/navigation/audit',
            QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.raw_sub=self.create_subscription(TwistStamped,'/navigation/raw_cmd_vel',self.on_velocity,1)
        self.command_sub=self.create_subscription(String,'/navigation/command',self.on_command,10)
        self.guard_sub=self.create_subscription(String,'/motion/state',self.on_guard,10)
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)

    def on_guard(self,msg):
        try:
            state=json.loads(msg.data)
            if type(state.get('latched')) is not bool:raise ValueError('invalid guard state')
            self.guard_state=state;self.guard_at=time.monotonic()
        except (ValueError,TypeError,AttributeError):
            self.guard_state=None

    def guard_fresh(self):
        return self.guard_state is not None and time.monotonic()-self.guard_at<.3

    def command_authority_clear(self):
        for topic, kind in (('/motion/request','geometry_msgs/msg/TwistStamped'),
                            ('/navigation/state','std_msgs/msg/String')):
            publishers=self.get_publishers_info_by_topic(topic)
            if (len(publishers)!=1 or publishers[0].node_name!=self.get_name()
                    or publishers[0].node_namespace!=self.get_namespace() or publishers[0].topic_type!=kind):
                return False
        raw=self.get_publishers_info_by_topic('/navigation/raw_cmd_vel')
        return len(raw)<=1 and all(p.node_name=='controller_server' and p.node_namespace=='/'
                                  and p.topic_type=='geometry_msgs/msg/TwistStamped' for p in raw)

    def on_health(self,msg):
        super().on_health(msg)
        if hasattr(self,'session'):
            # Preserve a moving observation even if stationarity recovers before
            # the next timer. A pending guard RPC cannot authorize the old target.
            if self.arm_future is not None and not self.lease.stationary:
                self.lease.invalidate('arm_stationarity_lost')
            if self.lease.stop_required and self.session.current is not None:
                self.session.cancel(self.session.current,'localization_invalid')

    def audit(self,event,**fields):
        """Observation of this callback, not guard application or physical motion."""
        self.audit_sequence+=1
        try:
            self.audit_pub.publish(String(data=json.dumps({
                'schema_version':1,'run_id':self.policy.run_id,'sequence':self.audit_sequence,
                'sim_stamp_ns':self.get_clock().now().nanoseconds,'event':event,
                'command_sequence':self.commands.sequence,'forwarded_commands':self.forwarded,
                'target':asdict(self.session.current) if self.session.current else None,
                **fields},allow_nan=False)))
        except Exception as error:
            # Observation must not interrupt a stop/cancel callback. The sticky
            # failure and sequence gap make the evaluator refuse a complete trace.
            self.audit_error=type(error).__name__

    def on_command(self,msg):
        command=None
        try:
            if not self.command_authority_clear():raise ValueError('Competing command authority')
            command=self.commands.decode(msg.data)
            op=command['op'];accepted=False
            if op!='cancel' and not self.lease.tick(time.monotonic()):
                self.last_reply={'sequence':command['sequence'],'accepted':False,'op':op}
                return
            if op=='request':
                accepted=self.session.request(command['waypoint']) is not None
            else:
                target=Target(**command['target'])
                if op=='cancel':accepted=self.session.cancel(target)
                elif (self.arm_future is None and target==self.session.current
                      and self.session.state=='waiting_for_arm' and not self.lease.stop_required
                      and self.lease.healthy and self.lease.stationary and self.guard_fresh()
                      and self.guard_state['latched'] and self.arm_client.service_is_ready()):
                    if self.policy.arm(target,stationary=True,now=time.monotonic()):
                        self.arm_target=target;self.arm_at=time.monotonic()
                        self.arm_future=self.arm_client.call_async(Trigger.Request())
                        accepted=True
            self.last_reply={'sequence':command['sequence'],'accepted':accepted,'op':op}
        except (ValueError,TypeError,KeyError) as exc:
            self.last_reply={'accepted':False,'error':str(exc)}
        finally:
            self.audit('command_reply',command=command,reply=self.last_reply)

    def on_velocity(self,msg):
        target=self.session.current
        now=time.monotonic()
        self.lease.tick(now)
        if (target is None or self.lease.stop_required or not self.session.motion_permitted(target)
                or not self.guard_fresh() or self.guard_state['latched']):return
        if not self.command_authority_clear():
            self.lease.invalidate('command_authority_ambiguous')
            self.session.cancel(target,'command_authority_ambiguous');return
        publishers=self.get_publishers_info_by_topic('/navigation/raw_cmd_vel')
        if len(publishers)!=1 or publishers[0].node_name!='controller_server':
            self.session.cancel(target,'controller_authority_ambiguous');return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        sim=self.get_clock().now().nanoseconds*1e-9
        v=msg.twist;values=(v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z)
        valid=(msg.header.frame_id in ('base_link','base_footprint')
               and all(math.isfinite(x) for x in values) and all(x==0 for x in (v.linear.y,v.linear.z,v.angular.x,v.angular.y))
               and abs(v.linear.x)<=.2 and abs(v.angular.z)<=.5 and -.1<=sim-stamp<=.2
               and self.dispatch_stamp is not None and stamp>=self.dispatch_stamp
               and (self.last_velocity_stamp is None or stamp>=self.last_velocity_stamp))
        if not valid:
            self.session.cancel(target,'invalid_controller_velocity');return
        out=TwistStamped();out.header=msg.header;out.header.frame_id='base_link';out.twist=msg.twist
        self.request_pub.publish(out);self.last_velocity_stamp=stamp;self.forwarded+=1
        self.audit('velocity_forwarded',target=asdict(target),
                   velocity_stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                   frame_id=out.header.frame_id,velocity=list(values))

    def arrival(self,target):
        try:
            sim=self.get_clock().now().nanoseconds
            if sim<=100000000:return None
            t=self.buffer.lookup_transform('map','base_footprint',Time(nanoseconds=sim-100000000,clock_type=ClockType.ROS_TIME))
            p=t.transform.translation;q=t.transform.rotation
            if not planar_pose_valid((p.x,p.y,p.z),(q.x,q.y,q.z,q.w)):return None
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            x,y,goal_yaw=self.session.waypoints[target.waypoint]
            xy=math.hypot(p.x-x,p.y-y);angle=abs(math.remainder(yaw-goal_yaw,2*math.pi))
            return {'xy_error_m':xy,'yaw_error_rad':angle,
                    'within_tolerance':xy<=ARRIVAL_XY_TOLERANCE_M and angle<=ARRIVAL_YAW_TOLERANCE_RAD}
        except TransformException:return None

    def tick(self):
        if hasattr(self,'session') and not self.command_authority_clear():
            self.lease.invalidate('command_authority_ambiguous')
        super().tick()
        if not hasattr(self,'session'):return
        now=time.monotonic()
        self.policy.observe(sim_time=self.lease.sim_time if self.lease.sim_time is not None else 0.,
                            valid=self.lease.healthy and not self.lease.stop_required,now=now)
        self.session.tick()
        if self.arm_future is not None:
            target=self.arm_target
            if self.arm_future.done():
                try:success=self.arm_future.result().success
                except Exception:success=False
                valid=(success and target==self.session.current and self.session.state=='waiting_for_arm'
                       and self.policy.armed and self.lease.healthy and self.lease.stationary
                       and not self.lease.stop_required and now-self.arm_at<.5)
                if valid and self.guard_fresh() and not self.guard_state['latched'] and self.guard_at>=self.arm_at:
                    self.arm_future=None
                    self.dispatch_stamp=self.get_clock().now().nanoseconds*1e-9
                    self.last_velocity_stamp=None
                    if not self.session.start(target,timeout=self.goal_timeout):self.lease.invalidate('goal_start_failed')
                elif not valid:
                    self.arm_future=None
                    self.lease.invalidate('late_or_failed_arm')
                    self.session.cancel(target,'late_or_failed_arm')
            elif now-self.arm_at>=.5:
                # Do not release the old task while an unresolved arm may still apply.
                self.lease.invalidate('arm_timeout');self.session.cancel(target,'arm_timeout')
        if self.session.state=='running' and (not self.guard_fresh() or self.guard_state['latched']):
            self.session.cancel(self.session.current,'guard_stopped_or_stale')
        if (self.session.state=='terminal' and self.arm_future is None and self.guard_fresh()
                and self.guard_state['latched'] and self.lease.healthy and not self.lease.stop_required and self.lease.stationary):
            target=self.session.current
            measured=self.arrival(target) if self.session.outcome=='nav2_succeeded_unverified' else None
            outcome=self.session.outcome
            if outcome=='nav2_succeeded_unverified':outcome='arrived' if measured and measured['within_tolerance'] else 'arrival_not_verified'
            self.last_completion={'target':asdict(target),'outcome':outcome,'measured':measured}
            self.session.release(target,stationary=True,stop_latched=True)
        self.nav_state_pub.publish(String(data=json.dumps({
            'run_id':self.policy.run_id,'command_sequence':self.commands.sequence,
            'sim_stamp_ns':self.get_clock().now().nanoseconds,
            'audit_sequence':self.audit_sequence,'audit_ok':self.audit_error is None,
            'target':asdict(self.session.current) if self.session.current else None,
            'action_state':self.session.state,'healthy':self.lease.healthy and not self.lease.stop_required,
            'guard_latched':self.guard_state['latched'] if self.guard_fresh() else None,
            'last_reply':self.last_reply,'last_completion':self.last_completion,'forwarded_commands':self.forwarded})))

    def destroy_node(self):
        self.session.destroy()
        self.listener.unregister()
        return super().destroy_node()
