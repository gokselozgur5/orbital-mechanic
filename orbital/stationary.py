"""Bounded sampled odometry history for a fresh, settled inspection precondition."""
import math

from orbital.localization import planar_pose_valid, STATIONARY_LINEAR_M_S, STATIONARY_ANGULAR_RAD_S

SETTLE_SECONDS = .5
SETTLE_NS = 500000000
MAX_GAP_SECONDS = .2
MAX_GAP_NS = 200000000


class StationaryWindow:
    def __init__(self):
        self.start_stamp = self.start_wall = self.last_stamp = self.last_wall = None
        self.samples = 0
        self.generation = 0
        self.reason = 'awaiting_odometry'
        self.max_linear = self.max_angular = 0.

    def reset(self, reason):
        if self.start_stamp is not None or self.reason != reason:
            self.generation += 1
        self.start_stamp = self.start_wall = None
        self.samples = 0
        self.reason = reason
        self.max_linear = self.max_angular = 0.

    def observe(self, msg, now, sim_ns, epoch_floor):
        try:
            sec,nsec = msg.header.stamp.sec,msg.header.stamp.nanosec
            if (type(sec) is not int or type(nsec) is not int or sec<0 or not 0<=nsec<10**9
                    or type(sim_ns) is not int or not 0<=sim_ns<2**63 or not math.isfinite(now)):
                raise ValueError('invalid_odometry')
            stamp = sec*10**9+nsec
            if not epoch_floor<=stamp<2**63 or stamp>sim_ns+100000000:
                raise ValueError('odometry_epoch_or_time_mismatch')
            p,q = msg.pose.pose.position,msg.pose.pose.orientation
            v,w = msg.twist.twist.linear,msg.twist.twist.angular
            position,orientation = (p.x,p.y,p.z),(q.x,q.y,q.z,q.w)
            linear,angular = (v.x,v.y,v.z),(w.x,w.y,w.z)
            if (msg.header.frame_id!='odom' or msg.child_frame_id!='base_footprint'
                    or any(type(x) not in (int,float) or not math.isfinite(x) or abs(x)>10000
                           for x in (*position,*orientation,*linear,*angular))
                    or not planar_pose_valid(position,orientation)):
                raise ValueError('invalid_odometry')
            if self.last_stamp is not None:
                if stamp<=self.last_stamp or now<self.last_wall:
                    raise ValueError('out_of_order_odometry')
                if stamp-self.last_stamp>=MAX_GAP_NS or now-self.last_wall>=MAX_GAP_SECONDS:
                    self.reset('odometry_gap')
            self.last_stamp,self.last_wall = stamp,now
            speed,turn = math.hypot(*linear),math.hypot(*angular)
            if speed>STATIONARY_LINEAR_M_S or turn>STATIONARY_ANGULAR_RAD_S:
                self.reset('moving')
                return False
            if self.start_stamp is None:
                self.start_stamp,self.start_wall = stamp,now
            self.samples += 1
            self.max_linear,self.max_angular = max(self.max_linear,speed),max(self.max_angular,turn)
            self.reason = 'settling'
            return True
        except (AttributeError,TypeError,ValueError,OverflowError):
            self.reset('invalid_or_unordered_odometry')
            return False

    def report(self, now, sim_ns, pose_stamp):
        base = {'settled':False,'reason':self.reason,'samples':self.samples,
                'generation':self.generation,'required_duration_s':SETTLE_SECONDS}
        if self.start_stamp is None:
            return base
        if (not math.isfinite(now) or not 0<=now-self.last_wall<MAX_GAP_SECONDS
                or sim_ns-self.last_stamp>=MAX_GAP_NS):
            self.reset('stale_odometry')
            return self.report(now,sim_ns,pose_stamp)
        if self.last_stamp>sim_ns or self.last_stamp<pose_stamp:
            return {**base,'reason':'awaiting_time_matched_odometry'}
        wall_span,sim_span = self.last_wall-self.start_wall,self.last_stamp-self.start_stamp
        settled = wall_span>=SETTLE_SECONDS and sim_span>=SETTLE_NS
        return {**base,'settled':settled,'reason':'settled' if settled else 'settling',
                'first_stamp_ns':self.start_stamp,'last_stamp_ns':self.last_stamp,
                'wall_span_s':wall_span,'simulation_span_s':sim_span/1e9,
                'received_age_s':now-self.last_wall,
                'max_linear_m_s':self.max_linear,'max_angular_rad_s':self.max_angular,
                'linear_limit_m_s':STATIONARY_LINEAR_M_S,'angular_limit_rad_s':STATIONARY_ANGULAR_RAD_S}
