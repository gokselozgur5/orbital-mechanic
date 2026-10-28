"""Read-only proximity evidence from health and exact-time localized robot pose.

The caller here is the trusted ROS adapter, never a planner supplying distance or
pose. A valid distance measurement alone does not authorize inspection/actions.
"""
import math

from orbital.health_lease import HealthLease
from orbital.inspection_geometry import ARRIVAL_XY_TOLERANCE_M, ARRIVAL_YAW_TOLERANCE_RAD
from orbital.localization import planar_pose_valid
from orbital.machines import MAX_CONTROL_DISTANCE_M
from orbital.navigation_client import _json

TF_DELAY_NS = 100000000
MAX_POSE_AGE_NS = 200000000


class Unavailable(ValueError):
    """A query lacks current measurement evidence."""


def timestamp(value):
    if type(value) is not int or not 0 <= value < 2**63:
        raise Unavailable('invalid_simulation_clock')
    return value


class ProximityEvidence:
    def __init__(self, registry, machine_id):
        matches = [b for b in registry.bindings if b.machine_id == machine_id]
        if len(matches) != 1:
            raise ValueError('unknown_machine')
        self.registry, self.machine = registry, matches[0]
        self.lease = HealthLease()
        self.source = None
        self.epoch_floor = None
        self.last_clock = None
        self.error = None

    def health(self, text, now):
        if self.error:
            return
        try:
            report = _json(text, 4096)
            if not HealthLease.valid_report(report):
                raise ValueError('invalid health schema')
            stamp = timestamp(round(report['sim_time']*1e9))
            source = report['run_id'], report['epoch']
            if self.source is None:
                self.source, self.epoch_floor = source, stamp
            elif self.source != source:
                self.error = 'localization_generation_changed'
                return
            accepted = self.lease.observe(report, now)
            if not accepted and report['healthy'] and self.lease.reason != 'ready':
                self.error = 'invalid_health_sequence_or_time'
        except (ValueError, TypeError, OverflowError):
            self.lease.invalidate('malformed_health')
            self.error = 'malformed_health'

    def requested(self, sim_ns):
        timestamp(sim_ns)
        if self.last_clock is not None and sim_ns < self.last_clock:
            self.error = 'simulation_regressed'
        self.last_clock = sim_ns
        if self.error:
            raise Unavailable(self.error)
        requested = sim_ns-TF_DELAY_NS
        if requested <= 0 or self.epoch_floor is None or requested < self.epoch_floor:
            raise Unavailable('awaiting_post_epoch_transform')
        return requested

    def measure(self, requested, sim_ns, now, transform):
        timestamp(requested)
        self.requested(sim_ns)
        self.lease.tick(now)
        # pending_stop belongs to motion ownership. This observer neither clears
        # it nor treats a distance observation as a stop acknowledgment.
        if not self.lease.healthy:
            raise Unavailable('localization_unavailable')
        health_stamp = timestamp(round(self.lease.sim_time*1e9))
        if not 0 <= sim_ns-health_stamp <= MAX_POSE_AGE_NS:
            raise Unavailable('health_simulation_time_mismatch')
        if not self.epoch_floor <= requested <= sim_ns or sim_ns-requested > MAX_POSE_AGE_NS:
            raise Unavailable('stale_or_pre_epoch_transform')
        header = transform.header
        sec, nsec = header.stamp.sec, header.stamp.nanosec
        if (type(sec) is not int or type(nsec) is not int or sec < 0 or not 0 <= nsec < 10**9
                or sec*10**9+nsec != requested or header.frame_id != 'map'
                or transform.child_frame_id != 'base_footprint'):
            raise Unavailable('transform_time_or_frame_mismatch')
        p, q = transform.transform.translation, transform.transform.rotation
        position, orientation = (p.x,p.y,p.z), (q.x,q.y,q.z,q.w)
        if (any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>10000
                for v in (*position,*orientation)) or not planar_pose_valid(position,orientation)):
            raise Unavailable('invalid_planar_transform')
        distance = math.dist(position[:2], self.machine.control)
        goal = self.machine.inspection
        # Homogeneous quaternion form avoids heading drift from the small norm
        # error permitted by the existing planar-pose validator; q and -q agree.
        x,y,z,w = orientation
        yaw = math.atan2(2*(w*z+x*y),w*w+x*x-y*y-z*z)
        xy_error = math.dist(position[:2],goal[:2])
        yaw_error = abs(math.remainder(yaw-goal[2],2*math.pi))
        within_distance = distance <= MAX_CONTROL_DISTANCE_M
        within_xy = xy_error <= ARRIVAL_XY_TOLERANCE_M
        within_yaw = yaw_error <= ARRIVAL_YAW_TOLERANCE_RAD
        pose_reason = ('outside_control_distance' if not within_distance else
                       'outside_inspection_position' if not within_xy else
                       'outside_inspection_heading' if not within_yaw else 'within_pose_bounds')
        return {'schema_version':2, 'valid':True, 'machine_id':self.machine.machine_id,
                'registry_identity':self.registry.identity, 'frame':'map',
                'control_xy_m':list(self.machine.control), 'robot_pose_xyz_xyzw':[*position,*orientation],
                'pose_stamp_ns':requested, 'observed_sim_stamp_ns':sim_ns,
                'health':{'run_id':self.lease.run_id, 'epoch':self.lease.epoch,
                          'sequence':self.lease.sequence, 'received_age_s':now-self.lease.received_at},
                'distance_m':distance, 'distance_limit_m':MAX_CONTROL_DISTANCE_M,
                'within_distance':within_distance,
                'within_inspection_pose':within_distance and within_xy and within_yaw,
                'inspection_pose':{'target_xy_yaw':list(goal), 'robot_yaw_rad':yaw,
                                   'xy_error_m':xy_error, 'xy_limit_m':ARRIVAL_XY_TOLERANCE_M,
                                   'yaw_error_rad':yaw_error, 'yaw_limit_rad':ARRIVAL_YAW_TOLERANCE_RAD,
                                   'reason':pose_reason},
                'scope':'localized distance and inspection pose bounds; not inspection or maintenance authorization'}
