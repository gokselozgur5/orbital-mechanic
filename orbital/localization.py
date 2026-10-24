"""ROS-free validation of trusted localization observations.

This checks freshness and consistency, not localization accuracy. A ROS adapter
must supply actual messages and a successful time-specific TF lookup; planner
arguments are never evidence. No command or target is emitted by this module.
"""
from dataclasses import dataclass
import math


def finite(values, size):
    return len(values) == size and all(math.isfinite(v) for v in values)


def quaternion_valid(q):
    return finite(q, 4) and abs(sum(v*v for v in q)-1.) <= .001


def level_orientation(q, limit=.05):
    if not quaternion_valid(q):
        return False
    x,y,z,w = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
    return abs(roll) < limit and abs(pitch) < limit


def planar_pose_valid(position, orientation):
    return finite(position, 3) and abs(position[2]) < .05 and level_orientation(orientation)


def planar_covariance_valid(cov):
    """Check the x/y/yaw marginal is symmetric, PSD and within commissioning limits."""
    if not finite(cov, 36):
        return False
    indices = (0, 1, 5)
    a = [[cov[i*6+j] for j in indices] for i in indices]
    bounds = (.25**2, .25**2, .35**2)
    if any(not 0 <= a[i][i] <= bounds[i] for i in range(3)):
        return False
    if any(abs(a[i][j]-a[j][i]) > 1e-9 for i in range(3) for j in range(3)):
        return False
    # Normalize by permitted variance so the numerical tolerance has a defined scale.
    a = [[a[i][j]/math.sqrt(bounds[i]*bounds[j]) for j in range(3)] for i in range(3)]
    if any(a[i][i]*a[j][j]-a[i][j]**2 < -1e-9 for i in range(3) for j in range(i)):
        return False
    det = (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])
           -a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
           +a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]))
    return det >= -1e-9


@dataclass(frozen=True)
class TransformSample:
    stamp: float
    parent: str
    child: str
    position: tuple
    orientation: tuple


@dataclass(frozen=True)
class Health:
    healthy: bool
    stationary: bool
    reason: str
    sim_time: float


@dataclass(frozen=True)
class Sample:
    stamp: float
    received_at: float
    valid: bool


class LocalizationHealth:
    INPUT_TIMEOUT = .5
    TF_MAX_AGE = .2
    MAX_TILT = .05

    def __init__(self):
        self.samples = {}
        self.last_wall = None
        self.last_sim = None
        self.stationary = False
        self.awaiting_seed = False
        self.seed_stamp = None
        self.pending_fault = None

    def reset(self):
        """Explicit initial-pose replacement must discard all pre-seed evidence."""
        self.samples.clear()
        self.stationary = False
        self.pending_fault = None

    def replace_initial_pose(self, stamp):
        """Require an AMCL result at/after this explicit seed, never a retained old pose."""
        self.reset()
        self.awaiting_seed = True
        self.seed_stamp = stamp if math.isfinite(stamp) and stamp >= 0 else None

    def clock_reset(self, sim_time):
        """Discard the old epoch immediately when the ROS clock callback regresses."""
        self.reset()
        self.last_sim = sim_time
        self.awaiting_seed = True
        self.seed_stamp = None
        return Health(False, False, 'simulation_reset', sim_time)

    def _record(self, name, stamp, now, valid):
        old = self.samples.get(name)
        valid = bool(valid and math.isfinite(stamp) and stamp >= 0 and math.isfinite(now)
                     and (self.last_wall is None or now >= self.last_wall))
        if not valid:
            self.pending_fault = self.pending_fault or name+'_invalid'
        if old is not None:
            if name != 'pose' and now-old.received_at >= self.INPUT_TIMEOUT:
                self.pending_fault = self.pending_fault or name+'_stale'
            if not valid or stamp < old.stamp or now < old.received_at:
                self.samples[name] = Sample(stamp, now, False)
                self.pending_fault = self.pending_fault or name+'_invalid'
                return False
            if stamp == old.stamp:
                # A duplicated message cannot revive invalid evidence or extend a lease.
                return False
        self.samples[name] = Sample(stamp, now, valid)
        return valid

    def scan(self, *, stamp, now, frame, ranges, range_min, range_max):
        sane = (frame == 'lidar_link' and math.isfinite(range_min) and math.isfinite(range_max)
                and 0 < range_min < range_max and len(ranges) == 360)
        if sane:
            sane = (all((math.isfinite(v) and range_min <= v <= range_max) or v == math.inf for v in ranges)
                    and sum(math.isfinite(v) for v in ranges) >= 180)
        return self._record('scan', stamp, now, sane)

    def odometry(self, *, stamp, now, parent, child, position, orientation, linear, angular):
        valid = (parent == 'odom' and child == 'base_footprint' and planar_pose_valid(position, orientation) and finite(linear, 3) and finite(angular, 3))
        accepted = self._record('odom', stamp, now, valid)
        if accepted:
            self.stationary = math.hypot(*linear) <= .01 and math.hypot(*angular) <= .02
        elif not valid:
            self.stationary = False
        return accepted

    def imu(self, *, stamp, now, frame, orientation):
        valid = frame == 'base_link' and level_orientation(orientation, self.MAX_TILT)
        return self._record('imu', stamp, now, valid)

    def pose(self, *, stamp, now, frame, position, orientation, covariance):
        valid = (frame == 'map' and planar_pose_valid(position, orientation)
                 and planar_covariance_valid(covariance))
        if self.awaiting_seed or self.seed_stamp is not None:
            valid = valid and self.seed_stamp is not None and stamp >= self.seed_stamp
        accepted = self._record('pose', stamp, now, valid)
        if accepted:
            self.awaiting_seed = False
        return accepted

    def evaluate(self, *, now, sim_time, transform):
        def result(reason):
            return Health(reason == 'ready', self.stationary if reason == 'ready' else False, reason, sim_time)
        if not math.isfinite(now) or (self.last_wall is not None and now < self.last_wall):
            self.reset()
            return result('wall_clock_invalid')
        self.last_wall = now
        if not math.isfinite(sim_time) or sim_time < 0:
            self.reset()
            return result('simulation_clock_invalid')
        if self.last_sim is not None and sim_time < self.last_sim:
            return self.clock_reset(sim_time)
        self.last_sim = sim_time
        if self.awaiting_seed:
            return result('initial_pose_required')
        if self.pending_fault is not None:
            reason = self.pending_fault
            self.pending_fault = None
            return result(reason)
        for name in ('scan', 'odom', 'imu', 'pose'):
            sample = self.samples.get(name)
            if sample is None or not sample.valid:
                return result(name+'_invalid')
            if sample.stamp > sim_time+.1 or sample.received_at > now:
                self.samples[name] = Sample(sample.stamp, sample.received_at, False)
                return result(name+'_future')
            # AMCL may retain a pose while stationary; current scan/IMU/odom/TF
            # remain mandatory. Covariance is a sanity bound, never accuracy proof.
            if name != 'pose' and (now-sample.received_at >= self.INPUT_TIMEOUT
                                   or sim_time-sample.stamp >= self.INPUT_TIMEOUT):
                return result(name+'_stale')
        if (not isinstance(transform, TransformSample) or transform.parent != 'map'
                or transform.child != 'base_footprint' or not math.isfinite(transform.stamp)
                or not 0 <= sim_time-transform.stamp <= self.TF_MAX_AGE
                or not planar_pose_valid(transform.position, transform.orientation)):
            return result('transform_invalid')
        return result('ready')
