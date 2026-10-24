"""ROS-free target lifecycle policy; not a deployed navigation controller.

The adapter supplies trusted localization health and stationary feedback, calls
``tick`` on a steady timer, and enforces ``stop_required`` through the motion
guard. It must confirm a cancelled goal is terminal (or was never dispatched).
A cancel-request acknowledgment alone is insufficient. No velocities are emitted.
"""
from dataclasses import dataclass
import math
import uuid


@dataclass(frozen=True)
class Target:
    run_id: str
    epoch: int
    sequence: int
    waypoint: str


class NavigationPolicy:
    HEALTH_TIMEOUT = 0.5

    def __init__(self, waypoints):
        self.waypoints = frozenset(waypoints)
        if not self.waypoints or any(not isinstance(w, str) or not w for w in self.waypoints):
            raise ValueError('Named waypoints are required')
        self.run_id = str(uuid.uuid4())
        self.epoch = 0
        self.sequence = 0
        self.target = None
        self.cancel_pending = None
        self.armed = False
        self.reason = 'startup'
        self.health_at = None
        self.sim_time = None
        self.last_wall = None
        self._healthy = False

    @property
    def stop_required(self):
        return not self.armed

    def _invalidate(self, reason):
        # Preserve cancellation ownership until the matching terminal result.
        if self.target is not None:
            self.cancel_pending = self.target
        if self.target is not None or self._healthy or self.armed:
            self.epoch += 1
        self.target = None
        self.armed = False
        self._healthy = False
        self.health_at = None
        self.reason = reason

    def _clock_ok(self, now):
        if not math.isfinite(now) or (self.last_wall is not None and now < self.last_wall):
            self._invalidate('invalid_wall_clock')
            return False
        self.last_wall = now
        return True

    def tick(self, now):
        if not self._clock_ok(now):
            return False
        if self._healthy and (self.health_at is None or not 0 <= now-self.health_at < self.HEALTH_TIMEOUT):
            self._invalidate('localization_timeout')
        return self._healthy

    def observe(self, *, sim_time, valid, now):
        """Supply an advancing trusted health sample, never planner arguments.

        ``valid`` must include fresh scan reception, current TF lookup and finite
        accepted pose/covariance. This policy does not calculate those predicates.
        A sample arriving after the deadline cannot conceal the expired lease.
        """
        self.tick(now)
        if not math.isfinite(now) or self.last_wall is None or now != self.last_wall:
            return False
        if not math.isfinite(sim_time) or not isinstance(valid, bool) or not valid:
            self._invalidate('localization_invalid')
            return False
        if self.sim_time is not None and sim_time < self.sim_time:
            self._invalidate('simulation_reset')
            self.sim_time = sim_time
            return False
        if self.sim_time is not None and sim_time == self.sim_time:
            return False
        self.sim_time = sim_time
        self.health_at = now
        self._healthy = True
        # Fresh evidence may make a new request possible, never re-arm old work.
        return True

    def replace_initial_pose(self):
        self._invalidate('initial_pose_replaced')
        # Require a subsequent advancing health sample before accepting a target.

    def request(self, waypoint, *, now):
        if not self.tick(now) or self.target is not None or self.cancel_pending is not None:
            return None
        if not isinstance(waypoint, str) or waypoint not in self.waypoints:
            return None
        self.sequence += 1
        self.target = Target(self.run_id, self.epoch, self.sequence, waypoint)
        self.armed = False
        self.reason = 'target_waiting_for_arm'
        return self.target

    def arm(self, target, *, stationary, now):
        if (not self.tick(now) or self.cancel_pending is not None or target != self.target
                or self.target is None or stationary is not True):
            return False
        self.armed = True
        self.reason = 'target_active'
        return True

    def cancel(self, reason='operator_stop'):
        self._invalidate(reason)

    def confirm_terminal(self, target):
        """Adapter confirms exact cancelled task is terminal/not dispatched."""
        if target is None or target != self.cancel_pending:
            return False
        self.cancel_pending = None
        return True

    def finish(self, target, *, now):
        """Close an active task and require stop, without asserting goal arrival.

        Success still requires measured position/orientation and stationary
        feedback in the adapter/evaluator. A Nav2 success flag is not that proof.
        """
        if not self.tick(now) or target is None or target != self.target or not self.armed:
            return False
        self.target = None
        self.armed = False
        self.reason = 'target_terminal'
        return True
