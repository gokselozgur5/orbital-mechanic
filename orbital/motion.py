"""Fail-closed velocity policy. Callers supply monotonic wall time and trusted odometry.

This is a simulator command guard, not a certified collision avoidance controller.
"""
import math


class MotionGuard:
    COMMAND_TIMEOUT = 0.35
    FEEDBACK_TIMEOUT = 0.5
    MAX_LINEAR = 0.2
    MAX_ANGULAR = 0.5

    def __init__(self):
        self.latched = True
        self.reason = 'startup'
        self.feedback_at = None
        self.sim_time = None
        self.speed = (0.0, 0.0)
        self.command_at = None
        self.velocity = (0.0, 0.0)
        self.armed_after = None
        self.last_command_stamp = None

    def stop(self, reason='operator_stop'):
        self.latched = True
        self.reason = reason
        self.velocity = (0.0, 0.0)
        self.command_at = None

    def feedback(self, linear, angular, sim_time, now):
        if not all(math.isfinite(v) for v in (linear, angular, sim_time, now)):
            self.stop('invalid_feedback')
            self.feedback_at = None
            return
        if self.sim_time is not None and sim_time < self.sim_time:
            self.stop('simulation_reset')
        # Replayed/stalled samples must not keep a watchdog alive.
        if self.sim_time is None or sim_time != self.sim_time:
            self.feedback_at = now
            self.speed = (linear, angular)
            self.sim_time = sim_time

    def _fresh(self, now):
        return (math.isfinite(now) and self.feedback_at is not None
                and 0 <= now - self.feedback_at < self.FEEDBACK_TIMEOUT)

    def reset(self, now):
        if not self._fresh(now):
            self.stop('feedback_timeout')
            return False
        if abs(self.speed[0]) > .02 or abs(self.speed[1]) > .05:
            self.stop('reset_while_moving')
            return False
        self.latched = False
        self.reason = 'armed_waiting_for_command'
        self.velocity = (0.0, 0.0)
        self.command_at = None
        self.armed_after = self.sim_time
        self.last_command_stamp = None
        return True

    def command(self, linear, angular, stamp, now):
        # Enforce an expired deadline even if the periodic timer was delayed.
        self.output(now)
        if self.latched:
            return False
        if not self._fresh(now):
            self.stop('feedback_timeout')
            return False
        if (not all(math.isfinite(v) for v in (linear, angular, stamp, now))
                or abs(linear) > self.MAX_LINEAR or abs(angular) > self.MAX_ANGULAR):
            self.stop('invalid_command')
            return False
        if (stamp <= self.armed_after or not -.1 <= self.sim_time - stamp <= .2
                or (self.last_command_stamp is not None and stamp < self.last_command_stamp)):
            self.stop('stale_command')
            return False
        self.velocity = (linear, angular)
        self.command_at = now
        self.last_command_stamp = stamp
        self.reason = 'command_active'
        return True

    def output(self, now):
        if not self.latched:
            if not self._fresh(now):
                self.stop('feedback_timeout')
            elif self.command_at is not None and not 0 <= now - self.command_at < self.COMMAND_TIMEOUT:
                self.stop('command_timeout')
        return (0.0, 0.0) if self.latched else self.velocity
