# Guarded motion

## Policy

`orbital/motion.py` owns the fail-closed state machine. It starts with a latched stop. An explicit reset requires fresh, stationary odometry. Reset clears old commands; it never resumes a previous velocity.

- Maximum linear speed: 0.2 m/s in either direction.
- Maximum angular speed: 0.5 rad/s in either direction.
- Command timeout: 350 ms of monotonic wall time, followed by a latched stop. A newly arriving command also evaluates the old deadline, so a delayed timer cannot conceal a command gap.
- Feedback timeout: 500 ms without an advancing odometry timestamp, followed by a latched stop. Replayed messages cannot refresh this timer.
- Commands carry simulation timestamps. Pre-arm, out-of-order, old (>200 ms behind odometry) and far-future (>100 ms ahead) commands are rejected and latch stop. The forward tolerance allows clock/odometry scheduling skew.
- Nonfinite and out-of-range values fail closed. A simulation-clock reset also latches stop.
- Reset is refused above 0.02 m/s or 0.05 rad/s measured speed.

The policy is independent of ROS and tested deterministically. A ROS adapter must use a steady-clock timer, validate unsupported axes and frames, obtain odometry itself and publish zero while stopped. Its process ownership must ensure a crashed guard cannot leave the simulated drive holding its last command.

This is not collision avoidance, a hardware emergency-stop circuit or a certified safety controller. Autonomous navigation and obstacle stopping are separate work. Do not expose raw motor inputs to an LLM.

Policy review for issue #2 reproduced a late-command deadline bypass when the periodic tick was delayed. The regression fails before the fix and passes after it; the policy now checks its existing deadline before accepting another command. This review does not claim ROS or physical stop acceptance.

## ROS integration

`scripts/motion-guard.py` uses a 20 Hz steady-clock timer, independent of `/clock`. It consumes odometry with `odom` / `base_footprint` frames and computes the magnitudes of all measured velocity axes before allowing reset. Invalid frame feedback clears the freshness credit; valid advancing feedback is required again.

Input is `geometry_msgs/TwistStamped` on `/motion/request`, with frame `base_link`, linear X and angular Z only. Unsupported axes, wrong frames, nonfinite values, stale stamps and speed violations latch the stop. `/motion/stop` and `/motion/reset_stop` are `std_srvs/Trigger` services; `/motion/state` reports the latched state and reason as JSON. Reset alone produces zero velocity.

Only `/motion/guarded_cmd_vel` is bridged to Gazebo `/demo/private/cmd_vel`. This is a trusted-local-process boundary, not authentication: another process with local ROS/Gazebo access can bypass topic conventions. Do not grant a planner arbitrary ROS publishing or shell access. Both command transport and the simulator remain local to Thor.

The guard runs inside the existing `orbital-demo` user-service cgroup. The supervisor waits for any component to exit, then tears down the demonstration; systemd owns all descendants. Fault-injection proof of guard death, pause/resume and simulation reset is recorded in [MOTION_EVIDENCE.md](MOTION_EVIDENCE.md).

Run `scripts/check-motion.py` with the ROS environment and domain 42, using system Python, from tmux. It moves only the simulated rover near the clear startup origin and leaves it latched stopped. It rejects a starting position outside a 0.25 m radius. The harness checks forward/reverse motion, timeout, blocked late commands, explicit stop, rotation, overspeed, wrong frame, unsupported axis and stale timestamps. Do not use it for physical hardware.

For simulator-only fault injection, run `python3 scripts/check-motion-failures.py` from `tmux orbital-work`, with ROS domain 42 and Gazebo partition `orbital-maintenance`. This intentionally kills the guard identified inside the demo cgroup, then restarts the full demo. It emits source hashes, limits, measurements and failures. Do not run it while another operator is exercising the robot. Gazebo reset uses the official [WorldControl transport API](https://gazebosim.org/api/sim/8/reset_simulation.html).
