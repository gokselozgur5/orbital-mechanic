# Guarded motion

## Policy (implemented, ROS integration pending)

`orbital/motion.py` owns the fail-closed state machine. It starts with a latched stop. An explicit reset requires fresh, stationary odometry. Reset clears old commands; it never resumes a previous velocity.

- Maximum linear speed: 0.2 m/s in either direction.
- Maximum angular speed: 0.5 rad/s in either direction.
- Command timeout: 350 ms of monotonic wall time, followed by a latched stop.
- Feedback timeout: 500 ms without an advancing odometry timestamp, followed by a latched stop. Replayed messages cannot refresh this timer.
- Commands carry simulation timestamps. Pre-arm, out-of-order, old (>200 ms behind odometry) and far-future (>100 ms ahead) commands are rejected and latch stop. The forward tolerance allows clock/odometry scheduling skew.
- Nonfinite and out-of-range values fail closed. A simulation-clock reset also latches stop.
- Reset is refused above 0.02 m/s or 0.05 rad/s measured speed.

The policy is independent of ROS and tested deterministically. A ROS adapter must use a steady-clock timer, validate unsupported axes and frames, obtain odometry itself and publish zero while stopped. Its process ownership must ensure a crashed guard cannot leave the simulated drive holding its last command.

This is not collision avoidance, a hardware emergency-stop circuit or a certified safety controller. Autonomous navigation and obstacle stopping are separate work. Do not expose raw motor inputs to an LLM.
