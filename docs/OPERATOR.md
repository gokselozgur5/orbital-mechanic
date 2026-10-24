# Bounded operator CLI

Run on Thor inside `tmux orbital-work`, from the project root:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 scripts/orbital-operator.py status
/usr/bin/python3 scripts/orbital-operator.py arm
/usr/bin/python3 scripts/orbital-operator.py jog forward --duration 0.5
/usr/bin/python3 scripts/orbital-operator.py stop
```

Directions are `forward`, `reverse`, `left`, `right`. Linear speed is fixed at 0.12 m/s and turning at 0.3 rad/s. Duration defaults to 0.5 seconds and must be finite, positive and at most 2 seconds. Every jog ends with a stop latch; arm explicitly before each next movement. Arm alone does not move. The client waits for simulation clock, fresh guard state and odometry; it publishes only stamped `/motion/request`, never raw drive output.

`status` reports stop reason, commanded and measured velocity, and camera/lidar message counts observed by that short-lived client. These counts are not frame-rate guarantees. Use the existing Gazebo/Foxglove views for scene and sensors; the composed operator view is still pending under issue #5. This CLI does not complete that issue.

Normal completion requests stop. Interruption or loss of the client is also covered by the independent guard's command timeout; the client does not own or disable that watchdog. This is simulation-only, with no collision avoidance yet.

## Measured exercise

Start a fresh world at the clear origin, then run `/usr/bin/python3 scripts/check-operator.py`. Thor tmux job `operator-acceptance-4` exited 0 on 2026-10-24:

- A jog while latched was rejected.
- 0.7-second forward/reverse jogs moved 0.0836 / 0.0843 m and ended stationary/latched.
- Left/right turns passed measured yaw magnitude and direction checks, then stopped.
- Killing the real operator subprocess during forward motion produced `command_timeout`, observed stop after 0.4394 seconds and 0.0497 m.

Earlier test attempts failed before adequate diagnosis was captured: one did not observe motion before disconnect; another failed the disconnect observation without detailed state being captured; a subsequent attempt correctly refused a rover outside the clear-origin precondition. The final harness keeps ROS callbacks active during the stop measurement and the final exercise ran in a fresh world. Raw evidence remains outside Git. User-cancel signal variants and the combined viewer remain follow-up acceptance work.
