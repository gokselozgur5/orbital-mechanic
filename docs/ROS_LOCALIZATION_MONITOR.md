# ROS localization observer

`orbital/ros_localization.py` connects the [health validator](LOCALIZATION_HEALTH.md) to real ROS messages and TF. `scripts/localization-monitor.py` runs it. It publishes `/navigation/health` JSON only; it does not publish velocity, TF or initial pose, call motion services, dispatch navigation, or read Gazebo truth.

The standalone command refuses an existing health publisher before starting. The observer also becomes unhealthy if its topic has multiple publishers. This detects accidental duplicate authorities; it is not ROS authentication. All local processes in the domain remain trusted.

## Wire contract

Each report includes `healthy`, `stationary`, `reason`, `sim_time`, `run_id`, `epoch` and `sequence`. A process restart changes `run_id`. A healthy-to-unhealthy transition, clock reset or initial-pose event advances `epoch`; later recovery preserves that new epoch. This retains evidence of invalidation even if a consumer misses the single unhealthy report.

A future navigation supervisor must bind a target to the health run/epoch, reject old or non-increasing report sequences, and invalidate its target on run/epoch changes. It must also expire health by its own monotonic receive deadline. Silence, reconnection or a later healthy report cannot authorize an old target. This observer alone never stops a robot: the supervisor still must enforce real guard stop and exact Nav2 cancellation.

ROS callbacks read actual `/scan`, `/odom`, `/imu`, `/amcl_pose`, `/initialpose` and `/clock`. Sensor subscriptions use sensor QoS; AMCL pose uses reliable transient-local delivery. Scan angle geometry is checked against the commissioned 360-ray full circle, and an IMU marking orientation unavailable is rejected. The steady timer runs every 50 ms, including during a paused simulation clock.

TF lookup is for simulation time minus 0.1 s, within the validator's 0.2 s bound, to allow the commissioned 20 Hz odometry transport to populate its buffer. A failed lookup is invalid, never replaced with an identity or cached success. Clock regression clears TF and health evidence immediately and requires explicit reseeding. An initial-pose event invalidates readiness; a later valid AMCL pose is not independent proof that the requested pose is correct or causally acknowledged. The evaluator must still measure localization accuracy.

## Isolated transport test

`scripts/check-localization-monitor.py` uses real DDS, ROS messages and tf2 with a synthetic room-consistent stream. It requires domain 84, checks the domain is empty before creating publishers, and refuses live domain 42. It starts no Gazebo process or system service and emits no velocity/guard commands. All nodes are destroyed when the test completes.

Run only in a dedicated empty test domain, from a Thor tmux window:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=84 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds-local.xml"
export ROS_LOG_DIR="$PWD/runtime/ros" TMPDIR="$PWD/runtime/tmp"
mkdir -p runtime/ros runtime/tmp
timeout --kill-after=5 60 /usr/bin/python3 scripts/check-localization-monitor.py
```

The test checks valid lookup and retained stationary pose; scan loss and replay; IMU tilt; TF loss; clock pause/reset; explicit reseeding; duplicate health authority; recovery epochs; and the monitor's actual publisher list. Synthetic transport acceptance does not prove AMCL accuracy, real cancellation/stop behavior, routes, collision avoidance or a completed navigation phase.

## Measured acceptance — 2026-10-24

Thor job `health-monitor-synthetic-2` exited 0 in the dedicated test checkout and domain 84. Scan loss/replay became unhealthy after 0.533/0.535 s, tilt after 0.042 s, missing TF after 0.165 s, clock pause after 0.491 s and duplicate authority after 0.042 s. These are observed test intervals including the 50 ms timer and transport scheduling, not hard real-time guarantees. Recovery retained the incremented epoch; reset required a new seed. The only monitor publishers were `/navigation/health`, `/parameter_events` and `/rosout`.

All 50 ROS-free unit tests also pass. The main demo and localization services remained active; no shared-world restart or motion was performed by these tests. Raw test output and exact deployed source hashes remain in private review artifacts. Real-world input integration and motion-stop enforcement are outstanding.
