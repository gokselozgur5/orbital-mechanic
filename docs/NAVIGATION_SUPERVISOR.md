# Composed navigation supervisor

This experimental component joins localization health, exact Nav2 action ownership and the only navigation velocity publisher. It is tested with real ROS transport and the real motion guard, but is not installed in the shared Gazebo demo. Real Nav2 routes and collision acceptance remain outstanding.

## Ownership

`orbital/ros_navigation_supervisor.py` consumes `/navigation/raw_cmd_vel` (`TwistStamped`) from exactly one publisher named `controller_server`. The controller must have no direct drive connection. The supervisor alone relays navigation requests to `/motion/request`; the independent motion guard owns `/motion/guarded_cmd_vel`. These are trusted local ROS processes, not authenticated identities. Production integration must exclude competing manual publishers and put the controller, supervisor and their descendants under explicit lifecycle ownership.

A command is forwarded only for the current running action, fresh valid localization and fresh unlatched guard feedback. It must be planar, finite, within ±0.2 m/s and ±0.5 rad/s, in the expected body frame, no older than 0.2 simulation seconds, and no earlier than action dispatch. Future tolerance is 0.1 s; timestamps cannot regress. Inputs are not restamped to hide age. Invalid commands cancel the exact target and request stop.

Health loss stops publication as well as requesting the guard stop service. If that service is unavailable, the guard watchdog can therefore stop the stream. Supervisor process death also ends publication even if the controller continues publishing raw commands. This assumes the deployed topic graph has no bypass or competing command publisher.

## Operator protocol

Read `/navigation/state` for the current `run_id`, `command_sequence` and exact `target`. Publish JSON in `std_msgs/String` to `/navigation/command`, with sequence exactly one greater than the reported command sequence:

```json
{"run_id":"CURRENT_RUN","sequence":1,"op":"request","waypoint":"NAMED_WAYPOINT"}
```

Request reserves a target without moving. Arm and cancel must echo its full `{run_id, epoch, sequence, waypoint}` identity in a `target` field instead of `waypoint`. Arbitrary poses and extra fields are rejected. Commands are limited to 4096 characters. A well-formed command consumes its sequence even when the requested action is refused; inspect `last_reply.accepted`. Malformed, replayed and wrong-run messages do not consume sequence. This is a single-operator protocol, not a concurrent-client arbitration service.

Arm requires fresh healthy stationary feedback, a latched guard and the guard reset service. Dispatch waits for both a successful asynchronous arm response and fresh unlatched guard feedback after the request. An unresolved arm call prevents task release, even after timeout; a late response triggers stop. Cancellation acknowledgment alone never releases Nav2 ownership: the exact result must be terminal, then fresh stationary and latched feedback must be observed. A dead remote action server can leave the task intentionally occupied; production recovery must tear down its owned Nav2 process group before permitting a replacement.

Nav2 success is checked against a time-specific `map` to `base_footprint` transform. XY error must be at most 0.10 m and yaw error at most 0.15 rad. The result is `arrived` or `arrival_not_verified`. This checks the localization estimate, not independent Gazebo truth or collision-free travel. The evaluator must add those measurements for real route acceptance.

## Measured isolated acceptance

Thor job `navigation-supervisor-synthetic-1` exited 0 on 2026-10-24, in an initially empty ROS domain 84 and separate source directory. `scripts/check-navigation-supervisor.py` starts synthetic sensors, the actual localization monitor and motion guard, a continuously publishing fake controller and a separate fake Nav2 action server. No Gazebo or actuator bridge is present.

Eight cases passed:

- Raw velocity blocked without a target and explicit arm.
- Scan loss stops guarded output in 0.499 s; recovery cannot re-arm the old target.
- Stale controller timestamps cancel the target.
- Loss of the stop service still reaches zero output through `command_timeout` in 0.928 s.
- Nav2 success away from the measured target is rejected.
- Matching measured TF is required for reported arrival.
- Exact-target cancellation stops and releases only after terminal completion.
- SIGKILL of the supervisor stops guarded output in 0.377 s while raw controller commands continue.

Times describe this synthetic ROS run, not physical braking or universal bounds. The test reaps its child process groups on exit. Source hashes and raw evidence remain in private review artifacts; no credentials, recordings or generated runtime outputs belong in Git.

To reproduce, use a dedicated checkout on a ROS 2 Jazzy host, source `/opt/ros/jazzy/setup.bash`, set `ROS_DOMAIN_ID=84` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and use the project Fast DDS profile. The script refuses a nonempty domain. Run `timeout --kill-after=5 90 /usr/bin/python3 scripts/check-navigation-supervisor.py` inside the operational tmux session. It is a test harness, not a production navigation launcher.

## Remaining deployment gates

Resolve shared-world ownership before integrating. Configure actual Nav2 planner/controller/behavior-tree lifecycle and stamped output remapping; exclude alternate command paths; verify startup, teardown and unresolved-goal recovery. Exercise localization reset/reseed and the unseeded negative gate in Gazebo. Then independently measure arrival at all three machines, obstacle behavior, collisions, physical stopping and failure recovery. Phase 2 remains open, and no LLM or maintenance mission has been executed.
