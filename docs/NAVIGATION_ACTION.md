# Nav2 action session — isolated protocol acceptance

`orbital/ros_navigation_action.py` wraps the real `NavigateToPose` action client around `NavigationPolicy`. It accepts named XY/yaw poses only, requires a simulation-time node, and does not publish velocities or arm the motion guard. The owning supervisor must supply verified health, perform real guard arm, and implement a nonblocking stop callback. This is a lifecycle component, not a complete navigator.

## Admission, cancellation and terminal ownership

`request()` reserves one target token containing run/epoch/sequence. `start()` requires that exact target, explicit policy arm and fresh policy health; the supervisor must already have acknowledged actual guard arm. A bounded monotonic goal deadline defaults to 120 seconds and accepts 0.1–300 seconds for commissioning. Invalid deadlines request stop without dispatch. The action pose is generated from the configured allowlist, not caller-supplied geometry.

Cancellation immediately requests stop and invalidates the policy target. If goal acceptance is still pending, the session retains ownership; a later accepted handle is cancelled immediately, while rejection proves no task was dispatched. Cancellation acknowledgment must name the exact goal UUID, but acknowledgment alone does not release it. The session waits for the matching action result to report succeeded, cancelled or aborted. Rejected/failed cancellation keeps the session occupied and stopped.

A terminal action still cannot be released until the caller supplies fresh measured stationary feedback and an engaged guard stop latch. Success is named `nav2_succeeded_unverified`, not arrival. Success arriving after a cancellation intent is `completed_after_cancel`. Actual pose/yaw/proximity checks remain the supervisor/evaluator's responsibility.

The velocity path must call `motion_permitted(target)` before forwarding each command. It evaluates deadlines immediately, so a delayed timer cannot briefly authorize an expired target. Old target tokens cannot cancel or release a subsequent task. Health loss requests cancellation through the same path.

An unresponsive or dead server does not constitute a terminal result. The session remains occupied, stopped and unable to start a new target. The owning service must escalate unresolved cancellation through controlled lifecycle teardown/restart of its Nav2 group. Destroying this client alone does not prove that a remote goal ended; shared process/cgroup ownership and shutdown acceptance remain required.

The implementation uses the [Jazzy rclpy asynchronous goal/cancel/result interfaces](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/action/client.py). It does not block inside ROS callbacks waiting for synchronous action calls.

## Test scope

`scripts/check-navigation-action.py` refuses any domain other than 84 and requires that domain to be empty. It starts a dedicated fake `NavigateToPose` server process and exercises the actual ROS action transport. The fake server delays acceptance, delays terminal cancellation after acknowledging it, rejects cancellation, returns success and is deliberately killed in a final case. It launches no controller, Gazebo world or velocity publisher. Its child process group is reaped on completion/failure.

Use the isolated environment from [ROS_LOCALIZATION_MONITOR.md](ROS_LOCALIZATION_MONITOR.md):

```bash
timeout --kill-after=5 60 /usr/bin/python3 scripts/check-navigation-action.py
```

This tests protocol/state ownership, not physical routes, stop-service composition, planner/controller configuration, obstacle avoidance or arrival accuracy. These components still need one supervisor that owns the sole Nav2-to-guard velocity path and couples health, stop/arm acknowledgments and exact action tokens.

## Measured acceptance — 2026-10-24

Thor `navigation-action-synthetic-3` exited 0 with 11 cases: acknowledgment versus terminal cancellation, cancellation before acceptance, rejected goal, deadline cancellation, deadline enforcement before a delayed timer, cancellation rejection, health loss, feedback-gated release/stale token rejection, invalid deadline, late success after cancellation, and server death without a fabricated terminal result. The stop callback was exercised 88 times; this counter is not evidence of physical stopping. The exact deployed source hashes match the committed candidates. All 57 ROS-free unit tests remain passing.
