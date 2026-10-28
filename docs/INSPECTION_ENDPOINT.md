# Owned inspection endpoint

OM-049 implements a bounded inspection endpoint over versioned JSON in ROS
`std_msgs/msg/String`. This follows the repository's existing local command
transport; it is **not a generated ROSIDL action type**. Scripted callers and
future planner adapters use the same request, cancellation and result rules.
Native DDS, service lifecycle and rendered inspection commissioning remain open.

The endpoint calls the actual [camera observation reader](CAMERA_OBSERVATIONS.md)
in a separate Python/ROS process. The server retains admission and terminal
ownership. A reader's per-query `rclpy.try_shutdown()` therefore cannot destroy
the long-lived server context. The server disables rclpy's automatic signal
shutdown so its own handlers can cancel and reap before destroying the context;
this option is defined in the
[Jazzy rclpy initialization API](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/__init__.py).

## Wire contract

All messages use `schema_version: 1`. IDs are 32 lowercase hexadecimal characters.
The server generates a new `run_id` at startup. Clients discover two advancing
heartbeats, pin the run and publisher GIDs, and require a state receipt under
300 ms old. Discovery/graph checks are sampled local evidence, not authentication.

| Topic | Direction | Meaning |
| --- | --- | --- |
| `/inspection/command` | Client → server | `inspect`, `cancel` or `query` |
| `/inspection/reply` | Server → client | Correlated admission/rejection and current action snapshot |
| `/inspection/result` | Server → client | One terminal result after exact worker reap |
| `/inspection/state` | Server → observers | 10 Hz bounded run/sequence, readiness, active action, last result and capacity |

Example command (replace the server run and use new mission/action/command IDs):

```json
{
  "schema_version": 1,
  "run_id": "11111111111111111111111111111111",
  "mission_id": "22222222222222222222222222222222",
  "action_id": "33333333333333333333333333333333",
  "command_id": "44444444444444444444444444444444",
  "op": "inspect",
  "arguments": {"machine_id": "pump", "timeout_s": 10.0}
}
```

`cancel` and `query` require the exact run/mission/action identity and empty
arguments. `command_id` correlates the transport reply; changing it does not
create a new action. Unknown fields, duplicate JSON keys, non-finite values,
invalid IDs, unknown machines and timeouts outside 0.1–10 seconds are rejected.
Commands are bounded to 4 KiB before decoding, with a character precheck before
UTF-8 encoding. ROS transport allocation occurs earlier and is not bounded by
this decoder. The server retains at most 16 queued commands and processes at
most four per ownership turn; queue overflow provides no acknowledgement.

One action is active at a time. The same action and normalized arguments return
its cached current/terminal snapshot without another capture. Reusing an action
ID with another mission or argument set is `action_conflict`. At most 64 admitted
actions are retained for the server run, with **no eviction/reexecution window**.
At capacity, explicitly restart the server after it is idle; a new run refuses
old-run commands. Existing capture storage has its separate 64-entry quota,
including partial bundles. Restart does not remove captures or reset that quota.

## Admission and completion

Admission starts one monotonic deadline covering initial settling, process
startup, capture, verification and final acceptance. The server independently
observes fresh localization health and stopped odometry, waits for a half-second
settling window, measures exact-time pose and pins the settling generation. It
continuously rechecks those preconditions while the worker is owned. Motion,
lost evidence, changed clock mode, source files, map files, publisher identities
or a competing inspection publisher refuse continuation. A later return to a
stopped pose does not erase an observed violation.

The worker uses the existing public telemetry, map binding, exact-time robot and
optical TF, image/CameraInfo, calibration, timestamp and storage checks. It binds
the following into the immutable capture manifest before publication:

- Server run, mission and action IDs, normalized request digest.
- Registry and sensor-authority identities pinned by the parent.
- Admission simulation/monotonic times and observation monotonic time.

It returns only a fixed observation ID and manifest digest. The parent reaps the
worker, reads and verifies the actual bounded manifest and RGB bytes, checks the
context against its owned request, then rechecks live robot/source state and
both clocks **after artifact I/O**. Image/info receipt ages include the elapsed
time since the recorded observation; both camera and telemetry simulation stamps
must still be within 200 ms at completion. A slow write, shutdown, pipe transfer
or parent-side read can therefore reject a valid earlier capture. No stored
timestamp is relabelled and no freshness bound is widened.

Terminal results contain the request identity/digest, machine, status, typed
error, start/finish simulation stamps, monotonic duration, observation reference
or null, and `state_change: null`. `succeeded` means a verified observation at the
recorded completion time. `failed`, `cancelled` and `timed_out` contain no accepted
observation reference. Unadmitted requests are rejected in their transport reply.
An ACK or a valid image file alone is never a terminal success.

## Cancellation, process loss and recovery

The first cancellation/deadline/failure intent is sticky. Cancellation processed
before terminal commitment wins even when the child has already exited and its
success bytes are waiting to be processed. An accepted cancel reply says only
that cancellation is underway. A live or unreaped worker continues to own the
slot and blocks a second action. Deadline expiry does not pretend the child exited.

The fixed executable receives a bounded private request through stdin. Stdout is
limited to 4 KiB and stderr to 8 KiB; malformed results, unexpected exit codes or
output overflow fail. No planner input selects executables, files, topics, poses,
images or calibrations. The exact `Popen` child receives SIGINT, then SIGTERM
after 250 ms and SIGKILL after 500 ms if necessary. An uninterruptible process
keeps ownership stopping until reaped; these are escalation intervals, not a
hard real-time termination guarantee.

The worker inherits the server's no-follow, single-link lifetime lock and a
parent lifeline pipe. Cancellation or pipe EOF interrupts capture. Closing the
parent descriptor does not unlock the child's inherited open file description,
so an overlapping server in the same unchanged checkout cannot acquire the
lock while the orphan worker remains. The user-service cgroup contains the
server and descendants. Lock/graph checks are not a defense against a privileged
or same-user attacker replacing the checkout or forging ROS traffic.

The client never automatically resubmits inspection. When a waiting inspect is
interrupted or loses a confirmed result, it attempts one exact cancellation and
waits briefly for the terminal result. If it cannot confirm that result, it
returns `uncertain`, including the action identity. Explicit `query` recovers
the cached result during the same server run. After restart, old actions are
unknown/wrong-run; orphan capture files are **not** promoted to success. Terminal
replay reports history and does not renew evidence freshness or action permission.

This endpoint only observes. It does not cancel navigation, hold the robot,
acknowledge a pending motion stop, diagnose a fault or authorize maintenance.
Cross-action mission ownership and interlocked maintenance remain later gates.

## Operator commands

After the native navigation/map/telemetry/camera prerequisites have been
explicitly deployed, seeded and accepted, inside Thor `tmux orbital-work`:

```bash
cd /home/jensen/.goske/codex/orbital-mechanic
python3 scripts/launch-inspection.py
source scripts/ros-env.sh
python3 scripts/inspect-machine.py status
python3 scripts/inspect-machine.py inspect pump --timeout 10
```

The launcher pins already-running demo, localization and navigation service
generations; it does not start or restart them. Stop the inspection service with
`systemctl --user stop orbital-inspection.service`. The foreground server is
`scripts/inspection-server.py`; use it only in the owned domain 42 or an explicitly
empty isolated localhost domain 84, preserving the user-service lifecycle contract.

`--no-wait` returns an admission receipt, not success. To recover or cancel,
copy the exact `run_id`, `mission_id` and `action_id` from that receipt:

```bash
python3 scripts/inspect-machine.py query --run-id RUN --mission-id MISSION --action-id ACTION
python3 scripts/inspect-machine.py cancel --run-id RUN --mission-id MISSION --action-id ACTION
```

## Evidence and limits

All 460 local tests pass, including 45 new endpoint/protocol/owner/client/guard
and process tests. The causal baseline, shell syntax, Python AST and XML checks
pass. Deliberately removing final freshness or cancellation-before-commit handling
makes the same relevant tests fail; the candidate passes both sensitivity controls.

Local tests cover strict input, replay/conflict/capacity, cancellation/result
races, deadlines, post-I/O expiry, generation/pose/source loss, client ACK versus
terminal/uncertain handling, bounded pipes, descriptor setup failures, signal
escalation, inherited lock lifetime and process reap. A real-clock controlled ROS
transport runs the production server, actual subprocess worker entry, actual
camera reader and real artifact I/O through success, cancellation, lifeline loss,
wrong-action evidence and a 250 ms parent-side read delay. Synthetic RGB and
controlled TF/map/health messages are test inputs; these are not native DDS,
rendered camera, stopping, three-machine tour or service-cgroup acceptance.

The initial late-read fixture sometimes failed earlier at sample selection:
the selected telemetry sample expired before a suitable post-settling camera
pair arrived. That failure log is retained. The fixture now schedules its first
telemetry pair after settling so the injected read delay actually exercises final
acceptance. Production timing, sample-selection rules and freshness limits are
unchanged. The complete acceptance receipts live outside Git under
`reviews/orbital-delivery-20261028/om049` in the operator workspace.
