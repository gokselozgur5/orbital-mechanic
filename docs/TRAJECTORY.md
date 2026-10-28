# Independent navigation trajectory evidence

OM-023 implements a read-only evaluator that records simulator truth, localization
at the same simulation timestamp and exact supervisor command identities. Local
tests pass. Native Gazebo Python bindings, DDS transport and moving-robot capture
have **not** been commissioned; Thor access is currently unavailable. This does
not accept a route, machine arrival, collision result or physical stopping test.

## Sources and timing

| Stream | Source | Meaning |
| --- | --- | --- |
| Truth | Gazebo `/world/maintenance/dynamic_pose/info`, `gz.msgs.Pose_V` | The exact `maintenance_robot` model pose, entity ID and simulation timestamp. Includes XYZ and the full quaternion. |
| Localization | TF `map -> base_footprint` | Interpolated lookup at each truth sample's exact simulation timestamp. Never uses latest TF or wheel odometry as ground truth. |
| State | ROS `/navigation/state` | Supervisor run, target, action state, health, guard observation, command/audit/forwarded counters and source simulation time. |
| Audit | ROS `/navigation/audit` | Each command callback result and each forwarded velocity, with the exact target, source stamp and ordered counters. |

Audit delivery uses volatile best-effort QoS with depth 100 so the observer does
not require reliable delivery acknowledgments on the control callback's audit
publication. Any observed sequence gap invalidates the capture. This is an
observability choice, not a bound on all native middleware publication latency.
The publisher/subscriber pair follows the
[ROS 2 QoS compatibility contract](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).

The world already loads SceneBroadcaster. Its dynamic-pose output contains model
and link entries, so the observer selects the exact model name and pins its entity
ID. The reviewed upstream implementation stamps pose batches with simulation time
and advertises the dynamic-pose topic. See the pinned
[SceneBroadcaster source](https://github.com/gazebosim/gz-sim/blob/43d9baf13eac134914ebbd826de800724fca0a87/src/systems/scene_broadcaster/SceneBroadcaster.cc).
These source observations do not establish the version of Thor's loaded binary.

The adapter uses Harmonic's `gz.transport13` and `gz.msgs10.pose_v_pb2` bindings;
subscription, unsubscribe and publisher discovery follow the reviewed
[Python wrapper](https://github.com/gazebosim/gz-transport/blob/869cb2eeb88601d7c17c1e77471abc89a0e864ef/python/src/__init__.py)
and [native bindings](https://github.com/gazebosim/gz-transport/blob/869cb2eeb88601d7c17c1e77471abc89a0e864ef/python/src/transport/_gz_transport_pybind11.cc).
Missing bindings produce a failed artifact before ROS observation starts. No
package installation, world edit or truth-to-ROS bridge occurs automatically.

World and map coordinates use the existing known-world contract. Grid origin is
an occupancy indexing offset, not a second transform of truth coordinates. The
truth model and `base_footprint` may differ in height; errors currently compare XY
and wrapped yaw, while all seven pose values remain in the trace for later body
stability analysis. This recorder does not infer collision absence between samples.

## Preparing a capture

After native commissioning, run inside `tmux orbital-work` on Thor from the
repository root. Deploy reviewed bytes, explicitly restart navigation and retain
its startup manifest. OM-021 also requires regenerated map artifacts, an explicit
localization reload and seed/map/readiness verification. Run the existing native
TF authority gate before moving trials. Do not replace an old startup manifest
just to make the recorder accept a mixed deployment.

```bash
source scripts/ros-env.sh
/usr/bin/python3 scripts/record-navigation-trajectory.py --name pump-trial-001 --duration 15
```

Use a new simple run name every time. Output is confined to
`runtime/evaluation/<name>` and an existing run is never overwritten. Production
scope is ROS domain 42, LOCALHOST discovery and Gazebo partition
`orbital-maintenance`. The CLI does not start services, seed localization, reserve
a target, arm, cancel, reset the world or send motor commands.

Warmup requires healthy idle navigation, a latched guard, fresh state, advancing
truth, a matched time-specific transform and unique expected state/audit/truth
publisher metadata. Command activity after the initial state baseline invalidates
warmup. Start the recorder before reserving a target; wait for its `ready.json`,
verify its recorded run ID/baseline against the current operator session, and keep
the recorder process under observation. A stale marker from another invocation is
not readiness. Armed trials still need the independent pidfd trial guardian and
their own exact-target cancellation/stop verification.

`--duration` is finite, 0.1–300 seconds, default 10, measured by the observer's
monotonic clock after readiness. `--warmup` is 0.1–30 seconds, default 10. A bounded
0.2–0.6 second post-roll allows TF and command/state callbacks to catch up after
truth intake closes. The artifact records truth cutoff and final counter fence
separately: audit events during post-roll are not extra physical trajectory data.
Preflight and final deployment checks have their own subprocess timeouts. These
are application wait bounds, not a real-time scheduling or native-library guarantee.

## Artifacts and refusal behavior

`events.jsonl` preserves raw normalized truth, state, audit and matched samples.
Record sequence is write order; receipt monotonic timestamps and each source's
simulation stamp remain separate. A matched sample points to its truth record and
the most recently received state **at truth receipt**, not a later state selected
after TF interpolation. These state references are observations, not simultaneous
snapshots or proof that a velocity reached the guard or actuator.

`summary.json` records file hash, byte/record counts, source/service/binding
identities before and after, the declared audit counter window, sample counts and
maximum truth sampling gaps. Completion requires:

- One advancing truth entity, one supervisor run and no clock/counter regression.
- Consecutive audit events after the declared baseline, exact target tokens and
  matching command/forwarded counters at the final received state.
- At least two capture samples, fresh navigation state at each truth receipt and
  no simulation or receipt gap greater than 250 ms during capture.
- Exact-time TF for every capture truth sample; interpolation can wait at most
  0.5 seconds before recording an explicit missing sample and failing.
- Unchanged observed service invocations, navigation source/installed artifact
  identity and loaded binding-file hashes across the run.

Queues are bounded to 2,048 input records and 512 unmatched truth samples. Output
is limited to 100,000 records, 64 MiB and 64 KiB per record; JSON inputs are bounded
to 16,384 characters and incoming decoded pose batches to 1 MiB. These limits
cover application queues/output, not all allocations inside ROS/Gazebo/protobuf.
Queue overflow, callback exceptions, malformed input, missing peers, restarted
owners, changed files or missing audit tails fail visibly. Previously written
evidence is retained. No extrapolated/latest pose or zero error substitutes for a
missing transform.

Exit 0 with `complete: true` means the declared observation/integrity checks
passed. It does not mean a navigation or maintenance task succeeded. Exit 1 means
capture/preflight failed; exit 2 means argument/environment/output validation
failed; interruption is 130. Ctrl-C ends observation and **does not cancel an armed
goal**. An interrupted process may leave only a JSONL prefix or partial marker;
absence of a valid complete summary is incomplete evidence.

Verify a saved run without ROS/Gazebo:

```bash
python3 scripts/record-navigation-trajectory.py --verify runtime/evaluation/pump-trial-001
```

The verifier bounds input, checks record order and compares the exact JSONL hash,
bytes and count with the complete summary. This proves self-consistency, not
authenticity, replay protection or independent semantic re-evaluation of a forged
manifest. Sampled graph metadata does not authenticate processes or attribute each
message to its publisher. File fences do not prove in-memory source or the loaded
world; the existing deployment and TF checks remain necessary.

## Local acceptance and remaining native gate

The 204-test local suite passes. New tests exercise the production supervisor's
request, held arm, forwarding and cancellation callbacks; an audit publication
failure preserves cancellation and makes subsequent state explicitly unsuitable
for complete evidence. Recorder tests cover clock/entity changes, missing/duplicate
audit events, exact target identity, TF timing, interpolation wait, malformed
callbacks, concurrency/overflow, bounded output, tampering and interrupted capture.
The actual native adapter is executed with controlled ROS/Gazebo transports and
proves it creates no control publisher or service client.

Those doubles do not establish native ABI, transport QoS, delivery rate or moving
robot accuracy. Next native acceptance must verify installed Python bindings,
capture an idle control with independent endpoint comparison, then a guarded
bounded movement with measured displacement and exact target audit. Preserve a
negative run with unavailable/wrong-time TF and prove its failure. Only after that
should this recorder supply physical-loss stopping and three-machine arrival
evidence. Collision evaluation remains OM-027.
