# Independent navigation trajectory evidence

OM-023 implements a read-only evaluator that records simulator truth, localization
at the same simulation timestamp and exact supervisor command identities. OM-027
extends it with observed link/obstacle poses and a versioned geometry snapshot for
[sampled collision evaluation](COLLISION_EVALUATION.md). Local tests pass.
Native bindings and an idle observer candidate have now been exercised on Thor.
The first installed-recorder attempt exposed the light-visual regression below;
the corrected installed capture, negative TF controls and moving-robot acceptance
remain pending. No route, machine arrival or physical stopping test is accepted.

## Native light-visual regression — 2026-10-29

The first actual capture on deployed `0373a75` refused the scene before recording
any samples. Five retained Gazebo protobuf batches contain 35 entities, including
`sunVisual`; the declared topology expected only 34. Gazebo's
[SdfEntityCreator](https://github.com/gazebosim/gz-sim/blob/43d9baf13eac134914ebbd826de800724fca0a87/src/SdfEntityCreator.cc#L660)
creates a child visual named `<light-name>Visual` for each declared light.

The parser now requires those exact generated names in its complete multiset.
It does not allow arbitrary extra visuals. Missing, duplicated or renamed light
visuals and aliases of collision-model names are refused. The same three new
regressions fail on the old parser; all 79 geometry/evaluator/trajectory tests
pass after the correction. The fixture now includes the generated light visual.

The old parser rejects all five retained native batches; the candidate accepts
each complete scene. A separately identified observer-only module override then
records 278 capture samples over five seconds with exact-time TF, zero command
activity and no sampled geometry event. Maximum truth gaps are 20 ms simulation
and 28.4 ms receipt time. Before/after live inventory and stationary pose checks
pass; its disposable cgroup exits. The trace explicitly records the candidate
module hash separately from the unchanged installed-source manifest.

Evidence: operator workspace
`reviews/orbital-navigation-rollout-20261029/observer-fix-evidence`, 71-file manifest
SHA-256 `d15d57cf89d362c01e51d8298ba776e7f8719fe3d0630b5023f707b997d9c1be`.
This candidate proof does not establish installed deployment, negative-TF,
moving-robot or between-sample collision acceptance.

## Sources and timing

| Stream | Source | Meaning |
| --- | --- | --- |
| Truth | Gazebo `/world/maintenance/pose/info`, `gz.msgs.Pose_V` | Robot model, actual robot links and static model/link poses at one simulation timestamp, with entity IDs and full XYZ/quaternions. |
| Localization | TF `map -> base_footprint` | Interpolated lookup at each truth sample's exact simulation timestamp. Never uses latest TF or wheel odometry as ground truth. |
| State | ROS `/navigation/state` | Supervisor run, target, action state, health, guard observation, command/audit/forwarded counters and source simulation time. |
| Audit | ROS `/navigation/audit` | Each command callback result and each forwarded velocity, with the exact target, source stamp and ordered counters. |

Audit delivery uses volatile best-effort QoS with depth 100 so the observer does
not require reliable delivery acknowledgments on the control callback's audit
publication. Any observed sequence gap invalidates the capture. This is an
observability choice, not a bound on all native middleware publication latency.
The publisher/subscriber pair follows the
[ROS 2 QoS compatibility contract](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst).

The world already loads SceneBroadcaster. The full pose output includes models,
links, visuals and lights, including each generated light child visual. The
observer checks the complete source-derived name multiset
and pins entity generations; it preserves all collision-bearing model/link poses.
Unexpected, missing or replaced scene entities invalidate capture. The reviewed
upstream implementation stamps each batch with simulation time. See the pinned
[SceneBroadcaster source](https://github.com/gazebosim/gz-sim/blob/43d9baf13eac134914ebbd826de800724fca0a87/src/systems/scene_broadcaster/SceneBroadcaster.cc).
These source observations do not establish the version of Thor's loaded binary.

Robot link poses are relative to their parent model; top-level model poses are
world-relative. The composition follows the reviewed
[physics pose update](https://github.com/gazebosim/gz-sim/blob/43d9baf13eac134914ebbd826de800724fca0a87/src/systems/physics/Physics.cc#L3390).
Static models may move/rotate and their observed poses are retained. Static links
must retain their declared relative transforms; a moving floor invalidates the
current support policy. The source geometry is fixed during a capture.

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
maximum truth sampling gaps. It also stores the bounded SDF source and SHA used
for collision geometry, checked against the verified navigation startup manifest.
The supported scene parser rejects ambiguous/unsupported geometry and frame
semantics. File/topology observations do not prove loaded collision geometry in
physics memory. Completion requires:

- One advancing truth entity, one supervisor run and no clock/counter regression.
- Consecutive audit events after the declared baseline, exact target tokens and
  matching command/forwarded counters at the final received state.
- At least two capture samples, fresh navigation state at each truth receipt and
  no simulation or receipt gap greater than 250 ms during capture.
- Exact-time TF for every capture truth sample; interpolation can wait at most
  0.5 seconds before recording an explicit missing sample and failing. An executor
  delay cannot make a late but now-available transform acceptable.
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

The 244-test local suite passes. Tests exercise the production supervisor's
request, held arm, forwarding and cancellation callbacks; an audit publication
failure preserves cancellation and makes subsequent state explicitly unsuitable
for complete evidence. Recorder tests cover clock/entity changes, missing/duplicate
audit events, exact target identity, TF timing, interpolation wait, malformed
callbacks, concurrency/overflow, bounded output, tampering and interrupted capture.
The actual native adapter is executed with controlled ROS/Gazebo transports and
proves it creates no control publisher or service client.

The strict TF wait regression fails on `5ff3a8e` and passes after the OM-027
correction. The old code could accept a successful lookup after its declared wait
when the observer executor resumed late. The evaluator also replays timestamps,
sample references and the original protocol counters before accepting a report.
Legacy model-only traces remain readable by the integrity verifier, but lack the
link/world evidence required for collision evaluation; recapture with this source.

Those doubles do not establish native ABI, transport QoS, delivery rate or moving
robot accuracy. The native candidate proof above exercises bindings and idle
capture; next verify the corrected installed recorder, then a guarded
bounded movement with measured displacement and exact target audit. Preserve a
negative run with unavailable/wrong-time TF and prove its failure. Only after that
should this recorder supply physical-loss stopping and three-machine arrival
evidence. OM-027 geometry evaluation is implemented locally; native collision and
complete route acceptance remain pending.
