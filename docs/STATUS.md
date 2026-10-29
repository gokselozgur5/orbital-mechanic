# Project status

Updated: 2026-10-29

## Active delivery goal

The user requested at least 100 substantive atomic commits toward the complete
maintenance MVP. [GOAL_100.md](GOAL_100.md) defines scope and evidence gates. The
isolated `codex/nav2-integration` branch preserves the dirty main checkout and
original localization branch; six migrated commits do not count as new deliveries.
[Draft PR #16](https://github.com/gokselozgur5/orbital-mechanic/pull/16) links #6/#7,
which remain open until full live localization/navigation acceptance and review.

**OM-049: cancellable inspection ownership passes local integration checks; native acceptance pending.**
The typed String command endpoint owns one run/mission/action at a time, invokes
the actual camera capture in a separate ROS process, and returns an ACK separately
from a terminal result. Exact replay returns cached state; conflicting reuse,
wrong runs and history overflow are rejected. Cancellation/deadline intent stays
sticky until the exact child is reaped. A bounded lifeline and inherited owner
lock preserve ownership across loss of the parent; the user-service launcher
contains descendants and pins existing prerequisites.

The parent independently checks live settled pose, source/publisher/map identity
and both clocks, verifies the actual manifest/image and owned request context,
then rechecks after I/O. Slow completion never renews capture freshness. The CLI
distinguishes acknowledged, terminal, rejected, not_sent and uncertain outcomes;
same-run query recovers a cached result, while restart never adopts orphan files.
This is a versioned JSON endpoint, not a generated ROSIDL action or a motion hold.

All 461 local tests pass (46 new), along with the causal baseline, shell syntax,
Python AST and XML checks. Two deliberate mutation controls fail the same tests
when final freshness or cancellation-before-commit handling is removed; the
candidate passes both. These are test-sensitivity controls, not historical bugs.
Tests cover process reap/escalation, idempotency/capacity, cancellation
races and full server → subprocess → actual capture → artifact → terminal flow
with controlled ROS transport and a shared test clock. The initial late-read fixture failed earlier at
sample expiry; the corrected provider supplies post-settling telemetry before
testing the intended 250 ms final-read delay. The first published PR CI rejected
a positive capture at final validation; its specific inner cause was not logged.
Eight later real-clock local controls passed. CI ownership tests now freeze their
shared clock after capture during actual worker exit, and a 350 ms delayed-exit
case proves terminal publication waits for reap. Production still uses real
monotonic time and unchanged freshness bounds; native timing remains open.
The fixture correction adds no separate OM-049 delivery credit.
Thor SSH timed out (exit 255); no native state changed. Native endpoint, camera,
map/navigation and three-machine acceptance remain open under #8 / draft #19.
Contract and operator commands: [INSPECTION_ENDPOINT.md](INSPECTION_ENDPOINT.md).

**OM-048: camera evidence passes local integration checks; native capture remains pending.**
`machine-observation.py --with-camera` and `capture_machine_observation` acquire
an image/CameraInfo pair on the joint observer. Image and telemetry stamps remain
separate, with an explicit maximum 100 ms difference. Both must be post-settling;
image-time robot and optical TF are measured and checked against the configured
mount, while telemetry-time pose and all existing gates remain required.
Source-bound dimensions, RGB stride/byte limits, calibration, frame/time/order,
publisher identity and 200 ms image/info freshness are enforced.

Schema 2 references a lossless RGB PPM and bounded manifest under ignored
`runtime/inspection/<observation_id>`. Exclusive no-follow writes, a nonblocking
writer lock, a 64-entry quota and bounded digest-verified reads prevent arbitrary
paths, overwrites or unlimited capture retention. Identities are prepared before
final validity checks; live validity and stored bytes are rechecked around
manifest publication. Managed failure/interruption removes the acceptance marker
and retains bounded partial evidence. Stored files alone are not action permission
or a successful inspection endpoint result; crash recovery remains future work.

All 415 local tests pass, including 32 new core/adapter/filesystem/CLI tests.
The causal baseline, shell syntax, Python AST, XML and whitespace checks pass.
A late-digest freshness failure in the initial new writer is reproduced and the
identical regression passes after correction. A real
monotonic-clock paced controlled transport probe accepts actual local capture and
rejects a 250 ms storage delay with no accepted manifest. Synthetic pixels and
controlled transports do not prove native rendered visibility, DDS timing or
vision diagnosis. Thor SSH again timed out (exit 255); no native state changed.
The map revision remains `f02e11a9cb0e4f2f`. Issue #8 / draft PR #19 remain open;
next are cancellable inspection ownership and all pending native gates.
Contract: [CAMERA_OBSERVATIONS.md](CAMERA_OBSERVATIONS.md).

**OM-047: joint machine/robot observations pass locally; native inspection remains pending.**
The production `read_machine_observation` reader and `machine-observation.py`
CLI use one live observer to bind a selected public telemetry sample to robot TF
at exactly the same simulation timestamp. The sample must be produced at or
after a fresh half-second stopped-odometry window. That window's generation and
the selected packet remain pinned through actual map collection and final checks.
Fresh lagging odometry may catch up within the fixed 200 ms sample lifetime;
new telemetry, movement, expiry or changed source/clock/publisher/map evidence
rejects the query. There is no later-sample or latest-TF fallback.

The target-specific schema-1 record includes public measurements, original
run/sequence/source/time identities and the complete measured robot-state report.
It contains no hidden fault fields, camera, diagnosis, stop acknowledgement or
action permission. The packet hash correlates to the full original public wire
sample, not a digest recomputable from the returned target-only measurements.

All 383 local tests pass, including 21 new core/adapter/CLI tests with controlled
transport. The causal baseline, shell syntax, Python AST, XML and whitespace
checks also pass. Native DDS timing and inspection acceptance remain unverified.
A fresh strict SSH read
to Thor timed out (exit 255); no native state changed. The map revision remains
`f02e11a9cb0e4f2f`, with the existing explicit reload/seed/readiness requirements.
Issue #8 / draft PR #19 remain In Progress. OM-048 adds local camera capture above;
native camera acceptance, cancellable inspection ownership and maintenance endpoints
remain subsequent work.
Contract: [INSPECTION_OBSERVATION.md](INSPECTION_OBSERVATION.md).

**OM-046: fresh public telemetry reading passes locally; native integration remains pending.**
The existing OM-041 public projection is reused rather than reimplemented for
OM-045 credit. `observe-station.py` and its production reader accept only the exact
source-bound public schema and require two advancing samples in one run. Both
receipt and simulation age must be below one second. Gaps/expiry discard the
pair; replay, run/clock regression, malformed/missing/future data and changed
publisher identity refuse the query. Model elapsed time remains distinct from
the absolute simulator timestamp. Invalid input cannot be erased by a later
valid packet, and input text is not reflected into decoder error messages.

The reader pins the clock and telemetry publishers before subscribing, then
rechecks source files, clock mode, generations, age and deadline before returning.
No control publisher/client, hidden fault field, diagnosis or action permission
is introduced. Numerically valid noisy values are retained without model-based
correction; causal diagnosis and noise confidence remain subsequent work.

All 362 local tests pass, including 30 new tests. Baseline, shell/AST/XML and
whitespace checks pass. The shipped publisher and reader/CLI run with controlled ROS
transport, covering normal samples, malformed/schema/source/time/order failures,
final-read races and cleanup. Review reproduced five station-reader and ten
existing proximity-adapter acceptance failures during result preparation. The
same tests pass after ID/digest work moves before the final validity checks;
proximity's old adapter is loaded directly from `24860f4` with unchanged runtime
dependencies. A fresh Thor SSH attempt timed out (exit 255); no
native source/process changed. The joint record is now integrated locally in
OM-047 above; camera capture, inspection endpoints and all native gates remain
open under issue #8.
Contract: [STATION_OBSERVATION.md](STATION_OBSERVATION.md).

OM-046's follow-up size correction rejects an oversized string before UTF-8
encoding can copy its full payload. The same 1 MiB allocation regression fails
before the correction and passes after; this receives no separate delivery credit.

**OM-044: settled inspection preconditions pass locally; native inspection remains pending.**
The production `read_inspection_state` reader and CLI `--require-settled-pose`
mode require the accepted inspection pose and direct stopped odometry spanning
at least 0.5 seconds in both monotonic receipt time and simulation time. All
velocity axes use the localization monitor's existing 0.01 m/s and 0.02 rad/s
norm limits. Motion, invalid/out-of-order data, stale/gapped samples and unhealthy,
moving, frozen or missing health reset the window. Health simulation freshness is
checked throughout accumulation, not only at the final pose. A post-settling
invalidation rejects this query even if later callbacks settle again.

The full map geometry is validated before ROS observation starts. During live
collection and after TF lookup, exact source/artifact fingerprints are checked;
actual server/retained cells and consumer parameters/subscriptions are still read.
This avoids spending the short odometry lease on repeated geometry computation.
The map revision remains `f02e11a9cb0e4f2f`; previous explicit native reload gates
still apply. Schema 3 exposes stationarity separately from distance/pose.

All 332 local tests pass, including 25 new tests covering the production reader, shipped CLI,
source changes, cleanup and invalidation races through controlled transport.
A discriminating regression reproduces two stale/future-health history acceptances
before the freshness correction and rejects both afterward. Baseline, shell,
Python AST and XML checks pass. With real wall-clock paced controlled callbacks,
recomputing geometry during observation expires the stopped history; the prepared
path accepts a fresh settled pose with the same live map checks. Native odometry/TF
timing and stopping remain uncommissioned; sampled settling is not a hold or an
inspection/action authorization. A fresh Thor SSH attempt timed out (exit 255),
with no native changes. Issue #8 / draft PR #19 remain In Progress. Next: fresh
public observation handling and inspection integration, plus pending native gates.
Contract: [PROXIMITY.md](PROXIMITY.md).

**OM-043: measured inspection pose passes locally; native inspection remains pending.**
The map validator and public registry now require each nominal inspection heading
to face its actual west-face control. The same fresh TF used for distance reports
measured XY/yaw errors against the existing 0.10 m / 0.15 rad arrival bounds.
Schema 2 distinguishes distance from `within_inspection_pose`; the CLI's
`--require-inspection-pose` option rejects a nearby robot with the wrong position
or heading. A valid pose still does not authorize inspection or maintenance.

All 307 local tests pass, including eleven new tests through nominal configuration, production evidence and the
actual CLI/map/TF adapters with controlled transport. The same regression rejects
opposite/misaligned configured headings only after the change; a valid diagonal
goal passes before and after. Boundary and quaternion sign/norm/wrap controls
pass. The causal model baseline and shell/AST/XML checks also pass.

Two map generations agree at revision `f02e11a9cb0e4f2f`; occupancy is unchanged
from `1c58f56cf5246735`, which current source now rejects. Native use requires an
explicit map regeneration/localization reload, seed and readiness check. Thor
SSH again closed the connection (exit 255); no native state changed. PR #19 and
issue #8 remain open/In Progress. Next: stationary inspection interlock and fresh
public observations, alongside the pending native commissioning gates. Contract:
[PROXIMITY.md](PROXIMITY.md).

**OM-042: measured proximity passes locally; native commissioning remains pending.**
The read-only `machine-proximity.py` query uses the registry, actual map-binding
gate, localization health and an exact-time `map` to `base_footprint` transform.
It pins publisher/run/epoch identity, checks current source and timing, and
reports planar control distance for one machine. Caller-supplied pose/distance,
stale or reset evidence and successful lookups that finish after validity gates
expire cannot produce an accepted measurement. No control RPC or publisher is
created; the observer never acknowledges a motion-stop obligation.

All 296 local tests pass, including 25 new production core/CLI and actual map/TF
adapter tests with controlled transports. Three lookup races (publisher generation,
clock mode and map age) fail before the final checks and pass afterward; a strict
timestamp regression also fails before and passes after. The native access check
could not establish an SSH session (exit 255). No native source, process or world changed. See
[PROXIMITY.md](PROXIMITY.md) for the evidence contract and commissioning steps.

This remains stacked on navigation through draft PR #19 / issue #8. A valid
distance is not inspection eligibility: native pose/source commissioning,
stationarity, telemetry freshness and action ownership remain open; OM-043 adds
local pose checks above. Next local work: stationary inspection interlocks;
native navigation/map/telemetry gates remain required before live integration.

**OM-041: machine registry and telemetry binding pass locally; native commissioning remains pending.**
The `codex/machine-integration` branch is based on navigation head `fd86e6e` and
contributes to [issue #8](https://github.com/gokselozgur5/orbital-mechanic/issues/8).
The existing telemetry process now binds its three machine IDs to configured SDF
models, collision boxes, controls and inspection poses. A versioned public catalog
identifies exact world/map source bytes; each measurement carries the catalog,
simulation timestamp, run/sequence and units while retaining the causal model's
existing observation fields. Scenario truth remains outside the projection.

All 271 local tests pass, including 27 new registry/core/actual-adapter cases.
The identical source-change regression fails on `fd86e6e` and passes now: changed
source stops publication before the next measurement. Duplicate/missing geometry,
invalid controls, source boundaries, clock pause/reset/gaps, clock-mode changes,
identity drift and publication errors have discriminating controls. Registry
poses are declared source geometry, not runtime truth or robot proximity.

This is a stacked maintenance change; navigation PR #16 and its native gates stay
open. Thor SSH still times out and no remote process/source changed. Native
telemetry/schema/source and measured proximity commissioning, orientation/stationarity,
freshness/noisy data handling and inspection/actions remain outstanding. See
[MACHINE_REGISTRY.md](MACHINE_REGISTRY.md) for contracts and deployment gates.

**OM-027: sampled collision evaluation passes locally; native contact/route acceptance is pending.**
The evaluator covers the body box, caster sphere and both finite wheel cylinders
against every declared static obstacle box at observed poses. The recorder now
uses the full Gazebo pose stream, captures actual link/static-model poses and
binds a supported SDF geometry snapshot to the verified deployment. Missing or
changed entities/frames/geometry refuse complete evidence. Cylinder boundary
uncertainty, normal floor support, body-floor contact and leaving the floor are
reported separately; sampled clearance never claims continuous collision freedom.

All 244 local tests pass, including 40 new cases and 300 analytic cylinder oracle
controls. The actual recorder adapter's controlled-transport artifact passes the
independent evaluator; moved-obstacle wheel contact, rotated geometry, tipping,
tampered/rehashed traces and missing data produce their required adverse outcomes.
An identical TF deadline regression fails on `5ff3a8e` and passes after a late
available transform is refused beyond the recorder's 0.5-second wait.

Thor SSH still times out. No native source, process, world or obstacle changed;
live deployment remains last verified at OM-017. Full scene capture, collision
controls, physical loss/recovery and route acceptance remain open. The evaluator
cannot by itself establish the mission's no-collision requirement. See
[COLLISION_EVALUATION.md](COLLISION_EVALUATION.md) for the geometry/coverage contract
and [TRAJECTORY.md](TRAJECTORY.md) for the updated recorder and native gates.

**OM-023: independent trajectory recording passes local tests; native capture is pending.**
The read-only recorder samples Gazebo model truth directly, looks up localization
at each truth timestamp and records exact supervisor target/command identities.
New supervisor audit events distinguish command replies from forwarded velocity;
neither implies physical movement. Ordered counters expose missing audit records.
Unique run output retains normalized events, receipt/source times, interpolation
results and a bounded hash/count summary, with service/source/binding fences.

All 204 local tests pass, including 40 new evaluator/audit tests. Production
supervisor callbacks and the actual recorder adapter run through controlled
transports. Missing TF/audit data, resets, changed identity, queue/output limits,
malformed callbacks, interruption and tampered artifacts fail visibly. The initial
test combined two negative scenarios on an already-invalid trace; they are now
independent cases. No native binding, DDS or moving-trajectory acceptance is claimed.

Thor SSH remains unavailable; the last verified live deployment is still OM-017.
Next native: commission OM-018/022, coordinate the revised map/localization reload,
deploy the reviewed observer/audit source and prove stationary then guarded moving
capture before physical-loss and route acceptance. The recorder never commands a
stop or cancels a goal; armed trials retain the independent guardian. See
[TRAJECTORY.md](TRAJECTORY.md) for capture timing, artifacts and evidence limits.

**OM-022: bounded navigation commands pass local tests; native DDS is pending.**
`scripts/orbital-navigate.py` observes status or sends one request, explicit arm
or exact-target cancel. It pins supervisor run/sequence, rejects stale targets
and ambiguous peers, uses a per-checkout/domain lock and never retries an
unacknowledged command automatically. JSON receipts distinguish acknowledgment,
rejection, no publication and uncertain delivery. Acknowledgment is not dispatch,
arrival or physical stopping; an armed goal remains owned by the supervisor after
the CLI exits. Request validates geometry; cancellation does not depend on
reloading the local map config or healthy localization.

All 164 local tests pass, including 37 new protocol/CLI cases. The actual CLI
adapter drives production supervisor callbacks with controlled ROS transport,
covering request/arm/cancel, retained pending arm, stale tokens and competing
publishers. Deadline, restart, sequence, malformed input, local lock and input
boundary tests pass. No native process was changed: Thor SSH still times out.
Next: native isolated CLI commissioning alongside OM-018, then the coordinated
OM-021 map reload, physical loss/recovery gates and measured tour. Independent
trajectory recording is implemented locally as OM-023 above. See
[NAVIGATION_CLIENT.md](NAVIGATION_CLIENT.md) for the operator/receipt contract.

**OM-021: inspection-pose geometry is accepted offline; deployment is pending.**
The waypoint loader, map generator and artifact binding now validate the three
machine IDs against actual static collision boxes. Controls must match each
machine's west-face center. Goals need a clear 0.47 m arrival envelope (0.37 m
robot plus 0.10 m XY tolerance), inflated connectivity from the origin and control
distance within the station's 0.75 m bound throughout that tolerance. Reviewed
footprint/arrival bounds cannot be reduced through JSON configuration. The ROS
entry point refuses missing geometry before constructing a command owner.

All 127 local tests pass. The identical 14-test regression file produces 21
failing subcases on `24df24d`; all cases pass on the new implementation, including
valid translated geometry and the exact proximity boundary. Other regressions
cover the production CLI, disconnected destinations, changed validation source
and tampered map evidence. Two map generations agree byte-for-byte at revision
`1c58f56cf5246735`; the occupancy image is unchanged. Each original pose has
0.05 m control-distance margin after arrival tolerance.

Map identity now binds the geometry validation dependencies and includes a
per-machine report. Existing generated artifacts are preserved. Before deploying
this change, regenerate the map, explicitly reload localization and verify seed,
map binding and readiness; an already-loaded old revision is intentionally
rejected. No native process or simulation was changed. OM-018/019/020 and the
measured tour remain pending; SSH access to the configured Thor is unresolved.

**OM-018: arm callback races are fixed locally; native commissioning is pending.**
The command callback now expires the health lease before admitting a request or
arm. If moving feedback arrives while a guard-arm response is pending, that
target is invalidated even when stationary feedback returns before the next tick.
Dispatch also rechecks health and stationarity when the response arrives.

Seven transport-controlled tests execute the production supervisor, health
interlock and action session. Four fail on `b712384` and all seven pass with the
fix; the full local suite passes 107 tests. Positive controls retain dispatch
after a fresh acknowledgment and ownership while an arm response is unresolved.
`scripts/check-navigation-arm.py` provides seven corresponding native ROS cases,
but has not run: SSH to the configured Thor timed out from the current network.
Local tests use ROS transport doubles and do not establish DDS or physical-motion
acceptance. The live deployment remains last verified at OM-017. Next: run the
native negative/positive controls, inspect current service ownership, deploy the
reviewed fix and finish stationary-arm commissioning before physical-loss trials.

**OM-017: map revision binding is deployed and accepted.**
Startup validates the loaded map-server revision and artifacts; readiness also
checks every published/GetMap cell, AMCL/planner map settings and subscriptions,
and the supervisor's read-only identity of its loaded waypoint configuration.
Seven isolated map cases and seven native Nav2 readiness stages passed, including
a runtime-only wrong map topic that triggered acknowledged stop/pause. Explicit
recovery passed. All 100 local tests pass; failed trials and their limits are in
[NAV2_INTEGRATION.md](NAV2_INTEGRATION.md).

Only navigation was restarted for deployment. Live activation, map binding,
per-message TF audit, idle/latched checks and independent pose comparison passed.
Gazebo and localization invocations stayed unchanged. Physical endpoint pose
remained `(0.000386, 0.000002, 0.012884)`; the new supervisor has zero commands and
no target. All 94 source hashes match native/live identity
`b7310e23ee1bb43dab9c7d90ed48e142b4442ceb9b976134fd5a31659531e646`.
The navigation startup manifest now records this deployed revision.

This is startup/readiness map binding, not continuous map-change enforcement or
proof of consumers' private memory. Next: stationary-arm commissioning (OM-018),
physical localization-loss stopping and the measured three-machine tour.

**OM-016: native per-message TF publisher attribution passed.**
A read-only C++ auditor maps each received DDS publisher GID to ROS endpoint
metadata in the same process. Seven isolated real-DDS controls pass, including a
wrong publisher with otherwise correct graph/parents, duplicate same-name owners,
wrong parent, nonfinite geometry, missing edge and frozen dynamic timestamps.
The positive control also proves late-join delivery of static transforms.

Live `tf-authority-proof/live-2` passed all eight edges across 256 messages. Source
identity `5b0d335e780bf2e7fd93fdf8b3faba193221af026e0c4545d2b39a44fc5e2ff2`
matches all 90 captured source files; executable SHA is separately recorded.
94 local tests pass. All service invocations stayed unchanged; the supervisor
remains healthy/idle/latched with 4 historical commands and no new requests.
Independent Gazebo pose/localization error remains unchanged from OM-015.

An initial conservative rejection exposed incomplete DDS node-name discovery;
unknown placeholders now wait for resolution, while unresolved final mappings
still fail. Failed runs are retained. This is sampled ownership evidence, not
authentication, continuous enforcement, route acceptance or reproducible-build
proof. Use the new audit before route trials. Next: map identity binding (OM-017),
stationary arm, physical loss-stop and the measured tour.

**OM-015: the first native Nav2 goal moved and was cancelled by reseeding.**
The bounded `navigation-reseed-1` trial requested/armed pump, observed running Nav2
and 0.0766208 rad/s odometry rotation, then published an uncertain origin seed. The
exact target ended `cancelled`, stationary and latched in 0.224119 s; its old arm was
rejected and the command stream stayed stopped. A fresh cooler target was accepted
then cancelled without arming. Current supervisor is healthy/idle/latched with no
target and 4 historical forwarded commands; it has not reached a machine yet.

A separately pinned pidfd trial guardian passed five native failure/ownership
cases before the motion trial. It disarmed after verified safe completion, leaving
all actual services on their existing invocations. Post-trial independent Gazebo
pose was (0.000386, 0.000002, 0.012884); localization error 0.0109483 m / 0.0208018 rad passed.
No independent physical braking distance or continuous collision proof is claimed.
All 85 sources match `413b13a500013aebec1017ee671720a93475258359efa56787d3ba7025f6fd14`;
92 local tests pass. Reproduction: [NAV2_INTEGRATION.md](NAV2_INTEGRATION.md).
Next: per-message TF authority, map binding, stationary arm and physical loss-stop
acceptance, then the measured tour.

**OM-014: live world reset invalidated the old unarmed target.**
Native `navigation-reset-1` reserved a pump target, reset actual Gazebo simulation
time to zero, and observed `initial_pose_required` plus terminal old-target state.
The old token's arm was rejected before and after explicit known-origin reseeding;
a new request was blocked before the seed and accepted afterward with a higher
epoch, then cancelled. All service/supervisor invocations stayed unchanged. Final
state: healthy/idle/latched, zero forwarded velocity commands, world reset to origin.

All 81 source hashes match identity `4742ab14f1354ab63ecef3a2494e57b3f7a0d81aad85c05a0fe21575c7d97dfb`; all 92 local tests pass.
This is unarmed target reset acceptance. Next: reseed during a dispatched goal,
then TF authority/stationary-arm/physical-stop gates and measured routes.

**OM-013: real lidar and AMCL freshness fault admission checks passed.**
Native `freshness-3` temporarily lowered only lidar to 0.1 Hz and separately
suspended the pinned AMCL process. With unaffected inputs advancing, the supervisor
reported `scan_stale` / `transform_invalid` and rejected new targets. Recovery
permitted a fresh unarmed request; each positive-control target was cancelled.
No velocity request, dispatch or odometry movement occurred, and the guard stayed
latched. Lidar recovered to 15 samples in 1.5 s after restoring 10 Hz; all service
and supervisor invocations are unchanged from OM-012 below.

Both source-bound cases passed at identity
`4592da65126331421671819fd672c8b5de977019fa6f8e32b11ab8c610a1b17e` (80 matching
source files). All 92 local tests pass. Retained failure evidence includes a scan
harness callback-age error corrected by spinning ROS while awaiting the helper.
AMCL's stationary retained pose is allowed only alongside valid current inputs/TF;
GOAL_100's freshness row now makes that existing contract explicit. No moving-goal
stopping or reset/reseed acceptance is claimed. Next: world reset/reseed ownership,
TF authority and stationary-arm gates before measured routes.

**OM-012: offset-seed convergence into native navigation passed.**
Thor `navigation-convergence-20261028T114740Z` restarted the known-origin world,
proved unseeded AMCL, supplied a deliberate (0.20 m, -0.15 m, 0.15 rad) seed, and
ran acknowledged bounded forward/left commissioning jogs. Independent Gazebo
comparison measured 0.0752675 m XY / 0.00673083 rad yaw error, inside the unchanged
0.10 m / 0.10 rad bounds. Native Nav2 then reached active state 3 with healthy
localization, guard latched and zero supervisor velocity requests. Accuracy was
unchanged after activation; measured startup displacement/yaw change was zero.

Current runtime is **seeded, Nav2 active, guard latched**, awaiting a new explicit
target. No goal was executed during OM-012; OM-015 above covers the first dispatched native goal. All 79 source hashes match identity
`3e31a0f418d6ce62ae7633b9e96704cbdce4ee0064c3c2bf4991eea961889788`.
All 92 local tests pass. Next: localization freshness/reset/loss and TF gates,
then the measured tour. Reproduction and evidence limits are in
[NAV2_INTEGRATION.md](NAV2_INTEGRATION.md).

**OM-011 historical acceptance: live unseeded admission refusal passed.**
The packaged Thor run `unseeded-20261028T114225Z` restarted only localization and
navigation. Active AMCL produced no pose/map transform while real sensors advanced.
All six supervisor commands (request/arm for each machine) were explicitly rejected;
no target was allocated, no velocity request emitted and the guard stayed latched.
Gazebo endpoint pose change was zero at output precision. At the end of that run, native Nav2 servers were
configured/inactive (state 2), deliberately awaiting explicit localization.
The simulator InvocationID stayed unchanged. Source identity:
`145b33f33caa9b155cc4019592a16f99ecf8d4b4e83afb5b1888c97b034a4cbe`.

A seeded negative control rejected the probe's unseeded premise. The first full
launch also failed closed on incomplete DDS discovery; its final graph was valid,
but accumulated earlier missing-endpoint observations rejected startup. The failed
report is retained alongside the successful explicit rerun. This conservative
startup false rejection remains a usability limitation, not a passed route test.
The runner was corrected for an already-stopped navigation unit and Thor's absent
ripgrep. All 92 local tests pass. Next: explicit offset-seed commissioning and
localization recovery, then measured routes. See the OM-011 reproduction below
in [NAV2_INTEGRATION.md](NAV2_INTEGRATION.md).

**OM-010 historical acceptance: the native navigation stack reached guarded active standby.**
Planner, controller and navigator passed unconfigured → inactive → active checks;
the health observer and supervisor are running. Guard remained latched, localization
was healthy and no supervisor velocity requests were emitted. A repeated read-only
check passed with no lifecycle commands. Independent Gazebo before/after pose queries
reported zero net translation/yaw change at their output precision, within the
1 mm / 0.001 rad acceptance bounds. No target or arm command was sent.

Native isolated readiness tests passed configuration, activation, repeated checks,
changed-source rejection with stop/pause, and explicit recovery after restoring
bytes. The first stop-RPC attempt timed out; persistent clients and bounded
discovery/response waits passed the subsequent proof. All 92 local tests pass. Earlier
controller, planner, graph, service and manifest proofs are documented in
[NAV2_INTEGRATION.md](NAV2_INTEGRATION.md) and [NAVIGATION_SERVICE.md](NAVIGATION_SERVICE.md).

Native and live readiness used identical source/artifact content identity
`4a5eda76f5780bcd255157fbf5eab1205c7ea7a4648b14a40ad93753afed0d5d`.
The live demo/localization InvocationIDs did not change. Deployment added navigation
sources and backed up three older localization test scripts before updating their
unseeded checks. Running world, robot and guard source files were already identical.
Raw results, source bindings and rollback metadata remain in private review artifacts.

Readiness is not autonomous route acceptance. Next: real unseeded/reseed/reset/loss
and TF-authority gates, then the measured three-machine tour and failure cases.
Physical arrival/collision/mission acceptance remains outstanding. No maintenance
mission, live LLM call or paid resource was started.

## Current phase and next action

Phases 0 and 1 are accepted. Phase 1 motion policy, guarded ROS integration, bounded operator CLI, failure stopping and the combined Foxglove view now have acceptance evidence. On 2026-10-24 the user imported `foxglove-operator-desktop` and confirmed all four panels were visible. Independent read-only capture showed the robot camera, lidar points, stop reason and four commanded/measured velocity series; topic counters were advancing.

The original accepted baseline was motion code `c63e9cd` (archive `eedc3fd`) and desktop layout `7fdf49c`. Frame correction `5378dac` is now also live-accepted: the complete packaged run passed after the transient-unit race fix `8ebdfb8`; exact evidence and source hashes are recorded below.

The user handles Foxglove clicks, keyboard input and layout import. Do not automate that UI without renewed authorization. Avoid restarting the simulator while the user is inspecting it without coordinating. Read-only probes and offline preparation can continue.

Next: complete Phase 2 known-map localization and guarded navigation. Localization commissioning exposed and isolated a physical tipping defect; the corrected body and bounded acceleration passed the full runtime regression described below. The six milestones and 15 issues are indexed in [TRACKING.md](TRACKING.md); [PHASES.md](PHASES.md) defines acceptance.

## Installed and running baseline

- Thor ARM64: Ubuntu 24.04.3/L4T R38.2.2, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, Nav2 and gz_ros2_control installed. Installation is not navigation integration.
- One room, three machine props and a wheeled robot render with Ogre1/llvmpipe. Ogre2 startup failed; no board reboot, blanket OS upgrade or PyTorch GPU inference was attempted.
- RGB 640 × 480, camera calibration, 360-sample lidar, IMU, wheel joints, odometry, transforms and simulation clock are bridged. Read-only machine telemetry uses a separate simplified causal model, not electrical physics or robot-integrated maintenance.
- Mac Gazebo viewing uses local noVNC through SSH; the user confirmed the image appeared. Native macOS Screen Sharing failed and is not the default viewer.
- Foxglove 3.4.2 is installed separately from the crashing old 1.84 client. The user completed pairing and opened it. Camera/calibration/TF subscriptions and the WebSocket handshake were verified. The optional source-built 0.8.5 bridge overlay is used; APT bridge 3.6.0 remains installed.

## Verified motion and safety

The guard accepts bounded stamped `/motion/request` commands. Startup, stop, timeout and invalid feedback latch motion off; explicit re-arm requires fresh stationary feedback. Old commands cannot restart motion. This is a trusted local-process safety boundary, not authentication or collision avoidance.

Eight live motion checks passed. The earlier operator baseline measured approximately 0.0841 m forward and reverse, verified both turn directions, and required subscriber discovery plus guard acknowledgment before reporting successful command delivery. SIGKILL/SIGINT/SIGTERM/SIGHUP cancellation stopped through the independent watchdog in 0.373–0.412 s and 0.040–0.048 m in that run.

Pause/resume and world reset did not resume old motion. In the final combined fault run, killing the guard removed all 14 sampled demo processes in 0.223 s; clean restart was latched and stationary. The service cgroup owns detached Gazebo descendants, and bounded cleanup fixes were verified. These measured simulation cases are not universal stopping-distance guarantees.

See [MOTION.md](MOTION.md), [MOTION_EVIDENCE.md](MOTION_EVIDENCE.md) and [OPERATOR.md](OPERATOR.md) for interfaces, limits and earlier failures. `scripts/check-phase-one.sh` now reproduces the full acceptance sequence and passed end-to-end on Thor, including frame checks before and after failure injection. It leaves a fresh latched world on success and stops the demo on failure.

## Camera freshness

Earlier camera delivery was intermittent and a reliable-QoS-only trial still ended stale. A project-local 16 MiB Fast DDS shared-memory profile and common `scripts/ros-env.sh` resolved the reproduced short-window failure. No global kernel settings or unrelated services were changed.

The final combined run received 97 images in 20 seconds with approximately 50 ms final receive age. Three subsequent 20-second windows received 100/96/100 images with final ages 24.6/37.2/63.4 ms. All passed strict receive freshness, image timestamp age and other sensor checks. This supports approximately 5 FPS in those windows, not long-duration stability or measured camera-to-screen latency.

## Offline navigation preparation

`python3 scripts/generate-known-map.py` derives a 210 × 170 occupancy grid at 0.05 m resolution from static SDF collision geometry. Generated PGM/YAML/manifest files stay in ignored `runtime/maps`; source, configuration and generator hashes identify them. The inflated-grid check finds paths to all three proposed inspection poses using a conservative 0.37 m radius.

All 24 unit tests pass on Mac and Thor, including six map cases. Repeated local generation is identical; Thor generated the same PGM SHA-256 `d08588af2b1186992411e32dbf1d703e47ea45d183666de13c04a84cbe9c5000`. The offline job exited 0 with the existing demo still active; it did not restart the world. Map and navigation-contract commits passed GitHub CI.

See [NAVIGATION.md](NAVIGATION.md). Offset-seed AMCL commissioning is described below. Stronger TF ownership evidence, complete live localization loss/reset behavior, obstacle-aware routes and actual arrival at machines remain unverified. The frame-height correction itself is live-accepted below. No autonomous navigation is claimed.

## Frame geometry correction — live acceptance passed

Read-only inspection found a 0.22 m vertical mismatch: live `odom→base_link` had z=0, while Gazebo reported the settled model near z=0 and its body at model-relative z=0.22. Consequently the ROS lidar/camera mount heights were 0.16/0.12 m instead of the model's nominal 0.38/0.34 m. Matching topic frame names did not establish matching physical geometry.

The source now separates planar `base_footprint` from the elevated `base_link`, adds the fixed 0.22 m body transform and adjusts the guard's expected odometry child. Velocity requests remain in `base_link`; the frames have the same planar axes and differ only in height. Three cross-model regressions failed before this change; all 27 local unit tests pass afterward.

Thor job `frame-runtime-acceptance-2` exited 0. Its source manifest matches the local frame, guard, launcher and runner files. All 36 unit tests passed on Thor; before/after live frame checks confirmed body/camera/lidar/wheel heights with zero positional error at the reported precision. Eight motion cases and four-direction/four-signal operator checks passed. Cancellation stops were 0.401–0.405 s and 0.041–0.045 m; independent forward/reverse timeout cases stopped in 0.051/0.058 m. Pause/reset stayed stopped, guard death removed 14 processes in 1.238 s, and a clean restart was latched.

The final sensor probe passed with 95 images in 20 seconds and 87.6 ms final image receive age. The first packaged attempt exposed a transient-unit collection race after its initial frame/motion checks; `8ebdfb8` fixes it with a reproduced regression. Reports remain in Thor `runtime/phase-one-20261024T135645Z` and local private review artifacts. `check-frames.py` verifies advancing odometry and expected geometry/parents/publisher nodes; it does not prove per-message TF ownership because this installed Jazzy metadata lacks publisher GIDs.

## Physical stability correction — runtime regression passed

Localization commissioning revealed a failure missed by planar wheel-odometry checks: after movement the physical body pitched forward by 0.522876 rad (about 30 degrees). Gazebo model pose and 42 IMU samples independently agreed. Lidar then saw the floor in front and empty space behind; its planar-map residual was about 1.84 m mean absolute error. A deliberately offset AMCL seed failed the unchanged 0.10 m accuracy bound at 0.109 m and 0.163 m. The earlier 8 mm result from a correct origin seed was not adequate convergence evidence.

The old combined center of mass was only 1.98 mm inside the wheel-axle edge of the support triangle. The body inertial origin now lies 0.08 m rearward, leaving about 65 mm minimum static support margin. DiffDrive acceleration is bounded to ±0.4 m/s² linear and ±0.8 rad/s² angular. Geometry and sensor mounts remain the same. A source regression fails for the old mass distribution; 37 local unit tests pass for the correction.

Thor job `body-stability-acceptance` exited 0 with the corrected SDF. Four-direction IMU stability passed with 253 samples, followed by all motion/operator/failure/frame/sensor checks. Timeout stops at ±0.15 m/s took 0.685/0.723 s and 0.078/0.085 m; limits remain 1.2 s and 0.12 m. Four client-disconnect cases stopped in 0.622–0.686 s and 0.056–0.065 m. Pause/resume stayed stopped after 0.0217 m travel; reset did not resume; guard death removed 14 processes in 1.240 s. The final camera probe received 101 frames in 20 s, final receive age 10.3 ms, with all sensors fresh.

These stopping times supersede the earlier instantaneous-drive measurements for the corrected model. The IMU regression rejects >0.05 rad roll/pitch and stops on failure; it is a commissioning test, not a continuous tilt interlock. Planar odometry alone cannot establish chassis stability. Raw report and source manifest: Thor `runtime/phase-one-20261024T141235Z`, local private review `body-acceptance.json`. The extended eight-case body probe also passed at operator speeds and guard maxima (±0.2 m/s, ±0.5 rad/s), with 461 advancing IMU samples and measured displacement/turn direction. Its peak pitch was below 0.000001 rad in this idealized simulation. Corrected-body AMCL commissioning subsequently passed the scoped initial-pose/lifecycle checks described below; no autonomous navigation is claimed.

## Known-map AMCL — initial commissioning accepted

AMCL/map-server launch, explicit uncertain seeding and evaluator probes are now preserved in this branch. The earlier live `body-max-localization` job exited 0: a deliberately offset seed converged after bounded motion to 0.073151 m XY / 0.006311 rad yaw error; a fresh-origin restart measured 0.008088 m / 0.011434 rad. Demo stop removed all seven localization processes in 1.402 s, and neither implicit world start nor implicit localization restart was allowed.

The unseeded gate was subsequently deployed and passed in OM-011 above. The integration branch is still separate from shared `main`. See [NAVIGATION.md](NAVIGATION.md) for exact scope. Initial commissioning does not close #6/#7 or prove autonomous routes.

## Offline target lifecycle policy

`orbital/navigation.py` now provides run/epoch/sequence target identity, named-target allowlisting, explicit stationary arm, 0.5-second advancing-health freshness and cancellation-terminal interlocks. Seven new unit cases cover reset/loss/replay and stale-result behavior; all 34 local unit tests pass. See [NAVIGATION_POLICY.md](NAVIGATION_POLICY.md).

The policy itself emits no commands. The isolated ROS adapters described below now exercise health validation, action/cancel races, deadlines and estimated arrival. Shared-world deployment and live route/failure acceptance remain required.

## Offline localization health validator

`orbital/localization.py` now validates independently fresh/advancing scan, odometry and IMU, finite planar pose/TF, normalized orientation and symmetric positive-semidefinite planar covariance. It retains a pending fault when a bad or late sample is followed by a good one before evaluation, so a callback burst cannot hide a broken lease. World reset requires an explicit seed and post-seed pose; old target identities remain cancelled through the existing navigation policy.

Thirteen new cases and all 50 local unit tests pass. A discriminating test first reproduced acceptance of a non-planar ground transform, then passed after enforcing height/tilt bounds. See [LOCALIZATION_HEALTH.md](LOCALIZATION_HEALTH.md) for the exact room-specific contract and adapter obligations. This work is isolated on `codex/localization-health`; it is not integrated with the shared simulator and does not complete live reset/loss/Nav2 acceptance.

## ROS observation adapter — isolated synthetic acceptance

The read-only `localization-monitor.py` now feeds the validator from actual ROS subscriptions and a time-specific tf2 lookup, publishing health with process identity, epoch and sequence. It preserves invalidation across recovery and refuses duplicate health authorities. It cannot publish velocity, TF or initial pose and does not call motion/navigation services.

Thor `health-monitor-synthetic-2` exited 0 in a separate checkout and initially empty ROS domain 84: loss/replay, tilt, TF loss, pause/reset/reseed and duplicate-authority cases all passed. Measured invalidation intervals were about 0.042–0.535 s. Main domain 42 and the running demo/localizer were not restarted. See [ROS_LOCALIZATION_MONITOR.md](ROS_LOCALIZATION_MONITOR.md). The actual guard-service connection is exercised below; ownership of the Nav2 velocity path, action lifecycle and collision/arrival tests remain outstanding.

## Health-to-stop adapter — isolated actual guard acceptance

The health lease now validates schema/run/epoch/sequence, expires after 0.25 s without advancing health and preserves stop obligations through recovery. Generation-tagged acknowledgments prevent an old stop reply from clearing a newer fault. Seven new lease cases and all 57 local unit tests pass.

`health-interlock-synthetic-2` exited 0 using the actual ROS motion guard and stop service in domain 84. Valid velocity requests continued during injected faults; guarded output reached zero with `operator_stop` in 0.559 s (scan loss), 0.062 s (tilt), 0.156 s (TF loss), 0.307 s (clock pause) and 0.247 s (health silence). Recovery did not re-arm. All 13 stop requests were acknowledged. No Gazebo or actuator bridge existed in that domain, so this does not measure physical braking.

See [HEALTH_INTERLOCK.md](HEALTH_INTERLOCK.md). The component is not installed in the shared demo and is not sufficient as a standalone safety boundary: the final supervisor must own command publication, stop on its own process loss and await exact Nav2 cancellation. Shared-world ownership coordination and full route/failure acceptance remain pending.

## Nav2 action session — isolated protocol acceptance

`NavigationActionSession` now reserves exact target identities, requires policy arm, dispatches named poses, enforces a bounded wall deadline and retains ownership until the matching Nav2 result is terminal. Cancel acknowledgment alone cannot release a task; delayed acceptance is immediately cancelled after invalidation. New targets also wait for measured stationary feedback and an engaged stop latch. A successful Nav2 result remains explicitly unverified arrival.

Thor `navigation-action-synthetic-3` exited 0 with 11 real action-transport cases against a dedicated fake server, including delayed timer/acceptance, rejected cancellation, late success after cancellation and server SIGKILL. No controller or velocity publisher was launched. See [NAVIGATION_ACTION.md](NAVIGATION_ACTION.md). The composed synthetic acceptance below extends this evidence; real routes, physical stop composition and independent arrival/collision acceptance remain outstanding.

## Composed supervisor — isolated ROS acceptance

The supervisor now owns navigation command publication, joins health and action lifecycle, and requires run-bound sequenced operator commands with exact target tokens. It waits for asynchronous guard arm acknowledgment plus fresh unlatched feedback, rejects invalid/stale controller output, and checks measured TF before reporting arrival. Unresolved arm/action calls prevent premature task release.

`navigation-supervisor-synthetic-1` exited 0 with eight cases using synthetic sensors/fake Nav2 and the actual ROS guard: unarmed blocking, health-loss stop/old-target rejection, stale output, stop-service failure, false arrival, measured arrival, explicit cancellation and supervisor SIGKILL. Scan-loss stop took 0.499 s; unavailable stop service reached watchdog stop in 0.928 s; supervisor death reached watchdog stop in 0.377 s while the fake controller kept publishing. These are zero-output times, not physical braking measurements. All 59 local unit tests pass.

See [NAVIGATION_SUPERVISOR.md](NAVIGATION_SUPERVISOR.md) for the protocol, reproduction and deployment gates. This branch remains separate from shared `main`, pending coordination with the other ROS session. No actual Nav2 route, collision test, production supervisor service or LLM mission is claimed. Next: integrate the actual Nav2 lifecycle and exclusive velocity path, then run independent Gazebo route and failure acceptance.

## Runtime ownership and boundaries

Thor root: `/home/jensen/.goske/codex/orbital-mechanic`. Perform operational work inside `tmux orbital-work`; use `scripts/launch-thor.sh` and `scripts/stop-thor.sh` to manage transient user service `orbital-demo`. There is no boot-start configuration. Source `scripts/ros-env.sh` for ROS tools.

ROS domain 42 is localhost-only; Gazebo partition is `orbital-maintenance`. VNC/Foxglove bind to Thor loopback ports 5942/8765, forwarded to Mac 15942/18765. Mac noVNC uses localhost 15943. Artifacts, logs and caches stay under the project root; `~/.gz` links to its runtime cache. Dependencies, generated outputs, media and credentials remain outside Git. Unrelated llama/openclaw services are untouched.

Anthropic is the future planner provider based on user-reported USD 200 credit; available balance is unverified. No API key is loaded, no paid calls or resources were opened. A scripted mission and independent evaluator precede LLM experiments. No integrated repair, robot arm, visual diagnosis, SLAM or LLM-controlled mission is claimed.
