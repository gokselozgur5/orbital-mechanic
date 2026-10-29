# Production navigation integration

The motion boundary remains controller → `/navigation/raw_cmd_vel` → navigation
supervisor → `/motion/request` → motion guard → drive. Nav2 must never publish
directly to the drive or the guard output. The controller is the sole permitted
navigation velocity source; recovery behaviors and velocity smoothers are not
implicitly permitted additional writers.

## Installed artifact gate (OM-002)

Run inside `tmux orbital-work` on Thor:

```bash
/usr/bin/python3 scripts/check-nav2-compatibility.py
```

This read-only gate hashes the installed server executables, package manifests,
plugin exports/libraries, NavigateToPose interface and stamped-publisher header.
Missing executables, required plugins, incompatible interfaces, mixed versions
or an unreviewed release series fail before a configuration trial. It starts no
ROS nodes and commands no movement. Tests inject missing libraries, mixed
versions, incompatible action fields, missing stamped support and unavailable
executables. A matching manifest is not a dynamic-linker or lifecycle proof.

On 2026-10-28, Thor's nine required packages reported **1.3.13**, all artifact
checks passed and the source interface contained the expected goal/result fields.
The installed header and [Jazzy controller documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/)
agree that `enable_stamped_cmd_vel` defaults to false. Explicitly set it true;
the supervisor requires TwistStamped and preserves input age. Never introduce
an adapter that stamps old unstamped commands as fresh.

## Controller configuration trial (OM-003)

`config/nav2-controller.yaml` configures Regulated Pure Pursuit, pose-based
progress, stopped-goal acceptance and a rolling 2D obstacle/inflation costmap.
The footprint radius is 0.37 m, matching the known-map contract. Desired forward
speed is 0.15 m/s; rotate-to-heading speed is 0.4 rad/s. That rotation parameter
is not a universal angular-output clamp: the supervisor still rejects every
command outside its 0.2 m/s and 0.5 rad/s limits. Actual route tuning remains open.

In an empty localhost-only domain 84, run these inside Thor tmux orbital-work
with the Jazzy environment loaded (do not source the domain-42 project helper):

```bash
export ROS_DOMAIN_ID=84 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 scripts/check-nav2-controller.py
/usr/bin/python3 scripts/check-nav2-controller.py --unstamped-control
```

The first must exit 0; the deliberate unstamped control must exit 1 specifically
with `Unstamped controller output rejected`. Both results were observed on Thor
on 2026-10-28. The actual controller, goal/progress checkers and obstacle/inflation
plugins configured successfully. The controller remained inactive. DDS discovery
showed exactly one stamped output publisher and no controller publisher on the
raw drive/guard-input/guard-output topics checked by the probe. Each child process
was reaped, and native logs recorded plugin cleanup. No world, route or movement
was started. These results prove this configuration and output interface, not
collision avoidance, measured speed tracking or the production graph.

## Known-map planner trial (OM-004)

`config/nav2-planner.yaml` loads Navfn with unknown-space rejection and zero goal
substitution tolerance. The global costmap uses the known static map, a 0.37 m
robot radius and 0.70 m inflation. Production parameters use simulation time.

Run `scripts/check-nav2-planner.py` with Jazzy loaded in empty localhost-only
ROS domain 84, inside Thor tmux `orbital-work`. The probe starts only the native
planner/global costmap, publishes the versioned SDF-derived occupancy grid and
an isolated synthetic TF, and overrides time to wall clock. It never launches a
controller or world. Each owned child is terminated and reaped on exit.

The 2026-10-28 native run passed six successful plans: origin to each machine,
pump to cooler, cooler to power unit, and a detour around the cooler to (4, 0).
The detour was 8.232 m rather than the obstructed 4 m straight line. Every segment,
including start/goal connectors, passed an independent swept-disc check against
original occupied/unknown cell squares. This check does not trust Nav2 inflation
or merely inspect sparse waypoints. A deliberately direct path through the cooler
was rejected. Native occupied/outside goals aborted with codes 206/204 and no path.
No velocity publisher appeared on the four checked command topics.

Planner parameters SHA-256:
`5c04bce652d44a536ac76ed0be3c19493361f13c94b45e08298faa29c28cca1a`.
The JSON result binds probe, map, world and checker hashes and preserves complete
route points. Six local regressions exercise sparse crossings, footprint-only
collisions, unknown cells, invalid/outside paths and corner geometry; all 72 local
tests pass. These results establish static route clearance, not tracking accuracy,
dynamic collision avoidance or physical arrival. The test TF is not localization.

## Supervisor executable (OM-005)

`scripts/navigation-supervisor.py` loads named map-frame poses from
`config/known-map.json` and starts the existing guarded action owner on a
single-threaded executor. It validates finite numeric poses, normalized yaw,
unique JSON keys, stable names and a bounded goal timeout before creating a ROS
publisher. This is input validation; geometric pose commissioning remains a
separate gate. Only localhost-only domains 42 (Orbital) and 84 (isolated tests)
are accepted. No target is requested or armed at startup.

A checkout/domain file lock rejects duplicate local launches. Two seconds of DDS
discovery precedes startup: an existing `/motion/request` or `/navigation/state`
writer, or an incompatible/competing raw controller writer, prevents ownership.
The supervisor also checks command authority on its steady timer and command
path. A newly discovered competitor invalidates health, cancels the active
navigation target and requests the guard stop. This is accidental-conflict
protection among trusted processes, not a ROS security/authentication boundary.
Cross-checkout races remain subject to DDS discovery; full graph readiness is
OM-008. Exit stops publication; the independent guard watchdog must remain alive.

On 2026-10-28, `check-navigation-supervisor.py` ran the actual entrypoint with a
fixture pose file on empty domain84. All 11 scenarios passed, including existing
writer refusal, duplicate process refusal, late-writer cancellation, stale input,
health loss, stop-service loss, measured-arrival rejection/acceptance, explicit
cancellation and supervisor process death. The fake controller kept publishing
during death/health-loss checks. Five new input regressions bring local tests to
77. Source hashes for every deployed file match the tested worktree.

The action server, sensors and guard feedback in this trial are fixtures. This
proves ROS integration behavior, not physical stopping distance, real Nav2
tracking or localization accuracy. No live domain42 service was started.

## Owned native launch (OM-006)

`scripts/navigation.launch.py` owns controller_server, planner_server, bt_navigator
and their lifecycle manager. Localization, the motion guard and supervisor remain
separate owners. The launch remaps velocity to `/navigation/raw_cmd_vel`, starts
unconfigured by default and never respawns a child. Any owned child exit shuts
down the remaining launch. Production service ownership is the next gate; do not
use this low-level launch as an accepted live deployment procedure yet.

`config/navigate-guarded.xml` contains only periodic ComputePathToPose and FollowPath.
No behavior server, spin/back-up recovery, smoother or waypoint follower is started.
The supervisor retains cancellation/recovery decisions. Default simulation time
can be overridden only for the isolated native trial.

On 2026-10-28, `scripts/check-navigation-launch.py` passed twice on empty domain84:
normal shutdown and `--kill-controller`. In both cases, all three servers began
unconfigured and activated through an explicit lifecycle-manager STARTUP request.
The actual BT loaded and a goal at the stationary fixture pose succeeded with only
zero velocity output. DDS showed the controller as the sole stamped raw-command
publisher, with no writer on the three checked bypass topics. Both normal SIGINT
and controller SIGKILL ended the launch and its four native child processes.

The initial acceptance probe incorrectly assumed the kernel exposed
`/proc/PID/task/TID/children`. Thor does not expose it. The final probe discovers
only launch descendants using the `/proc/*/stat` parent relation and records PID
start identities before any teardown. It never kills by executable name. Final
source-bound results are `launch-result.json` and `launch-kill-result.json` in the
private review directory; both match the committed launch/config/probe sources.

This is native composition and process ownership evidence with synthetic static TF,
scan and odometry. It is not a moving-world route, supervisor-to-guard integration
or physical collision/stopping acceptance.

## Startup graph gate (OM-008)

`check-navigation-graph.py --phase before` now runs before the service starts its
children. It observes the graph repeatedly after discovery, requiring the expected
TF/map/sensor/guard publishers and services and no existing navigation owner, route
action server or bypass command publisher. Duplicate nodes/endpoints, wrong message
types and conflicting/missing TF parents fail. `--phase configured` instead expects
the exact supervisor/monitor/controller/planner/navigator graph for the later
pre-activation gate. Neither mode arms or activates anything.

Both modes require advancing simulation time, fresh stationary odometry and a fresh
latched guard with zero output. The pre-start mode can accept an unseeded AMCL node;
localization health and explicit arm still gate motion. This is a startup snapshot,
not continuous graph monitoring or a Gazebo transport security boundary.

The read-only live domain42 run passed on 2026-10-28 without changing services or
motion. Six isolated domain84 DDS cases also passed: valid before/configured graphs
accepted; a second TF endpoint on the same node, competing navigation action, bypass
velocity publisher and stale guard report were each rejected for the expected reason.
Seven new policy regressions bring the local suite to 84 passing tests. Results and
source bindings are retained in private `graph-readonly-result.json` and
`graph-fixtures-result.json` artifacts.

The installed Jazzy Python callback metadata was re-probed: it contains timestamps
and sequence fields but no publisher GID. Endpoint GIDs are available from graph
introspection, but this does not associate each TF sample with an endpoint. Reports
therefore explicitly set `per_message_tf_authority_proven: false`. Node/endpoint
counts and parent checks detect common duplicate-tree errors; stronger TF edge
authority attribution remains OM-016. Trusted node names are not authentication.

To repeat the read-only live check, load the Orbital ROS environment and run
`python3 scripts/check-navigation-graph.py --phase before`. For fault injection,
use `check-navigation-graph-fixtures.py` only on empty localhost domain84; the script
refuses another domain or an already populated graph.

## Deployment source identity (OM-009)

`navigation-manifest.py` records all navigation Python/shell sources, required
world/map/robot/bridge/controller/planner/navigator settings, installed compatibility
artifacts and Nav2 shared libraries (including BT/costmap/checker plugins). Its
content identity excludes absolute checkout paths and generated runtime files.
Missing required files, escaping source symlinks, failed compatibility and artifacts
that change during capture fail. This is a content record, not a signed release or
complete operating-system dependency lockfile.

Service startup now captures `source-manifest.json` before the graph gate, then
re-hashes source/installation and writes `source-verification.json`, binding the
successful graph report's SHA-256 to the unchanged manifest identity. A failed gate
cannot receive a successful binding. This detects observed before/after changes;
it does not freeze a mutable checkout, prove in-memory loaded code or detect a
change that was reverted between observations. Versioned deployment and live
process identity still need their own acceptance.

On 2026-10-28, the disposable native proof captured **74 source files and 107
installed artifacts** and bound a fresh read-only domain42 graph result. Changing
controller speed from 0.15 to 0.19 in that copy invalidated verification. Restoring
the original bytes recovered the same identity; a deliberately failed gate was
refused. All captured source hashes match this worktree. Seven new regressions
cover changed source/binaries, extra BT libraries, missing files, escaping symlinks,
manifest tampering and runtime-output exclusion; all 91 local tests pass.

The native acceptance identity was
`b746ccb4c4f280f0ba1f3be92eeb325428516a965fa89290c6daecfa6057afdd`.
It will intentionally change with later source/config/plugin deliveries. Full
source/artifact tables and the gate binding remain in private review artifacts.
`check-navigation-manifest.py` is a mutation test restricted to disposable copies
under Thor's Orbital runtime directory. It restores the changed configuration in
`finally`; it never edits the live checkout or starts navigation.

## Guarded lifecycle readiness (OM-010)

From an already-started navigation service, with the Orbital ROS environment loaded:

```bash
python3 scripts/check-navigation-ready.py --configure
python3 scripts/check-navigation-ready.py --activate
python3 scripts/check-navigation-ready.py
```

The first command accepts the exact unconfigured graph, explicitly configures all
three native servers and checks their inactive graph. The second revalidates source,
idle supervisor ownership and latched stationarity before lifecycle RESUME. With no
flags, a successful check leaves an already configured/active stack unchanged. No mode sends
a goal or calls reset_stop. A failure requests guard stop and lifecycle PAUSE; the
independent motion watchdog retains responsibility for motor deadlines.

Every run verifies the source manifest before/after, watches for any target, supervisor
restart, unlatched guard or velocity request, and checks fresh stationary graph
observations. It reports localization health separately: infrastructure readiness is
not permission to drive with invalid localization. Source identity is a before/after
observation, not a write fence. Per-message TF attribution remains unproven.

The isolated systemd-owned native proof passed configuration, activation, repeat,
changed-source refusal and explicit recovery. The initial failed-source stop RPC timed
out; reusing a client created at startup and allowing a bounded three-second limit for each discovery/
response wait passed the final proof. This does not change the motion watchdog timeout.

Live domain42 then passed configure, activate and a read-only repeat with the same
native-tested identity `4a5eda76f5780bcd255157fbf5eab1205c7ea7a4648b14a40ad93753afed0d5d`.
All three servers are active, localization is healthy, guard is latched and no target
or supervisor velocity request occurred. Gazebo pose queries before/after activation
reported zero net change at output precision (bounds 1 mm / 0.001 rad). This is a
stationary endpoint comparison, not continuous route/collision evaluation.

Deployment preserved demo/localization service invocations and their running source.
Forty files were added/updated after matching the reviewed inventory; the three existing
changed files were older localization probes lacking the unseeded checks. Previous
bytes and a rollback manifest were saved under the scoped deployment runtime directory.
Private `readiness-native-*`, `readiness-live-*` and deployment artifacts retain raw
results and source hashes. Full live failure/route acceptance remains open.

## Next gates

1. Controller/local-costmap inactive configuration is verified above; validate actual motion after full composition.
2. Known-map planning and native behavior-tree composition are verified above.
3. Native lifecycle, startup graph and source binding now pass in live guarded standby.
4. Complete live unseeded, reseed/reset/loss and stronger TF-authority gates.
5. Run announced bounded motion, independent arrival/collision evaluation and
   failure stopping before closing Phase 2.

The artifact check does not complete any of steps 1–5. Private raw reports live
outside Git; retain their source hash when collecting later acceptance evidence.


## Unseeded admission commissioning (OM-011)

Inside `tmux orbital-work` on Thor, after coordinating the localization restart:

```bash
bash scripts/check-navigation-unseeded.sh --restart-localization
```

This stops navigation, verifies the base graph and latched stationary feedback,
restarts only localization, and proves that active AMCL has no automatic pose or
map transform while scan/odometry/IMU advance. It then launches the actual
navigation service and configures its native servers. Global costmap activation
requires map-to-base TF, so this unseeded check keeps Nav2 inactive; active-stack
readiness with a seeded localizer is covered separately by OM-010.

The live supervisor receives six schema-valid, run-bound commands: request and
arm for each of the three machine names. Each must return an explicit rejection
with the exact sequence and operation. Arm uses a well-formed but unowned target
because the preceding request must not allocate one. No target ownership, healthy
localization, velocity request, guard release or odometry movement is allowed.
Before/after Gazebo pose must agree within 1 mm / 0.001 rad; this endpoint check
is not continuous collision or trajectory evidence. The probe always requests
stop on exit, and the runner tears navigation down on failure.

Results and source binding are written under `runtime/unseeded-<UTC timestamp>`.
Success intentionally leaves localization **unseeded**, native Nav2 **inactive**,
and the supervisor running with the guard **latched**. Explicit seeding/recovery
is a separate operator step; never describe this state as ready to navigate.

Thor run `unseeded-20261028T114225Z` passed with AMCL state 3 and all native Nav2
servers state 2. Six commands were rejected, zero velocity requests were observed,
and independent pose endpoints agreed exactly at Gazebo output precision. Source
identity: `145b33f33caa9b155cc4019592a16f99ecf8d4b4e83afb5b1888c97b034a4cbe`.
A seeded negative control failed on the unexpected AMCL pose, as intended. An earlier
startup failed closed because the graph gate retained incomplete discovery samples,
even though its final graph was correct; the successful explicit rerun does not
remove that startup usability limitation. Both reports are retained privately.


## Offset-seed convergence into native navigation (OM-012)

This commissioning **restarts the world and moves the robot**. Coordinate before
running it inside `tmux orbital-work` on Thor:

```bash
source scripts/ros-env.sh
python3 scripts/check-navigation-convergence.py --restart-world-and-jog
```

The runner stops navigation, explicitly stops the guard and starts a new known-origin
world/localizer. Active unseeded AMCL must first pass the no-pose/no-map-TF gate.
The operator then supplies a deliberate `(0.20, -0.15, 0.15)` x/y/yaw seed with
uncertainty. Evaluator ground truth is never used to generate that seed. Existing
bounded controls command forward for 1 s at 0.12 m/s, then left for 0.7 s at
0.3 rad/s, with explicit arm and acknowledged stop for each jog. Navigation is
absent during these manual commissioning movements.

Independent Gazebo comparison must satisfy the unchanged 0.10 m XY / 0.10 rad yaw
localization limits, with fresh sensors, map agreement and a level stationary body.
Native navigation then configures/activates behind the latched guard, must observe
healthy localization, and must emit zero supervisor velocity requests. A second
independent comparison must still pass, with no startup pose change above 1 mm /
0.001 rad. Source/config/installed Nav2 identities must match before and after.
Failures stop navigation and the guard, falling back to stopping the owned world
if the stop command cannot be acknowledged.

Thor `navigation-convergence-20261028T114740Z` passed: XY error **0.0752675 m**,
yaw error **0.00673083 rad**, unchanged after native activation. Gazebo endpoint
translation/yaw change during startup was zero at reported precision. All three
Nav2 servers reached state 3, localization was healthy, and the guard stayed latched
with zero supervisor velocity requests. Source identity:
`3e31a0f418d6ce62ae7633b9e96704cbdce4ee0064c3c2bf4991eea961889788`.
All 79 captured source files match the delivered tree. This is one measured
convergence run into native readiness, not statistical localization accuracy,
continuous tilt/collision monitoring, autonomous arrival or recovery during a goal.


## Live localization freshness faults (OM-013)

Coordinate these stationary fault injections inside Thor `tmux orbital-work`:

```bash
source scripts/ros-env.sh
python3 scripts/check-navigation-freshness.py
python3 scripts/check-navigation-freshness.py --scan-gap
```

The default case finds the exact AMCL executable inside the owned localization
cgroup, pins its process with a Linux pidfd, and temporarily suspends it. A separate
helper requests resume after a two-second timeout or earlier stdin closure. The
scan case uses the discovered Gazebo `/scan/set_rate` Double→Empty service to
lower only lidar to 0.1 Hz; its helper restores the versioned 10 Hz rate after
stdin closes or a three-second timeout. Service calls have their own bounded
waits, so these are restoration attempts, not hard real-time timing guarantees.
No arm or velocity command is sent. Exceptions request guard stop, and the AMCL
case also resumes the pinned process in cleanup.

Both cases require initially healthy, idle native Nav2 with all servers active.
During the fault, fresh supervisor/guard/health/odometry must continue and the
specific `transform_invalid` or `scan_stale` reason must precede an explicitly
rejected target request. The TF case additionally requires advancing lidar.
Recovery must acknowledge stop and remain idle/latched; a new request is then
accepted as a positive control and explicitly cancelled without arming. No velocity
request, dispatch, odometry movement or service invocation change is allowed.

Final native `freshness-3` passed both cases. Rejection was observed 0.689874 s
(TF) and 1.165830 s (scan) after helper launch; these include process/service
setup and acknowledgment, and are **not physical braking times**. The recovered
scan stream supplied 15 advancing samples in 1.5 s. All service/supervisor run
identities remained unchanged, and both stop calls were acknowledged. Source
identity `4592da65126331421671819fd672c8b5de977019fa6f8e32b11ab8c610a1b17e`
matches all 80 delivered source files before and after both cases.

The first scan harness failed its receipt-age assertion because a blocking helper
acknowledgment wait delayed sensor callbacks. The final harness spins ROS during
that wait; both the failed report and successful rerun are retained. Separate
helper controls observed resume after approximately 2.03 s with the pipe held open
and 0.009 s after closure. Their first immediate process-state assertion raced
signal delivery; the corrected control waits for the stopped state. The helper
function is unchanged between those controls and the delivered script.

AMCL legitimately retains its last pose while stationary. Current advancing scan,
odometry, IMU and time-specific TF remain mandatory; retained pose/covariance alone
cannot make localization healthy. A replacement seed/reset separately requires
post-seed pose evidence. This freshness acceptance does not prove pose accuracy
or stopping during an active moving goal; OM-012 covers sampled accuracy and the
remaining reset/reseed/physical-stop gates cover those transitions.


## Live reset and old-target invalidation (OM-014)

This test resets the live Gazebo world to its known origin. Coordinate first,
then run inside Thor `tmux orbital-work`:

```bash
source scripts/ros-env.sh
python3 scripts/check-navigation-reset.py
```

With seeded active Nav2 and the guard latched, it reserves an **unarmed** pump
target and calls `/world/maintenance/control` with `reset { all: true }`. It must
observe an actual ROS-clock regression, a higher health epoch, terminal ownership
of the old target and persistent `initial_pose_required`. Both an arm of the old
token and a new request are rejected. One second of fresh post-reset input must
not bypass the explicit seed requirement.

The test supplies the reset world's known origin with uncertainty, without using
evaluator truth. It waits for a post-seed AMCL pose and healthy idle navigation,
then checks the old target's exact `not_dispatched` terminal record. The old arm
must still be rejected. A new cooler request must have a higher target epoch;
that target is cancelled without arming. Native service and supervisor invocation
identities must stay unchanged, so restarting ownership cannot satisfy the test.
Any velocity request, action dispatch, odometry movement or guard release fails it.

Native `navigation-reset-1` passed: simulation time regressed from 1340.630 s to
zero, the pump token at target epoch 14 was invalidated, and the new cooler token
used epoch 15. Both old-token arm attempts were rejected; the unseeded new request
was rejected, and the post-seed new request/cancel pair succeeded. The final state
was healthy/idle/latched with zero forwarded commands. Source identity
`4742ab14f1354ab63ecef3a2494e57b3f7a0d81aad85c05a0fe21575c7d97dfb` matches all 81 delivered source files before and after the run.

This proves reset of an allocated unarmed target in the actual native stack.
It does not prove cancellation or braking of a moving goal; the moving-goal reset
and reseed fault gates remain required. The direct test seed is restricted to the
acknowledged known-origin reset and a terminal, latched target; it is not a general
operator authorization to reseed arbitrary active motion.


## Dispatched-goal reseed and bounded trial ownership (OM-015)

This is a short **native motion** trial near the freshly reset origin. Coordinate
first, verify the disposable guardian proof, then run in Thor `tmux orbital-work`:

```bash
python3 scripts/check-navigation-trial-guard.py
source scripts/ros-env.sh
python3 scripts/check-navigation-reseed.py
```

`navigation-trial-guard.py` pins the navigation service's exact MainPID using a
Linux pidfd after checking InvocationID, cgroup membership and KillMode. It stops
that owner on deadline or parent-pipe loss, escalating to SIGKILL after one second
if necessary. A replacement invocation cannot be signalled through the old pidfd.
Only an exact `safe` acknowledgment disarms it; the trial sends that after verified
latched, stationary, idle completion. Existing systemd containment and the drive
guard remain responsible for descendants and physical braking. This guardian is
an operational trial bound, not a hard real-time or hardware safety certification.

Five native disposable-service cases passed: wrong-generation refusal, explicit
disarm, parent-pipe loss, deadline, and replacement-owner isolation. The failure
fixtures deliberately ignore SIGTERM in both owner and child; both processes were
gone after cgroup cleanup. No actual simulation service was stopped by those tests.

The motion trial pins a 12-second independent deadline, requests/arms the real pump
goal, and requires running Nav2 plus measured odometry motion before publishing a
new uncertain origin seed. It stays within 0.15 m / 0.3 rad at seeding; continuous
odometry-region and IMU-tilt bounds abort the trial. The exact dispatched target
must reach cancelled/aborted terminal state, measured stationarity and the stop
latch. The old token must fail re-arm and its command count must stay unchanged
through a recovery observation. A fresh cooler request is accepted, then cancelled
without arming. Failure closes the guardian pipe as well as requesting guard stop.

Native `navigation-reseed-1` passed. Four supervisor requests produced initial
rotation; measured angular speed before reseeding was 0.0766208 rad/s. The exact
pump target (epoch 16 / sequence 8) ended **cancelled** 0.224119 s after the seed, with
stationary odometry and a latched guard. This is not independently measured physical
braking time/distance. Old-token arm was rejected; the fresh epoch 17 / sequence 9
request/cancel succeeded and the guardian disarmed after 1.690 s. No service restart
was needed. A separate post-trial Gazebo check measured pose
`(0.000386, 0.000002, 0.012884)` and localization error 0.0109483 m / 0.0208018 rad.
That endpoint check confirms the small rotation, not continuous collision clearance.

All 85 captured sources match identity
`413b13a500013aebec1017ee671720a93475258359efa56787d3ba7025f6fd14` before/after
native execution; all 92 local tests pass. This is the first dispatched actual Nav2
goal and its reseed cancellation. Arrival at the three machines, blocked/changed
obstacles and independent physical stopping/collision evaluation remain open.

## Per-message TF authority (OM-016)

The Python graph gate retains its narrower endpoint/parent scope. The native
auditor adds the missing message-to-publisher relation using
`rclcpp::MessageInfo::get_rmw_message_info().publisher_gid`. It joins that GID to
endpoint metadata from the **same ROS context**; GIDs are not compared across
process contexts and node names are not authentication.

| Edge | Expected publisher | Topic |
| --- | --- | --- |
| map → odom | amcl | /tf |
| odom → base_footprint | ros_gz_bridge | /tf |
| base_link → left_wheel/right_wheel | robot_state_publisher | /tf |
| base_footprint → base_link | robot_state_publisher | /tf_static |
| base_link → lidar_link/camera_link | robot_state_publisher | /tf_static |
| camera_link → camera_optical_frame | robot_state_publisher | /tf_static |

Run inside Thor `tmux orbital-work`, with no motion commands:

```bash
bash scripts/build-tf-authority.sh
# Private proof refuses an occupied domain. No simulator or drive is launched.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=84 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 scripts/check-tf-authority.py --output runtime/tf-proof-new-run
# Read-only live gate; use before subsequent route trials.
source scripts/ros-env.sh
python3 scripts/audit-tf-authority.py
```

The build uses installed Jazzy rclcpp/tf2_msgs, CMake, a C++17 compiler and
nlohmann-json. Thor verification used GCC 13.3.0, rclcpp 28.1.22, Fast DDS RMW
8.4.4 and nlohmann-json 3.11.3. The executable stays in ignored runtime storage.
The live wrapper binds before/after source manifests and executable SHA-256;
it does not certify reproducible compilation or the complete runtime library set.
C++ and CMake sources are now mandatory manifest inputs. A regression first
failed because changing the auditor left the old Python-only manifest unchanged.

Seven real-DDS fixture cases passed in `tf-authority-proof/fixtures-3`: correct
owners, wrong owner, wrong parent, nonfinite translation, missing edge, duplicate
same-name publishers, and frozen dynamic timestamps. The wrong-owner case retains
the correct endpoint inventory and TF parents, isolating the missing relation.
Static transforms are published once before the auditor joins. Dynamic edges
must have multiple observations and advancing, nonregressing stamps. Unknown GIDs,
multiple owners and invalid geometry fail the gate.

Initial `fixtures-1` and `live-1` failed closed on node-name discovery placeholders.
`live-1` records `_NODE_NAME_UNKNOWN_`/`_NODE_NAMESPACE_UNKNOWN_` resolving to the
correct node. The auditor now defers incomplete metadata instead of treating it
as a conflicting identity. It still rejects unresolved messages/final endpoints
or any conflict between two complete identities. Failed evidence is retained.

Live `live-2` passed eight edges in 256 messages, with unchanged service identities,
healthy idle/latched navigation and zero new velocity requests. All 90 source files
match `5b0d335e780bf2e7fd93fdf8b3faba193221af026e0c4545d2b39a44fc5e2ff2`.
The native executable SHA is
`7e0794e504d7951e49f6dab48dfb4a5e3639b4119068251dcb2340252442f4f6`.
Independent Gazebo comparison still reports 0.0109483 m / 0.0208018 rad localization
error. All 94 local tests pass. A passing bounded sample is not a continuous TF
interlock, localization accuracy proof or machine-arrival/collision acceptance.

## Map revision binding (OM-017)

Navigation startup now runs `check-map-binding.py --localization-only` before
creating its children. Readiness runs the full map gate before activation, and
its existing failure path requests guard stop and lifecycle PAUSE. The gate sends
no motion command, seed or map update.

The gate binds these observations:

- The map server's `yaml_filename` names the revision derived from the current
  world, map configuration and generator bytes. PGM, YAML and manifest must match
  the independently regenerated expected bytes; escaping/symlinked paths fail.
- Both the retained `/map` message and the map server's GetMap response have the
  expected frame, dimensions, resolution, origin and every occupancy cell.
- Map server ownership and AMCL/global-costmap subscriptions have the expected
  names, types and retained/reliable QoS. Consumer frame/topic settings agree.
- The supervisor reports the SHA-256 of the exact configuration bytes from which
  its waypoints were parsed. This parameter is read-only and ignores overrides;
  changing a file does not relabel already-loaded waypoints as the new revision.

The map gate waits for incomplete DDS endpoint-name discovery, but rejects a
complete wrong/duplicate publisher. It rechecks artifact bytes before returning.
This is a bounded startup/readiness contract. It does not expose private AMCL or
costmap memory, prove Gazebo's loaded geometry, freeze files or continuously block
later map updates. Localization freshness, per-message TF auditing and independent
route/collision acceptance remain separate gates.

Read-only checks on configured Thor:

```bash
source scripts/ros-env.sh
python3 scripts/check-map-binding.py --localization-only
python3 scripts/check-map-binding.py
```

Private negative cases use `check-map-binding-fixtures.py --output NEW_DIRECTORY`
in empty localhost domain84. The native readiness fixture additionally changes
only AMCL's running `map_topic` parameter: source bytes remain unchanged, so the
map gate itself must reject activation and acknowledge stop/pause. Restore the
parameter and request explicit activation to recover. No drive exists in that
fixture. Its base-process readiness marker carries PID plus a per-run UUID;
stale marker files cannot start the next generation early.

Seven isolated map-gate cases passed: valid, wrong loaded revision, changed cells,
wrong AMCL topic, wrong planner topic, wrong supervisor configuration and restored
valid state. Seven actual Nav2 readiness stages also passed: configure, activate,
wrong runtime map consumer with stop/pause, explicit map recovery, read-only
repeat, changed-source refusal and explicit source recovery. That run used source
identity `b7310e23ee1bb43dab9c7d90ed48e142b4442ceb9b976134fd5a31659531e646`.

Retained failed trials exposed incomplete endpoint-name discovery and a stale
fixture-ready marker. Both are handled explicitly above. Another activation RPC
timed out; the manager log did not show RESUME, while the subsequent PAUSE was
acknowledged. The readiness client now creates its lifecycle RPC endpoint at
startup, alongside the stop client, so graph/source checks allow discovery before
activation. The final native run passed without increasing deadlines or adding
retries; the exact cause of that earlier transport failure remains unproven.

Live deployment then passed activation, map binding, the separate per-message TF
check, idle/latched observation and independent pose comparison. Only navigation
was restarted; demo/localization InvocationIDs stayed unchanged. Before/after
Gazebo pose was `(0.000386, 0.000002, 0.012884)` at reported precision. New supervisor
`20828d54-02c0-40ce-9f7b-a37bb06454c0` is healthy/idle/latched with no target and zero
forwarded commands. Navigation InvocationID is `5eed299576fe47009e74de5cb28eb30a`.
All 94 source hashes match the native identity above; 100 local tests pass.
Raw artifacts and pre-deployment backups are in private
`reviews/orbital-nav2-20261028/om017` and Thor `runtime/map-binding-proof`.
No route goal was requested; independent arrival and physical failure stopping
remain required. The prior startup-manifest mismatch is resolved by this explicit
deployment, while earlier manifests remain historical evidence.
## OM-018 arm callback regression: native acceptance pending

The supervisor must validate health on command admission, even if its periodic
timer is delayed. A guard response must not dispatch a target after an intervening
moving observation, including a move/recovery pair processed before the timer.

`python3 -m unittest discover -s tests -p test_supervisor_arm.py -v` executes seven
cases against the production supervisor, health interlock and action session with
controlled ROS transport and monotonic time. On `b712384`, four cases fail:
expired-health request, expired-health arm, moving-feedback acknowledgment and
move/recovery-before-tick. The fixed source passes all seven and the full
107-test local suite. The same test file was used on both source variants.

The native counterpart is `scripts/check-navigation-arm.py`, restricted to empty
localhost ROS domain 84 with the production guard/supervisor, synthetic health
and a Nav2 stand-in that records and rejects goals. Its seven cases are prepared,
not accepted: the configured Thor SSH connection timed out on 2026-10-28. No
runtime deployment or physical-arm acceptance follows from the local result.
Run the native regression and positive controls before deploying this change.
