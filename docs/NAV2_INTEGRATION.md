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
