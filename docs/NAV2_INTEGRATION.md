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

## Next gates

1. Controller/local-costmap inactive configuration is verified above; validate actual motion after full composition.
2. Known-map planning is verified above; compose the navigation behavior tree.
3. Configure actual lifecycle nodes in an empty private domain, inspect their
   graph and parameters, and verify the real stamped publisher type.
4. Deploy the owned navigation service with shared-world preflight and a latched
   guard. Confirm that no command bypass or competing controller exists.
5. Run announced bounded motion, independent arrival/collision evaluation and
   failure stopping before closing Phase 2.

The artifact check does not complete any of steps 1–5. Private raw reports live
outside Git; retain their source hash when collecting later acceptance evidence.
