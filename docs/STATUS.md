# Project status

Updated: 2026-10-28

## Active delivery goal

The user resumed Orbital work and requested at least 100 substantive atomic commits. [GOAL_100.md](GOAL_100.md) defines the product, evidence gates and counting rules. The isolated branch `codex/nav2-integration` is based on the corrected Gmail-attributed main; its six migrated navigation commits preserve the original source tree and do not count toward the new goal. Original branch and dirty main drafts are untouched. OM-002 now provides a read-only installed-artifact compatibility gate. Thor reports Nav2 1.3.13 consistently across nine required packages; server/plugin artifacts, stamped-publisher support and the action contract pass. Seven rejection/control tests bring the local suite to 66 passing tests. This does not prove plugin runtime loading or lifecycle compatibility. OM-003 now configures the real controller/local costmap and verifies plugin loading, inactive lifecycle state and sole stamped output on empty domain84; an unstamped negative control fails as required. No world or movement was started. OM-004 now verifies native known-map planning: six clear swept-footprint routes, occupied/outside goal rejection and an independently rejected straight path through a machine. All 72 local tests pass. Native result source hashes and route points are retained in private review artifacts; see NAV2_INTEGRATION.md. OM-005 adds the validated supervisor executable with duplicate-owner and continuous command-authority checks. All 77 local tests and 11 composed ROS fixture scenarios pass, including existing/late competing publishers and process death. Native source manifests and results remain in private review artifacts. OM-006 now composes the native planner/controller/navigator and lifecycle manager with a restricted behavior tree. Empty-domain native tests passed explicit startup, a zero-motion goal, normal shutdown and controller-death teardown of all five owned processes. Next: add production service ownership, graph preflight/source binding/readiness and then prove the first measured route. No new autonomous-motion acceptance has occurred yet.

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

The current source also adds an unseeded negative gate to the probe/runners; that addition is syntax-checked but not yet live-accepted. These scripts remain separate from the isolated health components and have not been redeployed or merged into shared `main` while another Claude's ownership is unresolved. See [NAVIGATION.md](NAVIGATION.md) for exact scope. Initial commissioning does not close #6/#7 or prove autonomous routes.

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
