# Project status

Updated: 2026-10-24

## Current phase and next action

Phase 0 design/setup is accepted. Phase 1 motion policy, ROS integration, bounded operator CLI and failure tests pass. Issue #5 remains open for user-rendered acceptance of the combined Foxglove camera/lidar/motion view. The initial layout import failed; its native desktop format is corrected, but the saved `foxglove-operator` record still contains the old notebook data (verified read-only). Import the distinctly named `config/foxglove-operator-desktop.json`; successful rendering has not been confirmed. Do not close Phase 1 based on JSON structure alone.

The user handles Foxglove clicks, keyboard input and layout import. Do not automate that UI without renewed authorization. Avoid restarting the simulator while the user is inspecting it without coordinating. Read-only probes and offline preparation can continue.

Next: confirm the rendered operator view, then complete Phase 2 localization and guarded navigation. Offline map preparation does not bypass that gate. The six milestones and 15 issues are indexed in [TRACKING.md](TRACKING.md); [PHASES.md](PHASES.md) defines acceptance.

## Installed and running baseline

- Thor ARM64: Ubuntu 24.04.3/L4T R38.2.2, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, Nav2 and gz_ros2_control installed. Installation is not navigation integration.
- One room, three machine props and a wheeled robot render with Ogre1/llvmpipe. Ogre2 startup failed; no board reboot, blanket OS upgrade or PyTorch GPU inference was attempted.
- RGB 640 × 480, camera calibration, 360-sample lidar, IMU, wheel joints, odometry, transforms and simulation clock are bridged. Read-only machine telemetry uses a separate simplified causal model, not electrical physics or robot-integrated maintenance.
- Mac Gazebo viewing uses local noVNC through SSH; the user confirmed the image appeared. Native macOS Screen Sharing failed and is not the default viewer.
- Foxglove 3.4.2 is installed separately from the crashing old 1.84 client. The user completed pairing and opened it. Camera/calibration/TF subscriptions and the WebSocket handshake were verified. The optional source-built 0.8.5 bridge overlay is used; APT bridge 3.6.0 remains installed.

## Verified motion and safety

The guard accepts bounded stamped `/motion/request` commands. Startup, stop, timeout and invalid feedback latch motion off; explicit re-arm requires fresh stationary feedback. Old commands cannot restart motion. This is a trusted local-process safety boundary, not authentication or collision avoidance.

Eight live motion checks passed. Final operator acceptance measured approximately 0.0841 m forward and reverse, verified both turn directions, and required subscriber discovery plus guard acknowledgment before reporting successful command delivery. SIGKILL/SIGINT/SIGTERM/SIGHUP cancellation stopped through the independent watchdog in 0.373–0.412 s and 0.040–0.048 m in that run.

Pause/resume and world reset did not resume old motion. In the final combined fault run, killing the guard removed all 14 sampled demo processes in 0.223 s; clean restart was latched and stationary. The service cgroup owns detached Gazebo descendants, and bounded cleanup fixes were verified. These measured simulation cases are not universal stopping-distance guarantees.

See [MOTION.md](MOTION.md), [MOTION_EVIDENCE.md](MOTION_EVIDENCE.md) and [OPERATOR.md](OPERATOR.md) for interfaces, limits and earlier failures. The local `scripts/check-phase-one.sh` convenience runner is an uncommitted draft and has not been executed end-to-end; the individual acceptance scripts have been run. That draft is not an additional phase gate.

## Camera freshness

Earlier camera delivery was intermittent and a reliable-QoS-only trial still ended stale. A project-local 16 MiB Fast DDS shared-memory profile and common `scripts/ros-env.sh` resolved the reproduced short-window failure. No global kernel settings or unrelated services were changed.

The final combined run received 97 images in 20 seconds with approximately 50 ms final receive age. Three subsequent 20-second windows received 100/96/100 images with final ages 24.6/37.2/63.4 ms. All passed strict receive freshness, image timestamp age and other sensor checks. This supports approximately 5 FPS in those windows, not long-duration stability or measured camera-to-screen latency.

## Offline navigation preparation

`python3 scripts/generate-known-map.py` derives a 210 × 170 occupancy grid at 0.05 m resolution from static SDF collision geometry. Generated PGM/YAML/manifest files stay in ignored `runtime/maps`; source, configuration and generator hashes identify them. The inflated-grid check finds paths to all three proposed inspection poses using a conservative 0.37 m radius.

All 24 unit tests pass on Mac and Thor, including six map cases. Repeated local generation is identical; Thor generated the same PGM SHA-256 `d08588af2b1186992411e32dbf1d703e47ea45d183666de13c04a84cbe9c5000`. The offline job exited 0 with the existing demo still active; it did not restart the world. Map and navigation-contract commits passed GitHub CI.

See [NAVIGATION.md](NAVIGATION.md). AMCL convergence, measured TF geometry/ownership, localization loss/reset behavior, obstacle-aware routes and actual arrival at machines remain unimplemented or unverified. No autonomous navigation is claimed.

## Frame geometry correction — live acceptance pending

Read-only inspection found a 0.22 m vertical mismatch: live `odom→base_link` had z=0, while Gazebo reported the settled model near z=0 and its body at model-relative z=0.22. Consequently the ROS lidar/camera mount heights were 0.16/0.12 m instead of the model's nominal 0.38/0.34 m. Matching topic frame names did not establish matching physical geometry.

The source now separates planar `base_footprint` from the elevated `base_link`, adds the fixed 0.22 m body transform and adjusts the guard's expected odometry child. Velocity requests remain in `base_link`; the frames have the same planar axes and differ only in height. Three cross-model regressions failed before this change; all 27 local unit tests pass afterward.

**This correction is not deployed into the running world yet.** Thor remains on the previous geometry while the user examines Foxglove. Only the read-only `scripts/check-frames.py` was transferred: it correctly exited 1 on the old frame tree while confirming fresh odometry and the expected publisher-node graph. After coordinated deployment/restart, run it plus the existing motion/operator/fault/sensor checks before claiming live acceptance. Its publisher-node and observed-parent checks do not prove per-message TF ownership; this installed Jazzy callback metadata lacks publisher GIDs.

## Runtime ownership and boundaries

Thor root: `/home/jensen/.goske/codex/orbital-mechanic`. Perform operational work inside `tmux orbital-work`; use `scripts/launch-thor.sh` and `scripts/stop-thor.sh` to manage transient user service `orbital-demo`. There is no boot-start configuration. Source `scripts/ros-env.sh` for ROS tools.

ROS domain 42 is localhost-only; Gazebo partition is `orbital-maintenance`. VNC/Foxglove bind to Thor loopback ports 5942/8765, forwarded to Mac 15942/18765. Mac noVNC uses localhost 15943. Artifacts, logs and caches stay under the project root; `~/.gz` links to its runtime cache. Dependencies, generated outputs, media and credentials remain outside Git. Unrelated llama/openclaw services are untouched.

Anthropic is the future planner provider based on user-reported USD 200 credit; available balance is unverified. No API key is loaded, no paid calls or resources were opened. A scripted mission and independent evaluator precede LLM experiments. No integrated repair, robot arm, visual diagnosis, SLAM or LLM-controlled mission is claimed.
