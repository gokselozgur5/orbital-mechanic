# Project status

Updated: 2026-10-24

## Working and measured

- Thor ARM64: Ubuntu 24.04.3/L4T R38.2.2; ROS 2 Jazzy, Gazebo Harmonic, Nav2, gz_ros2_control and Foxglove Bridge installed. No blanket OS upgrade or reboot performed.
- One maintenance room, three machine props and a wheeled robot render through Ogre1/llvmpipe. VNC screenshot inspected: scene is visible.
- Clean 20-second sensor probe passed after service stop/restart: 640x480 RGB images, camera calibration, 360-sample lidar, IMU, odometry, joints, dynamic/static transforms and simulation clock. No timestamp regressions. See EVIDENCE.md for exact measurements.
- Read-only machine telemetry is live. The latched-fault scenario reports a 24 V bus. It is a simplified model, not Gazebo electrical physics.
- Foxglove legacy WebSocket handshake succeeded through the Mac SSH tunnel. Both VNC and Foxglove bind only to Thor loopback.
- Stop/restart test confirmed no Gazebo process or listener remained after stopping the service. The next launch had exactly one server and one GUI.
- Eight ROS-free machine tests pass; scripted model baseline covers healthy, repaired latch and safely escalated jam.

## Mac client status

The existing Mac Foxglove Studio 1.84.0 crashes at startup, including with GPU acceleration disabled. Official Foxglove 3.4.2 was downloaded, its app signature verified, and installed separately under the local workspace. The user completed pairing and confirmed that Foxglove opened. Thor logs independently show the Mac client connected and subscribed to camera image/calibration and TF topics. No subscription purchase or data upload was initiated by the agent. macOS Screen Sharing was opened; the VNC transport and rendered desktop were independently verified.

The optional bridge 0.8.5 overlay was built successfully for the old protocol. APT bridge 3.6.0 remains installed. Keep this distinction explicit; changing the bridge does not fix the old Mac application's startup crash.

## Runtime ownership

Thor project: `/home/jensen/.goske/codex/orbital-mechanic`. Start/stop using scripts/launch-thor.sh and scripts/stop-thor.sh. Transient user service `orbital-demo`; ROS domain42; Gazebo partition `orbital-maintenance`; display42; localhost VNC5942/Foxglove8765. Mac forwarded ports15942/18765. No boot-start configuration.

All project artifacts and caches are under this root. The ~/.gz path is a compatibility symlink to runtime/gazebo-home, created after verifying that the original directory came from this session. Runtime files, source dependencies, media and credentials are excluded from Git.

## Decisions

Anthropic is the selected future planner provider based on user-reported USD200 credit. No API key loaded or credits spent. Select and price the exact model when connecting the credential, with explicit per-mission and experiment caps.

Use standard ROS control/navigation for motion, a restricted planner gateway, and independent evaluation. The current demo uses Gazebo's differential-drive plugin; the installed ros2_control/Nav2 packages are not yet integrated into autonomous motion.

## Phase status and next work

Phase 0 design/setup is accepted: the user approved the baseline and phase-tracking proposal, sensor/lifecycle evidence is recorded, and Mac Gazebo/Foxglove opening is confirmed. The six milestones and 15 dependency-ordered issues are indexed in [TRACKING.md](TRACKING.md). [PHASES.md](PHASES.md) supersedes the earlier exploratory M-numbered ordering.

Next: Phase 1 issue #5 provides bounded operator controls and a usable motion view. Policy review #2 is complete. The guarded ROS adapter/bridge is deployed and its eight live acceptance cases pass; see MOTION.md. Failure injection now passes; operator controls remain pending. No navigation, integrated repair, robot arm, visual diagnosis, SLAM or LLM-controlled mission is claimed.

## Mac Gazebo viewer fix

User reported native Screen Sharing could not communicate with localhost. Verified SSH master alive and RFB3.8/no-auth handshake reachable; native client closed before requesting frames. Added a Mac-local noVNC/WebSocket proxy in tmux orbital-viewer. The user confirmed the Gazebo display appeared in the browser. Default Mac opener now uses localhost15943 through the existing15942 SSH tunnel. No Thor runtime or motion code changed in this fix.

## Guarded motion integration

The Thor tmux `motion-integration-2` job passed 18 unit tests and eight live motion checks. Forward travel was 0.2233 m; the observed timeout latch arrived 0.3233 s after the drive loop returned (not a precise last-publish latency measurement). Reverse, rotation, explicit stop, blocked late commands and invalid input rejection passed. One server and one GUI remained. The robot was left latched stopped. See [MOTION.md](MOTION.md); subsequent process-death, pause/reset and stopping measurements are recorded below.

## Motion failure acceptance

Two final fault runs passed after fixing long teardown and a transient-unit restart race. Final measured timeout stopping: forward 0.0562 m / 0.3805 s, reverse 0.0551 m / 0.3677 s. Pausing/resuming and world reset did not restart motion; guard SIGKILL terminated all 14 sampled demo processes in 0.3227 s. Fresh restart was latched/stationary; duplicate launch was rejected. Full evidence, limits, earlier failures and source identity checks: [MOTION_EVIDENCE.md](MOTION_EVIDENCE.md).

Final 20-second sensor probe passed functionally, but camera delivery fell to 14 frames (~0.7 Hz); do not claim stable 5 Hz or unchanged camera performance. Lidar/odom/IMU remained about 10/20/20 Hz. Phase 1 stays open pending operator controls/view (#5).

## Operator CLI delivery

Bounded arm/stop/status/jog CLI and real-process acceptance are implemented; see [OPERATOR.md](OPERATOR.md). Final CLI exercise passed four directions and SIGKILL disconnect stopping (0.4394 s / 0.0497 m). Issue #5 remains open for the composed motion/camera/lidar view and cancel variants.

Camera investigation: Gazebo source measured ~4.98 Hz; an isolated 10-second ROS comparison received 1 best-effort versus 51 reliable frames. A subsequent multi-topic reliable probe received 36 frames but ended with old image timestamps, so changing QoS alone is not accepted as a complete fix. Probe freshness checks and continued transport/executor diagnosis are next; earlier functional sensor passes did not assert end-of-run image freshness.
