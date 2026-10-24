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

## Client limitation

The existing Mac Foxglove Studio 1.84.0 crashes at startup, including with GPU acceleration disabled. Official Foxglove 3.4.2 was downloaded, its app signature verified, and installed separately under the local workspace. It opens successfully but requires account sign-in for the live view. No sign-in, subscription or upload was completed. User input is pending on account availability. macOS Screen Sharing was opened; the VNC transport and rendered desktop were independently verified.

The optional bridge 0.8.5 overlay was built successfully for the old protocol. APT bridge 3.6.0 remains installed. Keep this distinction explicit; changing the bridge does not fix the old Mac application's startup crash.

## Runtime ownership

Thor project: `/home/jensen/.goske/codex/orbital-mechanic`. Start/stop using scripts/launch-thor.sh and scripts/stop-thor.sh. Transient user service `orbital-demo`; ROS domain42; Gazebo partition `orbital-maintenance`; display42; localhost VNC5942/Foxglove8765. Mac forwarded ports15942/18765. No boot-start configuration.

All project artifacts and caches are under this root. The ~/.gz path is a compatibility symlink to runtime/gazebo-home, created after verifying that the original directory came from this session. Runtime files, source dependencies, media and credentials are excluded from Git.

## Decisions

Anthropic is the selected future planner provider based on user-reported USD200 credit. No API key loaded or credits spent. Select and price the exact model when connecting the credential, with explicit per-mission and experiment caps.

Use standard ROS control/navigation for motion, a restricted planner gateway, and independent evaluation. The current passive demo uses Gazebo's differential-drive plugin; the installed ros2_control/Nav2 packages are not yet integrated into autonomous motion.

## Next work

1. Complete Foxglove client sign-in or choose an approved no-account viewer; verify camera/lidar panels.
2. Add a command watchdog, latched stop and bounded manual-motion test before bridging motor commands.
3. Add known-map navigation and trusted proximity checks.
4. Connect the existing machine model to robot inspections/actions and the independent evaluator.
5. Complete scripted missions before enabling Anthropic tool planning.

No robot arm, physical repair, visual diagnosis, SLAM or LLM-controlled mission is claimed. M0 simulation/transport gates are evidenced; client GUI acceptance is pending. M1 control and M2 integration remain open.
