# Verification evidence — 2026-10-24

## Scope

Measured on the selected ARM64 Thor after a full stop/restart of the demo user service. These are initial smoke-test results, not long-duration reliability or autonomous-mission acceptance.

## Sensor probe

Command: `python3 scripts/probe-sensors.py` with ROS domain42 and localhost discovery. Wall duration: 20.000 seconds. Clock-based real-time factor: approximately 0.998. All required topics appeared; image/scan stamps advanced without observed regressions; expected optical/lidar/base frames were present.

| Topic | Samples received | Notes |
| --- | ---: | --- |
| `/clock` | 9986 |  |
| `/imu` | 400 |  |
| `/odom` | 400 |  |
| `/joint_states` | 400 |  |
| `/tf` | 800 |  |
| `/scan` | 200 | 360 finite samples; approximately 10.00 received scans/s |
| `/tf_static` | 1 | Transient-local static transforms received |
| `/camera/camera_info` | 100 |  |
| `/camera/image_raw` | 58 | 640x480 RGB8; 921600 bytes/frame; approximately 2.90 received frames/s |

The camera is configured for 5 Hz, but the probe received about 2.9 images/s. This is a measured performance limitation to investigate; the configured rate is not claimed as delivered throughput. Lidar measured approximately 10 Hz. This test alone does not distinguish renderer scheduling from transport/probe image drops.

## Other checks

- Gazebo SDF validation: valid, with warnings for Gazebo-specific `gz_frame_id` extension elements; the resulting ROS frames were verified.
- Simulation desktop captured through VNC and visually inspected: room, three machines and robot visible.
- Foxglove old-protocol handshake through SSH: HTTP101 Switching Protocols; the user subsequently confirmed the new client opened; server logs prove its image/calibration and TF subscriptions. Detailed panel rendering has not been independently inspected.
- Read-only machine bus-voltage topic: 24.0 V in the latched-fault scenario.
- Stop test: no `gz sim` process and no localhost5942/8765 listener remained. Restart test: one server, one GUI, final sensor probe passed.
- Demo user-service memory snapshot: 576434176 bytes (about550 MiB). This includes its processes, not unrelated host workloads. Do not describe overall Thor RAM use using this number.
- Root disk after setup: approximately43 GiB free. Existing host services were left running.
- Eight model unit tests passed locally. Three scripted model-only outcomes passed: healthy/no intervention; latched fault/repaired and verified; jam/escalated.
- Shell syntax and XML parsing checked. CI definition is included; consult GitHub for its current result.

## Failures found and addressed

1. Ogre2 crashed in Mesa/EGL device enumeration. Ogre1 with llvmpipe rendered successfully; no device reboot occurred.
2. A combined Gazebo launch left independently grouped child processes alive after a shell-only stop. The transient user-service cgroup now owns the whole tree. A first probe contaminated by duplicate worlds was rejected; the table above is from the clean replacement test.
3. The generic Gazebo clock topic was missing. The bridge now explicitly subscribes to `/world/maintenance/clock`.
4. APT Foxglove Bridge3.6.0 rejected the old client's subprotocol. Official bridge0.8.5 at41f96cc6053632a472d9a821989952771b1117f2 built successfully as an isolated overlay.
5. Mac Foxglove Studio1.84.0 itself crashes at startup. A separately installed, signature-verified Foxglove3.4.2 opens; the user completed pairing and confirmed it opened. No account, subscription or paid action was created by the agent.

Raw logs and the simulated desktop image remain outside Git in the local setup review directory and Thor project logs. This document includes no credentials or private media.
