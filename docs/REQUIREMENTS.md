# Requirements and hardware/software decisions

Baseline: 2026-10-24. Requirements are targets unless linked to measured evidence.

## Selected deployment

- Simulation host: existing Jetson AGX Thor, ARM64, Ubuntu 24.04.3, L4T R38.2.2, 122 GiB RAM. Observed root free space: 47 GiB before install.
- Thor project root: `/home/jensen/.goske/codex/orbital-mechanic`. Scripts, logs, generated artifacts and runtime configuration belong here. APT packages use normal system paths.
- Operator host: existing Mac, with Foxglove Studio already installed.
- Robotics: ROS 2 Jazzy, Gazebo Harmonic, ros_gz, Nav2 and gz_ros2_control from ROS Ubuntu packages.
- Visualization: loopback-only TigerVNC display, verified SSH tunnel and Mac-local noVNC browser viewer (native Screen Sharing failed); Foxglove Bridge on loopback through a separate forwarded port.
- Runtime LLM: Anthropic provider selected based on user-reported USD 200 credit. Credit is unverified; no credential is loaded and no API calls are authorized by a dollar balance alone. Initial experiment cap proposal: USD 5, to be configured before paid tests.
- Claude Code: development assistant. Production task planner uses an explicit provider adapter and a restricted tool schema, not unrestricted coding-agent shell access.

## Fidelity decisions

Initial sensor rendering uses software OpenGL. This is a diagnostic baseline because a previous Thor GPU inference path caused reboots; it is not evidence that Gazebo GPU rendering is defective. No PyTorch/CUDA inference is required. Measure camera/lidar throughput before considering hardware acceleration.

The first sensor demo uses Gazebo's built-in differential-drive plugin. ROS motor-command bridging is deliberately absent until a watchdog/stop controller exists. This is a passive visualization demo, not completed M1 navigation. Transition to ros2_control is a separate integration step requiring joint-limit and timeout tests.

Machine electrical/thermal behavior is a documented simplified model. A camera seeing a colored box is not visual fault diagnosis. Planner-facing machine identity, inspection observations and a causal repair interface must be integrated explicitly.

## Functional requirements

| ID | Requirement | Acceptance |
| --- | --- | --- |
| F01 | Launch a repeatable local scene with robot and three machines | Valid SDF and a live Gazebo session |
| F02 | Expose camera, lidar, IMU, odometry and frames through ROS | Timestamped nonempty samples, valid dimensions/ranges, matching frames |
| F03 | View scene and telemetry on Mac | VNC desktop accessible, Foxglove WebSocket handshake and channels confirmed |
| F04 | Model a power anomaly caused by an individual machine | Model tests prove intervention changes load/voltage and a jam persists |
| F05 | Execute maintenance only with trusted proximity, stop and freshness checks | Negative-path tests; robot state comes from trusted adapter, never model-supplied numeric claims |
| F06 | Navigate with bounded commands and safe stop | Command-loss/cancel/process-failure integration tests before autonomous motion |
| F07 | Diagnose through restricted tools | Same interface works for scripted and LLM planners; no hidden state access |
| F08 | Verify recovery independently | All machines operating and stable voltage over a measured window; isolated load is not counted as fixed |
| F09 | Produce reproducible evidence | Software manifest, scenario identity, trace and evaluator outcome per run |

## Operational constraints

- No public VNC or Foxglove listener; both bind to loopback. SSH verifies the device identity.
- VNC uses no extra password only because its listener is loopback and access is through authenticated SSH. This assumes trusted accounts on Thor; add VNC authentication or per-user isolation before multi-user deployment.
- No public cloud relay, paid VM, model download or automatic cleanup of existing disk data.
- Stop before free disk falls below 10 GiB; bound recordings and keep logs under the project root. Retention automation remains to be implemented.
- Gazebo transport partition `orbital-maintenance`, ROS domain 42 with localhost discovery isolates accidental network discovery; this is not a security boundary against local ROS processes.
- Software installs must not perform a blanket OS/L4T upgrade. Normal package dependencies may update individual libraries; capture the package manifest.
- GUI success, sensor delivery, robot motion, autonomy and fault repair are separate acceptance claims.

## Deferred decisions

Exact Anthropic model ID and current pricing must be verified when the credential is connected. Do not guess a model name or promise a number of missions from the reported credit. Low-latency local inference, exact AI-chip emulation, physical robot hardware, arms, SLAM, multi-robot orchestration and public web access are outside the first demonstration.
