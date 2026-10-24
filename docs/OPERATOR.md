# Bounded operator CLI

Run on Thor inside `tmux orbital-work`, from the project root:

```bash
source scripts/ros-env.sh
/usr/bin/python3 scripts/orbital-operator.py status
/usr/bin/python3 scripts/orbital-operator.py arm
/usr/bin/python3 scripts/orbital-operator.py jog forward --duration 0.5
/usr/bin/python3 scripts/orbital-operator.py stop
```

Directions are `forward`, `reverse`, `left`, `right`. Linear speed is fixed at 0.12 m/s and turning at 0.3 rad/s. Duration defaults to 0.5 seconds and must be finite, positive and at most 2 seconds. Every jog ends with a stop latch; arm explicitly before each next movement. Arm alone does not move. The client waits for simulation clock, fresh guard state, odometry and the guard command subscriber; its bounded duration starts only after discovery. It requires an observed command-active acknowledgment before reporting success; it publishes only stamped `/motion/request`, never raw drive output.

`status` reports stop reason, commanded and measured velocity, and camera/lidar message counts observed by that short-lived client. These counts are not frame-rate guarantees. Use the existing Gazebo/Foxglove views for scene and sensors; the composed operator view is still pending under issue #5. This CLI does not complete that issue.

Normal completion requests stop. Interruption or loss of the client is also covered by the independent guard's command timeout; the client does not own or disable that watchdog. This is simulation-only, with no collision avoidance yet.

## Measured exercise

Start a fresh world at the clear origin, then run `/usr/bin/python3 scripts/check-operator.py`. Thor tmux job `operator-acceptance-4` exited 0 on 2026-10-24:

- A jog while latched was rejected.
- 0.7-second forward/reverse jogs moved 0.0836 / 0.0843 m and ended stationary/latched.
- Left/right turns passed measured yaw magnitude and direction checks, then stopped.
- Killing the real operator subprocess during forward motion produced `command_timeout`, observed stop after 0.4394 seconds and 0.0497 m.

Earlier test attempts failed before adequate diagnosis was captured: one did not observe motion before disconnect; another failed the disconnect observation without detailed state being captured; a subsequent attempt correctly refused a rover outside the clear-origin precondition. The final harness keeps ROS callbacks active during the stop measurement and the final exercise ran in a fresh world. Raw evidence remains outside Git. The later cancellation results are recorded below; the combined viewer remains a separate acceptance gate.

## Discovery and cancellation follow-up

A final integration trial returned success for a reverse jog with zero displacement. The client previously started its short publishing deadline before request-subscriber discovery completed. It now waits for discovery and verifies guard acknowledgment. Final four-direction acceptance measured0.0841m forward/reverse and checked actual turn direction. All four cancellation signals stopped via the independent command watchdog: SIGKILL0.4040s/0.0449m, SIGINT0.4117s/0.0478m, SIGTERM0.3770s/0.0401m, SIGHUP0.3729s/0.0449m. The client filename is `orbital-operator.py` to avoid shadowing Python's standard `operator` module; the old filename caused a reproduced CI import failure.

## Combined Foxglove view

Import `config/foxglove-operator.json` from the desktop Layout menu → Import from file. It contains an Image panel for `/camera/image_raw`, a 3D lidar view in `odom` following `base_link`, a raw `/motion/state` view, and commanded/measured linear/angular velocity plots. It contains no publish or direct-drive controls; use the guarded CLI for motion.

The file uses native desktop `configById` and mosaic `layout` data, not the SDK's notebook `content` tree. The first notebook-format attempt was rejected as incompatible; the generator now converts to native version1 and maps public `ThreeDee` to internal `3D`. Native importer shape/version and all four panel references were checked against the installed desktop app; user-rendered acceptance is still pending.

To regenerate, use an isolated development environment with `foxglove-sdk==0.30.2`, then run `python scripts/generate-foxglove-layout.py`. The simulator does not require that SDK. Foxglove UI manipulation is currently handled by the user; do not automate its clicks or keyboard input without renewed authorization.
