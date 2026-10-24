# Simulated motion failure acceptance

Date: 2026-10-24. Host: Thor ARM64, ROS 2 Jazzy/Gazebo Harmonic. All operational checks ran in `tmux orbital-work`, with the simulator owned by `orbital-demo.service`. This is simulated odometry evidence, not hardware braking certification or collision avoidance.

## Reproduction and limits

Start one clean demo with `scripts/launch-thor.sh`, source ROS Jazzy and set `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `GZ_PARTITION=orbital-maintenance`. Run system Python on `scripts/check-motion-failures.py`, then `scripts/check-motion.py`. The fault test intentionally kills only the guard found in the demo cgroup and restarts the world. Its JSON includes SHA-256 identities for the exact six tested source files.

Acceptance limits: actual odometry reaches below 0.02 m/s and 0.05 rad/s within 1.2 wall seconds and 0.12 m displacement after the bounded drive loop stops. Cgroup teardown must finish within 5 wall seconds. Timing starts after the final publishing loop returns, so these are observed harness intervals, not instrumented actuator or last-packet latency. Position samples arrive at approximately 20 Hz; report precision does not imply measurement accuracy beyond that sampling.

## Failures found before acceptance

- Initial guard-death test exceeded its original 12-second teardown limit: detached Gazebo Ruby process survived until systemd's 10-second stop timeout, after a 2-second local grace. Bound local grace to 0.2 seconds and systemd grace to 2 seconds. All demo descendants remain under the cgroup.
- Faster stop/restart exposed a transient-unit collection race. The launcher now waits up to 5 seconds for the old unit to disappear and rejects an already active demo. It never stops an active world merely to start a new one.
- An early test stopped at an assertion without enough diagnostic detail; its exact failed condition was not captured. The harness now waits for actual feedback readiness and records the traceback on failure. Repeated final checks are required before acceptance; this is not claimed as a diagnosed runtime bug.

## Measured fault run

Thor job `motion-failures-4` passed forward/reverse timeout stopping, pause/resume, world-clock reset, guard SIGKILL and clean latched restart.

| Check | Measured result |
| --- | --- |
| Forward timeout at 0.15 m/s | 0.0586 m, 0.4068 s to observed stop |
| Reverse timeout at −0.15 m/s | 0.0515 m, 0.3791 s to observed stop |
| Pause/resume | 0.0060 m displacement after resume; remained latched/stopped |
| World reset | Simulation time 13.45 → 0.95 s; `simulation_reset`, no restart |
| Guard SIGKILL while moving | All 14 sampled cgroup processes gone in 0.3255 s |
| Restart | Fresh world, stationary odometry and latched stop |

Raw evidence remains outside Git under the local reviews directory and Thor runtime jobs. Final repeat job `motion-final-acceptance` exited 0: forward stop 0.0562 m / 0.3805 s; reverse 0.0551 m / 0.3677 s; pause/resume displacement 0.0034 m; reset 50.2 → 1.0 s; 14 processes terminated in 0.3227 s; clean restart latched. All six source hashes were compared with the local commit candidates and matched exactly.

The same job rejected a second launch, passed all eight basic motion cases, and passed the 20-second sensor probe with no missing topics or timestamp regressions. One server and one GUI remained. RGB delivered only 14 frames (~0.7 Hz) in this run, below earlier 2–3 Hz observations and the configured 5 Hz; the functional sensor pass is not a throughput guarantee. Lidar delivered 200 messages and odometry/IMU/joints 399 each. Operator-view performance remains follow-up work.

## Transient-unit collection race — 2026-10-24

The packaged frame regression run passed its first frame/motion checks but failed when restarting: systemd collected the failed transient unit between reading `ActiveState=failed` and `reset-failed`. The launcher previously treated the resulting missing-unit response as fatal. It now tolerates that failure only after a new read confirms `LoadState=not-found`; an existing unit with a real reset error still fails closed, and an active world is still refused.

A shell-level regression reproduced the old failure and passed after the change; a second case verifies active worlds are not replaced. Thor job `frame-runtime-acceptance-2` then completed the full runner with exit 0, including repeated restarts, 36 unit tests, frame geometry, motion/operator/fault checks and fresh sensors. Its guard-kill case removed 14 processes in 1.238 s, below the 5 s test limit; this run does not preserve the earlier 0.223 s measurement as a universal guarantee.
