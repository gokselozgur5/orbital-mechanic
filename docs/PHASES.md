# Agreed development phases

Approved direction: 2026-10-24. This phase sequence supersedes the earlier M0–M6 exploratory ordering. GitHub milestones track phases; issues define independently reviewable deliveries. Closing a phase requires its exit evidence, not just merged code.

## Product baseline

One two-wheel differential-drive robot in a single artificial-gravity room with three machines. Sensors: RGB camera, 2D lidar, IMU and wheel joint/odometry feedback. Inspection and maintenance initially use proximity-gated simulated controls. No robot arm, physical part replacement, SLAM or multi-robot coordination in the first release.

Thor runs ROS 2 Jazzy and Gazebo Harmonic; the Mac views Gazebo through local noVNC/SSH and robot data through Foxglove. Anthropic is the future task-planning provider, not the motor controller. A scripted mission and independent evaluator precede paid LLM experiments.

| Phase | Deliverable | Exit evidence |
| --- | --- | --- |
| 0 — Design and setup | Requirements, deployment choices, phase plan and visible sensor baseline | User-approved scope; installed stack; sensor probe; clean stop/restart; user-confirmed Gazebo and Foxglove viewing |
| 1 — Safe motion | Bounded forward/reverse/turn control, deadman timer, latched stop and operator controls | Measured displacement and actual stop; stale/invalid commands rejected; guard/process failure and pause/reset tests; no unintended restart |
| 2 — Navigation | Known map, localization, named inspection poses and obstacle-aware navigation | Reach all three machines within tolerances; blocked target/timeout/cancel stop safely; no observed collision |
| 3 — Maintenance capabilities | Trusted inspection, power isolation/reset/restore and causal telemetry | Proximity/stationary/freshness interlocks; healthy/recoverable/persistent/ambiguous model cases; jam cannot be reset away |
| 4 — Scripted mission | Full non-LLM diagnosis/intervention/verification and independent evaluation | Repeatable scenario suite; healthy cases untouched; recovery measured; persistent faults escalated; hidden truth not exposed |
| 5 — Claude task planner | Versioned tool gateway, Anthropic adapter and spending limits | Matched-seed comparison to scripted baseline; malformed/timeout/replay tests; measured costs; hard safety gates pass |

## How work moves

1. Take an issue from the earliest open phase whose prerequisites are satisfied.
2. Specify the interface and discriminating acceptance test before deployment.
3. Commit one coherent change with its related tests/docs. Do not pad commit counts or mix unrelated deliveries.
4. Run appropriate local checks, then integration checks on Thor from tmux `orbital-work` when required.
5. Link commit and evidence to the issue; close it only when all criteria are verified.
6. Review the entire phase gate before closing the milestone. Future phase prototypes do not imply their phase is complete.

The motion policy and ROS integration are implemented; STATUS.md records the acceptance evidence and remaining Phase 1 gates. Existing machine-model code contributes to Phase 3 without completing it.

## Scope control

An issue may produce several atomic commits, each independently understandable. A phase is not a single giant commit. If the source changes after a test, verify the affected behavior again. Keep secrets, raw media, recordings, dependencies and generated outputs outside Git.

No exact delivery dates are promised. A new feature such as an arm needs a separate future phase proposal with its own resources and acceptance criteria. Paid Anthropic calls require a configured budget and credential; reported account credit is not proof of available balance or permission for unlimited spend.
