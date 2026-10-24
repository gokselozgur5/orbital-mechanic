# Delivery strategy and implementation backlog

Status: planning baseline, 2026-10-24. M0 installation, simulation and transport checks passed; Foxglove opening is user-confirmed with camera/TF subscriptions verified. Passive M1 geometry/sensors and a model-only M2 prototype exist. Control, navigation and robot-to-machine integration remain open.

## Strategy

Build one observable vertical slice before broadening the robot. Prove machine causality and a scripted mission before judging an LLM. Keep physical simulation, task planning and evaluation separately replaceable. Prioritize reliable evidence over visual polish. Add realism where it changes decisions: sensor delay/noise, obstacles, faulty controls and persistent faults.

## Milestones and exit gates

| Stage | Work | Required evidence before proceeding |
| --- | --- | --- |
| M0 Host feasibility | Inventory chosen host; pin Ubuntu/ROS/Gazebo pairing; launch minimal world and camera | Reproducible boot, lidar/RGB received, measured real-time factor and CPU/RAM/GPU use; chosen host recorded |
| M1 Robot foundation | Room, three machine props, differential drive, limits, sensor frames, stop path | Manual drive and stop work; transforms/timestamps valid; bounded command loss stops robot |
| M2 Causal machines | Power model, telemetry, seeded faults, maintenance controls | Healthy/recoverable/persistent model tests pass; wrong or unsafe intervention rejected; reset cannot fix jam |
| M3 Scripted mission | Known map, navigation, inspections, intervention and verification | Full recoverable mission plus healthy/persistent runs without LLM; independent evaluator and trace replay summary |
| M4 LLM planner | Typed gateway, provider adapter, schema validation, budgets | Same scenarios via tools; malformed requests, stale data, timeout, cancellation and duplicate requests tested |
| M5 Demonstration | Held-out scenario suite, operator timeline and reproducible launch | Evaluation report with failures, costs, versions and limitations; clean-machine reproduction |
| M6 Extensions | Arm, contact repair, richer perception, battery and multiple robots | Select one extension only after M5; each gets a separate acceptance contract |

The user selected Thor and authorized installation; package and sensor smoke checks are now recorded in EVIDENCE.md. The next engineering task is guarded motion, with the working Mac viewing path available. No paid cloud machine is needed.

## Proposed evaluation gate

Freeze a versioned suite of 25 trials: five each of healthy, recoverable, persistent, ambiguous and blocked scenarios, with matched seeds for scripted and LLM planners. Physics results may vary despite a seed; record tolerances and platform identity. Run cheap deterministic model/contract tests in CI and simulator integration on a verified Linux runner. Initial CI must not require a paid LLM.

Hard gates: zero collision events, zero accepted interlock violations, zero false repair claims and zero maintenance interventions in healthy trials. Initial task goal: at least four of five recoverable cases repaired within budget; persistent cases must escalate safely. Ambiguous/blocked scenarios have specified safe outcomes, not forced repair. A failed hard gate blocks the demo designation even if aggregate success is high. These are project targets, not measured results.

Report per-scenario outcomes, navigation failures, interventions, observation age, simulated mission time, wall time, real-time factor, model calls, token use and actual provider cost when available. Never infer cost from token count without the applicable price configuration.

## Initial configurable budgets

Provisional ceilings: 20 planner calls per mission, two attempts per navigation target, one reset per machine, 10 minutes wall time and a 30-second provider-call deadline. Tune after baseline measurement. A numeric paid-provider spending cap must be configured before enabling hosted inference; no cap means hosted mode stays disabled. Tool/camera payload sizes and context length must also be bounded.

## First implementation tickets

1. M0: select host and capture rendering/sensor smoke-test evidence.
2. Define repository ROS workspace layout and reproducible version manifest.
3. Add robot description with explicit mass, inertia, wheel limits and sensor transforms.
4. Add one-room world and guarded teleoperation launch.
5. Specify and unit-test machine power dynamics independently of Gazebo.
6. Implement common action/observation schemas and conformance tests.
7. Integrate machine controls, navigation and scripted mission.
8. Build evaluator before adding the LLM adapter.

## Risks and fallback decisions

| Risk | Response |
| --- | --- |
| Mac/VM graphics cannot sustain sensors | Measure first; use an existing suitable Linux host or lower camera rate; no automatic paid rental |
| LLM latency or unreliability | Keep ROS control independent; scripted baseline remains usable; bounded calls and safe escalation |
| Unrealistic repair shortcut | Causal telemetry model and persistent jam scenario prevent reset-everything success |
| Hidden ground-truth leakage | Separate evaluator process/files; audit gateway and observations |
| Navigation consumes entire scope | Known map and fixed inspection waypoints first; SLAM/exploration later |
| Attractive visuals obscure correctness | Require trace and evaluator evidence before demo polish |

## Decisions still requiring context

Which existing machine will run simulation? Should the first LLM experiment be local or use an explicitly budgeted hosted provider? Neither blocks architecture documentation; both must be resolved before their dependent implementation stages. No delivery dates or performance promises are committed yet.
