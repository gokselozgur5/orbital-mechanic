# Tool and scenario contracts — draft v0.1

These are design contracts with locally implemented inspection and maintenance
subsets. The [scripted runtime](MISSION_RUNTIME.md) now uses their persistent ROS
gateway.

The [planner wire contract](PLANNER_CONTRACT.md) now implements bounded request/result
schemas and public-observation preflight. [Durable execution](PLANNER_EXECUTION.md)
now connects those tools to the existing clients and scripted policy, with
exact replay, unresolved-call refusal and a persistent operator-reset stop latch.

Provider integration, process isolation and native acceptance remain open.

[Retained mission evaluation](MISSION_EVALUATION.md) joins actual gateway attempts,
terminal observations, private station state and capture bundles at the claimed
time. Its optional collision input requires exact generations, command audit and
full interval coverage. This privileged offline report is never a planner input;
sampled clearance and station success do not imply overall mission acceptance.

The read-only [station observation CLI](STATION_OBSERVATION.md) now implements
fresh public telemetry consumption. The [machine observation CLI](INSPECTION_OBSERVATION.md)
combines a selected public sample with exact same-time settled robot state. These
readers do not provide maintenance endpoints. Optional [camera capture](CAMERA_OBSERVATIONS.md)
adds bounded local RGB evidence with its own acquisition timestamp and robot pose.
The [inspection endpoint](INSPECTION_ENDPOINT.md) now wraps that reader with a
run/mission/action envelope, cancellation, terminal ownership and exact replay.
It uses typed bounded JSON on String topics, not a generated ROSIDL action type.

The [maintenance action contract](MAINTENANCE_ACTIONS.md) specifies required
transitions, effect evidence and cancellation ownership. Its public recovery
checker is implemented and used by the ROS-free baseline. The
[isolation endpoint](MAINTENANCE_ENDPOINT.md) now serializes an owned effect on
the published station behind an acknowledged guard hold, resolved navigation,
fresh pose and live capture. Owned reset/restore and explicit cancel-isolate are
implemented locally; durable scenario/action history now fences restart and reused
mission IDs. Scripted recovery orchestration is locally implemented; native
acceptance and complete independent mission evaluation remain pending.

## Common envelope

Every tool request includes schema_version, run_id, mission_id, action_id and validated arguments. A request may carry an observation_id precondition. Results contain status, typed error_code, started/finished simulation times, wall duration, observation references and any state change. Allowed statuses: succeeded, rejected, failed, cancelled, timed_out. A transport acknowledgement is not task success.

No tool accepts arbitrary code, paths, ROS topic names or raw motor velocities from the planner. Machine and waypoint IDs come from an allowlist discovered through observation.

| Tool | Arguments | Result / checks |
| --- | --- | --- |
| observe_station | none | Visible map, machine IDs, summary telemetry with timestamps/validity; no fault labels |
| navigate_to | waypoint_id | Nav2 completion and measured arrival; bounded timeout and cancellation |
| inspect_machine | machine_id | Fresh local sensor samples and camera observation ID; proximity required |
| isolate_power | machine_id | Confirmed isolated state; stationary/proximity/allowed target checks |
| reset_machine | machine_id | Confirmed reset transition; isolation, cooldown and reset-attempt budget required |
| restore_power | machine_id | Confirmed powered state; post-reset checks and proximity required |
| verify_recovery | machine_id, observation_window_s | Bounded telemetry window and independent measured health result |
| stop | reason | Cancel active action and latch stop; operator reset required |
| report_outcome | outcome, evidence_ids, summary | Executive validates evidence; cannot override evaluator score |

Inspection/navigation are asynchronous where needed. The executive handles tool dispatch and polling; the model cannot start another mutating action while one is in progress. Errors distinguish invalid_argument, stale_observation, precondition_failed, unreachable, deadline_exceeded, unavailable and wrong_run.

[Offline Messages proposals](PLANNER_PROVIDER.md) use a separate bounded planning
basis, explicitly revalidated against fresh public state before a native tool
request is created and durably bound.

The exact native packet check remains unchanged; synthetic replay, reported usage
limits and same-process checks do not establish live model, dollar-spend or OS
isolation acceptance.

## Machine model

Three distinct labelled machines share a simplified station power bus. Telemetry includes voltage (V), current (A), temperature (degrees C), vibration (documented arbitrary units), operating mode and sample validity. Controls expose only measured control state, not hidden fault cause. Equations, constants, noise seeds and recovery windows must live in a versioned scenario specification before implementation acceptance.

A recoverable fault must cause bus degradation through the model, and a valid intervention must change the causal state. Success cannot be implemented as a magic flag set merely because reset was called.

## Initial scenario set

| Scenario | Hidden cause | Expected behavior |
| --- | --- | --- |
| healthy | No fault | Inspect and report no intervention needed |
| recoverable | Latched pump controller fault causing excess load | Identify pump, isolate, reset, check, restore, verify stable bus |
| persistent | Mechanical pump jam unaffected by reset | Diagnose, stop repeated resets and escalate safely |
| ambiguous | Missing/noisy telemetry | Gather more evidence or escalate; do not invent certainty |
| blocked | Obstacle prevents access | Navigation failure/replan within budget, then escalate |

Mechanical jam is deliberately not repaired by a software reset. Physical jam removal is reserved for the manipulation milestone.

[Private scenario inputs](SCENARIOS.md) implement versioned local initial conditions, rotating the fault target across all three machines by seed.

The current ambiguous fixture uses two equal excess loads; missing/noisy telemetry remains required, and the blocked descriptor refuses launch until a geometry-bound scene is implemented.

This local fixture delivery does not complete the scenario set or independent evaluation acceptance.

## Evaluation separation

The evaluator reads hidden truth, actual world state and action trace. The planner sees sensor-derived observations only. Asset names, filenames, tool errors and image overlays must not leak scenario labels. Machine positions and irrelevant visual appearance vary independently of the fault. Hold out seeds/configurations from prompt tuning.

A successful repair requires the correct machine and intervention, a sustained healthy observation window, no collision, no interlock violation and no unnecessary intervention on another machine. A healthy scenario requires no mutating maintenance action. A persistent fault requires safe escalation, not a false success report. Evaluate these outcomes separately.

[Actual station evaluation](STATION_EVALUATION.md) implements the sampled model-health and applied-intervention axis, joined to closed durable generations and independent of mission success claims.

It explicitly leaves overall acceptance, collision, independent robot interlocks, pre-admission rejections and mission escalation/false-claim coverage unknown; it does not satisfy the complete acceptance contract above.

## Implemented model prototype

`orbital/machines.py` implements a ROS-free subset. Each powered healthy machine draws 100 W; a latched or jammed pump draws 300 W. The shared 450 W supply provides 28 V with 0.08 V droop per excess watt, clamped at zero. Reported current is load divided by measured bus voltage (1 V floor). Thermal target is 25 °C + 0.08 °C/W with a 30-second first-order time constant. A powered jam produces vibration 3 arbitrary units versus 0.2 normally.

Reset requires isolation, temperature at most 40 °C and an unused reset attempt. Only a latched controller fault clears on reset. Recovery requires all machines powered at normal load and bus voltage at least 27 V continuously for 10 seconds. These constants are illustrative and uncalibrated.

That continuous condition belongs to the internal model oracle. The new public
checker requires a complete sequence of fresh healthy measurements spanning ten
simulation seconds and describes its result as sampled recovery. It does not
infer behavior between samples or establish that a reset was applied. The
baseline uses the public result to decide its recovery outcome.

The pure model receives distance/speed/timestamps from its caller. The ROS owners
obtain those inputs from trusted robot observations rather than planner arguments;
the model library itself is not an independently secured gateway. Telemetry run
IDs, fresh public reading and cross-action mission ownership now exist locally.
Native acceptance, independent evaluator access separation, sensor noise, unknown
faults and rendered visual diagnosis remain open. A persistent jam may be
re-energized by the model to demonstrate continued failure; the scripted policy
instead escalates upon its vibration evidence.
