# Tool and scenario contracts — draft v0.1

These are design contracts, not implemented endpoints. The same tools serve scripted and LLM planners.

The read-only [station observation CLI](STATION_OBSERVATION.md) now implements
fresh public telemetry consumption. It does not yet expose this common mission
envelope, combine robot inspection state or provide maintenance endpoints.

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

## Evaluation separation

The evaluator reads hidden truth, actual world state and action trace. The planner sees sensor-derived observations only. Asset names, filenames, tool errors and image overlays must not leak scenario labels. Machine positions and irrelevant visual appearance vary independently of the fault. Hold out seeds/configurations from prompt tuning.

A successful repair requires the correct machine and intervention, a sustained healthy observation window, no collision, no interlock violation and no unnecessary intervention on another machine. A healthy scenario requires no mutating maintenance action. A persistent fault requires safe escalation, not a false success report. Evaluate these outcomes separately.

## Implemented model prototype

`orbital/machines.py` implements a ROS-free subset. Each powered healthy machine draws 100 W; a latched or jammed pump draws 300 W. The shared 450 W supply provides 28 V with 0.08 V droop per excess watt, clamped at zero. Reported current is load divided by measured bus voltage (1 V floor). Thermal target is 25 °C + 0.08 °C/W with a 30-second first-order time constant. A powered jam produces vibration 3 arbitrary units versus 0.2 normally.

Reset requires isolation, temperature at most 40 °C and an unused reset attempt. Only a latched controller fault clears on reset. Recovery requires all machines powered at normal load and bus voltage at least 27 V continuously for 10 seconds. These constants are illustrative and uncalibrated.

The pure model receives distance/speed/timestamps from its caller. In future ROS integration these must come from trusted robot state, never planner arguments. This library is not an independently secured gateway. Telemetry run IDs and fresh public reading now exist; mission/action run ownership, sensor noise, unknown faults and integrated navigation/camera diagnosis remain unimplemented. A persistent jam may be re-energized by the model to demonstrate continued failure; the scripted policy instead escalates upon its vibration evidence.
