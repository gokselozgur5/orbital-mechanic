# Delivery strategy and implementation backlog

Status: agreed phase plan, 2026-10-24. Phase 0 setup/design accepted. Phase 1 is next; its policy prototype does not imply deployed motion. See TRACKING.md for authoritative GitHub links.

## Strategy

Build one observable vertical slice before broadening the robot. Prove machine causality and a scripted mission before judging an LLM. Keep physical simulation, task planning and evaluation separately replaceable. Prioritize reliable evidence over visual polish. Add realism where it changes decisions: sensor delay/noise, obstacles, faulty controls and persistent faults.

## Milestones and exit gates

The agreed phase sequence and workflow are in [PHASES.md](PHASES.md). GitHub milestones and issue links are indexed in [TRACKING.md](TRACKING.md). The phase order is design/setup → safe motion → navigation → maintenance → scripted mission → Claude planner.

Earlier M-numbered sketches were exploratory and are superseded by this sequence. Existing machine-model prototypes remain useful but do not bypass navigation or integrated acceptance.

## Proposed evaluation gate

Freeze a versioned suite of 25 trials: five each of healthy, recoverable, persistent, ambiguous and blocked scenarios, with matched seeds for scripted and LLM planners. Physics results may vary despite a seed; record tolerances and platform identity. Run cheap deterministic model/contract tests in CI and simulator integration on a verified Linux runner. Initial CI must not require a paid LLM.

Hard gates: zero collision events, zero accepted interlock violations, zero false repair claims and zero maintenance interventions in healthy trials. Initial task goal: at least four of five recoverable cases repaired within budget; persistent cases must escalate safely. Ambiguous/blocked scenarios have specified safe outcomes, not forced repair. A failed hard gate blocks the demo designation even if aggregate success is high. These are project targets, not measured results.

Report per-scenario outcomes, navigation failures, interventions, observation age, simulated mission time, wall time, real-time factor, model calls, token use and actual provider cost when available. Never infer cost from token count without the applicable price configuration.

## Initial configurable budgets

Provisional ceilings: 20 planner calls per mission, two attempts per navigation target, one reset per machine, 10 minutes wall time and a 30-second provider-call deadline. Tune after baseline measurement. A numeric paid-provider spending cap must be configured before enabling hosted inference; no cap means hosted mode stays disabled. Tool/camera payload sizes and context length must also be bounded.

## Immediate work

Complete Phase 0 tracking, then take the Phase 1 guard integration issue. Preserve the existing tested policy and undeployed integration drafts. Verify actual stopping behavior and process failure before enabling operator motion. Do not start navigation or Anthropic integration simply because their packages are installed.

## Risks and fallback decisions

| Risk | Response |
| --- | --- |
| Mac/VM graphics cannot sustain sensors | Measure first; use an existing suitable Linux host or lower camera rate; no automatic paid rental |
| LLM latency or unreliability | Keep ROS control independent; scripted baseline remains usable; bounded calls and safe escalation |
| Unrealistic repair shortcut | Causal telemetry model and persistent jam scenario prevent reset-everything success |
| Hidden ground-truth leakage | Separate evaluator process/files; audit gateway and observations |
| Navigation consumes entire scope | Known map and fixed inspection waypoints first; SLAM/exploration later |
| Attractive visuals obscure correctness | Require trace and evaluator evidence before demo polish |

## Remaining decisions

Host, simulator and provider are selected: Thor ARM64, Jazzy/Harmonic and future Anthropic planning. Exact model ID, paid experiment budget, navigation tolerances and scenario constants must be chosen in their owning issues. No delivery dates or performance guarantees are committed.
