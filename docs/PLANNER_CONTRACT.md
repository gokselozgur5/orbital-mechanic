# Planner wire contract

OM-081 implements a provider-neutral, versioned request/result vocabulary and a public telemetry projection.

The wire library and catalog remain independent of ROS; OM-082 adds a separate
[durable executive](PLANNER_EXECUTION.md) and same-contract scripted baseline.

## Inspect the contract

```bash
python3 scripts/planner-tools.py
```

The command prints nine tool schemas without importing ROS, opening a network connection or changing runtime state.

The schemas use the [JSON Schema 2020-12 vocabulary](https://json-schema.org/draft/2020-12/json-schema-validation).

The runtime additionally enforces strict JSON integer tokens, an 8,192-byte UTF-8 bound, duplicate-key refusal, finite numbers, context identity and cross-field result semantics.

Schema validation alone does not cover these additional checks or live preconditions.

| Tool | Arguments | Required public precondition |
| --- | --- | --- |
| `observe_station` | Empty object | None; execution still needs a fresh view |
| `navigate_to` | Configured `waypoint_id`, `timeout_s` 0.1–120 | Current issued observation |
| `inspect_machine` | Configured `machine_id`, `timeout_s` 0.1–10 | Current issued observation |
| `isolate_power` | Configured `machine_id`, `timeout_s` 0.1–30 | Current issued observation |
| `reset_machine` | Configured `machine_id`, `timeout_s` 0.1–30 | Current issued observation |
| `restore_power` | Configured `machine_id`, `timeout_s` 0.1–30 | Current issued observation |
| `verify_recovery` | Configured `machine_id`, `observation_window_s` 10–30 | Current issued observation |
| `stop` | Enumerated reason | None; invalid telemetry must not prevent parsing a stop |
| `report_outcome` | Enumerated outcome, up to 16 unique evidence IDs, 1–1,024-character summary | None; this is an unverified claim |

The current reviewed map has `pump`, `cooler` and `power_unit` as both machine and named inspection-waypoint IDs.

Adding a machine or waypoint requires a reviewed source/schema change; callers cannot introduce one through an argument.

No tool accepts caller-provided paths, ROS topics, shell commands, pose, velocity, fault selectors, control revisions or owner identity.

The executor must derive owner run IDs, action envelopes and interlock inputs from its trusted live clients.

## Request and result binding

Every request has exactly `schema_version`, `kind`, `run_id`, `mission_id`, `action_id`, `tool`, `observation_id` and `arguments`.

`run_id` identifies the executive session, not the station or maintenance owner; the host supplies expected run/mission IDs to `request()`.

An observation precondition is required for the six action/verification tools and explicitly null for observe, stop and report.

Canonical request hashes bind every field and normalize numeric timeout/window spellings.

Every terminal result binds the original request hash and identities, simulation start/end stamps, wall duration, observation references and effect state.

Statuses are `succeeded`, `rejected`, `failed`, `cancelled` and `timed_out`, with a fixed compatible error-code set.

An acknowledgement is not a terminal result.

Maintenance effects are `none`, `reported` or `unknown`; a failed/cancelled/timed-out operation may retain a reported or unknown effect.

A reported effect requires matching target/operation, control revision, powered state and an observation reference.

Successful maintenance requires a reported effect; rejection cannot carry effect credit.

These checks enforce wire consistency only: they do not authenticate the source, validate referenced artifacts or independently establish that an effect happened.

A successful `report_outcome` result can only mean the executive recorded the claim, never that the evaluator accepted it.

Summary text is untrusted data, even when it resembles instructions; this module never interprets or executes it.

## Public observation preflight

`PlannerObservation` consumes actual callback text through the existing `TelemetryEvidence` decoder with host-supplied receipt/simulation clocks.

A view requires two advancing schema-v2 samples, the configured registry, unchanged sources and freshness under one second on both clocks.

It projects only configured control/inspection geometry, public measurements, public applied-control identity and freshness.

Private initial conditions, profile/seed, fault labels, source paths, camera files and evaluator reports are never read or exported by this component.

An opaque observation ID is stable for one exact public packet and changes for every new packet, including control changes.

Only the latest issued packet is retained; reading a view does not refresh its receipt time.

`validate()` rechecks source files, then clocks, and rejects a missing, invented, old or expired reference.

A native request referring to an old packet must obtain a new observation; the executive also rechecks its original packet after binding I/O, immediately before publication.

The separate [offline Messages proposal boundary](PLANNER_PROVIDER.md) retains an older planning basis and explicitly records its revalidation against a fresh frame before creating a new native request.

It does not relax this native packet rule or silently update an already admitted request.

Stop and outcome-claim parsing remain available after observation failure; parsing a stop does not prove that cancellation or physical stopping occurred.

## Validation and remaining integration

Tests exercise all tools and terminal envelopes, malformed/duplicate/nonfinite/oversized inputs, cross-run requests, error privacy, effect uncertainty and stable catalog output across process hash seeds.

Public-observation tests use the actual causal station model and an actual isolation transition, plus changed sources, expired clocks, restart/replay, legacy telemetry and private-field injection.

These are local library/model tests, not a ROS/native robot trial or a provider call.

The separate executive now supplies command dispatch, durable call replay, pending-call budgets, bounded public evidence and owned inspection results.

Its catalog additionally describes the nonterminal, nondurable pending acknowledgement for a stop requested during another call.

The existing scripted runtime can opt into the same contract; local composed tests include actual station and camera artifacts.

Synthetic recorded Messages transport is implemented separately; live provider/network integration, model image input, process/OS isolation and native commissioning remain open.

Issue #13 stays open, and earlier phase acceptance/#12 remain prerequisites for complete planner acceptance.
