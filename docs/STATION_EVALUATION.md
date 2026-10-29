# Actual station observations and offline evaluation

OM-079 adds a private producer inside the trusted station process and an offline consumer independent of mission success reports.

This is one coherent station-metrics delivery, not complete mission acceptance or separate credit for OM-072 through OM-078.

## Record and evaluate

Start a deliberately prepared [private scenario](SCENARIOS.md) with the contained demo launcher.

The station records its actual causal-model state before returning each public telemetry packet, and records each applied intervention after the durable action outcome.

Stop the station normally after the observation window and settled actions, then run from the same source checkout:

```bash
python3 scripts/evaluate-station.py --trial-id OPAQUE_TRIAL_ID
```

The command acquires the existing station owner lock, reads retained evidence and prints one JSON report without starting ROS, publishing controls or amending history.

Exit zero means a valid report, including a valid failure report; it does not mean a successful mission.

An active station, incomplete evidence, changed source/input or invalid history produces exit two and a bounded generic error.

## What the score means

| Metric | Evidence and limit |
| --- | --- |
| Sustained healthy state | Actual sampled voltage at least 27 V, all machines powered, no remaining model fault and each load at most 110 W for at least 10 simulation seconds. A sample gap over one second or any applied intervention restarts the window. |
| Applied interventions | Exact private effects joined to durable admitted action IDs, owner/station generation and control revisions; counts isolate, reset and restore separately. |
| Unnecessary interventions | Applied actions on machines whose configured initial fault was absent. Timed fault injection is not implemented; unrecorded discrete fault changes refuse evaluation. |
| Recorded deviations | Recorded final distance/speed and power/temperature/reset-budget arguments, plus actual discrete operation outcomes. These arguments are not an independent robot pose or interlock witness. |
| Durations | Observed simulation interval and producer monotonic recording interval, not planner latency or end-to-end mission completion time. |
| Station criteria | Sustained healthy state with no unnecessary intervention or recorded precondition/effect deviation. |

Power isolation alone does not establish repair, even when bus voltage recovers.

The consumer does not use the model's `recovered` flag, owner success status or mission claims to manufacture a healthy window.

`overall_acceptance` is always null: collision, independent robot interlocks, commands rejected before admission, mission escalation and false-success claims are not covered by this report.

Zero applied actions does not prove zero attempted or rejected commands.

## Retention and closure

Private records live at `runtime/station-service/station-observations/<opaque-trial-ID>.jsonl`, outside Git, in an owned 0700 directory with 0600 single-link regular files.

Canonical hash-linked records bind the exact private input digest, registry, station/owner runs, declared source files and actual initial state.

Limits are 32 MiB, 20,000 records and 16 KiB per line; capacity is reserved before final actuator admission and bounded end-record headroom is retained.

Every append uses one write, fsync and read-back; a short or ambiguous append is retained and invalidates the producer instead of being retried or truncated.

File/source verification and reservation occur before the final live actuator gate, so storage delay cannot consume stale permission.

Failure after an actual effect preserves its durable applied record and uncertain ownership; it cannot become a terminal success or a complete score.

A clean, settled shutdown writes the final private snapshot and action-journal prefix, syncs and reads it back, then writes an `observations_closed` event to the authoritative journal.

The evaluator requires that separate event immediately after the referenced journal prefix and matching the private chain tip, record count and source identity.

A visible final payload whose fsync failed cannot create that closure event and is refused.

The closure seals the generation against further admissions; later explicitly prepared generations can coexist and historical closed trials remain readable with matching source/registry files.

No physical power-loss persistence test or client receipt of closure is claimed.

## Trust and acceptance boundaries

The producer observes the actual simplified model in the trusted station process; it is not a separate physical sensor or an independently authenticated world observer.

Private permissions, hashes and exclusive ownership protect cooperative integrity and accidental disclosure, not malicious same-user reads, coherent rollback, another checkout or authenticated DDS access.

Declared file hashes are version pins, not loaded-code attestation; changed executable sources deliberately refuse old trial evaluation until their exact version is restored.

The public packet digest records packet construction, not subscriber receipt.

OS separation, independent collision/interlock and mission-terminal joins, blocked/noisy fixtures, held-out configurations, native commissioning and the 25-trial freeze remain open under #10/#12.

## Validation

Local tests use the shipped station adapter, actual causal model, owner, private files, action journal and real capture subprocesses with controlled ROS/robot/camera peers.

They distinguish healthy, recoverable, persistent and ambiguous outcomes; reject fabricated success, missing effects, gaps, mixed identities, changed files and incomplete shutdown; and exercise actual owned-process SIGKILL before and after effect recording.

The final-payload fsync regression failed before the separate authoritative closure was added and passes afterward.

These tests establish local producer/consumer behavior, not native simulator timing, physical collision freedom or complete mission acceptance.
