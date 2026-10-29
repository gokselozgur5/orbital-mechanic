# Retained maintenance sequence evidence

OM-060 adds a read-only recorder for one owner, mission and machine, ending at a named terminal action.

It extends independent observation beyond the existing single-isolation recorder to reset, restore and cancel-isolate outcomes.

The recorder subscribes to maintenance command/state/result and station telemetry topics; it creates no control publisher, service client, retry or cancellation.

It never reads the owner's journal, hidden fault, reset counter or camera artifact to establish the reported model postconditions.

## Capture and replay

Use the already commissioned, contained Orbital services and the configured Thor checkout, tmux, localhost domain 42 and orbital-maintenance partition.

The observer starts no world or service, and its readiness marker grants no action permission.

Choose a fresh mission ID and action IDs before starting the scenario, and obtain the current maintenance owner run with `maintain-machine.py status`.

Start the observer before the first isolation, naming the last expected action:

```bash
python3 scripts/record-maintenance.py --name pump-maintenance-001 \
  --machine-id pump --run-id "$maintenance_run" \
  --mission-id "$maintenance_mission" --action-id "$restore_action" \
  --duration 120 --warmup 10 --require-recovery
```

The variables above must be the actual chosen identities, not an old recording's values.

Wait for `runtime/evaluation/pump-maintenance-001/ready.json`, then perform the already authorized isolate/reset/restore commands in another pane with the same owner/mission and distinct action IDs.

Each reset/restore command still requires its exact prior action ID, current station state and the endpoint's independent live interlocks.

For cancellation, name the explicit cancel-isolate workflow ID as the recorder's final `--action-id` and omit `--require-recovery` when the intended final state is isolated.

The observer ends only when every observed intervention command has its separately observed terminal result, the named final action has completed, and a fresh current pair plus idle owner state agrees.

An unmatched or rejected intervention command, unknown effect, missing result or source loss leaves an incomplete recording; this recorder does not subscribe to admission replies or convert a rejection into a terminal action.

Observe rejection/admission diagnostics through the existing client separately.

Replay needs no ROS, Gazebo, model or active owner:

```bash
python3 scripts/record-maintenance.py --verify runtime/evaluation/pump-maintenance-001
```

Replay requires the same configured registry/world bytes and verifies all retained record sequence/length/checksum bounds before accepting the final report.

Recordings remain under ignored `runtime/evaluation`; never reuse an output directory or old readiness marker.

## Independent checks

| Evidence | Required binding |
| --- | --- |
| Root request | Actual observed command bytes, exact owner/mission/action, machine, station generation and semantic request digest. A new command ID does not create another action. |
| Prior result | The exact separately observed isolation/reset result, not merely a copy nested inside a later success result. |
| Applied effect | The request's next revision, complete observed control chain and exact public applied identity; no unaccounted revision or repeated reset/restore attempt. |
| Reset eligibility | Both retained powered references resolve to actual source packets and measurements; observed high vibration remains a refusal. This covers the recorded history, not unobserved earlier server history. |
| Postcondition | Both consecutive packets at the reported verification reference carry the exact control and measured state; both were fresh at the independent paired observation. |
| All intermediate samples | Observed power/current/vibration must agree with the recorded control, including samples outside the selected verification pair. |
| Cancellation | Observed root/target commands and separately observed target/child results match the workflow; its final references resolve to actual retained packets. |
| Final state | A fresh pair after the named decision and a fresh idle owner state agree with the latest accounted control. |

An internal cancellation child has no external command publication.

Its request is explicitly labeled `workflow_child_report`; the child result and effect packets must still be separately observed.

The recorder does not mislabel that reported internal intent as an independently witnessed external admission.

Cross-topic callback order does not establish source order; bounded later packets may satisfy a result reference that arrived first.

Historical decisions may arrive late after storage I/O without renewing their source samples.

The verifier checks retained pairs at their observation time and separately requires fresh samples at recording completion; it does not independently establish the server's historical wall-clock admission deadline.

## Reading the outcome

`complete=true` and exit 0 mean a consistent retained observation, not necessarily a successful intervention or mission.

Each action preserves its server status, applied/not-applied disposition, source references and reported hold release.

Cancellation before a known effect can establish only sampled unchanged power/control, not absence of an unobserved transient effect.

`observed_reset_restore_chain` means the exact applied reset/restore ancestry and source samples were observed, even if a later cancellation re-isolated the machine.

The final measured power and server/workflow status remain separate fields.

`post_restore_recovery_verified` additionally reuses the public consecutive ten-second recovery checker after the last observed restoration effect: all machines powered, bus voltage at least 27 V and observed power at most 110 W per machine.

`--require-recovery` keeps recording until this postcondition is verified or the bounded observation fails.

A new unhealthy measurement discards that window; an energized machine is not automatically recovered.

A healthy window following a restoration before the recorded mission cannot count as that mission's observed recovery.

The candidate regression uses two actual durable-owner missions and a cancelled, unapplied isolation in the second mission; it fails before and passes after requiring a restoration revision inside the captured chain.

A workflow reporting `unverified` stays explicitly unverified even when the independent observer subsequently collects a fresh current state.

## Limits and acceptance

The existing trace limits remain 64 MiB, 100,000 records and 65,536 bytes per line.

The in-memory verifier additionally retains at most 4096 telemetry packets and 64 action identities without eviction; exhausting a bound refuses completion.

Warmup is 0.1–30 wall seconds, followed by a 0.1–120 second observation deadline; paused simulation cannot extend it.

Source/service/GID checks are sampled consistency evidence, not publisher authentication or protection against an entirely fabricated trace and manifest.

Hold release remains a server report; camera/pose admission, actual navigation containment, electrical isolation, continuous physical health and repair causation are separate acceptance gates.

Local tests exercise the actual durable station/model with controlled sensor inputs, the real subscriber/CLI with forbidden control interfaces, and composed station/guard/navigation with three or four real camera children.

Native DDS/timing, three-machine maintenance traces, full mission evaluation and native recovery acceptance remain pending.

OM-058's existing model/reset persistent-fault refusals and OM-059's existing observation-loss interlocks are reused without duplicate delivery credit.
