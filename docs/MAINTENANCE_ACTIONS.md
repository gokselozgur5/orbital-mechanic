# Maintenance actions and observable postconditions

OM-051 defines the contract for issue #9 and implements its public station
recovery checker in `orbital/maintenance_observation.py`.

The ROS-free baseline uses that checker instead of the model's hidden
`Station.recovered` oracle. OM-052 now implements an
[owned isolation endpoint](MAINTENANCE_ENDPOINT.md), exact guard inhibition and
versioned applied-effect telemetry. Reset/restore and native maintenance
acceptance remain unimplemented. The inspection
endpoint supplies observations; its cached success grants no maintenance rights.

## Ownership and admission

The existing `MachineTelemetry` owns one causal `Station`. The isolation actuator
mutates that exact instance, serialized with sampling and clock reset.
Starting a second model in the inspection process would change a private copy
and cannot satisfy an operation on the published station.

An operation binds the maintenance-owner run, mission/action IDs, station run,
machine ID, normalized request digest, expected control revision and deadline.
The owner generates its timing and evidence bindings; the caller supplies no
robot pose, velocity, source identity, observed power state or cooldown claim.
IDs use the inspection protocol's bounded format. Exact replay returns the
previous record; conflicting reuse, a different run or exhausted history refuses
admission. Do not evict replay records to make space for another intervention.

Before each effect, the executor must independently establish:

- Current configured registry/map and publisher identities, advancing simulation
  time and fresh public telemetry from the same station run.
- The measured control pose: distance at most 0.75 m, inspection position error
  at most 0.10 m and heading error at most 0.15 rad.
- At least 0.5 s settled odometry in receipt and simulation time, with the existing
  0.01 m/s and 0.02 rad/s bounds and 200 ms pose/sample freshness gates.
- Exclusive maintenance ownership plus an acknowledged drive latch and resolved
  navigation ownership. New navigation dispatch must remain blocked while the
  maintenance hold exists. A sampled stopped pose alone is not this hold.
- A new live inspection bound to this target and action, including the applicable
  operation preconditions below. Revalidate after I/O and immediately before the
  serialized mutation; neither an old file nor a successful ACK renews freshness.

OM-052 implements these gates for isolation and tests their local composition.
The same gates remain requirements for future reset/restore operations.
The pure `Station.intervene` caller-supplied geometry checks are insufficient for
ROS admission. The hold must compose with the existing motion supervisor and
drive guard; adding an unrelated boolean in the maintenance process is not enough.

## Allowed transitions

The diagnostic policy selects a target from public evidence. The actuator checks
the evidence and interlocks independently; a request cannot grant itself an
exception. The current illustrative model constants are not hardware thresholds.

| Operation | Required prior state | Permitted effect | Evidence needed for operation completion |
| --- | --- | --- | --- |
| Isolate | Freshly observed powered target; owned live control pose and hold | Set that target's power off once | Matching applied-operation record, followed by new same-run telemetry with target `powered=false`, zero current and zero vibration |
| Reset | Verified isolation; temperature at most 40 °C; an unused reset attempt; eligible pre-isolation evidence | One reset pulse while power stays off | Matching reset-application record plus new telemetry confirming continued isolation and cooldown; this does not confirm the fault cleared |
| Restore | A verified reset in this owned sequence; no unresolved action; cooled isolated target; fresh post-reset inspection; no retained persistent-fault refusal | Set that target's power on once | Matching applied-operation record plus new same-run telemetry with `powered=true`; recovery is verified separately |

A new isolate request for an already isolated target, a live reset, an overheated
reset/restore, restore without its verified reset or a second reset attempt is
rejected before mutation. An exact duplicate of a completed action returns its
historical result, even if the current state has changed; that history is never
an assertion of current power or permission for another operation.

Reset eligibility requires retained *powered* observations consistent with the
recoverable signature: excess load and no high-vibration indication. Unknown,
missing or contradictory evidence does not establish eligibility. The existing
baseline's vibration threshold is 1 arbitrary unit; a higher reading prevents
generic reset/restore and requires escalation. Zero vibration after isolation
must not erase the prior powered high-vibration evidence. The one-reset budget
belongs to the station/mission owner, not caller arguments or process-local
planner memory. A new server run cannot silently renew an old mission's attempt.

The causal model can still demonstrate a reset followed by re-energization of a
jam for evaluator tests. That model capability does not authorize this sequence
through the future maintenance gateway. A healthy mission requests no mutation.

## ACK, effect record and observation are distinct

An ACK confirms admission only. The model's current `status=succeeded` receipt
confirms a synchronous library call, without a public effect identity. It is not
a complete maintenance result or evidence of repaired machinery.

Schema-1 isolated measurements cannot confirm a reset. A latched machine without
a reset, a successfully reset latch and a reset mechanical jam produce identical
public isolated measurements at matched temperatures/times. The executable
observability control in `test_maintenance_observation.py` demonstrates this on
the actual model. Waiting for another unchanged isolated sample cannot solve it.

Before exposing reset as an endpoint, extend the public actuator state through a
versioned telemetry contract. The selected design is a monotonically increasing
control revision with a bounded record of the exact applied mission/action,
machine, operation, request digest and simulation stamp. The station owner
records it only after that serialized effect, and subsequent telemetry includes
the committed revision. A reset application is public control state; hidden
fault labels and the fault-clearing decision remain evaluator-only.

The endpoint must match its exact action record and require a strictly later
telemetry sequence/stamp from the pinned station run and revision. An unrelated
revision, an admission reply or a zero-current measurement alone cannot confirm
its reset. A complete trace needs retained source samples and the action record,
not merely a hash or caller-supplied identifier. The new wire schema, actuator
record/replay storage are implemented for isolation in OM-052. Telemetry schema 2
adds `control.revision` and the exact `control.last_applied` record; strict
readers also accept legacy schema 1 for read-only observation. Reset/publication
semantics remain pending. The endpoint requires schema 2 for its live inspection.

The simplified model's power switch and current represent simulated isolation.
They do not prove physical electrical isolation or absence of residual energy on
hardware. The shared bus can remain at 28 V while a single target is isolated.

## Public recovery checker

`RecoveryEvidence` consumes strict schema-1 or schema-2 station telemetry via
`TelemetryEvidence`. It pins the complete three-machine catalog and registry,
one station run, ordered sequence/stamps, model-time consistency and receipt plus
simulation ages below one second. The live adapter must also keep the source
binding current, as the existing telemetry/inspection adapters do.

Every accepted sample in the recovery window must show all machines powered,
bus voltage at least 27 V, and `voltage_v * current_a <= 110 W` for every machine.
These are the existing model's recovery conditions expressed through its public
measurements. Temperature and vibration remain available for diagnosis and
operation eligibility; the checker does not manufacture a fault classification.

A fresh consecutive pair first establishes observation validity. The window
starts at that pair's last sample and requires ten simulation seconds of healthy
consecutive samples. At nominal 2 Hz this takes 22 input samples, of which the
last 21 span the full window. It is not a count-only gate. Polling repeatedly or
advancing the query clock without new samples cannot extend the window.

An unhealthy sample, missing sequence or expired receipt/simulation age discards
the window. A new complete healthy window is required. Malformed/replayed input,
a changed run or clock regression remains a sticky reader failure. A final
`measure` rechecks freshness, including after a previously verified result; callers
must revalidate at publication after any further I/O. The result contains the
criteria, run/registry, first/last sample identities, count, freshness and SHA-256
of each canonical public sample plus newline in order. Storage stays constant in
size; recomputing the digest requires retaining the original packets separately.

This is **sampled station recovery**, not proof of continuous physical health,
successful reset, repair causation or control authorization. To bind it to a
maintenance sequence, the future executor must begin a new observer after the
last verified effect, require samples strictly after that effect and retain its
operation identity alongside the window. Standalone healthy observations cannot
identify who repaired a machine or even prove that a repair was necessary.

`scripts/run-machine-baseline.py` now decides its recovery outcome using this
public checker. The model's hidden `recovered` property remains an evaluator
oracle and existing model test surface. Scenario names and expected outcomes in
the baseline report are evaluator output, not planner observations. The baseline
still supplies synthetic robot geometry and executes synchronous model calls;
it does not implement the admission/hold/effect-confirmation contract above.

## Cancellation, deadlines and unresolved effects

Admission, effect commitment, observation verification and terminal publication
are separate stages. Cancellation/deadline before commitment prevents the effect.
After commitment it cannot erase that effect, report `not_applied`, repeat the
action under a new ID or declare success from an ACK. Retain the exact action and
its power/effect evidence until the actuator outcome is established.

Cancellation never automatically restores power. An already isolated target
stays isolated. If restore may have committed, record that uncertainty, keep the
drive latch/maintenance ownership and reconcile the exact action. Any compensating
isolation is a separately identified effect under the current interlocks, not a
hidden rollback or blind retry. If fresh preconditions or an applied result cannot
be established, send no further maintenance effects and require reconciliation.
Do not describe an unknown or energized outcome as a safely isolated cancellation.

The deadline bounds requested work; it does not authorize releasing unresolved
ownership. Client transport loss can return `uncertain` while the owner remains
active. A final result must retain both the requested terminal status and known
effect disposition (`not_applied`, `applied` or `unknown`) with evidence references.
Failure/cancellation after a real effect must never masquerade as a mutation-free
rejection. The isolation implementation retains unknown effects as nonterminal
`unresolved` ownership; it emits no terminal result until a known disposition
and exact hold release are verified. Restart never adopts that old action.
Reset/restore require their own boundary and loss controls. The current API is
documented separately.

## Acceptance and remaining work

Local checks execute the actual causal model and public decoder: isolated reset
indistinguishability, power isolation versus recovery, jam receipts without
recovery, complete-window timing, dropped/expired/unhealthy samples, sticky run
changes, final freshness and digest binding. A baseline regression makes any read
of the hidden `Station.recovered` property fail; the baseline must still return
healthy / repaired / escalated with 0 / 3 / 0 interventions.

Run `python3 -m unittest discover -s tests -v` and
`python3 scripts/run-machine-baseline.py` from the repository root.

OM-052 implements the station-owner isolation endpoint, drive hold, versioned
effect telemetry and local isolation verification/cancellation/replay controls.
OM-053 must add independently retained/native isolation acceptance; it must not
count the same verifier implementation again. Reset/restore, reconciliation and
native sequence acceptance remain open through OM-060. Reuse this checker in
that integration; do not count its implementation twice. The full mission needs
independent evaluation of causality, collisions, unnecessary interventions and
safe escalation as well as this observation result. Thor access and preceding
navigation/inspection acceptance remain open; no maintenance deployment occurred.
