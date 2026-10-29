# Mission transition contract

OM-061 implements `orbital.mission_state.MissionState`, the bounded transition contract for the scripted mission and its shared gateway.

It connects existing navigation, inspection and maintenance message identities in one mission without equating an acknowledgement with completion.

The contract is exercised with the actual clients, inspection owner and durable maintenance station over controlled transport.

The [public inspection policy](MISSION_POLICY.md) supplies bounded inspection order
and diagnosis. The [persistent runtime](MISSION_RUNTIME.md) now owns the shared
ROS gateway locally; native mission acceptance and independent scenario scoring
remain open under #10/#11.

## State and outcomes

| Phase | Required evidence or permitted next step |
| --- | --- |
| `inspect` | Reserve a navigation request, or inspect the machine whose exact navigation target completed. |
| `navigating` | Retain the reserved request and exact target. Arm is separately reserved before its gateway call. An ACK alone cannot advance the mission. |
| `inspecting` | Await the exact inspection terminal result; an ACK, lost reply or running capture is insufficient. |
| `diagnose` | Record a policy decision bound to that successful inspection's action and observation reference. |
| `intervene` | Admit only the next isolate/reset/restore operation for the selected machine and current accounted control revision. |
| `intervening` | Retain the exact action until its terminal result, including cancellation after an applied effect. |
| `verify` | Collect a new complete public recovery window after the decision or successful restoration. |
| `escalate` | No step remains pending; the controller may record an operator escalation instead of another effect. |
| `unresolved` | Delivery, completion, cleanup, clock or deadline is uncertain while a step remains pending. No new step is permitted. |
| `terminal` | One frozen outcome: `healthy`, `recovered`, `escalated`, `cancelled`, `timed_out` or `failed`. |

`escalated` means the mission stopped pursuing its task and reported the evidence; it is not an independent assertion of electrical isolation or physical stopping.

Cancellation and timeout likewise preserve the last accounted effect, rather than implying rollback or power off.

## Controller and gateway obligations

Construct the contract with a fresh, schema-2 public station baseline, the matching registry, a new mission ID and the controller's actual monotonic and simulation clocks.

The gateway must establish current source/service ownership, enforce its existing transport bounds and use the existing server-side live interlocks.

The contract validates the identity and transitions of those observations; it does not authenticate arbitrary caller-supplied dictionaries or grant actuator permission.

Use `dispatch(request, invoke)` to reserve navigation, inspection or maintenance intent before entering the gateway callback.

Use `dispatch_arm(command, invoke)` for the separate exact navigation arm command; it can be reserved only once for that target, before publication.

The callback must obey the remaining mission budget and the existing client deadline; this synchronous library cannot preempt an unbounded caller.

Exceptions, including interruption, retain possible delivery and prohibit an automatic second dispatch.

The lower-level `begin_navigation`, `begin_navigation_arm` and `begin_action` methods allow a nonblocking controller to reserve before publishing; they do not themselves publish or observe a send.

The controller then feeds the corresponding source-checked receipt, result or navigation state.

The final application must keep this contract inside its exclusive gateway owner; constructing a second Python object does not acquire exclusive ROS ownership or recover an interrupted mission.

## Result binding

Navigation request and arm receipts must match the reserved commands and exact run/epoch/sequence/waypoint target.

Completion additionally requires a fresh advancing supervisor state, the same target in `last_completion`, idle target ownership, healthy state, a latched guard and the supervisor's measured `arrived` report.

That report remains supervisor evidence; independent trajectory/collision acceptance is separate.

Inspection and maintenance results must match the mission, owner, action, semantic request digest, selected machine and action timestamps.

Owner generations cannot silently change between steps.

Maintenance additionally binds station generation, the next control revision and exact separately retained isolation/reset prerequisite results.

A valid result from the real owner for a different timeout or request cannot resolve this mission's step.

An applied effect is preserved even if the action or mission ends cancelled or timed out.

A late terminal result can resolve ownership, but cannot turn an expired mission into success.

An `uncertain` receipt, unconfirmed rejection, failed client cleanup or malformed gateway result keeps the step pending.

Explicit `not_sent` cannot contradict a previous possible delivery.

## Decisions and public verification

`decide` accepts one of `healthy`, `recoverable`, `persistent`, `ambiguous` or `inspect_again`, bound to the latest successful inspection reference.

These are controller policy decisions, not diagnoses inferred by this library.

The separate policy reads the verified public inspection artifact and current observations, handles uncertainty and keeps evaluator labels out of its inputs.

The contract does not read images, artifact measurements, hidden fault labels or reset counters to select a diagnosis.

`persistent` and `ambiguous` select escalation; `inspect_again` permits another bounded inspection step; `recoverable` permits the owned isolate/reset/restore sequence.

`healthy` still requires a new complete public healthy window with no maintenance in that path.

`observe_telemetry` and `verify` reuse `RecoveryEvidence`: all machines powered, voltage at least 27 V and each observed machine power at most 110 W over ten consecutive simulation seconds.

The window starts after the verification phase begins, remains fresh in both clocks and must agree with the accounted control.

`recovered` also requires this mission's exact successful restoration; energization, an old healthy window or a later persistent fault cannot provide that outcome.

This is sampled public model recovery, not repair causation or continuous physical health.

## Cancellation, bounds and restart

`cancel()` records sticky cancellation intent and returns the exact pending action or navigation target for the controller.

It sends no cancellation itself, and remains unresolved until the matching terminal result arrives.

An unknown navigation request target stays unknown; a new supervisor target is not guessed or adopted.

The runtime now propagates bounded exact cancellation through actual clients.
Explicit mission cancel-isolate policy, complete native process containment and
independent final stopping evidence remain open.

Mission duration is at most 600 wall seconds, with 32 steps by default and a hard maximum of 64; each normalized gateway record is at most 65,536 UTF-8 bytes.

Step identities never evict or reset within a session. Each navigation target has at most one separately reserved arm command.

Snapshot copies cannot mutate the active contract, and completed decisions ignore later telemetry rather than changing their historical outcome.

This state object remains in memory. The runtime adds a cross-process owner lock,
durable prior intent and unfinished-mission refusal. There is no resume API or
durable adoption of an interrupted mission.

The station's existing durable history still refuses reused maintenance identities; it does not make this mission controller restart-safe.

## Validation and remaining acceptance

Local tests use actual NavigationClient, InspectionClient, InspectionOwner, MaintenanceClient and the durable maintenance owner/model.

Navigation peer messages, camera workers and hold feedback are controlled; these are not native Nav2 routes or real camera-child mission acceptance.

Controls cover acknowledgement versus arrival, exact arm reservation, request/result mismatch, actual effect followed by gateway loss, cleanup uncertainty, cancellation/deadline precedence, bounded identity history, old health windows and new faults after restoration.

Candidate regressions cover telemetry after completion or at the idle deadline, the actual supervisor's optional maintenance-hold refusal reason and malformed arm receipts; retained identical tests fail before and pass after their corrections.

Three retained sequences replay healthy, recovered and effect-with-lost-reply states through this same contract; this establishes reproducibility, not independent mission scoring.

The cumulative suite also runs the earlier composed station/camera endpoint controls, separately from the mission fixture's controlled workers.

The policy and persistent gateway/runner are locally implemented. Interrupted
mission reconciliation, independent hidden-truth isolation/scoring and the
complete native scenario matrix remain open.

Do not close #10/#11/#12 or declare Phase 4 accepted from this contract's tests.
