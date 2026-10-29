# Cancellation with observed isolation

OM-056 adds an explicit `cancel-isolate` policy to the station owner. It cancels
one exact owned maintenance action, resolves its outcome and obtains fresh
evidence that the target is powered off. If the target is still powered, it
admits one separately identified isolation action under the existing live
interlocks. It never hides that action inside the original result.

Ordinary `cancel` retains its existing meaning: stop the named action without
rolling back power. A cancelled restoration can therefore remain energized.
Use the explicit policy when the requested final condition is observed model
isolation, and inspect its result before treating that condition as established.

## Operator request

After native commissioning and sourcing the Orbital ROS environment:

```bash
python3 scripts/maintain-machine.py cancel-isolate pump \
  --run-id RUN --mission-id MISSION --target-action-id ORIGINAL --timeout 20
python3 scripts/maintain-machine.py query \
  --run-id RUN --mission-id MISSION --action-id CANCELLATION
```

The exact owner/mission/target IDs are required. The CLI generates a new
cancellation action ID unless `--action-id` is supplied. The response also names
the server-reserved isolation child ID. The request must name an action in the
same owner, mission, machine and station generation. A superseded terminal
action, a nested cancellation or another active action cannot authorize this
workflow. A completed restoration can be targeted while its effect is current;
its original successful result remains unchanged.

The schema-1 wire operation is `cancel_isolate`, with `machine_id`,
`station_run_id`, `target_action_id` and `timeout_s`. It accepts no caller pose,
power measurement, path, hold token or compensation ID. The finite 0.1–30-second
deadline covers the complete workflow; it never gives each phase a fresh budget.
Completion rechecks that deadline after evidence I/O. A still-fresh isolated
measurement can accompany a timed-out workflow; it cannot turn expiry into success.

`--no-wait` reports admission only. Client loss, interruption or an observation
timeout returns uncertainty without retrying or implicitly aborting the explicit
isolation policy. The server keeps its own deadline and ownership. A new client
may query the exact original cancellation ID. An explicit ordinary `cancel` of
that ID prevents future compensation and cancels an already admitted child; it
does not erase an applied effect or promise isolation.

## Ownership and execution

1. Reserve both the workflow history slot and a distinct child identity before
   stopping the target. Neither history nor an unused child reservation is
   evicted. Exact replay returns the same record; changed arguments or another
   workflow targeting the same original action are refused.
2. Keep the original action, capture child and hold owned until its outcome is
   known and its normal release completes. An unexpected actuator exception
   remains unresolved even when telemetry appears to show power off. No new
   effect is authorized by that appearance.
3. Pin the current station/control identity and require two new consecutive
   public packets after resolution. Both members must remain younger than one
   second in receipt and simulation clocks. A contradictory first measurement
   cannot be replaced by a fresh latest one.
4. If both packets show power off, zero current and zero vibration, complete
   without another mutation. This is read-only confirmation; it grants no pose
   permission and does not require an unnecessary capture or reset.
5. If both show power on and time remains, admit exactly one new `isolate`
   action. It acquires its own hold, resolves navigation, runs a new actual camera
   capture and rechecks pose/source/stationarity/freshness after artifact I/O.
   The ordinary station actuator owns the effect and observation verification.
   A rejection, deadline or unknown outcome never becomes an automatic retry.
6. After the child's known result and hold release, require another new paired
   observation before reporting the workflow's final condition. A cached child
   result alone is insufficient. Recheck the authoritative control revision
   after assembling that pair; fresh packets may predate a newer effect.
   Preserve both original and child records.

Unrelated maintenance admissions are blocked throughout the workflow. The
original hold is released before the new isolation hold is acquired; these are
separate interlocked actions, not a continuously held token. The guard's existing
release behavior stays latched, and each new effect requires fresh navigation
quiescence and a settled inspection pose. This does not establish a mission-wide
navigation lock or authorize movement between actions.

## Results and failure boundaries

Workflow results use schema 4. Existing isolate/reset/restore results retain
schemas 1/2/3, and command/state envelopes stay schema 1. Clients must understand
the new action-view/result variants; old decoders reject them. The active view
exposes original/child IDs and phases. Result nesting is bounded to one original
isolate/reset/restore chain and one isolation child; cancellation nesting fails.

The result binds the normalized request, original request/result, child
request/result or admission rejection, actual paired sample hashes and measured
power/current/vibration. `final_state` has three meanings:

| Value | Meaning |
| --- | --- |
| `isolated` | Two new exact-control samples observed power off, zero current and zero vibration. |
| `energized` | Two new exact-control samples observed power on after a known failed or stopped workflow. |
| `unverified` | The current condition could not be established; historical child measurements are not renewed. |

Only a successful `isolated` workflow claims its requested condition completed.
A deadline or explicit cancellation may occur after actual child isolation; the
nested record retains that applied effect while the workflow can still report
`unverified` because it did not obtain its own post-release pair. Source loss
never becomes a safe-state assertion. A known failure can terminate with a
truthful energized/unverified result once all owned children and holds resolve.

Unknown original or child effects stay nonterminal with ownership retained,
including after the deadline. Shutdown reaps the current child without inventing
a result or releasing an unresolved hold. Restart does not adopt this workflow,
its files or old identities. [Durable history](DURABLE_MAINTENANCE.md) now retains
these boundaries, fences station-only restart and refuses reused mission/action
IDs in a new explicit scenario. Live mission recovery remains open.

## Validation scope

Local tests execute the actual station/guard/navigation adapter and separate
camera workers over controlled transport. The after-restoration case uses a
fourth real capture and reads its request-bound image/manifest. Cases cover
before-effect cancellation, late artifact expiry and cancellation after the
new isolation effect. Unit/client controls include reserved capacity, exact
replay, unknown effects, source loss, deadlines, both clocks, contradictory
samples, lost ACK/result and disconnected observers.

Review reproduced a candidate which could publish completion from a fresh pair
after another actual model effect had advanced the control revision. The same
regression fails on the retained candidate and passes after the final authoritative
revision check. This was corrected before publication within OM-056.

The cooperative fixture clock isolates ownership from host scheduling; production
uses real monotonic clocks and unchanged freshness limits. These controls do not
prove native DDS/timing, complete sequence replay, physical electrical isolation,
repair or mission acceptance. The independent isolation recorder does not yet
replay this composite workflow. Source-bound evidence is retained in operator
workspace `reviews/orbital-delivery-20261028/om056`.
