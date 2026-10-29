# Independent retained mission evaluation

OM-080 joins the actual gateway's private observations, its durable mission
journal, the independently retained station state and real capture files.

The result checks the claimed terminal time, so later recovery cannot validate
an early success claim.

## Capture and closure

The production ROS gateway creates a mandatory private witness under
`runtime/mission-service/mission-observations/<mission-id>/`.

It retains raw callback input with receipt wall/simulation clocks, actual native
publication-call attempts, exact prior journal prefixes, endpoint GIDs and the
verified manifest/world/three service invocation identities before and after.

The existing sole command publisher/subscriber guard remains enforced.

Input callbacks execute before witness disk writes; their receipt age is not
renewed by storage delay.

Publication entry copies clocks and the already durable journal prefix in memory,
then records the native call's outcome after it returns or raises.

A returned native publication is not acknowledgement or proof of actuation.

File/source/capacity checks run before the existing final freshness gates.

An ambiguous or failed write invalidates complete evidence, while exact cleanup
cancellation and incoming cleanup callbacks remain available.

Complete closure requires a terminal mission journal, unchanged runtime/graph
identities, successful adapter cleanup and payload fsync before creating the
separate summary manifest.

The stream is bounded to 128 MiB, 65,536 records, 128 KiB per record and 32 KiB per
raw message, with at most 64 retained mission directories.

Files are exclusive private regular files with identity/read-back checks and
hash links; they are not authenticated storage or protection against coherent
same-user rewriting/rollback.

The producer shares the trusted gateway process; it is separate from policy
claims but does not establish OS/process isolation or loaded-code attestation.

## Offline evaluation

After cleanly stopping the station and gateway, without starting another mission:

```bash
python3 scripts/evaluate-mission.py --trial-id TRIAL_ID --mission-id MISSION_ID
python3 scripts/evaluate-mission.py --trial-id TRIAL_ID --mission-id MISSION_ID \
  --collision-trace PATH_TO_RETAINED_TRACE_DIRECTORY
```

The evaluator creates no trajectory and publishes no control command.

Exit 0 means valid evidence and a complete report, including a negative outcome;
exit 2 means evaluation unavailable and emits only a bounded error.

The evaluator holds station ownership before mission ownership, validates exact
public packet digests against actual private samples, joins native publication
attempts to prior durable intents, and binds replies/results to those attempts.

Pre-admission rejections remain counted even when no station action was admitted.

Terminal owner states must have been observed as raw inputs; journal claims
cannot replace missing observations.

Actual image/manifest bundles are reopened and checked against their exact
result references, public station packets, registry and capture context.

Maintenance's internal inspect request has its own argument digest; its outer
maintenance digest is separately bound through the actual durable result and
publication, not falsely equated with the nested inspection digest.

Recovery requires at least ten simulation seconds of sampled healthy state at
the original mission boundary, with no sample gap above one second.

Every actual effect resets that window; initial healthy-machine mutations,
repeated mutating action attempts and recorded effect/precondition deviations
cannot pass the station criterion.

`success_claim_supported` applies to this station criterion only; reported
escalation/cancellation does not establish physical safety.

The privileged offline report contains profile/seed, exact input and observation
digests, source/runtime identities and a matched-conditions digest excluding only
the opaque trial ID.

This enables comparisons under identical declared conditions; it does not turn
different native trials into equivalent observations or supply a batch benchmark.

Never send this evaluator output to a planner or public observation interface.

## Optional collision join

A trace must pass the existing complete geometry/protocol replay and match the
exact runtime/service envelope and navigation run.

Its first and last captured truth samples must cover the entire mission on both
wall and simulation clocks; matching world files or timestamps alone is insufficient.

Every actual navigation publication must correspond to exactly one audited
command/reply in the mission interval, and additional or missing commands refuse
the join.

The returned `covering_trace` scores the entire trace, including samples outside
the mission; adverse pre/post samples are not silently discarded.

`continuous_collision_free` and `overall_acceptance` remain null even when every
sample is clear.

Final source/file checks run after analysis and before returning the report.

## Evidence and remaining gates

Local tests use the real station producer, ROS mission adapter, protocol clients,
capture adapter and image files with controlled graph/navigation/robot/camera and
worker lifecycle; they do not run a complete native ROS/Gazebo mission.

The collision join controls use explicitly synthetic geometry and observer
replies populated with actual retained mission identities/commands.

Healthy, recoverable, persistent, ambiguous, rejected-action and navigation
cancellation paths, false early success, unnecessary intervention, coherent
rehash corruption and final file/source changes are exercised.

Evidence is retained in operator workspace `reviews/orbital-delivery-20261028/om080`.

Independent robot interlocks, physical/native collision acceptance, authenticated
sources, OS/planner isolation, real blocked/noisy scenarios, interrupted mission
reconciliation, full cancellation containment, matched native trials and the
25-trial freeze remain open.

This is one coherent local delivery, with no duplicate OM-072–078 acceptance credit.
