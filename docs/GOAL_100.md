# Orbital Mechanic: 100 substantive deliveries

Objective: a simulated robot visits three machines, diagnoses a fault from permitted observations, performs an interlocked repair when appropriate, verifies recovery and produces an independently scored mission report.

The user requested at least 100 meaningful atomic commits on 2026-10-28. Product acceptance and commit quality are both required. This is a delivery backlog, not evidence that any item is implemented or a promise of 100 GitHub contribution squares on one day.

## Counting and quality

- Counting baseline: `4215aef2dd11cb60705e1759b98ffee3e0742e48`. The six migrated navigation commits and the email-history repair count as **zero new deliveries**.
- Use `Delivery: OM-NNN` commit trailers, Gmail author/committer identity, English content and one coherent change with its necessary tests/docs. OM-001 is this goal/acceptance decision.
- A qualifying commit adds behavior, fixes a reproduced failure, adds a discriminating acceptance check or records a necessary design decision. No empty commits, timestamp manipulation, cosmetic churn or splitting one inseparable change merely to increase the count.
- Existing components must be reused. An item already satisfied by existing code needs a meaningful integration or missing acceptance contribution; do not reimplement it for credit. Adjust the backlog transparently when evidence changes the work, preserving at least 100 substantive deliveries as the requested goal.
- Count distinct new nonempty changes once. Rebase, cherry-pick, merge wrappers and repeated pushes do not add credit. Mark published status only after readback; GitHub contribution display is separate from the delivery count.
- Review each slice and run appropriate checks before normal publication. Keep atomic commits when integrating; do not force-push existing published branches for this work.

## Boundaries and acceptance

Work in the isolated `codex/nav2-integration` worktree. Preserve the original localization branch and dirty main drafts. Thor operations stay in the project root and tmux orbital-work; own only the Orbital process tree/domain42/GZ orbital-maintenance. Never touch .229, Signal's files/services or Foxglove UI. Coordinate and announce bounded simulator movement/restarts before acceptance runs.

Independent maintenance integration uses the isolated `codex/machine-integration`
worktree/branch, stacked on the navigation branch with its own issue/PR scope.
This permits local progress while native navigation prerequisites remain open;
it does not waive any phase acceptance gate or add history-migration credit.

Phases 0/1 retain their recorded acceptance. Phase 2 requires actual Nav2 routes, independent arrival and collision evidence, and measured failure stopping. Synthetic action tests alone do not close it. Maintenance and scripted missions require fresh public observations and explicit safe failure outcomes. Keep hidden fault truth exclusively in the evaluator.

Planner integration is tested offline with replay fixtures. No paid provisioning or live model calls without explicit authorization. Offline success is not a claim of live provider quality or full Phase 5 acceptance. Raw recordings, credentials, generated outputs and large artifacts stay outside Git.

## Deliveries

All items below start pending except the goal-contract commit itself. Dependencies follow the group order; coherent independent work may overlap. Keep current progress in STATUS.md and commit trailers rather than marking planned work complete here.

### Navigation deployment

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-001 | Delivery contract and evidence rules | Record scope, ownership, counting baseline and all acceptance gates. |
| OM-002 | Installed Nav2 compatibility contract | Probe installed plugin, action and stamped-command support before launch. |
| OM-003 | Guarded controller configuration | Controller output reaches only the supervisor input; limits match the guard. |
| OM-004 | Obstacle-aware planner configuration | Known-map plans respect inflated robot footprint and occupied cells. |
| OM-005 | Navigation supervisor executable | Load validated named poses and refuse competing command publishers. |
| OM-006 | Owned Nav2 launch composition | Start only required planner/controller/navigator lifecycle nodes. |
| OM-007 | Navigation service lifecycle | Demo/localization loss tears down the owned navigation process tree. |
| OM-008 | Startup ownership preflight | Reject duplicate TF, action, controller or bypass publishers before activation. |
| OM-009 | Runtime source manifest | Bind deployed config/code/plugin identities to acceptance results. |
| OM-010 | Repeatable navigation readiness check | Prove lifecycle readiness with the guard latched and no robot movement. |

### Localization in the integrated system

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-011 | Unseeded navigation refusal | Real Gazebo navigation cannot arm before an explicit valid seed. |
| OM-012 | Offset-seed convergence acceptance | Compare localization against independent simulator pose after bounded motion. |
| OM-013 | Localization freshness commissioning | Expired scan/current-TF evidence blocks the integrated supervisor; retained stationary AMCL pose must be valid and satisfy the post-seed gate. |
| OM-014 | Localization reset integration | World reset invalidates the old target and requires new localization. |
| OM-015 | Reseed ownership integration | Reseeding during a goal stops it and cannot resume its previous command stream. |
| OM-016 | TF authority audit | Verify one owner per dynamic edge and reject inconsistent frames. |
| OM-017 | Map identity binding | Reject localization/navigation configurations built for a different map revision. |
| OM-018 | Stationary arm commissioning | Fresh stationary feedback and guard acknowledgment precede dispatch. |
| OM-019 | Localization-loss physical stop acceptance | Measure actual Gazebo stopping after loss of valid localization. |
| OM-020 | Localization recovery contract | Fresh recovery permits a new explicit target, never automatic old-target motion. |

### First autonomous tour

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-021 | Named inspection-pose validation | Validate machine pose geometry, clearance and map bounds. |
| OM-022 | Bounded operator navigation command | Request/arm/cancel through the existing exact-target protocol. |
| OM-023 | Independent trajectory recorder | Measure simulator truth, localization and command identities together. |
| OM-024 | First machine arrival | Reach machine one with independent position/yaw evidence. |
| OM-025 | Second machine arrival | Reach machine two from the first using obstacle-aware planning. |
| OM-026 | Third machine arrival | Reach machine three and validate inspection clearance. |
| OM-027 | Observed collision evaluator | Detect body/world contacts or geometric overlap throughout the route. |
| OM-028 | Tour sequence ownership | Advance only after the prior target has a verified terminal result. |
| OM-029 | Return-to-safe-pose action | Return to a validated staging pose through guarded navigation. |
| OM-030 | Three-machine tour acceptance | Reproduce the complete tour from a documented initial state. |

### Navigation failure acceptance

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-031 | Blocked-goal commissioning | An unreachable named pose terminates without indefinite motion. |
| OM-032 | Goal-timeout physical stopping | Timeout cancels the exact goal and measured speed reaches zero. |
| OM-033 | Mid-route operator cancellation | Cancellation cannot release ownership before terminal stop evidence. |
| OM-034 | Controller-death acceptance | Controller loss stops safely and requires explicit recovery. |
| OM-035 | Navigator-death acceptance | Retain ownership when the action result cannot be established. |
| OM-036 | Supervisor-death physical acceptance | Independent guard stops the physical simulation after supervisor exit. |
| OM-037 | Guard-loss navigation teardown | No navigation producer survives the loss of the drive guard. |
| OM-038 | Clock pause and resume acceptance | Stale pre-pause commands never restart motion. |
| OM-039 | Delayed command rejection acceptance | Reject replayed and out-of-order controller samples under real integration. |
| OM-040 | Navigation phase acceptance package | Publish reproducible route/failure evidence and exact limitations. |

### Robot-to-machine observations

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-041 | Machine registry integration | Bind simulation props to stable public machine identities. |
| OM-042 | Trusted proximity calculation | Inspection eligibility uses fresh measured robot pose. |
| OM-043 | Inspection orientation constraint | Reject observations from outside the valid viewing/control pose. |
| OM-044 | Stationary inspection interlock | Inspection cannot run while moving or before settling. |
| OM-045 | Public telemetry projection acceptance | Reuse OM-041's projection; commission the actual native publisher/inspection consumer across scenarios and prove injected truth stays absent. |
| OM-046 | Telemetry freshness semantics | Unknown/stale samples cannot support an intervention. |
| OM-047 | Inspection observation records | Bind observations to source time, machine and robot pose. |
| OM-048 | Camera evidence capture | Capture bounded timestamped observations locally without committing media. |
| OM-049 | Inspection action endpoint | Provide typed request, cancellation and terminal result. |
| OM-050 | Three-machine inspection acceptance | Collect reproducible observations at all three validated poses. |

### Interlocked maintenance

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-051 | Maintenance action contract | Define allowed transitions and observable postconditions. |
| OM-052 | Power-isolation integration | Require verified proximity/stationarity before isolation. |
| OM-053 | Independent isolation acceptance | Read-only source recorder and offline postcondition replay implemented locally; native fault/cancellation proof remains pending. Do not recount OM-052's owner verifier. Sampled model power-off does not verify camera/hold admission or hardware electrical isolation. |
| OM-054 | Bounded reset integration | Local endpoint uses retained eligible powered evidence, owned isolation, cooldown, a new inspection and one reserved attempt; exact post-effect pair precedes release. Native acceptance remains pending. |
| OM-055 | Power-restoration integration | Local endpoint requires exact verified reset, current cooldown, a new inspection and one reserved attempt. Paired new telemetry confirms energization; recovery and native sequence acceptance remain separate. |
| OM-056 | Maintenance cancellation | Explicit cancel-isolate resolves the owned action, admits one fresh interlocked isolation if needed and verifies a new final pair. Unknown outcomes remain owned; native acceptance is pending. Existing ordinary cancellation controls are reused without duplicate credit. |
| OM-057 | Durable intervention identity | Local scenario claims and retained mission/action/attempt boundaries fence station restart and reused IDs, including real process-crash cuts. New worlds require new mission/action IDs; no state adoption. Existing same-run replay is reused without duplicate credit; native acceptance remains pending. |
| OM-058 | Persistent-fault refusal | Model/reset healthy, jam and sticky high-vibration controls already exist in OM-051/054/055; reuse without extra delivery credit. Native full-scenario refusal remains pending. |
| OM-059 | Observation-loss intervention stop | Existing OM-052/054/055/056 final pose/telemetry/hold/source/deadline controls prevent new effects; reuse without extra delivery credit. Native loss acceptance remains pending. |
| OM-060 | Maintenance sequence evidence and acceptance | Read-only full sequence retention and independent raw-sample replay extend isolation to reset/restore/cancel-isolate, with separate public recovery. Native complete sequence acceptance remains pending. |

### Deterministic mission

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-061 | Mission state contract | Local executable transitions bind existing client/owner results, retain uncertain steps and require new public recovery. Live gateway ownership and native mission acceptance remain pending. |
| OM-062 | Evidence-driven inspection order | Local bounded order and diagnosis bind actual inspection files to retained public samples. One coherent delivery with OM-063; live gateway integration remains pending. |
| OM-063 | Deterministic diagnosis policy | Implemented with OM-062: public model hypotheses, sticky contradiction history and explicit uncertainty. No separate delivery credit or physical diagnosis claim. |
| OM-064 | Healthy-machine preservation | OM-062 locally inspects all three healthy machines without intervention, then requires a new public health window. Reuse without extra credit; native acceptance remains pending. |
| OM-065 | Ambiguous-evidence handling | OM-062 locally bounds repeated inspection and changing candidates before escalation. Reuse without extra credit; native acceptance remains pending. |
| OM-066 | Recoverable-fault mission | Persistent ROS gateway integrates public diagnosis and actual owned client results, durable intent and new recovery. Controlled adapter/station tests pass; native fresh-scenario recovery remains pending. One coherent delivery includes shared cancellation and restart-refusal foundations, without duplicate credit. |
| OM-067 | Persistent-fault escalation | Report evidence and leave the machine safe when recovery is unavailable. |
| OM-068 | Mission cancellation propagation | OM-066 connects exact cancellation to actual clients and retains lost results/effects locally. Reuse without extra credit; complete native owner containment and final electrical isolation remain pending. |
| OM-069 | Interrupted mission recovery | OM-066 durably refuses unfinished missions, including actual process death. Resume/reconciliation still requires revalidating exact machine and robot ownership; refusal alone is not recovery acceptance. |
| OM-070 | Complete scripted mission acceptance | Run the original station-power mission without an LLM. |

### Independent evaluation

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-071 | Scenario fixture schema | Define repeatable seeds and initial conditions outside planner observations. |
| OM-072 | Hidden-truth isolation check | Prove planner/tool inputs cannot read evaluator-only fields. |
| OM-073 | Healthy scenario acceptance | Score correct non-intervention and unnecessary actions. |
| OM-074 | Recoverable scenario acceptance | Score measured recovery rather than reported success. |
| OM-075 | Persistent scenario acceptance | Score safe escalation rather than forced task completion. |
| OM-076 | Ambiguous scenario acceptance | Score evidence gathering and bounded uncertainty handling. |
| OM-077 | Fault-timing scenarios | Inject faults during travel and inspection with recorded timing. |
| OM-078 | Navigation/maintenance combined faults | Evaluate safe outcomes across action-boundary failures. |
| OM-079 | Run metrics and event trace | Record success, interventions, collisions, duration and termination causes. |
| OM-080 | Repeatable evaluation report | Compare matched scenarios with source/config/seed identities. |

### Planner adapter without paid calls

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-081 | Versioned planner tool schemas | Expose typed bounded actions and explicit error outcomes. |
| OM-082 | Tool validation boundary | Reject malformed, extra, nonfinite and out-of-range arguments. |
| OM-083 | Planner observation boundary | Send only public fresh observations and permitted machine metadata. |
| OM-084 | Provider adapter interface | Separate provider transport from deterministic execution and policy. |
| OM-085 | Recorded-response replay adapter | Exercise realistic tool turns without network or paid inference. |
| OM-086 | Bounded planning loop | Cap turns, retries and pending actions; stop on exhausted limits. |
| OM-087 | Refusal and truncation handling | Provider stop conditions cannot masquerade as valid actions. |
| OM-088 | Tool-call replay protection | Repeated provider calls reuse results without duplicate effects. |
| OM-089 | Budget and credential boundary | Enforce configured ceilings and keep secrets out of traces/artifacts. |
| OM-090 | Matched offline planner evaluation | Compare replay fixtures with scripted behavior; never claim live model quality. |

### Reproducible delivery

| ID | Deliverable | Acceptance gate |
| --- | --- | --- |
| OM-091 | Operator mission entrypoint | Start, inspect and cancel a bounded mission from one documented interface. |
| OM-092 | Readable mission evidence view | Show observed facts, actions and verified outcomes with freshness. |
| OM-093 | Run artifact validation | Validate checksums, source identities and complete terminal records. |
| OM-094 | Fresh-checkout reproduction | Run supported local checks without private or untracked dependencies. |
| OM-095 | Scoped simulator deployment | Deploy versioned artifacts and fail safely on compatibility mismatch. |
| OM-096 | Demonstrated rollback | Return to the prior accepted simulator artifact without affecting other projects. |
| OM-097 | Robustness review closure | Reproduce and resolve concrete findings across failure and trust boundaries. |
| OM-098 | Demonstration recording workflow | Record a real accepted mission with reproducible setup and local media. |
| OM-099 | Professional project handoff | Document quick start, architecture, evidence, limitations and next work. |
| OM-100 | Final acceptance and commit audit | Verify product gates plus at least 100 qualifying published commits. |
