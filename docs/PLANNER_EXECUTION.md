# Durable planner tool execution

OM-082 connects the nine [wire tools](PLANNER_CONTRACT.md) to the existing owned mission clients.

`PlannerRuntime` runs the existing public-measurement diagnosis policy through `PlannerGateway`; it does not invoke a language model.

The ROS adapter, command publishers, owner identities, capture validation and independent mission evaluator remain the existing implementations.

## Run the same-contract scripted baseline

From the configured Thor project inside `tmux orbital-work`, with localhost discovery on domain 42 and all documented prerequisites already commissioned:

```bash
python3 scripts/run-scripted-mission.py run --tool-contract --timeout 600
```

This is a locally tested integration entrypoint, not a claim that native tool missions have been commissioned.

The command does not start the simulator or services.

Without `--tool-contract`, the existing scripted route remains available, subject to the same unfinished-mission and durable stop fences.

The tool mode observes public telemetry, navigates to the selected configured machine, inspects it, applies the existing scripted diagnosis policy, requests any eligible isolation/reset/restoration, measures recovery and records its outcome claim.

Healthy scenarios inspect all three machines without intervention; persistent and ambiguous scenarios escalate.

Inspection applies the deterministic diagnosis policy automatically in this baseline; a recorded provider can now choose bounded tools through the separate [proposal loop](PLANNER_PROVIDER.md), but cannot choose an independent diagnosis.

## Admission, execution and replay

The private mission journal owns one executive run ID, a maximum of 32 ordinary logical calls and one reserved stop slot.

Each request is validated against host-owned context, mission phase, permitted target/operation, live sources and its original public observation.

A fsynced `tool_intent` precedes any native action, and a fsynced `tool_binding` joins the exact logical call to each already-journaled native command before publication.

The final in-memory publication check rejects an expired or superseded original observation after journal and source I/O; it never substitutes a newer packet silently.

Navigation request and arm share one logical deadline, measured from admission, while existing owner, freshness and guard checks still govern each publication.

Action timeouts use the remaining logical budget; verification honors the requested 10–30-second measured window rather than silently shortening it to ten seconds.

Deadlines are cooperative checks with bounded polling and owned cleanup, not an independent real-time or distributed effect deadline.

A terminal reply is returned only after the exact native result and bounded public records have been validated and its `tool_result` persisted.

An identical completed call returns its original evidence without publishing another command or renewing measurement timestamps.

A changed request under the same action ID is refused, even when only its timeout differs.

A pending call is never replayed as fresh work; other ordinary calls are refused until its ownership is resolved.

Lost result delivery after a real effect remains unknown to the executive until the exact owner's original result is observed; a timeout or cancellation does not erase the reported effect.

The existing mission admission refuses every unfinished journal before transport construction, including a process killed after a tool result became durable but before the mission closed.

This implementation provides restart refusal and historical replay, not automatic interrupted-mission reconciliation.

## Stop and explicit operator reset

A stop arriving during another call immediately sets an in-memory cancellation request and returns `planner_tool_pending`, with `terminal: false` and `durable: false`.

This acknowledgement grants no completion, durability or physical-stop claim.

The serial owner attempts its existing exact cancellation, records the queued stop when it can, and exposes a separately persisted terminal result for an identical retry.

Unknown original ownership remains pending; failed stop delivery is visible in the runtime report and cannot produce a complete mission.

A durably admitted tool stop latches the journal against a new mission, including legacy-mode invocations and process restarts.

Only after the stopped mission has closed, an explicit operator may acknowledge that admission latch:

```bash
python3 scripts/run-scripted-mission.py reset-tool-stop --acknowledge-stop
```

This records a separate audit identity and clears only the planner admission latch.

It does not reset the motion guard, start another mission, energize a machine, reconcile unfinished ownership or modify prior evaluation records.

Never delete the journal to bypass an unresolved action.

Stop evidence reports fresh idle owners and the navigation guard latch; independently measured physical stopping, complete cancellation containment and final electrical isolation remain open.

## Evidence and limits

Public observation IDs and exact native result bindings are retained inside bounded mission history.

Maintenance effect revision and powered state must match the bound owner result; a caller cannot supply observations or inject an evaluator score.

`report_outcome` must match the executive's computed state and known evidence references.

A healthy/recovered claim also requires all earlier logical calls to have succeeded; the private evaluator still scores the original claim time independently.

Local composed tests use the production mission adapter, actual clients, causal station history and actual capture files, with controlled transport, navigation, robot and camera peers.

They cover healthy/recoverable/persistent/ambiguous profiles and stopped missions; operator acknowledgement preserves the original mission evaluation unchanged.

Owned child-process SIGKILL controls cover binding before publication, actual isolation before logical result persistence, and a persisted result before return.

They establish observed process-death behavior, not physical power-loss durability or native ROS acceptance.

The library is trusted code in the executive process; its child proposal loop adds only synthetic recorded transport.

No external tool server, OS/process isolation, independent source authentication or live model call is implemented.

Earlier native navigation/maintenance gates, independent robot pose/interlock evidence, blocked/noisy and held-out inputs, the 25-trial freeze and the full MVP remain incomplete.

The OM-082 backlog entry replaces argument validation already delivered by OM-081 with this missing execution integration; this is one coherent delivery, with no extra OM-083/086/088 credit.
