# Owned station maintenance

OM-052 adds `isolate`, `cancel`, `query` and `status` to the station process.
It changes the exact `Station` instance that publishes `/station/telemetry`.
No second station model is started in the capture process. OM-054 adds
[bounded reset](RESET_OPERATION.md) after an exact owned isolation, using retained
powered observations, cooldown and a second live inspection. OM-055 adds
[owned power restoration](RESTORE_OPERATION.md) after verified reset and a third
inspection. OM-056 adds an explicit [cancel-isolate workflow](CANCELLATION_OPERATION.md)
which resolves the original and, when needed, admits one new interlocked
isolation. OM-057 adds [durable claims and historical reconciliation](DURABLE_MAINTENANCE.md).
Station-only restart and old mission/action reuse are refused; no live state is adopted.
Mission recovery orchestration remains separate work.

Local controls execute the actual station adapter, motion guard, navigation
supervisor, separate capture process and artifact reader with controlled ROS
transport. Native DDS, service containment, three-machine commissioning and
physical route/maintenance acceptance remain pending. This implements simulated
power isolation, not hardware electrical isolation or repaired machinery.

## Operator entry point

After commissioning the exact source and sourcing the Orbital ROS environment:

```bash
python3 scripts/maintain-machine.py status
python3 scripts/maintain-machine.py isolate pump --timeout 15
python3 scripts/maintain-machine.py reset pump --run-id RUN --mission-id MISSION --isolation-action-id ISOLATION --timeout 15
python3 scripts/maintain-machine.py restore pump --run-id RUN --mission-id MISSION --reset-action-id RESET --timeout 15
python3 scripts/maintain-machine.py query --run-id RUN --mission-id MISSION --action-id ACTION
python3 scripts/maintain-machine.py cancel --run-id RUN --mission-id MISSION --action-id ACTION
python3 scripts/maintain-machine.py cancel-isolate pump --run-id RUN --mission-id MISSION --target-action-id ACTION --timeout 20
```

The IDs are placeholders for the exact 32-character lowercase hex values in the
admission receipt. Only localhost domain 42 or a verified empty isolated domain
84 is accepted. `cooler` and `power_unit` are also allowlisted. The finite deadline
is 0.1–30 seconds. The CLI discovers the current station run and control revision;
it sends no pose, velocity, paths, topics or caller evidence.

`--no-wait` returns an admission receipt, not an effect or completed action.
`transport` distinguishes `not_sent`, `rejected`, `acknowledged`, `terminal` and
`uncertain`. Exit 0 may mean an ACK when explicitly requested; inspect the
transport and result fields. A non-success terminal or uncertainty returns 2.
The client sends each isolation/reset/restore once. Loss/timeout/interrupt triggers at most one exact
cancel attempt, never another power request. A lost reply can be recovered with
same-run query. Server restart requires reconciliation, not replaying the
operation with a new run or action ID.

For `cancel-isolate`, client loss returns uncertainty without sending an implicit
cancel of that policy. The server continues within its own deadline. Its schema-4
result preserves original and isolation child outcomes and distinguishes observed
isolation, observed energization and an unverified current state. No unknown
effect can authorize compensation; see the separate workflow contract above.

## Transport and identity

`station_telemetry` owns `/maintenance/state`, `/maintenance/reply`,
`/maintenance/result` and `/motion/maintenance` (`std_msgs/msg/String`). It
subscribes to `/maintenance/command`. This is typed bounded JSON, not ROSIDL.
Commands are at most 4096 UTF-8 bytes; responses are bounded to 16384 bytes in
the client. The server queue holds at most 16 commands and consumes at most four
per turn. Oversized/overflow messages may be dropped without an ACK.

The exact schema-1 command fields are `run_id`, `mission_id`, `action_id`,
`command_id`, `op`, `arguments` and `schema_version`. Isolation arguments are
`machine_id`, `station_run_id`, `control_revision` and `timeout_s`. Cancel/query
have empty arguments. UUID hex identities, integer revisions, finite numbers,
allowlisted operations and exact fields are required; duplicate JSON keys fail.
The request digest excludes only `command_id`. Conflict, wrong run/revision,
busy owner and full history refuse admission. Exact replay returns history;
history never evicts an action for reexecution.

State publishes at 10 Hz with owner/station runs, control revision, advancing
sequence, simulation stamp, readiness, capacity, active action and last result.
The client requires two advancing simulation states and fresh receipts. A paused
stamp cannot renew readiness. Source/run/revision regression is fatal to that
client. Cached terminal results remain history, never renewed freshness or
permission for the next operation.

## Admission and effect

1. Reserve the exact action and a random hold token before publishing acquire.
2. Require the guard to acknowledge that token and remain latched. The supervisor
   invalidates/cancels navigation and refuses requests, arm and velocity
   forwarding. It reports quiescence only after its target, pending arm and exact
   action ownership resolve, with fresh healthy stationary feedback.
3. Start the actual inspection child under the station lifetime lock. Keep its
   ownership until reap. The parent retains its own stopped-pose history while
   the child captures and stores the actual RGB/manifest bundle.
4. Verify that bundle, request context, registry/map/publisher identity and
   station run/revision. Recheck live pose/settling and both clocks after I/O,
   immediately before the synchronous effect. Existing 200 ms pose/camera gates
   and the 0.5 s settled window are unchanged.
5. Apply isolation on the published station, increment its control revision and
   record the exact applied identity. An unexpected actuator exception is not
   evidence of no mutation.
6. Read a new consecutive telemetry pair through the station's own subscription.
   Require the same run, exact applied record, a strictly later sample stamp,
   and target `powered=false`, `current_a=0`, `vibration_au=0`. Recheck both
   telemetry ages below one second at completion. There is no hidden fault read.
7. Request exact hold release, await fresh guard and navigation confirmation,
   then publish the terminal result. The guard remains latched after release;
   only a later explicit reset and new navigation command can permit motion.

The guard hold never expires because the maintenance process or client went
away. Guard reset is refused while held. Release needs matching identity and
fresh stationary guard feedback. A release before its delayed acquire creates
a permanent cancellation record; that acquire cannot resurrect ownership.
Histories are bounded to 64 entries per process run.

The station samples and mutates in one executor. File I/O precedes final live
admission; sampling cannot interleave inside an effect. Durable mode records an
intent before final live interlocks and an outcome after mutation, without claiming
an atomic transaction spanning the model effect and filesystem or physical power-loss
recovery. Sources are pinned
by sampled names, namespaces, types and endpoint GIDs. These are consistency
checks on trusted local ROS, not message authentication or real-time guarantees.

## Cancellation and uncertainty

Before commitment, sticky cancel/deadline prevents the effect but still awaits
capture reap and exact hold release. After commitment, cancel does not undo
isolation: fresh verification precedes release and the terminal records both
`status=cancelled` and `state_change.disposition=applied`. An expired deadline
similarly retains unresolved ownership. No path automatically restores power or
retries the actuator.

`unresolved` is nonterminal. Unknown actuator outcome, changed station run,
expired observations or lost hold evidence may retain the action and external
inhibitor indefinitely. There is no forced release or restart adoption command.
The durable history CLI provides retained records for diagnosis; it grants no live
permission or resumed result. Preserve identities and evidence before proceeding.
Do not kill/restart components to turn an unknown action into success. Durable
mode refuses clock discontinuities; explicit new worlds require new mission/action
IDs and cannot establish an old mission outcome.

A known terminal result includes action/request IDs, machine/station run,
status/error, times, inspection artifact reference, `hold_released=true`
and `state_change`. An applied change includes the exact effect plus newer
telemetry run/sequence/stamp, control revision, public sample digest and measured
isolated values. `not_applied` has no effect or verification. Unknown effects
produce no terminal result. The digest is a reference, not a retained complete
trace. The [independent recorder](ISOLATION_EVIDENCE.md) now retains source packets
for isolation replay; it explicitly refuses reset results as isolation evidence.
Reset results use schema 2 with the prior isolation result and powered eligibility
references, plus temperature in their post-effect verification. Existing isolation
results keep schema 1. OM-060's [sequence recorder](MAINTENANCE_SEQUENCE_EVIDENCE.md)
adds independent local reset/restore/cancellation replay; native acceptance remains open.
Restore results use schema 3 with their prior reset chain and actual energized
measurements. Cancel after restore can report a verified applied effect with
power on; it is not a rollback or an isolation claim. Successful restoration is
separate from observed station recovery.

## Startup, shutdown and validation

The existing demo launcher starts this station owner. Telemetry can begin before
navigation/map/camera readiness; maintenance remains unavailable. The station
uses `runtime/station-service/owner.lock`, held by the parent and inherited by
its capture child. Ordinary shutdown cancels and reaps that child before closing
the lock. An unresolved external hold survives loss of its station owner.
The demo cgroup remains the descendant containment boundary. Guard restart
creates a new guard run; no old owner may adopt it silently.

Controls cover replay/capacity, delayed arm, non-expiring holds, early-release
tombstones, source loss, stale evidence, actual post-effect exceptions,
cancel before/after mutation, paused clients and schema validation. The composed
fixture sends actual parent station samples to the separate real capture worker,
rather than giving it another model. A shared test clock freezes during capture
teardown and resumes for post-effect sampling; this isolates ownership from host
scheduling. Production uses real monotonic time. These tests do not establish
native scheduling margins or DDS behavior.

Before deployment, recheck Thor/source/service ownership and pending
navigation/inspection gates. Commission guard/supervisor/station together on an
empty isolated domain, then exercise the three actual control poses and native
fault/cancellation/source-loss sequence with retained evidence. Rebuild source
manifests; their enumeration includes these new Python modules.
