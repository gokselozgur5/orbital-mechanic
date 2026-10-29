# Durable maintenance ownership

OM-057 binds the live station to a retained scenario claim and action journal.

A station-process restart cannot create a fresh instance of the old mission or renew its reset/restore budget.

This extends the existing in-process replay checks; it does not count those checks as a second implementation.

## Starting a scenario

The supported full-world entry point remains:

```bash
bash scripts/launch-thor.sh
```

The launcher first requires the previous demo service to finish cleanup.

Before spawning any component, `start-demo.sh` now runs `maintenance-journal.py prepare --new-world` under the station owner lock.

That explicit full-demo start prepares a new opaque scenario generation, then starts a new Gazebo world.

The station consumes that generation exactly once, binding its owner run, station run and current registry identity.

Starting only `machine-telemetry.py` without a prepared generation, or restarting it against a claimed generation, fails before publishing station measurements.

A clean shutdown also consumes the claim; it does not authorize adoption of old state on restart.

Previous mission and action IDs remain reserved in this journal across new scenarios.

A new world therefore requires new mission/action IDs, while a fresh command ID alone cannot bypass retained history.

The operator `prepare --new-world` command asserts that a new full world will be started; it is not a recovery command for a surviving world or a claim that Gazebo is stopped.

Use the service launcher for normal operation and preserve its containment/preflight requirements.

Standalone tests explicitly prepare their own fresh scenario fixtures.

A backward clock jump or a forward gap above ten seconds now refuses the durable station instead of rebuilding its causal model and clearing attempts.

The ROS-free model retains its explicit prototype reset behavior when no live durable owner is attached.

Deploy this change together with the launcher and client sources during a deliberate new-scenario commissioning; do not introduce it as a transparent restart of an old mission.

## Retained boundaries

The journal lives at `runtime/station-service/events.jsonl`, outside Git.

| Record | Meaning |
| --- | --- |
| Scenario | An explicit new-world generation was prepared against a registry identity. |
| Claim | One owner/station generation consumed it; another claim is refused. |
| Admission | Exact request, mission and action identity were retained before hold acquisition. A cancellation also reserves its child identity before stopping the original. |
| Effect intent | One named attempt was retained before the final live checks and actuator call. An intent alone never proves application. |
| Applied / rejected | The synchronous call returned a known effect, or a known rejection before application. A missing outcome remains unresolved. |
| Terminal | A complete historical result was retained after known effect verification and exact hold release, before result publication. |

Reset and restore reservations are independent of the model's in-memory counters.

A known, durably recorded rejection before application can release that reservation; an unknown call or failed write cannot.

The final registry, pose, hold, freshness and deadline checks run after intent persistence and before mutation.

A storage delay can therefore prevent an effect rather than spending stale permission.

The terminal decision rechecks its deadline and postcondition after hold-release I/O.

Terminal timestamps describe that decision, before disk persistence and delivery; a delayed append or later query never renews the observations.

A terminal record proves a historical decision, not that a client received the result or that the measured state is still current.

The cancellation workflow keeps original and child records separate and validates both against their retained history.

## Unknown outcomes and operator inspection

Read retained records without starting ROS or changing the journal:

```bash
python3 scripts/maintenance-journal.py status
python3 scripts/maintenance-journal.py query --action-id ACTION
```

These responses carry `historical_only=true` and grant no actuation permission, live process claim, fresh measurement, hold release or restart authority.

Status reports bounded unresolved records and explicitly indicates truncation of that display.

Admission without an effect intent, intent without an outcome, and an applied effect without a terminal record remain distinct.

Even when a crashed process actually changed the model, an absent outcome record cannot be reconstructed from that assumption.

A failed or ambiguous journal write stops further admissions/effects and retains unresolved action ownership.

If ordinary hold release already completed before terminal persistence fails, that historical release is not repeated or described as a forced release.

The server does not publish a newly invented terminal result or retry a failed append.

The added typed rejections are `mission_reused`, `action_reused`, `journal_full` and `journal_unavailable`; matching client/server versions are required, and older strict decoders fail closed.

No API adopts old camera files, restores a stored machine model or resumes an interrupted mission.

A new explicit world with genuinely new IDs is a separate scenario; it does not establish the old mission's outcome.

## Storage and limits

Writes use the existing exclusive station owner lock, also inherited by capture children.

Directory-relative descriptors reject followed links, non-regular or multiply linked files, changed directory/file identities and non-private journal storage.

Each bounded canonical record carries a sequence and chained digest.

Partial, malformed, reordered or inconsistent histories refuse startup/preparation; no implicit truncation, eviction or repair creates permission.

The limits are 16 MiB, 8192 events, 256 scenarios and 32 KiB per record, with the existing 64 action/reservation slots per owner and bounded admission headroom for outcomes.

Quota exhaustion requires operator attention; this implementation provides no journal reset or destructive cleanup command.

The implementation uses descriptor writes followed by `os.fsync` and syncs the containing directory when creating the file; exclusive nonblocking ownership uses `fcntl.flock`. ([Python fsync](https://docs.python.org/3.11/library/os.html#os.fsync), [Python flock](https://docs.python.org/3.11/library/fcntl.html#fcntl.flock))

The digest chain is an integrity check, not authentication or protection against coherent rollback, deleting the journal, a different checkout or a malicious same-user writer.

Keep the retained journal with its station workspace; copying only source code does not transfer its historical authority.

A missing/corrupt journal is not permission to recreate old missions under reused IDs.

## Validation boundary

Local tests run actual station/owner code and real filesystem operations, with controlled robot/camera inputs.

Five real owned subprocesses are killed after admission, after intent, inside the actual mutation before its outcome record, after the applied record, and after the terminal record before publication.

Each cut preserves history, refuses a station-only restart and refuses old mission/action IDs in a new explicit scenario; a genuinely new mission can complete.

Controlled partial writes, sync failures, path/link/permission changes, capacity, clock discontinuity and delayed persistence cover refusal paths.

Composed station/guard/navigation tests also verify that results from separate actual camera children match retained terminal records.

These are local process-crash and controlled-transport checks, not native DDS, surviving-world recovery, filesystem-snapshot rollback resistance, physical power-loss guarantees, electrical isolation or full mission acceptance.

## Native startup proof

On 2026-10-29, `scripts/check-station-lifecycle.py` passed six controls on Thor
ARM64, ROS 2 Jazzy and Python 3.12.3. Nine real telemetry, observer and journal
CLI children used an empty-checked localhost domain 84 and a synthetic clock:

- An unprepared generation exits before publishing station measurements.
- A prepared station supplies two fresh public schema-2 samples with the actual
  registry identity, no applied action and no hidden fault/scenario fields.
- A graceful station-only restart refuses the consumed generation, preserving
  journal bytes and publishing no new measurements.
- An explicitly prepared fresh fixture uses new station/scenario identities
  while retaining both generations and zero mission/action records.
- A five-second backward clock jump terminates the actual station.
- The real observer then refuses to return a fresh measurement.

Each preparation represents a new isolated model fixture, not a surviving Gazebo
world. The bounded disposable user-service cgroup exits; all 159 staged source
pins and the live demo/localization/navigation inventory remain unchanged. This
proves native startup and observation behavior, not maintenance effects, physical
stopping, route acceptance or recovery of a running world.

Evidence: operator workspace
`reviews/orbital-navigation-rollout-20261029/station-lifecycle-evidence`.
Its 30-file manifest SHA-256 is
`2352761c5cfe5dbe6ef28d24db597721f175814fce64560e3a50e913cde2245e`;
the driver SHA-256 is
`30c50b6047c1c256e991786a29ee66ef012b29eef660fae288ae269c290315a0`.
Use a fresh staged checkout under the owned runtime root and a bounded cgroup;
the driver requires domain 84/LOCALHOST and refuses an existing proof directory.
