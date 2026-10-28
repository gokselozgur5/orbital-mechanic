# Bounded navigation command client

`scripts/orbital-navigate.py` provides `status`, `request`, `arm` and `cancel`
through the supervisor's existing run/sequence and exact-target protocol. OM-022
is implemented and tested locally. Native DDS commissioning is still pending;
no route, physical stop or machine arrival is accepted by these client tests.

## Operating sequence

After native commissioning, source/deployment verification and a passing guarded
readiness gate, run on Thor inside `tmux orbital-work`, in the project root.
The OM-021 map revision must already be loaded and verified; the client does not
start services, reload localization or activate Nav2.

```bash
source scripts/ros-env.sh
/usr/bin/python3 scripts/orbital-navigate.py status
mkdir -p runtime/operator
/usr/bin/python3 scripts/orbital-navigate.py request pump > runtime/operator/pump-request.json
```

Use an unused receipt filename and preserve it. `request` validates the local
inspection geometry and reserves a target without arming. Inspect its JSON
receipt: it must say `delivery: acknowledged`, `accepted: true` and contain the
requested machine's target. Available names are `pump`, `cooler`, `power_unit`.

Starting that exact target requires a separate explicit command:

```bash
/usr/bin/python3 scripts/orbital-navigate.py arm --target-file runtime/operator/pump-request.json
```

**An armed goal can continue after this client exits.** The supervisor owns the
execution deadline (120 seconds by default, configured separately within its
0.1–300 second bound), health/guard checks and action cancellation. Ctrl-C ends
the client's wait; it does not prove that a command was undelivered or stop an
already accepted goal. Armed acceptance trials still require the independent
trial guardian described in [NAV2_INTEGRATION.md](NAV2_INTEGRATION.md).

Observe progress or explicitly cancel the same target:

```bash
/usr/bin/python3 scripts/orbital-navigate.py status
/usr/bin/python3 scripts/orbital-navigate.py cancel --target-file runtime/operator/pump-request.json
```

`--target` also accepts a literal JSON object with exactly `run_id`, `epoch`,
`sequence`, `waypoint`. The client never silently selects a current target from a
status snapshot. A stale token is refused. A completed or failed request receipt
cannot be reused to select another target. Cancel remains available when
localization is unhealthy or the local map config has become invalid, provided
fresh supervisor state and the exact current target are still available.

## What the result proves

| Result / exit | Meaning |
| --- | --- |
| `observed` / 0 | A recent supervisor state was received; `status` creates no command publisher. |
| `acknowledged` / 0 | The matching run/sequence/operation was accepted and its target matched. |
| `rejected` / 1 | The matching supervisor reply explicitly refused the command. |
| `not_sent` / 2 | This invocation did not enter command publication. Argument errors also exit 2. |
| `uncertain` / 3 | Publication was attempted but a valid matching acknowledgment was not established. |
| interrupted / 130 | Waiting was interrupted; JSON distinguishes `not_sent` from `uncertain`. |

An arm acknowledgment can precede the guard response and Nav2 dispatch. A cancel
acknowledgment can precede a terminal action and physical stopping. The receipt
reports the observed `action_state`, `guard_latched` and `last_completion`; it
does not turn acknowledgment into arrival or stopping evidence.

After an uncertain result, inspect state before a new explicit command. The
client never resends automatically. For an immediate stop outside this protocol,
the existing `scripts/orbital-operator.py stop` command explicitly latches the
motion guard; that acknowledgment also needs separate measured stopping evidence.

## Bounds and ownership

`--timeout` goes before the operation, for example `--timeout 3 status`. It is a
finite 0.1–10 second monotonic deadline covering ROS discovery and acknowledgment
after argument/config preflight and ROS construction. Received state must be
younger than 0.3 seconds. This bounds the client's wait, not the mission duration.

Each invocation publishes at most once to `/navigation/command`, with no direct
velocity, action-server or guard-arm interface. It pins one supervisor run and
refuses regressed/advanced command sequences, changed target ownership, malformed
JSON and mismatched acknowledgments. Request/arm require the appropriate healthy
idle/waiting state and a latched guard; the supervisor independently owns actual
stationarity and guard acknowledgment checks.

A local advisory lock serializes command clients in the same checkout/domain.
Sampled graph checks require one expected state publisher, this client's sole
command publisher and the supervisor's sole command subscription. Discovery
placeholders may resolve within the deadline; wrong or duplicate peers fail.
These checks are not authentication or a cross-host transaction. Trusted local
ROS processes remain the deployment assumption. Domain 42 must use LOCALHOST;
domain 84 is reserved for an independently verified isolated fixture.

## Evidence and next native gate

All 164 local tests pass, including 37 new protocol and CLI tests. Controlled
transport drives the actual CLI adapter and production supervisor callbacks for
request/arm/cancel, stale-token refusal, competing publishers and discovery.
Negative cases cover missing/late replies, restart, sequence races, interrupted
publication, malformed input, changed pre-publication state, wrong environment,
local lock contention and geometry refusal before ROS. Cancellation through an
invalid local map config remains accepted by the production callback fixture.

The callback tests deliberately retain a pending arm response and show that the
client can acknowledge intent without any Nav2 goal dispatch. They are not native
DDS, service-lifecycle or physical-motion evidence. Once Thor access returns,
commission real CLI subprocesses in empty-checked domain 84 with production
guard/supervisor, synthetic health and a goal-rejecting action stand-in. Verify
receipt/exit semantics, no duplicate publication and negative ownership cases
before using the CLI in a guarded physical route trial.
