# Bounded reset on the published station

OM-054 extends the [owned maintenance endpoint](MAINTENANCE_ENDPOINT.md) with
one reset attempt after a verified isolation. The same station executor owns
sampling, replay history and mutation. A reset application is observable control
state; it does not prove fault clearance, successful repair or recovery. Power
stays off. OM-055 now supplies [owned restoration](RESTORE_OPERATION.md) as a
separate explicit action. OM-057 supplies [durable restart refusal and historical records](DURABLE_MAINTENANCE.md); live mission recovery remains open.

## Request and eligibility

`reset` requires a machine ID, current station run/control revision, finite
0.1–30 second deadline and `isolation_action_id`. The CLI requires explicit
maintenance `--run-id`, `--mission-id` and `--isolation-action-id`; it may generate
a new reset action ID. The prior isolation must be a terminal, verified applied
action belonging to that same owner, mission, station generation and machine.
A cancellation after verified isolation can be such a prerequisite. An ACK,
unknown outcome, another owner or a different mission cannot be adopted.

Before isolation, the owner retains two consecutive fresh public packets from
the pinned source and registry, at the inspected control revision. Their target
must be powered, with `voltage_v * current_a > 110 W` and vibration at most
1 arbitrary unit. At least one retained packet must exactly match the inspection
digest, timestamp and measurements. These are illustrative model constants,
not calibrated hardware thresholds or a hidden fault classification.

High powered vibration is sticky for that machine. Malformed, missing, reordered,
run-changed or contradictory source/control history prevents reset eligibility;
an observation gap of at least one second also prevents it. Later isolated zero
vibration cannot erase earlier powered evidence. A healthy or jam scenario
cannot establish eligibility. Missing eligibility does not prevent isolation.

The source adapter pins station/guard/navigation publisher identities before
retaining packets. GIDs and hashes are consistency evidence on trusted local ROS,
not authentication. Source generations cannot silently replace that history.

## Hold, cooldown and pulse

Reset acquires a new exact non-expiring drive hold and waits for navigation
ownership to resolve. While holding the stopped pose it waits for fresh isolated
public measurements with temperature at most 40 °C. Hot measurements do not
start the capture child. The caller's deadline includes this wait; timeout or
cancel before the pulse preserves the unused attempt.

Once cool, a new capture child obtains actual RGB/manifest and pose evidence for
this reset action. The parent reaps it, validates the artifact and rechecks live
source, pose, hold, telemetry, cooldown and the existing 200 ms freshness gates
after I/O. The actual station independently rechecks isolation and temperature
before mutation. No request supplies its own geometry or power evidence.

The station reserves the machine's one attempt immediately before the synchronous
pulse. A known mutation-free model rejection can release that reservation; an
unexpected exception retains it even when `reset_count` did not advance. Another
action or owner cannot use an unknown attempt again. Successful exact replay
returns history without another gate or pulse. OM-057 additionally persists scenario,
mission/action and attempt boundaries in the live station. A station-only restart
refuses publication; a new explicit world requires new mission/action IDs.
Historical records do not adopt an old mission or establish power-loss recovery.

## Verification, cancellation and limits

After the pulse, two new consecutive public packets must carry the exact reset
identity and revision, strictly later stamps, continued isolation, zero current
and vibration, and temperature at most 40 °C. Both samples must remain younger
than one second in receipt and simulation time, including while awaiting the
release acknowledgement. The owner rechecks freshness before
exact hold release. Cancellation after commitment waits for this verification
and reports both the cancellation and applied reset. It never restores power.
Unknown effects, missing verification or lost ownership remain nonterminal and
cannot authorize a new release. If release was already sent, later evidence loss
cannot prove the token is still held; the owner stays unresolved and the drive
latch remains required. There is no forced release or blind retry.

Reset terminal results use schema 2 and contain the validated prior schema-1
isolation result, compact powered eligibility references and measurements, new
inspection reference and exact effect/verification. Nested reset prerequisites
are forbidden. Command/state envelopes remain schema 1 and public telemetry
remains schema 2; isolation results remain unchanged. This historical result is
not current permission or a retained independent source trace. The isolation
recorder cannot verify a reset. OM-060's separate
[sequence recorder](MAINTENANCE_SEQUENCE_EVIDENCE.md) now resolves reset/restore
and cancellation evidence to actual retained public packets locally; native acceptance remains pending.

Local controls use the actual station/guard/navigation code, separate real
capture child and artifact reader over controlled ROS transport. They cover
two inspections, cooldown/deadline, missing/contradictory evidence, sticky high
vibration, replay/budget, final freshness, cancellation before/after the pulse,
unknown mutation and exact post-effect measurements. Deliberate disabled-gate
copies check test sensitivity; they are not historical production defects.
The cooperative fixture clock is unchanged; production uses real monotonic time.

Run `python3 -m unittest discover -s tests -v` and the causal baseline. Exact
source-bound logs and negative controls live in operator workspace
`reviews/orbital-delivery-20261028/om054`. Native DDS/timing, three-machine
commissioning, physical electrical isolation, fault clearance and full maintenance
acceptance are not established by these local controls.
