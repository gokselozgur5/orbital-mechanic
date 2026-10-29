# Owned power restoration

OM-055 adds `restore` to the [maintenance endpoint](MAINTENANCE_ENDPOINT.md).
It energizes the same published station after an exact verified reset, current
cooldown and a new inspection. A successful restore means the commanded machine
was observed powered on. It does not mean the station recovered or a fault was
cleared. Those outcomes require separate fresh observations.

## Admission and one restoration attempt

The CLI requires a machine, explicit maintenance-owner run and mission, and
`--reset-action-id`. It discovers the current station run/control revision and
generates a new action ID unless supplied. The finite deadline is 0.1–30 seconds.
An isolation action, an unapplied cancelled reset, another owner/mission/machine,
an old station generation or an unresolved result cannot authorize restoration.
A verified applied reset remains usable after cancellation only through a new
explicit restore request and all current interlocks.

The owner retains the reset's eligible powered history. Missing or contradictory
source history and prior high powered vibration remain refusals. New zero
vibration readings while isolated cannot erase that refusal. A historical
successful reset grants no live pose, source, temperature or hold permission.

The new action acquires its own exact drive hold, waits for navigation ownership
to resolve, and observes the isolated target at temperature at most 40 °C.
Cooldown uses the action deadline. Then a third live camera capture in the
isolate/reset/restore sequence produces a new request-bound RGB/manifest bundle.
The owner reaps the child, checks the artifact and rechecks source, pose,
stationarity, hold, current telemetry and 200 ms freshness after I/O.

The actual station independently validates the prior reset's owner, mission,
machine and operation before final admission, then rechecks isolation and
temperature. It reserves one restoration attempt before energizing. A known
mutation-free rejection may release the reservation; an unexpected exception
retains it even if power appears unchanged. Exact replay returns history without
another call. Another ID, owner or later isolation cannot reuse the old reset to
obtain another attempt. [Durable scenario/action history](DURABLE_MAINTENANCE.md)
now refuses station-only restart, old mission/action reuse and clock-triggered
model reconstruction. Historical records never authorize resumed restoration.

## Applied effect and cancellation

The station commits a new schema-2 control revision and exact restore identity.
Two strictly later, consecutive packets must carry that effect and show the
target powered on. Both members must remain younger than one second in receipt
and simulation time through release acknowledgement. The terminal retains the
measured voltage, current, vibration and temperature. These values are not
replaced with healthy constants, and unhealthy values do not erase a real
energization. Exact hold release follows verification; the drive stays latched.

Cancel or deadline before commitment prevents energization and preserves the
unused attempt, while still waiting for capture reap and release. After
commitment the result can be cancelled/timed out **and applied with power on**.
Cancellation never silently rolls back power or claims safe isolation.
Compensating isolation, if required, must be another identified action under
current interlocks; it is not part of this operation.

OM-056's explicit [cancel-isolate workflow](CANCELLATION_OPERATION.md) now owns
that separate sequence. Ordinary restore/cancel behavior is unchanged.

Unknown actuator outcomes, missing verification or lost ownership stay
nonterminal and cannot authorize a new release. If release was already sent,
later observation loss cannot prove the token remains held. Preserve that
uncertainty and the drive latch; there is no forced release, adoption or retry.

Restore results use schema 3 and include the prior schema-2 reset result, which
contains its schema-1 isolation and powered eligibility. Operation/version gates
bound this chain to three records; recursive restore prerequisites are refused.
Command/state envelopes and public telemetry keep their existing versions.
The isolation recorder does not verify restoration or full sequence recovery.

## Recovery and evidence limits

The existing `RecoveryEvidence` independently requires all machines powered,
at least 27 V and at most 110 W per machine for a full consecutive ten-second
simulation window. The controlled composed test observes the actual station
after restoration, binds each packet to the exact applied identity/run and
passes that checker without using hidden fault truth. A separate actual-model
control injects a jam after energization: restoration is observed, but recovery
is never verified. This reuses OM-051; it earns no duplicate implementation credit.

The endpoint's restore result does not itself collect that recovery window or
provide a mission-level recovery API. Native sequence traces, independent
causal evaluation, recovery orchestration and native restart acceptance remain open.
No successful reset/restore result alone authorizes a claim of repair.

Local tests run the actual station, guard, navigation supervisor and three real
capture children with controlled ROS transport and the existing cooperative
clock. Cancellation and expiry use the real artifact reader. Unit controls cover
ownership, cooldown, budgets, late loss, unknown outcomes, replay and paired
freshness. Deliberately disabled-gate copies check sensitivity; they are not
historical bugs. Native DDS/timing, three-machine commissioning and physical
electrical safety remain unverified. Source-bound validation lives in operator
workspace `reviews/orbital-delivery-20261028/om055`.
