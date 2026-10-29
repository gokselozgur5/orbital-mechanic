# Public inspection and diagnosis policy

OM-062 adds `orbital.mission_policy.MissionPolicy` to the existing mission transition contract.

Inspection order and diagnosis form one delivery; the OM-063 policy and local OM-064/065 controls reuse this implementation without additional delivery credit.

The policy consumes public station packets and actual verified inspection artifacts; it neither receives hidden scenario labels nor calls a maintenance actuator.

Its version is `public-measurements-v1`, specific to the documented simplified station model.

## Inputs and evidence

Construct one policy around the active `MissionState`, using the same registry and actual monotonic/simulation clocks.

Feed each received packet once through `observe_telemetry`; it validates the public schema and supplies the same packet to mission recovery.

`next_machine()` requires two consecutive current public packets, the expected station generation/control revision and unchanged registry source bytes.

The controller navigates to that machine and obtains a successful inspection through the existing clients and owners before calling `apply()`.

`apply()` reads the real observation manifest and RGB file with the existing bounded artifact reader.

It binds the owner, mission, action, request digest, registry, machine geometry, source hashes and capture timestamps to the successful result.

The selected telemetry packet must still exist in the policy's bounded source history; an artifact cannot reconstruct a missing packet.

All selected metadata, units and measurements must match that packet, including JSON types.

The selected sample and latest consecutive pair must remain younger than one second in both clocks after artifact reads and hashing finish.

Cancellation, phase changes, altered registry bytes or changed control during that work prevent a decision.

The image is verified as evidence attached to the inspection; this policy does not classify pixels or infer visual damage.

## Order and hypotheses

An explicit ambiguity retry takes priority; otherwise choose the eligible machine with greatest measured electrical load, then prefer an unconfirmed/unvisited machine and catalog order for ties.

Each machine gets at most two inspections and the whole mission at most six.

Thresholds reuse existing recovery/reset gates: minimum bus voltage 27 V, maximum normal machine power 110 W and maximum reset-eligible vibration 1 a.u.

No new physical calibration is claimed.

| Public observations | Hypothesis and next step |
| --- | --- |
| All machines powered, bus at least 27 V, each load at most 110 W and vibration at most 1 a.u. | Inspect all three machines before selecting `healthy`; then require the existing new ten-second recovery window. |
| One machine above 110 W, bus below 27 V, all other machines at low vibration | Select that machine as candidate; inspect it before permitting any intervention. |
| The inspected candidate also has vibration at most 1 a.u. | `recoverable` hypothesis; the mission may request the owned isolate/reset/restore sequence, whose independent live gates still apply. |
| The inspected candidate has vibration above 1 a.u. | `persistent` hypothesis; select escalation without resetting. |
| Unpowered station, multiple excess loads, inconsistent load/voltage/vibration, or changing selected/current classifications | Request another bounded inspection or escalate with explicit uncertainty. |

The selected artifact sample and latest two packets must agree on the complete classification.

A powered high-vibration observation remains sticky for that machine throughout this policy instance; later low readings cannot erase it and authorize recovery or declare the station healthy.

These are public model hypotheses, not hidden-cause attribution or repair proof.

An escalation records that the mission needs an operator; it does not assert physical stopping, electrical isolation or a safe powered state.

## Bounds and outputs

The policy retains at most 64 raw packets, six consumed action identities and one high-vibration reference per machine.

Eviction of a selected capture packet causes refusal even if that sample would still be fresh by time alone.

Invalid telemetry or unavailable inspection evidence latches failure; restoring a changed file does not silently clear it.

Reports identify the policy version, mission, registry, station/control, inspected machine/action/artifact, exact sample hashes, historical vibration references, criteria and uncertainty reason.

Snapshot copies cannot mutate the active plan, history or budgets.

The caller must retain the public input stream if later audit needs the raw packets behind historical references; this bounded in-memory policy is not a durable event recorder.

## Validation and integration limits

The focused tests run the actual capture adapter, artifact storage/reader, inspection client/owner and durable station model over controlled ROS peers.

The capture adapter performs its real map, pose, settling, camera and calibration checks; its worker lifecycle, navigation completions and maintenance camera/hold feedback remain controlled test inputs.

Five retained sequences reproduce healthy, recovered, persistent, ambiguous and blocked outcomes through the same state/policy code, including the actual captured files and exact gateway results.

This proves reproducibility, not independent scenario scoring, native ROS transport, a camera-child mission or collision-free navigation.

Controls cover other candidate machines, hidden-label changes without new observations, contradictory history, changing candidates, global budgets, coherent manifest tampering, missing/evicted samples, image mutation and expiry/cancellation/source changes during artifact I/O.

The [persistent runtime](MISSION_RUNTIME.md) now integrates the policy with actual
clients, durable intent and bounded exact cancellation. Interrupted-mission
reconciliation, complete cancellation containment, independent evaluator access
separation and full native scenario acceptance remain open under #10/#11/#12.
