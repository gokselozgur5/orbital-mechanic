# Same-time machine inspection observations

OM-047 adds a read-only, target-specific record that binds public machine
telemetry to a localized, settled robot pose at the telemetry sample's exact
simulation timestamp. One live ROS observer acquires both kinds of evidence.
This is local software integration toward issue #8. Optional
[camera capture](CAMERA_OBSERVATIONS.md) now extends it locally; the common
inspection action envelope and native commissioning remain pending.

After commissioning the owned Orbital runtime and sourcing ROS:

```bash
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 scripts/machine-observation.py pump --timeout 5
```

The other allowed targets are `cooler` and `power_unit`. The timeout is finite
and bounded from 0.1 to 10 seconds. Optional `--with-camera` selects schema-2
local capture, described separately. There are no caller-supplied poses,
telemetry files, clocks, paths, topics or tolerance overrides. Domain 84 is for a
controlled fixture after verifying its graph is empty. The library entry point
is `orbital.ros_proximity.read_machine_observation`.

## Acquisition and invalidation

The observer shares the actual map, health, odometry and TF acquisition used by
[the settled-pose reader](PROXIMITY.md). It also subscribes to the existing
public telemetry through [the strict station decoder](STATION_OBSERVATION.md).
Before telemetry admission it pins a complete publisher snapshot, including the
unique expected clock and telemetry publishers and their nonzero endpoint GIDs.
It requires a positive simulation clock and maintains `use_sim_time=true`.

1. Validate registry and full map geometry before starting ROS observation.
   This preparation remains inside the overall timeout.
2. Observe health and direct odometry until accepted stopped samples span at
   least 0.5 seconds in both monotonic receipt time and simulation time. Initial
   movement can delay this window; once it completes, pin its generation and
   the current simulation stamp. Any later invalidation rejects the query,
   including movement while waiting for telemetry followed by another stop.
3. Require two advancing valid telemetry samples in one run, with a consecutive
   final pair. Select only a sample whose source timestamp is at or after the
   pinned settling stamp and no more than 200 ms old. Earlier samples cannot
   satisfy this condition merely because the robot later settles.
4. Freeze that exact canonical public packet and timestamp. Collect the actual
   live map service/retained data and consumer parameters using the prepared
   source bundle. Map collection follows sample selection, so waiting for the
   nominal 2 Hz telemetry does not spend the map observation's freshness lease.
5. Request `map` to `base_footprint` TF at exactly the selected source stamp.
   Validate frame/time, distance, configured inspection position and heading,
   fresh health, the retained stopped history and all source/map identities.
   Fresh odometry that is slightly behind this stamp may catch up within the
   same fixed sample lifetime. No latest-TF or alternate-time fallback exists.
6. Prepare the result identity before final source, publisher, clock,
   generation, freshness and deadline checks. Return one complete record only
   while every required gate still passes.

The joint pose/sample age bound is inclusive at 200 ms in simulation time;
standalone telemetry also retains its stricter-than-one-second receipt and
simulation age checks. The map observation must remain younger than 0.5 seconds
in receipt time. Odometry and health retain their existing independent bounds.
These constants are not user-configurable through this reader.

After selection, a different accepted telemetry packet, even a valid newer one,
invalidates this query. Missing TF can be retried only at the pinned timestamp
while that sample remains eligible. Sample expiry, malformed/replayed/reset
telemetry, localization generation changes, clock regression, motion or changed
source/map/publisher identity cannot be hidden by resettling or silently selecting
another sample. A new query is required after rejection.

## Record and CLI contract

Successful JSON has outer `schema_version: 1`, `valid: true` and
`kind: machine_inspection_observation`. It includes:

| Field | Meaning |
| --- | --- |
| `observation_id`, `authority_identity` | Unique record ID and digest of the sampled publisher inventory. |
| `machine_id`, `machine_binding` | Selected registry target and its public scene/control geometry. |
| `registry_identity`, `source_sha256` | Registry identity and configured source hashes. |
| `sample_stamp_ns` | Absolute simulation timestamp of the selected telemetry and requested robot TF. |
| `observed_sim_stamp_ns`, `settled_at_sim_stamp_ns` | Final observed clock and the stamp when the initial settling condition was pinned. |
| `measurements`, `units` | Bus voltage plus only the selected machine's public power, temperature, current and vibration measurements, with existing declared units. |
| `telemetry_source` | Original wire schema/source/run/sequence/stamp/model time and canonical packet SHA-256. |
| `telemetry_freshness` | Receipt/simulation age, accepted-pair count and missing-sequence count. |
| `robot_state` | Full schema-3 proximity/pose/stationarity report measured at `sample_stamp_ns`. |
| `map_binding`, `map_observed_age_s` | Actual validated map binding and its receipt age. |
| `scope` | Explicit limits of this read-only observation. |

`telemetry_source.time_s` remains elapsed model time, not the absolute sample
clock. Its `sample_sha256` hashes the full canonical public wire packet, while
this record intentionally returns only the selected machine's measurements.
The hash supports correlation with a retained original packet; it cannot be
recomputed from the target-only subset and is not an authenticity proof.
Returned nested objects cannot mutate the observer's cached evidence.

Exit 0 means a complete observation passed these checks. Evidence/runtime
failures return exit 2 and JSON `valid: false` with an error and no partial
measurements. Invalid arguments use parser diagnostics. Ctrl-C returns 130 with
invalid JSON. The shared node, listener and ROS context are cleaned up on
success, failure and interruption. The timeout bounds accepted results rather
than guaranteeing hard real-time termination of a stalled runtime call.

There is no camera observation, diagnosis, confidence score, maintenance action,
stop acknowledgement or held motion interlock in this record. Sampled stopped
odometry is not continuous physical immobility. A stored result grants no later
action permission. Publisher inventories are sampled consistency checks, not
per-message authentication; source hashes do not prove the loaded simulator's
memory. Hidden evaluator state and simulator truth are not observation inputs.

## Validation and remaining acceptance

Focused tests execute the production evidence core, combined reader, CLI and
actual map-binding adapter with controlled ROS transport. The joint fixture
uses the production `MachineTelemetry` core; the separate station-reader tests
also execute the shipped telemetry publisher. Validation decisions are not
mocked. Coverage includes all three targets, exact sample-time TF, delayed
post-settling samples and odometry, movement before/after selection, malformed
and changing telemetry, stale samples, wrong pose/map/frame, source/GID/clock
changes during result preparation, deadlines, interruption and cleanup.

These checks do not commission native DDS discovery, callback timing or the
complete inspection action. Recheck Thor ownership and the exact deployed
generations before native work. Complete the pending map regeneration/reload,
seed/readiness and publisher/pose/settling gates. On a verified empty isolated
domain, test the real combined reader with positive and invalidation controls;
then collect all three controls at independently verified arrival poses. Any
armed movement retains the trial guardian. No native movement or deployment was
performed for this delivery.

Camera evidence now has local capture/validation coverage; native camera acceptance,
cancellable inspection ownership, noise confidence, causal diagnosis and
interlocked maintenance remain separate work. Separately stored
station and proximity CLI results still cannot substitute for this live capture.
