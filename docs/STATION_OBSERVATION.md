# Fresh public station observations

OM-046 adds the actual read-only consumer for the existing OM-041 public telemetry
projection. It contributes to issue #8 without duplicating that projection or
claiming OM-045 as a new delivery. Local software tests execute the shipped
publisher and reader under controlled transport. The separate
[joint machine/robot reader](INSPECTION_OBSERVATION.md) now passes local tests.
Native ROS commissioning, complete inspection and maintenance actions remain pending.

On the commissioned Orbital runtime, after sourcing ROS:

```bash
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 scripts/observe-station.py --timeout 5
```

The reader accepts only a finite timeout from 0.1 to 10 seconds. There are no
caller paths, topic names, clocks, values or freshness-limit overrides. Domain 84
is allowed only for a controlled fixture whose empty graph was verified first.
Other projects' domains are refused. A short timeout can expire before the two
required nominal 0.5-second publisher samples arrive.

## Public schema and identity

`orbital.telemetry_observation.TelemetryEvidence` decodes exactly the existing
version 1 wire fields: source, registry, run/sequence, absolute simulation stamp,
units and model observations. It requires the full locally configured public
catalog, not merely an echoed identity string. Canonical catalog content must
match exactly, including numeric types; JSON whitespace/key order may differ.
All three known machine IDs and all measured fields must be present, with no
extra scenario/fault/reset fields. Units and source are fixed by the publisher
contract. The run is its existing 32-character lowercase hexadecimal ID;
sequence/stamp are positive integers below 2^63, never booleans or coerced floats.

The bounded parser permits at most 16384 UTF-8 bytes, rejects duplicate keys,
nonfinite numbers and malformed/deeply nested JSON, and does not expose rejected
input in errors. A character-count check precedes UTF-8 encoding so already
oversized transport input does not allocate another full payload copy. This
bounds decoder copying; ROS has already allocated the incoming message.
Power state must be boolean. Temperature, current, vibration,
voltage and model time must be finite numeric values, never missing, null or
boolean. Defensive protocol ranges are: voltage 0–10000 V, current and vibration
0–1e6 in their declared units, temperature −273.15–1e6 °C, and model time 0–(2^63−1)/1e9
seconds. These are representation bounds, not calibrated sensor ranges, alarm
thresholds or evidence of healthy operation.

Unknown/missing fields refuse the observation rather than becoming zero or a
diagnosis. Finite noisy readings inside the protocol bounds remain unchanged;
this reader does not smooth them, infer faults or check physical/model equations.
Its output is a fresh public measurement record, not proof of measurement
accuracy or sufficient confidence for an intervention.

## Time and ordering

One query pins one telemetry run. It requires two accepted samples with strictly
increasing sequence numbers and source timestamps, including a consecutive final
pair. A sequence gap records the missing count and starts a new pair. Expired
history also requires two new samples. Run changes, replays, nonadvancing source
timestamps, source-clock mismatch and malformed input invalidate the whole query;
later valid packets cannot erase them. A new query is required after a reset.

The source timestamp is absolute simulation nanoseconds. `time_s` is elapsed
model time and cannot substitute for it. Within one run, changes in elapsed model
time must match source-time deltas within 1e-6 relative/absolute seconds. The
source's documented reset behavior rotates the run rather than preserving old
observation authority.

Both monotonic receipt age and simulation age must be strictly below 1 second,
chosen as two nominal publisher periods. A future or already-stale received
packet is rejected; no transport-order tolerance is silently added. The local
simulation clock must advance during the query, and the final sample must be at
least as recent as its first positive observed clock. Two queued older packets
cannot make a frozen/new query appear live. Wall or simulation clock regression
invalidates the query. A recently paused stream can remain fresh only until
these bounds expire; no continuously running-clock guarantee is implied.

## Actual ROS reader and result

`orbital.ros_station_observation.run` loads the registry and starts a node with
simulation time enabled. Before subscribing it requires a positive clock and
the unique expected publishers: `ros_gz_bridge` for `/clock`, and
`station_telemetry` for `/station/telemetry`, with their exact namespaces/types
and nonzero endpoint GIDs. The same publisher snapshot must remain present
during collection and after final source verification. Clock mode, source bytes,
data age and overall deadline are checked before accepting the result.
Result ID/digest preparation precedes those final checks, so time or source changes
during local result preparation cannot bypass them.

Successful JSON uses outer `schema_version: 1` and contains:

- `valid: true`, a unique `observation_id` and the publisher-snapshot digest.
- `telemetry`, the complete validated public wire measurement with its original
  registry/run/sequence/source stamp/units/model values.
- The observed local simulation stamp and `freshness` with both ages, the fixed
  limit, consecutive sample count and missing-sequence count.
- An explicit scope stating that the record is not a diagnosis, local inspection
  or action authorization. Returned objects do not mutate cached evidence.

Exit 0 means a fresh station observation. Evidence/runtime failures return exit 2
and JSON `valid: false` without measurements; argument errors use the parser's
diagnostic. Ctrl-C returns 130 with invalid JSON. Node/ROS cleanup runs on success,
error and interruption. The deadline bounds accepted results, not hard real-time
process termination if local computation or a runtime call stalls.

There are no control publishers, service clients, stop acknowledgments, scenario
reads or Gazebo truth subscriptions. Graph names/GIDs are sampled consistency
checks, not message authentication or detection of every intervening source
change. Source hashes do not prove loaded simulator memory. The unversioned
`/station/bus_voltage` display topic is never used as action evidence.

## Validation and remaining integration

Tests run the strict production core, actual telemetry entry point, ROS reader
and shipped CLI with controlled transport. They cover every current model fault,
raw noisy values, missing/extra fields, wrong catalog/units/types, malformed and
bounded JSON, dual-clock expiry, reset/replay/gaps, frozen-clock queued samples,
final verification races, source drift, timeout, cleanup and output independence.
The publisher fixture emits real production messages; the reader's validation
and state decisions are not mocked. Transport doubles do not prove native DDS
ordering, discovery, QoS or installed callback behavior.

Review reproduced five late-result acceptance failures in the initial station
reader and ten in the existing proximity adapter from `24860f4` (both distance
and settled modes). The same tests inject source/GID/mode changes or elapsed
time during result identity preparation. They fail before the final checks are
moved after that work and pass afterward. Injection marks a scheduling boundary;
it does not claim UUID generation normally changes external state. These are
local source/transport regressions, not observed native incidents.

A separate allocation regression sends an already allocated 1 MiB string. The
initial byte-limit check copied it before rejecting it; the same test now rejects
it below a 64 KiB additional tracked-allocation bound. This correction belongs
to OM-046 and does not add a separate delivery credit.

Before native use, recheck Thor ownership and commission OM-041's exact telemetry
source/schema together with this reader on an empty isolated domain. Exercise
correct delivery, paused clock, stale/future/replayed/unknown data, source/run/GID
changes and positive recovery through a new query. Then bind the actual deployed
scene and owned demo/localization/navigation generations. Coordinate the pending
map regeneration/reload/seed/readiness and preserve the guardian for armed trials.

Do not combine stored CLI JSON with separately sampled robot pose and call it
inspection. The joint reader acquires current telemetry and settled robot state
together and binds their source/time identities through result acceptance.
Optional [camera capture](CAMERA_OBSERVATIONS.md) now extends that live reader.
Native camera acceptance, maintaining gates through inspection ownership, endpoint cancellation,
noise confidence, causal diagnosis and maintenance permissions remain separate
acceptance work. No native deployment, movement or maintenance action occurred
for this delivery.
