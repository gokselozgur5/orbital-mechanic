# Measured machine proximity

OM-042 adds a bounded, read-only query for the distance between the localized
robot and one registered machine control. It uses the actual map-binding gate,
localization health and time-specific TF. A planner cannot supply a pose or
distance. Local software tests pass; native transport and physical measurement
commissioning remain open.

On the commissioned Orbital runtime, after sourcing ROS:

```bash
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 scripts/machine-proximity.py pump --timeout 5
```

The three IDs are `pump`, `cooler` and `power_unit`. The timeout accepts finite
values from 0.1 to 10 seconds and bounds accepted results across registry load,
discovery, map verification and TF lookup. Slow local computation can delay an
error return; this is not a hard real-time process-termination guarantee. Domain
84 is reserved for an isolated fixture whose empty graph was checked before its
controlled publishers started. Never use another project's domain.

## Evidence and result

The query loads the immutable public registry, then requires the expected unique
publishers for `/tf`, `/tf_static`, `/clock`, `/map` and `/navigation/health`.
Endpoint GIDs must remain unchanged across collection and the final pose lookup.
The existing map-binding adapter checks exact source/artifact/cell identity,
map-server/AMCL/planner parameters, retained map delivery and consumer subscription
settings. Its configuration digest must match the registry. Map collection must
have completed less than 0.5 monotonic seconds before the accepted observation.

The health report pins one localization run and epoch. Replays, malformed input,
generation changes and clock regression invalidate this query. A frozen clock
cannot renew a health lease. Receipt age must be below 0.25 seconds; health
simulation time must not be in the future or more than 0.2 seconds old. The
time-specific `map` to `base_footprint` transform is requested at simulation time
minus 0.1 seconds. Its timestamp must match exactly, be within the observed epoch
and be no more than 0.2 simulation seconds old when measured. Nonfinite,
nonplanar, wrong-frame or invalid-quaternion poses are refused. Source hashes,
publisher generation, clock mode, map age, health and the overall deadline are
checked again before accepting a successful lookup.

A valid JSON result includes:

- Registry identity, stable machine ID and configured control XY in metres.
- Measured robot XYZ/quaternion, pose and observation simulation timestamps.
- Health run/epoch/sequence and monotonic receipt age.
- Map binding, age since collection completed and publisher-snapshot digest.
- A unique observation ID, `distance_m`, the inclusive `distance_limit_m: 0.75`
  and `within_distance`.

Exit 0 means a valid distance within the bound; 1 means a valid distance outside
it. Exit 2 means unavailable/invalid evidence, input or runtime; evidence failures
produce JSON with `valid: false` and no distance, while argument errors use the
standard parser diagnostic. Interruption returns 130 with JSON `valid: false`.
The adapter destroys its node/listener and map clients on completion or failure.

**A valid distance is not permission to inspect or intervene.** This query can
observe a moving robot. It does not enforce viewing direction, settling,
telemetry validity, action ownership or electrical isolation. It never clears
the health lease's pending motion-stop obligation. Future inspection/action
owners must acquire fresh evidence internally and enforce their remaining gates;
stored CLI JSON is not an action token.

## Trust boundary and commissioning

There is no control publisher, navigation/maintenance action call or Gazebo robot
truth subscription. Map services only read parameters and cells. This query uses
configured control geometry, not runtime machine/entity truth. Source hashes and
graph snapshots do not authenticate messages, prove private AMCL/costmap memory,
detect every intervening publisher change or continuously enforce map identity.
Map age is measured from completed collection, not the age of every cell or
parameter read. Retain the native per-message TF authority audit and independent
pose comparison when commissioning; neither is replaced by this local adapter.

The 25 new tests exercise the production core, CLI and actual proximity/map
adapters with controlled transports. They cover all three controls, distance
boundaries, map cell/parameter mismatches, missing/paused/stale/unhealthy data,
reset/replay/generation changes, invalid poses, source drift, cleanup and late
results. Review reproduced three acceptance races: a changed publisher GID,
changed clock mode or expired map observation during a successful TF lookup was
initially accepted. Identical cases now reject it. A separate strict-timestamp
regression rejects a float masquerading as integer nanoseconds. These are local
software controls, not native DDS or physical acceptance.

Before native use, recheck host/service ownership and commission pending
navigation and telemetry changes, including explicit map regeneration/reload,
seed, readiness and exact source identities. On an empty isolated domain, run
the real executable with positive and wrong-map/frame/source/generation/clock
controls. Then measure all three controls against independent simulator pose,
keeping ground truth outside the query's input graph. Preserve the trial
guardian for any armed movement. No native deployment, movement or maintenance
action was performed for this delivery. Orientation, stationarity, fresh public
observations and inspection endpoints remain subsequent work under issue #8.
