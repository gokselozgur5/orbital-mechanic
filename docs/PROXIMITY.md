# Measured machine proximity

OM-042/043 provide a bounded, read-only query for distance and inspection pose
bounds relative to one registered machine control. It uses the actual map-binding
gate, localization health and time-specific TF. A planner cannot supply a pose,
distance or tolerance. Local software tests pass; native transport and physical
measurement commissioning remain open.

On the commissioned Orbital runtime, after sourcing ROS:

```bash
ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 scripts/machine-proximity.py pump --timeout 5 --require-inspection-pose
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

A valid JSON result now uses `schema_version: 2` and includes:

- Registry identity, stable machine ID and configured control XY in metres.
- Measured robot XYZ/quaternion, pose and observation simulation timestamps.
- Health run/epoch/sequence and monotonic receipt age.
- Map binding, age since collection completed and publisher-snapshot digest.
- A unique observation ID, `distance_m`, the inclusive `distance_limit_m: 0.75`
  and `within_distance`.
- `within_inspection_pose` and an `inspection_pose` report with the configured
  target XY/yaw, measured yaw, XY/yaw errors, fixed limits and rejection reason.

Without `--require-inspection-pose`, exit 0/1 retains its distance-only meaning.
With the flag, exit 0 requires both distance and the measured inspection pose;
exit 1 means valid evidence outside those bounds. Exit 2 means unavailable/invalid
evidence, input or runtime; evidence failures
produce JSON with `valid: false` and no distance, while argument errors use the
standard parser diagnostic. Interruption returns 130 with JSON `valid: false`.
The adapter destroys its node/listener and map clients on completion or failure.

## Inspection position and heading

The same accepted transform supplies the measured position and quaternion yaw.
The robot must be within the existing navigation arrival bounds: 0.10 m XY from
the registered inspection goal and 0.15 rad shortest angular difference from its
heading, as well as within 0.75 m of the control. Comparisons use the computed
errors and include the boundary; the CLI cannot override these constants. A
robot facing away or standing outside the validated arrival area cannot satisfy
`within_inspection_pose` merely because it is close to a control.

The map validator and registry share a nominal-heading check. Each configured
goal must approach from west of the control and point at its actual control
coordinates, within 1e-8 rad source-rounding tolerance. A valid diagonal goal is
supported; zero yaw is not hardcoded. Runtime quaternion extraction treats q and
-q identically, accounts for the permitted small norm error and wraps angular
differences. This is a desired planar control pose, not optical visibility,
camera capture, an occlusion check or a diagnosis.

**A valid pose is not permission to inspect or intervene.** This query can
observe a moving robot. It does not enforce settling, telemetry validity,
action ownership or electrical isolation. It never clears
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

The original 25 tests exercise the production core, CLI and actual proximity/map
adapters with controlled transports. They cover all three controls, distance
boundaries, map cell/parameter mismatches, missing/paused/stale/unhealthy data,
reset/replay/generation changes, invalid poses, source drift, cleanup and late
results. Review reproduced three acceptance races: a changed publisher GID,
changed clock mode or expired map observation during a successful TF lookup was
initially accepted. Identical cases now reject it. A separate strict-timestamp
regression rejects a float masquerading as integer nanoseconds. These are local
software controls, not native DDS or physical acceptance.

OM-043 adds 11 tests for nominal configuration and measured pose, including the
actual shipped CLI with the real map/proximity adapters under controlled
transport. The same nominal-heading regression fails on `c2242c4` and passes
after the shared check; its valid diagonal control passes both. Other controls
cover all machines, exact computed bounds, wrong side/position/heading, quaternion
sign/norm/wrap and distinct distance/pose rejection. No physical or camera
acceptance is inferred from these results.

Before native use, recheck host/service ownership and commission pending
navigation and telemetry changes, including explicit map regeneration/reload,
seed, readiness and exact source identities. On an empty isolated domain, run
the real executable with positive and wrong-map/frame/source/generation/clock
controls and near-but-wrong-pose refusals. Then measure all three controls against
independent simulator pose,
keeping ground truth outside the query's input graph. Preserve the trial
guardian for any armed movement. No native deployment, movement or maintenance
action was performed for this delivery. Native pose commissioning, stationarity,
fresh public observations and inspection endpoints remain open under issue #8.

The nominal-heading validator changes the generated map revision from
`1c58f56cf5246735` to `f02e11a9cb0e4f2f` even though the occupancy image is
unchanged. Two generations agree byte-for-byte and current source rejects the
retained old revision. Regenerate the map and explicitly reload localization,
seed, check readiness and capture the new source/map identities; do not relabel
old artifacts or overwrite a startup manifest to bypass that requirement.
