# Public scene registry and machine telemetry

OM-041 connects the existing read-only machine telemetry process to the configured
scene. The three stable IDs (`pump`, `cooler`, `power_unit`) identify the same
named SDF models, collision boxes, map inspection poses and west-face controls.
Local source/adapter tests pass. Loaded-world identity, native ROS transport and
robot proximity/inspection remain uncommissioned; this does not close issue #8.

Inspect the public catalog without ROS or simulator access:

```bash
python3 scripts/machine-registry.py
```

## Registry contract

The catalog has `schema_version: 1`, a SHA-256 `identity`, `frame: map`,
`world_name: maintenance` and `geometry_source: configured_world`. Its three
machine entries contain:

| Field | Meaning |
| --- | --- |
| `model_name` | Exact configured top-level SDF model name, equal to the stable machine ID. |
| `model_pose_xyz_xyzw` | Configured world-relative translation in metres and unit quaternion. |
| `collision_center_xyz_m`, `collision_size_xyz_m` | The one supported machine box, including model/link/collision transforms. |
| `control_xy_m` | Configured west-face center in map XY; this is the planar control convention. |
| `inspection_xy_yaw` | Named inspection destination from the map contract; metres/radians. |

`source_sha256` binds the exact world and map-configuration bytes. The identity
hash covers the complete catalog content excluding its own identity field. The
projection contains no local paths, source XML, scenario labels, reset budgets or
hidden model fields. Each call returns independent dictionaries/lists; caller
mutation cannot rebind the immutable registry.

The loader reuses the existing machine IDs, waypoint parser and supported SDF
scene parser. It requires exactly the three configured machine identities, one
static axis-aligned collision box per machine and a control point matching that
box's west face. The nominal inspection goal must approach from west and its
heading must point at that control, using the same check as map validation.
Missing/duplicate models, extra dynamic models, unsupported or
ambiguous source geometry, wrong frames and invalid control points are rejected.
Sources must be regular files under the checkout without symlinks; each read is
bounded to 32 KiB. Startup and each measurement recheck source hashes. A source
change fails the process; there is no automatic adoption of a new world/map.

This is a catalog of **declared geometry**, not observed robot pose or runtime
Gazebo entity identity. Source hashes and before/after reads do not prove loaded
physics memory, authenticate publishers or detect an in-memory moved prop. Native
commissioning must bind the deployed world and its actual props. Navigation
continues to own full arrival clearance/reachability validation. Catalog lookup
does not authorize navigation, establish proximity or perform maintenance.

## Versioned telemetry

`scripts/machine-telemetry.py` now invokes the production adapter and registry
loader. Existing topics remain `/station/telemetry` (`std_msgs/msg/String`) and
`/station/bus_voltage` (`std_msgs/msg/Float32`). The latter is a display value
without identity/time evidence and must not authorize an action.

Every complete JSON telemetry sample includes:

- `schema_version: 1` and `source: simplified_station_model`.
- The full public `registry`, including its source identity and bindings.
- `run_id`, monotonically increasing per-run `sequence` and absolute simulation
  `stamp_ns` as a nonnegative integer. Measurements begin at a positive timestamp.
- Explicit units for voltage, current, temperature, vibration, positions,
  orientations, source time and model elapsed time.
- The existing `time_s`, `voltage_v` and `machines` observation fields unchanged.

`time_s` is elapsed **model** time; it is not the absolute simulator timestamp.
The original causal model and allowed temperature/current/vibration/power
projection are reused. All current fault scenarios produce the same registry;
the sample never serializes a model instance or publishes its hidden fault cause.
The operator-only ROS `scenario` parameter remains outside the future planner
gateway. A public projection is not process isolation or ROS authentication.

The adapter requires `use_sim_time=true` at startup and before every measurement.
The existing demo launcher already passes that setting. The upstream
[Jazzy TimeSource](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/time_source.py)
declares and responds to this parameter; checking only startup would miss a later
mode change. These source observations do not commission Thor's installed binary.

A zero or repeated clock produces no new sample. A backward clock or forward gap
greater than ten seconds resets the model, preserving the old prototype's reset
policy, but now rotates `run_id` and restarts the sequence. A gap of exactly ten
seconds advances the model in bounded one-second steps. Restarting the model can
restore initial power/fault state, so future action owners must invalidate work
across a run change rather than resume an old intervention. No maintenance action
endpoint exists in this slice. Consumer freshness, clock authority, missing/noisy
measurement handling and interlocked actions remain subsequent integration work.

Registry drift, station identity drift, invalid clock, changed clock mode and
publication failures propagate instead of publishing a fabricated measurement.
Normal exit, interruption and errors destroy the node and shut down ROS. The
existing demo cgroup launcher owns teardown if this child exits. There is no
robot truth subscription, command publisher or maintenance service/client.

## Validation and native gate

All 271 local tests pass, including 27 registry/core/adapter tests. They execute
the shipped telemetry entry point and actual adapter with controlled ROS
transport, and cover translated models/link offsets, duplicate/missing/aliased
IDs, malformed/linked/bounded sources, source changes, immutable projections,
fault-label exclusion, clock reset/pause/gaps, wall-clock refusal and cleanup.
An identical source-change test fails on `fd86e6e` and passes with this change.
A separate regression fails before the runtime clock-mode check and passes after.
These are local software results, not native DDS, timing or physical evidence.

Before live use, verify the current Thor owner/source generations. Commission
the exact executable and schema on an empty isolated ROS domain with controlled
clock and source-change negatives, then coordinate deployment/restart of the
owned demo/localization/navigation services with the pending map/seed/readiness
work. The machine process belongs to the demo cgroup; do not kill a similarly
named process or start another world to update it. Compare all three native model
and control bindings and retain exact source/registry identities. This turn made
no native deployment, movement, world change or maintenance intervention.

The [measured proximity and settled inspection pose query](PROXIMITY.md) now
passes local core/adapter tests; native commissioning remains open. It combines
fresh robot geometry with a bounded direct-odometry settling history. Next:
observation freshness/validity, then inspection and maintenance endpoints. Keep
the independent collision evaluator and hidden scenario state outside planner
inputs throughout that integration.
