# Known-map navigation contract — preparation

This is the Phase 2 design contract for issues #6/#7. It is not a claim that localization or navigation is running. Phase 1's rendered operator-view acceptance remains open. No live navigation deployment is performed by this document.

## Map and coordinates

The versioned source of geometry is `worlds/maintenance.sdf`: one 10 × 8 m floor centered at world (0, 0), walls centered at x=±5 and y=±4 with 0.15 m thickness, and three 0.8 × 0.8 m machine footprints centered at (3,1.8), (3,0), (3,−1.8). Coordinates are metres, right-handed, Z up; yaw 0 faces +X and positive yaw rotates toward +Y.

Generate the occupancy grid deterministically from the static collision geometry, not visual meshes. Planned resolution 0.05 m; origin(−5.25,−4.25), size 210 × 170 cells. Store occupied cells as 0, free as 254 and unknown as 128 in a generated PGM, with negate 0, occupied threshold 0.65 and free threshold 0.25. PGM rows run from highest Y to lowest Y; the YAML origin is the lower-left world coordinate. Mark the interior floor free, collision footprints occupied and space outside the room unknown. Refuse unsupported poses/geometry rather than silently producing a misleading map.

Version source geometry, generator and generation parameters in Git. Generated map files and a source/config SHA-256 manifest belong in ignored `runtime/maps`; a clean checkout must regenerate identical map bytes. Verify obstacle centers, clear origin, wall coverage, pixel/world round trips and an inflated free-space path to each inspection pose before live use.

## Inspection poses and actual control proximity

| Machine | Machine center | West-face control point | Planned robot goal (x,y,yaw) |
| --- | --- | --- | --- |
| pump | (3,1.8) | (2.6,1.8) | (2.0,1.8,0) |
| cooler | (3,0) | (2.6,0) | (2.0,0,0) |
| power_unit | (3,−1.8) | (2.6,−1.8) | (2.0,−1.8,0) |

These are proposed reachable goals, not measured arrivals. The station model's 0.75 m proximity bound applies to a physical control point, not the machine center: each goal is 0.60 m from its control. XY arrival tolerance 0.10 m and yaw tolerance 0.15 rad leave margin, but the machine gateway must still measure actual control distance and measured speed ≤ 0.01 m/s independently. A navigation success flag alone never authorizes maintenance.

Use a conservative circular collision radius 0.37 m initially (body half-diagonal ≈ 0.34 m, wheels extend laterally to 0.27 m). Check it against the full collision geometry. Nav2 inflation adds a margin; do not shrink the footprint merely to force a route through. Camera/scan observations must show which labelled machine is being approached; fixed poses are public geometry and cannot encode hidden fault truth.

## Transform and localization ownership

- `map→odom`: AMCL only, using the generated known map and `/scan`. No parallel identity transform or ground-truth publisher masquerading as localization.
- `odom→base_link`: existing Gazebo differential-drive bridge only. Validate timestamps and compare the model/link origin before accepting the 3D frame geometry.
- `base_link→sensor/wheel frames`: robot_state_publisher only. Verify the URDF/SDF body-origin offset and scan mounting height; do not infer correct geometry solely from matching frame names.
- Use simulation time for sensor/TF alignment, steady wall time for safety deadlines. AMCL may future-date TF by its configured transform tolerance; the validator must use lookup-at-current-simulation-time and an explicit allowed tolerance, not reject all future transform stamps indiscriminately.

Start from a documented initial pose at the clear origin, with finite conservative covariance. AMCL must converge before accepting navigation. A stationary robot need not produce new `/amcl_pose` messages at a fixed rate, so localization health must also consider fresh scan input and current TF availability; using only pose-message age would create false loss alarms.

## Loss, reset and stale-target behavior

The navigation supervisor owns one target and a run/epoch identity. A world-clock regression, explicit initial-pose replacement or localization loss invalidates the active epoch, cancels Nav2 and latches motion stop. A prior target cannot become valid again merely because TF returns. Resuming requires a newly accepted target in the current epoch and explicit re-arm after fresh stationary feedback; cancellation of the old task must be confirmed before releasing that interlock.

Initial bounds to verify: scan receive age ≤ 0.5 wall seconds, current map/odom/base transform lookup available, finite pose/covariance, and localization XY/yaw covariance thresholds documented in the implementation. Do not select arbitrary covariance thresholds and claim they prove localization accuracy; compare AMCL to simulator ground truth only in an independent evaluator. The planner must not receive that hidden evaluation feed.

## Acceptance evidence still required

Issue #6: generated map identity and geometry checks; exactly one owner per TF edge; initial-pose convergence and timing; reset/loss invalidates old target identity. Issue #7: measured arrival at all three inspection poses; obstacle/blockage, timeout and cancel tests; safe stopping through `/motion/request` only; no observed collision. Installed Nav2 packages and this contract are not acceptance evidence.
