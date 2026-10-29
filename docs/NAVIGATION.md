# Known-map navigation contract — preparation

This is the Phase 2 contract for issues #6/#7. Known-map AMCL has passed initial commissioning on the corrected physical model. Named-pose navigation, action cancellation and route/collision acceptance remain incomplete. The health observer and guard-stop connection have separate isolated ROS evidence; they are not yet integrated into the shared simulator.

## Map and coordinates

The versioned source of geometry is `worlds/maintenance.sdf`: one 10 × 8 m floor centered at world (0, 0), walls centered at x=±5 and y=±4 with 0.15 m thickness, and three 0.8 × 0.8 m machine footprints centered at (3,1.8), (3,0), (3,−1.8). Coordinates are metres, right-handed, Z up; yaw 0 faces +X and positive yaw rotates toward +Y.

Generate the occupancy grid deterministically from the static collision geometry, not visual meshes. Resolution 0.05 m; origin(−5.25,−4.25), size 210 × 170 cells. Store occupied cells as 0, free as 254 and unknown as 128 in a generated PGM, with negate 0, occupied threshold 0.65 and free threshold 0.25. PGM rows run from highest Y to lowest Y; the YAML origin is the lower-left world coordinate. Mark the interior floor free, collision footprints occupied and space outside the room unknown. Refuse unsupported poses/geometry rather than silently producing a misleading map.

Version source geometry, generator and generation parameters in Git. Generated map files and a source/config SHA-256 manifest belong in ignored `runtime/maps`; a clean checkout must regenerate identical map bytes. Verify obstacle centers, clear origin, wall coverage, pixel/world round trips and an inflated free-space path to each inspection pose before live use.

## Inspection poses and actual control proximity

| Machine | Machine center | West-face control point | Planned robot goal (x,y,yaw) |
| --- | --- | --- | --- |
| pump | (3,1.8) | (2.6,1.8) | (2.0,1.8,0) |
| cooler | (3,0) | (2.6,0) | (2.0,0,0) |
| power_unit | (3,−1.8) | (2.6,−1.8) | (2.0,−1.8,0) |

These are proposed reachable goals, not measured arrivals. The station model's 0.75 m proximity bound applies to a physical control point, not the machine center: each goal is 0.60 m from its control. XY arrival tolerance 0.10 m and yaw tolerance 0.15 rad leave margin, but the machine gateway must still measure actual control distance and measured speed ≤ 0.01 m/s independently. A navigation success flag alone never authorizes maintenance.

Use a conservative circular collision radius 0.37 m initially (body half-diagonal ≈ 0.34 m, wheels extend laterally to 0.27 m). Check it against the full collision geometry. Nav2 inflation adds a margin; do not shrink the footprint merely to force a route through. Camera/scan observations must show which labelled machine is being approached; fixed poses are public geometry and cannot encode hidden fault truth.

### Enforced offline geometry — OM-021

`orbital/inspection_geometry.py` validates the complete known-map contract before
map generation and before the navigation CLI imports ROS or acquires ownership.
The artifact binding check recomputes the same geometry. Minimal pose-only input
remains available to isolated library fixtures; the production entry point always
requires the world and geometry fields.

- Exactly the station's three machine IDs must have goals. Each must resolve to
  one static collision box, intersected by the configured map slice.
- Each finite control point must be the matching box's west-face center. A point
  inside a machine or on another machine cannot substitute for that control.
- The reviewed radius 0.37 m and arrival tolerances 0.10 m / 0.15 rad are enforced;
  the supervisor uses the shared arrival constants. JSON cannot weaken them.
- The full arrival footprint, radius 0.47 m around the nominal goal, must avoid
  occupied, unknown and outside cells. The existing continuous disc checker is
  reused. A single four-connected flood fill checks inflated reachability from
  the clear origin to every goal, without diagonal corner cutting.
- Nominal control distance plus XY arrival tolerance must remain within the
  station's shared 0.75 m maintenance bound. Current goals are 0.60 m from their
  controls, leaving 0.05 m after the 0.10 m arrival tolerance.

The manifest records these per-machine checks. Its revision binds the world,
exact config bytes and geometry/parsing/generation dependencies. Two generations
of revision `1c58f56cf5246735` match exactly; image SHA remains
`d08588af2b1186992411e32dbf1d703e47ea45d183666de13c04a84cbe9c5000`.
Changing the validator or config rejects an older loaded revision even when map
pixels are unchanged. Regenerate and explicitly reload localization before the
next native seed, map-binding and readiness checks; do not overwrite old evidence
or pretend that an old running map has loaded the new contract.

All 127 local tests pass. The same 14 geometry regression tests yield 21 failing
subcases on `24df24d` and pass on this implementation. Positive cases include
matching translated machine/control/goal geometry and the exact 0.75 m boundary.
CLI refusal, dependency identity and tampered report checks also pass.

This is static geometry acceptance. It neither measures localization error nor
proves actual arrival, observation orientation or collision-free execution.
Maintenance still requires fresh independently measured distance and stationary
feedback; navigation success alone cannot authorize it. Native deployment and
the three-machine tour remain pending.

## Transform and localization ownership

- `map→odom`: AMCL only, using the generated known map and `/scan`. No parallel identity transform or ground-truth publisher masquerading as localization.
- `odom→base_footprint`: Gazebo differential-drive bridge only. Wheel odometry is planar; the child is the ground projection, not the elevated body.
- `base_footprint→base_link→sensor/wheel frames`: robot_state_publisher only. The fixed body height is 0.22 m, lidar height 0.38 m, camera mount height 0.34 m and wheel centers 0.12 m above the nominal ground plane. The SDF spawn clearance is not a TF offset. Verify live heights and timing before acceptance.
- Use simulation time for sensor/TF alignment, steady wall time for safety deadlines. AMCL may future-date TF by its configured transform tolerance; the validator must use lookup-at-current-simulation-time and an explicit allowed tolerance, not reject all future transform stamps indiscriminately.

Start from a documented initial pose at the clear origin, with finite conservative covariance. AMCL must converge before accepting navigation. A stationary robot need not produce new `/amcl_pose` messages at a fixed rate, so localization health must also consider fresh scan input and current TF availability; using only pose-message age would create false loss alarms.

## Loss, reset and stale-target behavior

The navigation supervisor owns one target and a run/epoch identity. A world-clock regression, explicit initial-pose replacement or localization loss invalidates the active epoch, cancels Nav2 and latches motion stop. A prior target cannot become valid again merely because TF returns. Resuming requires a newly accepted target in the current epoch and explicit re-arm after fresh stationary feedback; cancellation of the old task must be confirmed before releasing that interlock.

Initial bounds to verify: scan receive age ≤ 0.5 wall seconds, current map/odom/base transform lookup available, finite pose/covariance, and localization XY/yaw covariance thresholds documented in the implementation. Do not select arbitrary covariance thresholds and claim they prove localization accuracy; compare AMCL to simulator ground truth only in an independent evaluator. The planner must not receive that hidden evaluation feed.

## Acceptance evidence still required

Issue #6: generated map identity and geometry checks; exactly one owner per TF edge; initial-pose convergence and timing; reset/loss invalidates old target identity. Issue #7: measured arrival at all three inspection poses; obstacle/blockage, timeout and cancel tests; safe stopping through `/motion/request` only; no observed collision. Installed Nav2 packages and this contract are not acceptance evidence.

## Offline map prototype

`python3 scripts/generate-known-map.py` now generates PGM/YAML/manifest files under ignored `runtime/maps/<input-and-generator-digest>/`. Source geometry, `config/known-map.json` and generator code all contribute to the identity. Existing different artifacts are refused, not overwritten. The robot/world is not started or changed by generation.

Local acceptance: all 24 unit tests pass, including six map cases covering reference obstacles/free space, pixel/world coordinates, PGM vertical orientation, inflated reachability to the three goals, narrow-corridor rejection and explicit rejection of unsupported rotated/non-box static geometry. Two generations produced identical metadata and map bytes. Current grid: 210 × 170 cells at 0.05 m; PGM SHA-256 `d08588af2b1186992411e32dbf1d703e47ea45d183666de13c04a84cbe9c5000`.

Offline reachability is not live navigation acceptance. Subsequent frame and AMCL commissioning evidence is described below; integrated target reset/loss and route acceptance remain outstanding.

## Frame regression and live check

The initial world published planar odometry directly as `base_link`, placing the body and all sensors 0.22 m too low in the ROS tree. The corrected tree separates `base_footprint` from `base_link`. Cross-model tests compare URDF cumulative mount positions with SDF body/sensor/wheel positions. The initial SDF model spawn at z=0.02 is clearance for settling, not a fixed transform.

On Thor, source `scripts/ros-env.sh` and run `/usr/bin/python3 scripts/check-frames.py`. It reads TF/odometry only, checks expected body/sensor/wheel positions, advancing fresh odometry, planar dynamic TF, observed parents and publisher-node counts, and exits nonzero on mismatch. It does not move the robot or prove AMCL accuracy. Per-message publisher ownership still requires stronger evidence than node counts; the installed subscription metadata does not expose a publisher GID. Current deployment status and required post-change tests are in [STATUS.md](STATUS.md).

The planar source behavior is also visible in the [Gazebo DiffDrive implementation](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.cc): its wheel odometry populates XY/yaw and uses the configured child frame name; it does not add the body's SDF height. Live measurements above establish the behavior of this installed runtime independently of that moving source branch.

## Map-server and AMCL commissioning

`config/localization.yaml` and `scripts/{launch,start}-localization.sh` target the installed Nav2 1.3.13 packages. They launch the official map-server/AMCL lifecycle group only, using the generated map, simulation time and `base_footprint`. There is no controller, velocity publisher or automatic pose seed. The transient `orbital-localization` service requires an already-active demo; it does not start the world or enable boot startup. Explicit demo stop propagates through `PartOf`, and a monitor handles unexpected demo termination. Its cgroup owns all localization descendants.

Two reproduced startup issues are fixed: the installed launch PythonExpression requires capitalized `False` for composition/respawn, and an empty YAML `yaml_filename` masked the launch map override. The map filename is now supplied only by the official launch argument.

`seed-localization.py --fresh-world-origin` latches stop, requires fresh stationary corrected odometry near the origin, refuses a running navigation controller and publishes an uncertain explicit pose. It reports processing only, not accuracy. Optional bounded seed offsets support commissioning. `check-localization.py --compare-gazebo` checks exact map cells, fresh scan/odometry/IMU, body level, current-time TF, finite bounded covariance and expected publisher nodes. Ground truth is evaluator-only and is never supplied to AMCL or the planner.

With the corrected physical model, Thor job `body-max-localization` exited 0. A deliberately wrong seed (0.20 m, −0.15 m, 0.15 rad), followed by bounded forward/turn movement, converged to independent error 0.073151 m / 0.006311 rad, within unchanged 0.10 m / 0.10 rad commissioning limits. This is one commissioning case, not a guaranteed route accuracy bound. A fresh-origin restart measured 0.008088 m / 0.011434 rad.

The same run verified service lifecycle: stopping the demo removed all seven sampled localization processes in 1.402 s; starting localization with the demo off was refused; restarting the demo did not implicitly restart localization. The new world was latched before localization was explicitly started and seeded again. Reports and source manifest remain in Thor `runtime/localization-acceptance-20261024T141639Z` and private review artifacts.

The current source adds `--expect-unseeded` to the probe and invokes it before seed in the commissioning/lifecycle runners. This extra negative gate is syntax-checked but has not been run against the live AMCL service yet. The earlier live lifecycle acceptance did not include that check. `check-localization-commissioning.sh` restarts the shared world and moves the robot; do not run it concurrently with another operator/session. Shared-world ownership is currently awaiting coordination, so these preserved scripts have not been redeployed from this branch.

Issue #6 remains open: commission the new negative gate and integrate source/epoch health with actual target cancellation/reset. Issue #7 remains open: own the Nav2 velocity path, verify all three arrivals, blockage/change/cancel/timeout behavior and collision evidence.
