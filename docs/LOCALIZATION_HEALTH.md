# Localization health validation — offline component

`orbital/localization.py` validates observations for a future trusted ROS adapter. It emits readiness and a reason; it does not publish motion, seed AMCL, dispatch a Nav2 action or modify the running simulator. This isolated delivery does not complete issues #6/#7.

## Evidence contract

The adapter must obtain scan, odometry, IMU and AMCL pose from ROS, use `time.monotonic()` for reception/deadlines and read the actual simulation clock. Planner arguments are not health evidence. Input frames must match the commissioned `lidar_link`, `odom→base_footprint`, `base_link` IMU and `map` pose conventions.

Scan/odometry/IMU each require independently advancing timestamps and less than 0.5 seconds of both reception and simulation age. Replayed stamps cannot renew their lease. Invalid observations and a late sample after an expired lease remain visible to the next evaluation even if a fresh sample arrives before the timer runs. A fresh-looking callback burst cannot conceal an intervening fault.

The closed-room scanner contract requires 360 ranges with at least 180 finite in-range observations; positive infinity is allowed for other rays, NaN/negative infinity are rejected. This is a conservative sanity requirement for this world, not a generic observability proof. A different scanner/world needs an explicit contract change and new commissioning evidence.

AMCL pose must be finite, with a normalized quaternion and a symmetric positive-semidefinite x/y/yaw covariance marginal. Commissioning bounds are 0.25 m XY standard deviation and 0.35 rad yaw standard deviation. These limits do not establish accuracy; independent Gazebo comparison belongs only to the evaluator. A stationary AMCL pose may remain unchanged while fresh scan/odometry/IMU and a current TF lookup continue.

The adapter supplies a successfully looked-up `map→base_footprint` transform for an instant no more than 0.2 simulation seconds old. It must contain finite position and normalized orientation; the flat-floor ground frame must remain within 0.05 m of z=0 and 0.05 rad of level. The same planar checks apply to AMCL pose and wheel odometry. The adapter must not fabricate an identity transform, reuse a failed lookup or forward simulator truth. Future-dated AMCL source transforms are not themselves the lookup instant.

IMU roll/pitch must stay below 0.05 rad, independently of planar wheel odometry. Stationary readiness requires full 3D linear speed ≤0.01 m/s and angular speed ≤0.02 rad/s; it is false whenever overall health is invalid.

## Epoch and target handling

A simulation-time regression discards observations and requires an explicit new initial pose. The adapter calls both `LocalizationHealth.replace_initial_pose(seed_stamp)` and `NavigationPolicy.replace_initial_pose()` when it observes a seed. Retained pre-seed AMCL messages cannot satisfy the new epoch. Evaluate the simulation clock/reset before accepting post-reset callbacks. Do not silently seed from ground truth.

Feed every health result into `NavigationPolicy.observe` and run its steady-clock timeout even when no callbacks arrive. Invalid health cancels the target identity and requires stop. A later healthy sample does not revive that target. The adapter must latch the real motion guard, cancel the exact Nav2 action and wait for its terminal outcome; cancel-request acknowledgment alone is insufficient. This document and pure policy tests do not prove those ROS side effects.

## Verification and remaining integration

Thirteen new deterministic cases cover missing inputs, duplicate and independently stale streams, delayed fresh-message bursts, invalid-then-valid bursts, covariance asymmetry/non-PSD/nonfinite values, physical tilt, wrong frames, missing/future TF, backward wall time, simulation reset and pre-seed pose rejection. The tilt case integrates the health result with the existing target policy and proves the old target cannot be re-armed after recovery.

A read-only ROS observation adapter is now available and exercised with isolated synthetic DDS/TF; see [ROS_LOCALIZATION_MONITOR.md](ROS_LOCALIZATION_MONITOR.md). A real navigation supervisor, action cancellation races, continuous motion-stop enforcement and shared-world reset/loss tests are still required. No runtime interlock or autonomous navigation acceptance is claimed by this offline component.
