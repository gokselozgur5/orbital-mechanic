# Navigation target lifecycle prototype

`orbital/navigation.py` is a ROS-free policy prototype. It emits no velocity and is not connected to the running robot. It contributes to the localization-loss/reset contract in [NAVIGATION.md](NAVIGATION.md), but does not complete issues #6/#7.

## Identity and authorization

Each accepted named target receives an immutable run UUID, epoch and sequence. Only allowlisted waypoint names are accepted. There is at most one target; requesting one does not arm motion. The trusted adapter must separately confirm fresh stationary feedback before calling `arm` with that exact target identity.

Every process start generates a new run UUID. Simulation-clock regression, explicit initial-pose replacement, invalid localization, expired health or operator cancellation invalidates the target and requires stop. A recovered transform or new scan never re-arms it. The adapter must enforce `stop_required` with the existing motion guard and run `tick` on an independent steady-clock timer.

## Freshness

A health sample advances simulation time and is valid only when the adapter has checked fresh scan reception, a usable current transform and finite accepted pose/covariance. The policy expires that evidence after 0.5 wall seconds. Replayed timestamps cannot refresh the lease. Deadline expiry is evaluated before accepting new evidence, so a late callback cannot conceal a gap and preserve the old target.

The boolean input is a trusted adapter result, not planner-supplied localization validity. This module does not calculate covariance thresholds, TF accuracy, scan age or independent ground-truth error. Those must be implemented and measured in live localization acceptance.

## Cancellation ownership

An invalidated target remains in `cancel_pending`. New targets are refused until the adapter confirms that exact underlying task is terminal, or proves it was never dispatched. A Nav2 cancellation-request acknowledgment is not terminal confirmation. A wrong-run, old, duplicate or null terminal response cannot release another task's interlock.

After terminal confirmation, fresh health, a new target and explicit stationary re-arm are all still required. An old task's late success cannot finish or authorize a newer target. Normal `finish` also requires stop; it does not assert measured arrival or authorize maintenance.

## Verification and remaining integration

Seven unit cases cover startup/allowlist/explicit arm, late health after expiry, clock reset and initial-pose replacement, replayed health, wrong/duplicate cancellation results, invalid health/backward wall time, and terminal/late-result handling. These tests exercise state transitions without ROS.

Still required: ROS timer and localization-health adapter; actual Nav2 action acceptance/cancellation races; initial-pose interception; bounded target deadlines; measured arrival; velocity forwarding only through the guard; and live reset/loss/cancel/failure acceptance. The policy alone is not a safety boundary against arbitrary local ROS publishers.
