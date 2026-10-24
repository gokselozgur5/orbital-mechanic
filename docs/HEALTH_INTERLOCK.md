# Health-to-stop adapter — isolated acceptance

`orbital/health_lease.py` validates versioned health reports. `orbital/ros_health_interlock.py` connects that lease to the existing `/motion/stop` service. The adapter never arms motion, publishes velocity or dispatches Nav2 actions. It is not installed in the shared demo and is not a complete navigation supervisor.

## Stop obligation and replay handling

Health reports must match schema version 1, contain finite time and correctly typed fields, and identify a canonical UUID run plus nonnegative epoch and positive sequence. Sequence must increase across epochs within the same run. Old epochs, replayed sequences, retired process identities and malformed reports invalidate health. Retired identities are bounded to 64; additional new sources are refused until an explicit process restart. This is replay discipline within a trusted local ROS domain, not publisher authentication.

The health receive lease is 0.25 monotonic seconds. A new sequence with a frozen simulation timestamp cannot renew it. Receiving a late healthy report checks the previous deadline first. Source/epoch changes and loss create a pending stop obligation even when the most recent report already says healthy. Thus a missed unhealthy message cannot disappear behind a later healthy report.

Only a successful stop-service response for the current invalidation generation clears that obligation. A delayed acknowledgment for an older generation cannot clear a newer fault. Invalid health continues to require stop even after acknowledgment; fresh health alone never re-arms the guard.

The ROS adapter checks every 50 ms, refuses zero/multiple health publishers, and retries stop requests no faster than 100 ms. An outstanding call times out after 500 ms and is retried. A timed-out ROS service request cannot cancel execution already underway at the server; any late effect is another idempotent stop, never an arm.

## Required ownership before real navigation

This component alone is not a fail-safe command boundary. If the stop service is unavailable, it can report/retry but cannot prove that an actuator stopped. If this interlock dies while an independent command publisher continues, its checks no longer run.

The navigation supervisor must therefore own both this interlock and the sole Nav2-to-guard velocity path. It must withhold commands whenever health, target identity, cancellation or stop acknowledgment is invalid. Its process loss must stop command publication so the existing independent guard watchdog expires. Nav2 raw output must never reach the drive directly. Launcher/cgroup ownership, exact action terminal cancellation, measured arrival and real-world fault tests remain outstanding.

The future supervisor must also require independently fresh stationary feedback before re-arm; stop acknowledgment only proves the guard latched, not that physical deceleration has finished.

## Discriminating ROS test

`scripts/check-health-interlock.py` requires an otherwise empty ROS domain 84. It runs the actual `motion-guard.py` adapter with synthetic odometry and the real health observer/interlock. No Gazebo or actuator bridge is created. The test first arms the guard explicitly, then keeps publishing valid stamped velocity requests while injecting scan loss, tilt, TF loss, a paused clock or a silent health publisher.

Acceptance requires the actual guarded ROS output to become zero with `operator_stop`, while command publication continues. A normal command timeout cannot satisfy that assertion. Recovery must leave the guard latched despite more old requests; the next case explicitly arms again. The test also checks the interlock's publishers and shuts down all test nodes.

Run with the same isolated environment described in [ROS_LOCALIZATION_MONITOR.md](ROS_LOCALIZATION_MONITOR.md), replacing the test command with:

```bash
timeout --kill-after=5 60 /usr/bin/python3 scripts/check-health-interlock.py
```

These are ROS control-output measurements, not physical displacement, braking or Nav2 cancellation evidence. The first trial passed but its fixture clock advanced faster across short polling windows; the corrected fixture keeps one publication schedule across windows. Report final timings only from the corrected trial.

## Corrected-clock acceptance — 2026-10-24

Thor `health-interlock-synthetic-2` exited 0. Guarded output reached zero after scan loss in 0.559 s, tilt in 0.062 s, TF loss in 0.156 s, clock pause in 0.307 s and health silence in 0.247 s. Between 3 and 26 valid requests continued during each fault, and every stop reason was `operator_stop`. All recovery cases remained latched; 13 stop requests received matching acknowledgments. These observed intervals include scheduling and are not universal guarantees.

Seven new lease unit cases cover source binding, late reports, missed invalidation epochs, replay/frozen time, retired identities, stale stop-ack races and malformed/old-clock input. All 57 local unit tests pass. Source hashes of the isolated deployment match the committed candidates. The common simulator was not used or restarted.
