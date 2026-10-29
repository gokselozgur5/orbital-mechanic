# GitHub delivery tracking

Created 2026-10-24. GitHub is the authority for current open/closed state; this file is the navigation index. Phase definitions and gates: [PHASES.md](PHASES.md).

## [Phase 0 — Design and setup](https://github.com/gokselozgur5/orbital-mechanic/milestone/1)

- [#1 — [P0] Accept Phase 0 scope and setup evidence](https://github.com/gokselozgur5/orbital-mechanic/issues/1)

## [Phase 1 — Safe motion](https://github.com/gokselozgur5/orbital-mechanic/milestone/2)

- [#2 — [P1] Review and accept the fail-closed motion policy](https://github.com/gokselozgur5/orbital-mechanic/issues/2)
- [#3 — [P1] Integrate guarded velocity commands with ROS and Gazebo](https://github.com/gokselozgur5/orbital-mechanic/issues/3)
- [#4 — [P1] Verify physical stopping and process-failure behavior](https://github.com/gokselozgur5/orbital-mechanic/issues/4)
- [#5 — [P1] Provide bounded operator controls and a usable motion view](https://github.com/gokselozgur5/orbital-mechanic/issues/5)

## [Phase 2 — Navigation](https://github.com/gokselozgur5/orbital-mechanic/milestone/3)

- [#6 — [P2] Establish the known map and localization contract](https://github.com/gokselozgur5/orbital-mechanic/issues/6)
- [#7 — [P2] Integrate Nav2 through guarded motion](https://github.com/gokselozgur5/orbital-mechanic/issues/7)
- [#30 — [P2] Complete native navigation acceptance after integration merge](https://github.com/gokselozgur5/orbital-mechanic/issues/30)

## [Phase 3 — Maintenance capabilities](https://github.com/gokselozgur5/orbital-mechanic/milestone/4)

- [#8 — [P3] Connect machine inspection to trusted robot state](https://github.com/gokselozgur5/orbital-mechanic/issues/8)
- [#9 — [P3] Implement interlocked causal maintenance actions](https://github.com/gokselozgur5/orbital-mechanic/issues/9)

## [Phase 4 — Scripted mission](https://github.com/gokselozgur5/orbital-mechanic/milestone/5)

- [#10 — [P4] Build independent scenario evaluation and trace evidence](https://github.com/gokselozgur5/orbital-mechanic/issues/10)
- [#11 — [P4] Complete the scripted end-to-end maintenance mission](https://github.com/gokselozgur5/orbital-mechanic/issues/11)
- [#12 — [P4] Freeze the baseline suite and reproducible demo](https://github.com/gokselozgur5/orbital-mechanic/issues/12)

## [Phase 5 — Claude task planner](https://github.com/gokselozgur5/orbital-mechanic/milestone/6)

- [#13 — [P5] Define and test the versioned planner tool contract](https://github.com/gokselozgur5/orbital-mechanic/issues/13)
- [#14 — [P5] Add Anthropic adapter with explicit spending controls](https://github.com/gokselozgur5/orbital-mechanic/issues/14)
- [#15 — [P5] Evaluate Claude against the scripted baseline](https://github.com/gokselozgur5/orbital-mechanic/issues/15)

## Current handoff

Phase 0 scope and setup are accepted based on the user-approved baseline, docs/EVIDENCE.md, confirmed Mac Gazebo/Foxglove viewing and this dependency-ordered backlog. Its acceptance issue and milestone are closed after publishing these documents.

Policy review #2, ROS integration #3, failure acceptance #4 and operator controls
#5 are complete for the recorded Phase 1 baseline. Evidence is in STATUS.md,
MOTION.md, MOTION_EVIDENCE.md and OPERATOR.md. The next active phase gate is native
navigation acceptance in #30; implementation merges do not close that gate.


## Navigation integration PR

[PR #16](https://github.com/gokselozgur5/orbital-mechanic/pull/16) was merged by the
owner on 2026-10-29, closing #6 and #7 while preserving its individual commits.
The remaining source deployment, localization recovery, live routes, collision
and physical stopping checks are tracked in #30. Native arm and CLI protocol
proofs pass, but do not close those physical acceptance gates or the milestone.

## Scripted mission contract

The owner merged the machine, maintenance and scripted-mission implementations in
[PR #19](https://github.com/gokselozgur5/orbital-mechanic/pull/19),
[PR #20](https://github.com/gokselozgur5/orbital-mechanic/pull/20) and
[PR #21](https://github.com/gokselozgur5/orbital-mechanic/pull/21) on 2026-10-29.
They include the [state contract](MISSION_STATE.md),
[public diagnosis policy](MISSION_POLICY.md) and
[persistent gateway](MISSION_RUNTIME.md). These bind actual client/owner results,
durable intent, evidence and bounded cancellation while preserving uncertainty.
Issues #8, #9 and #11 remain open for their native acceptance. Interrupted-mission
reconciliation, independent evaluation #10 and the full scenario suite remain
pending; local adapter tests and merged implementations do not close those gates.
