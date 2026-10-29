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

Policy review #2 is complete. ROS integration #3 passed its Thor deployment and motion checks; evidence is in STATUS.md and MOTION.md. Failure acceptance #4 passed; detailed results are in MOTION_EVIDENCE.md. Operator controls and user-rendered camera/lidar/state/velocity panels now pass #5; accepted identities and evidence are in OPERATOR.md. Phase 1 is accepted for that baseline. Next: #6, beginning with live verification of the prepared frame-height correction, then known-map localization.


## Navigation integration PR

[Draft PR #16](https://github.com/gokselozgur5/orbital-mechanic/pull/16) links #6 and
#7 with closing references. Both issues remain open until localization recovery,
live route/obstacle/stopping acceptance and review are complete. Native isolated
configuration and service proofs are recorded in STATUS.md; they do not close the
live acceptance gates. Preserve the individual atomic commits when merging.

## Scripted mission contract

Issue [#11](https://github.com/gokselozgur5/orbital-mechanic/issues/11) now has a local
[executable state contract](MISSION_STATE.md) on `codex/scripted-mission`, stacked
on the maintenance work in [draft #20](https://github.com/gokselozgur5/orbital-mechanic/pull/20).
It binds existing client/owner results and preserves uncertain ownership.
[Draft #21](https://github.com/gokselozgur5/orbital-mechanic/pull/21) also carries the
[public inspection/diagnosis policy](MISSION_POLICY.md), with bounded uncertainty
and artifact-to-source validation. The live mission gateway, independent evaluation
#10 and native scenario acceptance remain open. Issue #11 and draft #21 stay
In Progress; local policy tests do not close the earlier phase gates or this issue.
