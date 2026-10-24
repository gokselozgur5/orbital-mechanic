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

Next: issue #2 (motion policy review), then #3 (ROS integration), #4 (failure acceptance), #5 (operator controls). The existing policy is a tested prototype. The ROS adapter, bridge edit and motion-check script are still undeployed drafts; do not report Phase 1 as complete.
