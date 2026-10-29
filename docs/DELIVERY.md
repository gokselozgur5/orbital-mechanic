# Delivery workflow

[Project #6](https://github.com/users/gokselozgur5/projects/6) tracks the existing
phase issues and their pull requests. [PHASES.md](PHASES.md) defines acceptance;
[TRACKING.md](TRACKING.md) maps the issues to milestones.

## Board and queues

| View | Purpose |
| --- | --- |
| [Delivery Kanban](https://github.com/users/gokselozgur5/projects/6/views/1) | All work, in columns grouped by Status |
| [All Work](https://github.com/users/gokselozgur5/projects/6/views/2) | Assignment, labels, milestone and linked pull requests |
| [To-do](https://github.com/users/gokselozgur5/projects/6/views/3) | Scoped work awaiting prerequisites or implementation |
| [In Progress](https://github.com/users/gokselozgur5/projects/6/views/4) | Active implementation and acceptance, including draft PRs |
| [In Review](https://github.com/users/gokselozgur5/projects/6/views/5) | Ready work awaiting review or integration |
| [Done](https://github.com/users/gokselozgur5/projects/6/views/6) | Accepted and integrated deliveries |

The Status options are ordered **To-do → In Progress → In Review → Done**.
Moving a card does not prove acceptance, approve a PR or close a milestone.
Status is maintained explicitly; no automatic synchronization is claimed.

## Each issue and pull request

- Assign the actual owner. For the current personal project, that is
  `gokselozgur5`.
- Set a type label and an area label: `area:setup`, `area:motion`,
  `area:navigation`, `area:maintenance`, `area:evaluation`, `area:planner` or
  `area:delivery`.
- Select the phase milestone whose exit criteria the work advances. Workflow
  infrastructure uses `Delivery — Project workflow`. Do not invent due dates.
- Add both the issue and its PR to Project #6, with their actual Status.
- Describe the behavior, prerequisites and discriminating acceptance checks.
  Record the tested revision, result and any skipped or blocked check.
- Use `Closes #N` only when the PR covers that issue's complete acceptance.
  Use `Refs #N` for partial contributions; leave the issue open.

Read back the saved values. The repository's English issue and PR templates
provide prompts, but cannot automatically set a milestone or validate evidence.

## Commit, review and merge

Validate each atomic commit locally, then publish a batch once it is ready.
The model workflow checks pull requests and pushes to `main`; feature-branch
pushes do not start a duplicate run. Reading existing run logs starts no jobs.
Retry a failed run only after diagnosing its failure.

1. Keep each commit to one coherent change with its related tests and docs.
   Verify repo-local personal email `gokselozgur5@gmail.com` before committing.
2. Run the checks relevant to the changed behavior. Keep negative controls and
   failed attempts; distinguish local tests, native ROS tests and Gazebo evidence.
3. Push normally and verify the exact published head and CI. Keep incomplete
   deliveries draft / In Progress.
4. Move ready work to In Review. Request an eligible reviewer when available.
   With only the PR author as collaborator, the owner's solo-maintainer exception
   permits no reviewer assignment; it does not imply independent approval.
5. Inspect required reviews and protections, current head, checks and acceptance
   before merging. Do not bypass repository rules. Use a merge commit to preserve
   the atomic history.
6. Read back the merge, linked issue closure and Project status. Mark Done only
   for accepted scope. Close a phase milestone only after its complete exit gate.

Do not overwrite another author's issue, PR or review text without the owner's
explicit authorization. Keep credentials, private media and generated outputs out
of Git. Board work never authorizes paid resources or live planner calls.

## Navigation handoff

The owner merged [PR #16](https://github.com/gokselozgur5/orbital-mechanic/pull/16)
on 2026-10-29; its closing references closed #6 and #7. Preserve those records.
The remaining native deployment, route, collision and failure gates now live in
[#30](https://github.com/gokselozgur5/orbital-mechanic/issues/30), in the open
Phase 2 milestone. Merged source and isolated protocol proofs do not satisfy the
phase's physical acceptance criteria. Current evidence and the next deployment
step are in `docs/STATUS.md`, `docs/NAV2_INTEGRATION.md` and
`docs/NAVIGATION_CLIENT.md`.
