# Agent handoff

Read README.md and docs/STATUS.md before working. Keep technical documentation in English.

A passive Gazebo/ROS sensor demo and a ROS-free machine model are implemented. Do not claim autonomous navigation, integrated robot repair or LLM control. Read the current measured evidence and limitations in docs/STATUS.md. Update docs/STATUS.md when meaningful project state changes.

Keep work scoped to the single-room maintenance MVP. Verify host and simulator compatibility before pinning dependencies. Implement and test a deterministic baseline before adding the LLM.

Keep hidden fault state out of planner observations. Enforce safety interlocks outside the planner. The LLM requests bounded actions; ROS 2 owns motor control.

Never commit credentials, private media, model weights, build outputs or recordings. Do not provision paid resources without authorization. Check staged files for secrets and unintended artifacts before publishing.

Thor work belongs under `/home/jensen/.goske/codex/orbital-mechanic`; system packages use normal system paths. Use the user-service launcher so Gazebo descendants cannot survive stop/restart. Do not run a second world on the same Gazebo partition. Preserve the existing Mac Foxglove app; a newer client is installed separately under the local workspace. No Anthropic credential or paid call has been used.

Follow docs/PHASES.md and docs/TRACKING.md. Use atomic commits: one coherent delivery with related tests; do not combine unrelated issues or create artificial commit counts. Thor operational commands run inside tmux orbital-work, while the simulator remains owned by its user-service cgroup. Do not deploy the existing motion integration drafts before their Phase 1 acceptance checks.
