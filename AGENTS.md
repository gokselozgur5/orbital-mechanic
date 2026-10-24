# Agent handoff

Read README.md and docs/STATUS.md before working. Keep technical documentation in English.

This is a concept scaffold, not a working robotics application. Do not claim simulation, navigation or repair capabilities until implemented and tested. Update docs/STATUS.md when meaningful project state changes.

Keep work scoped to the single-room maintenance MVP. Verify host and simulator compatibility before pinning dependencies. Implement and test a deterministic baseline before adding the LLM.

Keep hidden fault state out of planner observations. Enforce safety interlocks outside the planner. The LLM requests bounded actions; ROS 2 owns motor control.

Never commit credentials, private media, model weights, build outputs or recordings. Do not provision paid resources without authorization. Check staged files for secrets and unintended artifacts before publishing.
