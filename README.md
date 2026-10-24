# Orbital Mechanic

A simulated maintenance robot that investigates and repairs faults in a space station using ROS 2 and an LLM task planner.

**Status: concept and project scaffold. No simulator or robot implementation exists yet.**

## First mission

“Station power is dropping. Find the cause and fix it.”

In a single simulated room containing three machines, a mobile robot collects camera observations and temperature, current and vibration measurements. It forms a diagnosis, moves to the affected machine, performs an allowed maintenance action and measures again to verify recovery.

The first version uses interactive switches and reset controls. Physical part replacement and a robot arm come later.

## Proposed architecture

- Simulation: room, differential-drive robot, collisions, machine state and injected faults.
- ROS 2: sensor interfaces, robot state, navigation and bounded action execution.
- Task planner: an LLM receives observations and calls explicit tools to inspect, navigate, isolate power, reset and verify.
- Safety supervisor: enforces action preconditions, timeouts, stop behavior and maintenance interlocks independently of the LLM.
- Evaluation: repeatable scenarios with a hidden fault, an event trace and measurable outcomes.

The LLM does not drive the motor control loop. Fault ground truth belongs to the evaluator; the planner sees only observations. Machine telemetry may initially use a documented simplified model rather than an electrical or mechanical digital twin.

Claude Code can develop and test the system. Using it as the runtime task planner is an optional experiment; no provider is selected yet.

## MVP acceptance

1. Start a reproducible single-room scenario with three machines and one injected fault.
2. Accept the mission in natural language.
3. Gather evidence and identify the affected machine without reading hidden fault state.
4. Navigate to its maintenance control without collision.
5. Perform an interlocked corrective action and verify recovery from new measurements.
6. Report evidence, actions and outcome; stop safely on uncertainty or failure.

Include a healthy scenario and an unrecoverable fault so success cannot mean blindly resetting every machine. Record task success, incorrect interventions, collisions, elapsed time and planner calls.

## Roadmap

- [ ] Choose development host, ROS 2 distribution and simulator after checking compatibility.
- [ ] Build the room, robot and manual control baseline.
- [ ] Implement deterministic fault scenarios and machine telemetry.
- [ ] Add navigation and maintenance actions with safety tests.
- [ ] Complete the mission with a scripted baseline.
- [ ] Add the LLM tool interface and compare against that baseline.
- [ ] Extend to manipulation, battery constraints and multiple robots.

## Continue development

Read [AGENTS.md](AGENTS.md) and [docs/STATUS.md](docs/STATUS.md). This repository currently has documentation only; there are no installation commands, runnable demos or passing simulation tests to claim.
