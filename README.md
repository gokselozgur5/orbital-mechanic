# Orbital Mechanic

A simulated maintenance robot that investigates and repairs faults in a space station using ROS 2 and an LLM task planner.

**Status: guarded simulated motion and fresh sensor delivery are verified on ARM64 Thor. Operator CLI safety tests, the rendered Foxglove view and the frame-height correction are accepted; the complete runtime regression runner passes. Known-map generation and a machine model are tested offline. The native Nav2 stack passed guarded active standby, unseeded admission refusal, measured offset-seed convergence and cancellation after reseeding a moving native goal. Autonomous routes, integrated maintenance and LLM missions remain unverified.**

## Run the current prototype

On the configured Thor host:

```bash
cd /home/jensen/.goske/codex/orbital-mechanic
bash scripts/launch-thor.sh
```

Stop with `bash scripts/stop-thor.sh`. The demonstration runs as a transient user service; it does not start at boot. See [operations](docs/OPERATIONS.md) for installation, Mac SSH forwarding, Foxglove compatibility and sensor verification.

The world contains a wheeled robot, three machine props, camera, lidar, IMU and odometry. Machine telemetry comes from a separate simplified causal model. A public source registry binds its machine IDs to configured scene/control geometry. A read-only localized proximity query passes local tests; native proximity, inspection and maintenance actions remain uncommissioned.

Run the ROS-independent checks on a development machine:

```bash
python3 -m unittest discover -s tests -v
python3 scripts/run-machine-baseline.py
```

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

Anthropic is selected for the future task planner. No API credential is connected and no paid calls have been made. A scripted mission and independent evaluator come first; Claude Code may assist development separately from the runtime planner.

## MVP acceptance

1. Start a reproducible single-room scenario with three machines and one injected fault.
2. Accept the mission in natural language.
3. Gather evidence and identify the affected machine without reading hidden fault state.
4. Navigate to its maintenance control without collision.
5. Perform an interlocked corrective action and verify recovery from new measurements.
6. Report evidence, actions and outcome; stop safely on uncertainty or failure.

Include a healthy scenario and an unrecoverable fault so success cannot mean blindly resetting every machine. Record task success, incorrect interventions, collisions, elapsed time and planner calls.

## Roadmap

- [x] Select Thor ARM64, install ROS 2 Jazzy/Gazebo Harmonic, and verify sensor rendering.
- [x] Build the room, robot and passive sensor demonstration.
- [x] Implement and verify guarded manual controls and failure stopping.
- [x] Accept the composed operator view and complete Phase 1.
- [ ] Integrate localization and autonomous navigation.
- [x] Prototype deterministic machine faults and read-only ROS telemetry.
- [ ] Integrate robot proximity, inspection and maintenance with the model.
- [ ] Add navigation and maintenance actions with safety tests.
- [ ] Complete the mission with a scripted baseline.
- [ ] Add the LLM tool interface and compare against that baseline.
- [ ] Extend to manipulation, battery constraints and multiple robots.

## Design documents

- [Agreed phases](docs/PHASES.md)
- [GitHub phase and issue tracking](docs/TRACKING.md)

- [Hardware/software requirements](docs/REQUIREMENTS.md)
- [Operations and Mac viewing](docs/OPERATIONS.md)
- [Architecture and component ownership](docs/ARCHITECTURE.md)
- [Tool and scenario contracts](docs/CONTRACTS.md)
- [Delivery strategy, milestones and evaluation](docs/PLAN.md)

## Continue development

Read [AGENTS.md](AGENTS.md) and [docs/STATUS.md](docs/STATUS.md). The status file distinguishes measured capabilities, prototype-only behavior and remaining work. Foxglove client opening is user-confirmed and its camera/TF subscriptions are visible in the server logs.

[Named navigation commands](docs/NAVIGATION_CLIENT.md) provide explicit target
request, arm and cancel operations. Their local protocol/CLI tests pass; native
commissioning and route acceptance remain pending.

[Independent trajectory recording](docs/TRAJECTORY.md) measures Gazebo truth,
same-time localization and exact command identities without publishing controls.
Local observer tests pass; native capture and physical route acceptance are pending.

[Sampled collision evaluation](docs/COLLISION_EVALUATION.md) checks observed body,
wheel and support geometry against the recorded world. Local tests pass; this
does not establish continuous collision freedom or native route acceptance.

[Machine registry and telemetry](docs/MACHINE_REGISTRY.md) bind each public
measurement to configured scene identities, source revisions and simulation time.
Local integration tests pass; native commissioning remains pending.

[Fresh public station observations](docs/STATION_OBSERVATION.md) read the existing
telemetry through strict schema, source, sequence and dual-clock freshness checks.
The CLI returns measurements without inferring a diagnosis. Local publisher-to-reader
tests pass; native transport and joint robot/inspection integration remain pending.

[Measured machine proximity and inspection pose](docs/PROXIMITY.md) combine
registered control geometry, verified map binding, localization health and
exact-time robot TF. The query can require the measured inspection position,
heading and a fresh half-second stopped-odometry window. Local core/adapter tests
pass. Native measurement and inspection/action acceptance remain open; sampled
robot state alone grants no permission.
