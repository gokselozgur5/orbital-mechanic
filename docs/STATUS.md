# Project status

Updated: 2026-10-24

## Confirmed direction

Fully virtual ROS 2 maintenance robot in an artificial-gravity space-station room. The user authorized architecture and planning work after repository creation.

## Completed

Documentation design v0.1: component ownership, planner/tool boundary, fault scenarios, safety behavior, evaluation gates and milestone backlog. These are proposed implementation decisions; no runnable robot exists.

## Proposed baseline

Ubuntu 24.04 amd64, ROS 2 Jazzy, Gazebo Harmonic, Nav2, ros2_control, Python mission/evaluation logic. One wheeled robot, three machines, a known map and proximity-gated maintenance controls. Scripted baseline precedes the LLM. No arm or electrical chip emulation in MVP.

## Next action

M0: select an existing development host and verify simulator camera/lidar output and performance before dependency pinning. See PLAN.md for exit gates.

## Open decisions and limitations

Host, exact package pins and LLM provider remain unresolved. No dependencies installed, simulator smoke test performed, paid resources provisioned or runtime tests passed. Documentation checks do not establish simulator compatibility on the eventual host.

## Host preflight — 2026-10-24

User-selected Thor host was reached after its ED25519 fingerprint was independently confirmed by the user. Observed Ubuntu 24.04.3, aarch64, L4T R38.2.2, 122 GiB RAM and 47 GiB free on a 95%-used root filesystem. Docker is present. No /opt/ros installation or gz executable was found by the initial checks. No installation or simulation workload was started.

The implementation target is now ARM64 Thor, diverging from the proposed amd64 baseline. Gazebo rendering and package availability still require a smoke test; do not treat M0 as passed. Do not delete existing data to make space. The old Thor R38.4 snapshot does not describe this inspected host. Keep addresses and authentication details outside repository documentation.
