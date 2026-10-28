# Production navigation integration

The motion boundary remains controller → `/navigation/raw_cmd_vel` → navigation
supervisor → `/motion/request` → motion guard → drive. Nav2 must never publish
directly to the drive or the guard output. The controller is the sole permitted
navigation velocity source; recovery behaviors and velocity smoothers are not
implicitly permitted additional writers.

## Installed artifact gate (OM-002)

Run inside `tmux orbital-work` on Thor:

```bash
/usr/bin/python3 scripts/check-nav2-compatibility.py
```

This read-only gate hashes the installed server executables, package manifests,
plugin exports/libraries, NavigateToPose interface and stamped-publisher header.
Missing executables, required plugins, incompatible interfaces, mixed versions
or an unreviewed release series fail before a configuration trial. It starts no
ROS nodes and commands no movement. Tests inject missing libraries, mixed
versions, incompatible action fields, missing stamped support and unavailable
executables. A matching manifest is not a dynamic-linker or lifecycle proof.

On 2026-10-28, Thor's nine required packages reported **1.3.13**, all artifact
checks passed and the source interface contained the expected goal/result fields.
The installed header and [Jazzy controller documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/)
agree that `enable_stamped_cmd_vel` defaults to false. Explicitly set it true;
the supervisor requires TwistStamped and preserves input age. Never introduce
an adapter that stamps old unstamped commands as fresh.

## Next gates

1. Configure the controller/local costmap with footprint and bounded speeds.
2. Configure the known-map planner/global costmap and navigation behavior tree.
3. Configure actual lifecycle nodes in an empty private domain, inspect their
   graph and parameters, and verify the real stamped publisher type.
4. Deploy the owned navigation service with shared-world preflight and a latched
   guard. Confirm that no command bypass or competing controller exists.
5. Run announced bounded motion, independent arrival/collision evaluation and
   failure stopping before closing Phase 2.

The artifact check does not complete any of steps 1–5. Private raw reports live
outside Git; retain their source hash when collecting later acceptance evidence.
