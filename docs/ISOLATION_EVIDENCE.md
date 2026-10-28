# Retained isolation observations

`record-isolation.py` observes one exact maintenance run/mission/action and
retains the public source messages needed to check its model postcondition
again. It subscribes to the command, state, result and station telemetry topics.
It creates no control publisher, service client, automatic retry or cancellation.
Stopping the observer does not stop or resolve the maintenance owner.

This is independent evaluation of a **sampled model power state**. It does not
independently verify camera/pose admission, hold release, continuous physical
stationarity, electrical isolation or repair. Those gates remain separate.
The owner still enforces the [maintenance endpoint](MAINTENANCE_ENDPOINT.md).

## Capture one operation

The configured native services and source manifest must already be verified.
Use the owned Thor checkout, tmux `orbital-work`, localhost ROS domain 42 and
Gazebo partition `orbital-maintenance`. Domain 84 is permitted only for an
already isolated commissioning graph. The observer starts no service or world.
Native navigation/inspection/hold commissioning remains pending.

Read the current maintenance `run_id` with:

```bash
python3 scripts/maintain-machine.py status
```

Choose fresh mission/action IDs and use the same values in both panes. Replace
the run placeholder with the exact current 32-character hexadecimal ID.

```bash
maintenance_run='COPY_CURRENT_RUN_ID_HERE'
maintenance_mission=$(python3 -c 'import uuid; print(uuid.uuid4().hex)')
maintenance_action=$(python3 -c 'import uuid; print(uuid.uuid4().hex)')
python3 scripts/record-isolation.py --name pump-isolation-001 \
  --machine-id pump --run-id "$maintenance_run" \
  --mission-id "$maintenance_mission" --action-id "$maintenance_action" \
  --duration 60 --warmup 10
```

Wait for `runtime/evaluation/pump-isolation-001/ready.json` before issuing the
already authorized maintenance command in another pane with those exact IDs:

```bash
python3 scripts/maintain-machine.py isolate pump --run-id "$maintenance_run" \
  --mission-id "$maintenance_mission" --action-id "$maintenance_action" --timeout 15
```

The readiness marker means the observer has pinned the graph and obtained a
fresh powered baseline with a ready idle owner. It grants no action permission.
A command observed before readiness is refused. The run directory is exclusive;
never reuse an old directory or its readiness marker for a new capture.

The warmup limit is 0.1–30 seconds. The subsequent observation deadline is
0.1–120 wall seconds and includes waiting for the command, terminal result and
postcondition samples. A stalled simulation cannot extend it. An unresolved
owner, missing result, source loss or interruption leaves `complete=false` with
the bounded retained prefix. Use the endpoint's exact query procedure to inspect
an uncertain action; a missing recording never permits another isolation.

## Replay

```bash
python3 scripts/record-isolation.py --verify runtime/evaluation/pump-isolation-001
```

Replay needs no ROS, Gazebo or running service. It requires the same configured
registry/world bytes, decodes the retained wire messages again and checks the
JSONL sequence, length and SHA-256. A visitor cannot accept before the final
integrity check. Trace bounds are 64 MiB, 100,000 records and 65,536 bytes per
line; messages retain their existing smaller protocol limits. Output permissions
and exclusive paths use the same writer as trajectory evidence. Recordings stay
under ignored `runtime/evaluation`, never in Git.

The checker derives the exact request digest and compares run/mission/action,
machine, station generation and revision with the terminal effect. It resolves
the server's verification reference to the actual retained source packet. It
also requires two consecutive fresh post-effect packets with that exact control
identity, power off, zero current and zero vibration. A result may arrive before
its reference packet; callback order across topics is not treated as source
order. Bounded later observations must still supply the missing evidence.

Cancellation after an observed effect can yield `server_status=cancelled` with
`verified_isolation=true`. Cancellation before an effect yields
`verified_isolation=false` and `sampled_unchanged=true` only after two fresh
post-terminal packets retain the powered baseline and control revision. This
does not prove the absence of a transient unobserved effect. Exit 0 and
`complete=true` mean a complete consistent observation, not necessarily isolation
or mission success. Inspect the explicit fields.

## Binding and limits

The graph reader checks unique expected station publishers and the clock source,
pins their GIDs, and rechecks during every callback and polling turn. Native
navigation source/world identity and demo/localization/navigation service
invocations must agree before and after capture. These are sampled identity
checks, not cryptographic publisher authentication or per-message attribution.
The public command topic records intent; observing a command does not establish
who sent it or authorize it. Hashes detect changed retained bytes; they cannot
authenticate an entirely fabricated trace and manifest.

`hold_released_reported` preserves the server claim. This recorder does not
independently measure guard/nav ownership or copy the camera artifact. The
inspection reference remains in the complete retained result; validating those
artifacts and native movement containment is separate work.

Local controls execute the real station/owner and replay real published packets,
including the composed guard/navigation/capture-child flow. The adapter runs on
controlled ROS transport and refuses control interfaces in tests. Negative
controls cover contradictory measurements despite a matching forged digest,
missing packets/commands, run/revision/request changes, frozen/expired clocks,
graph/source changes, corruption, cancellation, bounds and cleanup. This is not
native DDS/timing, three-machine or physical acceptance. OM-053 native fault and
cancellation commissioning remains open.
