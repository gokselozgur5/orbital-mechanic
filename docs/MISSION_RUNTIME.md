# Persistent scripted mission gateway

`scripts/run-scripted-mission.py run` connects the public mission policy to the
existing navigation, inspection and maintenance owners through one persistent
ROS session. It does not start prerequisites, inject scenarios or read hidden
fault labels. This is the local OM-066 implementation; native mission acceptance
and independent scenario scoring remain open.

## Admission and execution

The gateway acquires the private `runtime/mission-service` owner lock, validates
its append-only `events.jsonl`, and refuses an unfinished mission before creating
ROS publishers. One explicit invocation admits one new mission UUID. Every
original command has an exact durable intent before transport publication.

The adapter pins publisher names, namespaces, types and endpoint GIDs for public
clock/telemetry and the three owner protocols. Each command topic must have only
the gateway publisher and the expected single owner subscriber. File/graph reads
finish before the final receipt-age and simulation-time checks. Sampled graph
checks and a checkout-local lock are cooperative fences, not DDS authentication
or cross-host access control.

The policy chooses a machine from public measurements. Navigation requests a
target, arms that exact target and waits for the matching measured arrival.
Inspection acquires the existing owned camera artifact. A recoverable diagnosis
then requests isolate, reset and restore in order, each bound to the prior exact
result and control revision. A fresh ten-second public recovery window is still
required. Healthy missions inspect all three machines without maintenance.
Ambiguous/persistent cases escalate without claiming electrical isolation or
physical stopping.

The wall-clock budget includes source discovery and startup. The default is 600
seconds, with explicit values from 1 through 600. Action waits use the remaining
budget. Bounded cancellation/reconciliation may extend beyond the mission
deadline to observe the already-owned action; they never retransmit it.

## Cancellation and uncertainty

SIGINT/SIGTERM request cancellation. The driver stops admitting new work and
attempts at most one exact cancellation of its current target/action. Cleanup
may use that family's pinned sources despite loss of an unrelated public source.
A changed owner generation cannot be adopted. An ACK is not completion.

Missing navigation ACKs retain the unknown target. Lost maintenance observations
retain the original action even when the station actually changed. Observed
effects remain in the mission result after cancellation; there is no automatic
reset, restoration or rollback. This runner does not invoke `cancel_isolate` or
claim final electrical isolation for every cancellation path. Full native
containment remains part of OM-068 acceptance.

Completion requires no pending action, idle/guard-latched navigation, idle
inspection/maintenance owners, matching station/control identity and fresh owner
observations retained in the terminal journal record. `complete` means that the
gateway finalized its observed result; inspect `mission.outcome` separately.
The CLI exits zero only for finalized healthy, recovered or escalated outcomes.
Failed node cleanup makes the report incomplete even after a durable terminal
record; that record still states the earlier observed quiescence, not physical
or graph disappearance after cleanup.

Cleanup shuts down only a ROS context successfully initialized by this adapter.
An initialization refusal cannot shut down a context already owned by its caller;
a node-construction failure still cleans up this adapter's newly created context.

## Durable history and restart boundary

Events have ordered SHA-256 links and strict bounded records. The private regular
file, directory and lock identities are checked around reads/writes. Appends use
one write followed by fsync; short writes/sync failures latch unavailable, with
no truncation, repair or retry. Limits are 32 MiB, 16,384 events, 64 missions and
64 KiB per event. Exhaustion refuses further work.

Process death releases the lock but does not clear durable admission. An
unfinished, changed or unreadable history refuses another mission. There is no
resume/reset/clear command in this delivery: retain the journal and diagnose the
exact remote ownership. OM-069 still requires evidence-supported reconciliation;
do not delete history to bypass it. The integrity chain does not protect against
coherent same-user rollback or administrative filesystem replacement.

## Explicit invocation and validation

Only after commissioning the prerequisite owners, from the Orbital workspace in
Thor's `orbital-work` tmux session:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 scripts/run-scripted-mission.py run --timeout 600
```

This command can move the simulated robot and operate the station model. Domain
84 is reserved for explicitly isolated fixtures after an empty-graph check;
domains 77/78 and other projects remain out of scope. The CLI starts no world or
service and makes no model/provider call.

Local tests compose the actual ROS adapter, clients, capture/manifest reader and
durable station through controlled graph/clock/navigation peers, worker lifecycle
and hold feedback. They cover recovery for pump/cooler, three healthy inspections,
ambiguity/persistence, blocked routes, cancellation, lost observations, durable
admission after actual SIGKILL and storage failures. Final graph-read delays
originally admitted stale publication/finalization; the same controls pass after
the final freshness fence. None of these proves native DDS/Nav2, camera-child
mission execution, collision freedom, rendered diagnosis or independent scoring.
