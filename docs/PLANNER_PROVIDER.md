# Offline Messages proposals

OM-084 connects a bounded Messages client-tool dialect and an explicitly synthetic replay transport to the existing owned executive.

Recorded turns choose tools; the public diagnosis policy still constrains targets, intervention order and outcome claims.

This is an offline integration prototype, not a live Anthropic client or evidence of independent model diagnosis.

## Why a separate proposal exists

The native [tool contract](PLANNER_CONTRACT.md) requires the exact latest public packet at admission and again before publication.

A controlled five-second response delay advanced telemetry from sequence 8 to 258; the original request was correctly refused as `stale_observation`, with zero publications despite currently fresh telemetry.

Continuously renewing that native request would silently change its original precondition.

Instead, the model-facing input is a distinct proposal: `{basis_id, arguments}` for one named tool.

The host provides a planning frame containing a public observation, mission phase, arrival, next eligible operation, outcome, known evidence IDs and sticky vibration contradictions.

Before a new proposal can become a native request, the executive obtains a fresh public frame and compares source/registry identity, complete control state, powered states, public diagnosis classification and executive state.

Both wall and simulation age must remain below 30 seconds; ordinary admission also remains before the clipped mission/turn deadline.

Measurements may advance while these decision inputs remain equivalent; changed classification, control or a transient recorded vibration contradiction refuses the proposal.

A durable `provider_binding` records the original proposal, its planning basis, the separate fresh frame and the exact host-owned native request.

The existing native final packet, phase, target, source, owner and interlock checks then run unchanged.

This explicitly authorized host revalidation is narrower than accepting arbitrary stale commands, and it is not independent evidence of robot pose or physical safety.

`observe_station` asks for a new current read; `stop` remains independent of planning-frame freshness, while outcome reporting must still agree with the executive and known evidence.

## Messages subset and limits

The [Messages API](https://platform.claude.com/docs/en/api/messages/create) represents client calls as `tool_use` and accepts matching `tool_result` content in the next user turn.

Requests use [automatic tool choice with parallel calls disabled](https://platform.claude.com/docs/en/agents-and-tools/tool-use/parallel-tool-use); the decoder independently requires exactly one tool for a `tool_use` stop.

The fixed local dialect supports nonstreaming text/client-tool content, mandatory input/output token counts and optional cache-read/cache-creation counts.

Unknown fields, server tools, thinking blocks, parallel calls, duplicate JSON keys, nonfinite numbers and malformed arguments are refused before any proposal is dispatched.

Refusal, truncation, unsupported stop reasons and an `end_turn` without a supported outcome stop the mission; plain text never establishes success.

Requests currently specify `thinking: {type: disabled}`; [model support differs](https://platform.claude.com/docs/en/build-with-claude/thinking-troubleshooting), and this subset has not been validated against a real model or full current response envelope.

The fixture model name `orbital-recorded-fixture-v1` is not a provider model selection.

| Limit | Default or maximum |
| --- | --- |
| Turns, including exact replays | 32 |
| Reply bytes | 16 KiB |
| Request/history bytes | 256 KiB |
| Operator recording bytes | 1 MiB |
| Per-reply output-token claim | 2,048 default; 16–4,096 configurable |
| Total reported token claim | 100,000 default; 1–1,000,000 configurable |
| Rejected proposals before stopping | 2 default; 0–4 configurable |
| Reply/planning time | At most 30 seconds and remaining mission time |
| Native ordinary calls | Existing 32-call budget plus reserved stop |

Token accounting sums the supported reported usage fields; it is neither independently measured usage nor a dollar-spend guarantee.

No credentials, provider SDK, HTTP transport, environment-based model selection or automatic paid retry is present.

The transport's `submit`, `poll`, `cancel` and `close` are trusted nonblocking operations; callbacks, owner/source checks and cancellation continue while the fixture waits.

Cooperative checks cannot interrupt a blocked Python call or establish OS/resource isolation.

## Durable ownership

The private mission journal records provider configuration and each turn, reply, binding, receipt and end before the corresponding externally visible continuation.

A provider call ID maps deterministically into the host executive's UUID namespace; the model cannot supply native run, mission or action IDs.

An exact repeated completed proposal returns the original result and observations, including their old timestamps, without executing again.

A changed proposal under the same ID is refused; a bound proposal with an unresolved native effect remains pending.

A provider receipt must match the exact durable native request/result; neither a timeout nor a failed journal write can turn an unknown effect into a no-effect rejection.

Incomplete provider history prevents a terminal mission record, and a reopened unfinished mission refuses before transport construction.

Owned child SIGKILL checks cover the proposal binding before native admission, actual isolation before provider receipt and persisted receipt before returning it to the model.

These are process-death observations, not power-loss or native-robot proofs.

After a protocol error, exhausted budget or operator cancellation, the host attempts the existing reserved stop; the [durable operator-reset latch](PLANNER_EXECUTION.md) remains in force.

If source or ownership resolution fails, the mission remains unfinished for explicit reconciliation.

## Run a synthetic fixture

Local tests require no ROS installation or network:

```bash
python3 -m unittest discover -s tests -p 'test_planner_provider*.py' -v
python3 -m unittest discover -s tests -p 'test_planner_messages.py' -v
```

The existing ROS entrypoint accepts an operator-selected recording after the documented runtime prerequisites have been commissioned:

```bash
python3 scripts/run-scripted-mission.py run \
  --provider-replay /absolute/operator/fixture.json \
  --mission-text 'Station power is dropping. Find the cause and fix it.' \
  --provider-max-turns 16 --provider-max-reported-tokens 50000 \
  --timeout 600
```

Production domain 42 still requires the Thor workspace and `tmux`; isolated fixture domain 84 requires localhost discovery too.

`--provider-replay` and `--tool-contract` are exclusive, and provider settings are unavailable to `reset-tool-stop`.

The recording is loaded as one bounded stable regular file, refusing final symlinks, directories and FIFOs before ROS construction.

The file format is exactly `schema_version: 1`, `kind: planner_messages_recording`, `provenance: synthetic-test-fixture`, `model` and `steps`.

Each step has a finite `delay_s` from 0–60 and either `{response, bind}` or a zero-based `repeat` reference to an earlier rendered response.

The operator fixture may bind only `$current_basis` in `input.basis_id` and the exact `['$known_evidence']` list in `arguments.evidence_ids`; no model text, path or arbitrary field is interpreted as a template.

The test fixture author supplies expected tool sequences; the 60-second delay allowance exists to exercise the 30-second refusal.

`tests/planner_provider_fixture.py` and `tests/test_planner_provider_execution.py` contain the authored recipes; generated recordings and raw run artifacts stay outside Git.

## Evidence boundaries

Local composed runs use actual mission/ROS adapter code, owned clients, station histories and capture files, with controlled transport, navigation, robot and camera peers.

Healthy, recoverable, persistent and ambiguous profiles are scored independently; stopped and malformed-response runs cannot become success claims.

The recovery fixture includes five-second replies, actual isolation/reset/restoration and a measured new recovery window.

The model receives public data and bounded owner evidence, not the private scenario profile, seed, hidden fault or evaluator score.

Prompt-like text remains data in a recorded response; this does not establish a real model's resistance to prompt injection.

Native commissioning, process/OS separation, authenticated sources, independent pose/interlock evidence, full cancellation containment/final isolation, interrupted reconciliation, real blocked/noisy and held-out inputs, the 25-trial freeze and real model/cost comparisons remain open.

This is one coherent delivery for #13 with preparatory #14 work, not separate credit for OM-083/085/086/087/088 or complete Phase 5 acceptance.
