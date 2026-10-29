# Private station scenario inputs

OM-071 versions initial station conditions and binds their exact bytes to an explicit new-world generation.

The actual telemetry/maintenance process consumes those conditions under its existing owner lock; it no longer declares or reads a ROS `scenario` parameter.

This implements local fixture preparation, not independent scoring, native trial acceptance or an operating-system boundary against another process using the same account.

## Schema and repeatability

Private schema 1 identifies `station-profile-v1`, an unsigned 32-bit seed, a random trial ID, registry identity, declared source revision, exact initial machine states and a navigation requirement.

The source revision hashes the generator, private loader, causal model and telemetry adapter model sources; the registry separately binds the configured world and known-map source.

These are declared file identities, not an attestation of loaded executable code or a running Gazebo world.

For the same seed/profile/source, initial conditions repeat; trial IDs remain independently random.

`seed % 3` chooses pump, cooler or power unit, in that order.

Each machine starts powered, with temperature `28 + (first four SHA-256 bytes modulo 601) / 100` degrees C; the hashed string is `station-profile-v1:temperature:SEED:MACHINE`.

Temperature variation is independent of profile; geometry and appearance currently remain fixed.

| Profile | Initial condition | Runnable station fixture |
| --- | --- | --- |
| healthy | All machines have no fault. | Yes |
| recoverable | Selected machine has one latched controller fault. | Yes |
| persistent | Selected machine has a mechanical jam. | Yes |
| ambiguous | Selected machine and its next cyclic neighbor have equal excess loads from latched faults. | Yes |
| blocked | Selected machine has a latched fault and requires a blocked navigation scene. | No; preparation refuses until a geometry-bound scene exists. |

The ambiguous fixture exercises conflicting candidate loads; it does not implement the missing/noisy telemetry scenario required by the full contract.

The blocked descriptor never fabricates a navigation failure or silently runs on an unobstructed world.

Causal equations, load/vibration values, thermal time constant and ten-second recovery window remain defined in `orbital/machines.py` and are included in the source identity.

## Operator launch

The normal `bash scripts/launch-thor.sh` prepares recoverable/seed 0 for a fresh world.

To select another fixture, first stop the old demo and confirm its service cleanup, then run the trusted operator preparation:

```bash
python3 scripts/maintenance-journal.py prepare --new-world --profile persistent --seed 17
bash scripts/launch-thor.sh --trial-id TRIAL_ID_FROM_OUTPUT
```

Preparation prints only a public assignment and `new_world_required=true`; it never prints the private input, profile, seed or private digest.

The assignment allowlist is schema, kind, random trial ID, registry identity and the common `restore_station_power` task.

Only the opaque trial ID enters the long-lived launch arguments and `ORBITAL_TRIAL_ID` environment; fault selection belongs to the short-lived trusted preparation command.

The contained launcher keeps its existing active-service refusal, validates the selected unclaimed input before starting components, and forwards that identity to station startup.

The station checks it again under the owner lock, so replacing the prepared generation between preflight and startup cannot launch a different station silently.

Validation is not a reservation or proof that Gazebo is stopped; the existing service containment and explicit new-world requirement still apply.

`check-prepared --trial-id ID` returns the public assignment without claiming or replacing it; a consumed generation cannot be reused, even after clean shutdown.

Deploy launcher and station changes together during deliberate new-world commissioning; legacy history remains readable but cannot launch the live station without a new configured generation.

## Retention and refusal

Private files live in `runtime/station-service/scenario-inputs/<opaque-ID>.json`, outside Git.

The directory must be owned by the current user with mode 0700; files must be 0600 regular files with one link, at most 16 KiB, with at most 256 retained directory entries before new preparation refuses.

The writer uses exclusive creation, fsync and read-back before recording a versioned scenario event containing the file SHA-256; failed/partial files are retained and never silently overwritten or pruned.

The reader refuses links, mode changes, oversized or duplicate-key JSON, unsupported schemas, mismatched registry/source/seed/conditions, changed bytes and replaced directory/file identities.

Claiming the configured generation requires its exact private digest; an unrelated default model cannot claim it.

The retained input and sources are rechecked before new measurements and again after effect intent persistence, before final live interlocks and mutation.

A changed input cannot be restored into permission within that process; station restart still refuses the consumed generation.

This protects cooperative ownership and accidental public disclosure, not malicious same-user reads/edits, coherent journal rollback, another checkout or authenticated DDS access.

[Station evaluation](STATION_EVALUATION.md) now consumes the exact private generation and retained actual station state offline, after an authoritative clean closure.

The full hidden-truth isolation/leakage audit, enforced process/filesystem boundary, blocked geometry, noisy/missing observations, held-out seeds, complete independent mission scoring and 25 native trials remain open under #10/#12.

## Local validation

Tests execute the shipped ROS adapter with controlled transport, four prepared profiles, public packet decoding, real private file operations, journal claims and actual isolation/reset effects under controlled robot/camera inputs.

The shipped shell preflight is executed with component startup cut off before cache, ROS or Gazebo creation; systemd is a controlled peer.

Negative controls cover coherent input replacement, read/write/sync/capacity failures, late changes before an actual effect, stale or consumed launch IDs and generation changes between preflight and station startup.

These fixture checks do not score a mission or establish native timing; station-axis scoring has its own tests and still does not replace independent collision/interlock evidence.
