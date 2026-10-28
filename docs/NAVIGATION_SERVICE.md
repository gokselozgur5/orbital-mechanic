# Navigation service ownership

`orbital-navigation.service` owns the localization health observer, guarded
navigation supervisor and Nav2 launch. It requires an already-running demo and
localization service. It never starts or restarts either prerequisite.

The launcher is `scripts/launch-navigation.py`; the foreground entry is
`start-navigation.sh` → `navigation-service.py`. The launcher is restricted to
Thor's Orbital checkout inside tmux. The service starts Nav2 **unconfigured**.
Activation, graph preflight, deployed-source binding and live route acceptance
remain separate gates. OM-010 deployed this service into the live simulation and verified guarded standby.
See STATUS.md for the source identity and evidence; route acceptance remains open.

## Failure contract

`orbital/service_owner.py` pins both prerequisite InvocationIDs before creating
children, then observes active/running state and identity on a steady loop. A
child exit, dependency exit, restart or status-query failure ends navigation.
Each child has its own process group. The transient unit's `KillMode=control-group`
and two-second stop timeout also contain descendants that create a new session
or ignore SIGTERM. No respawn or boot enablement is configured. Stopping navigation
does not stop its prerequisites.

Do not add `Requisite`, `Requires` or `PartOf` merely to express this dependency.
A native trial demonstrated that `Requisite` propagated an explicit localization
restart into a new navigation invocation even with `Restart=no`. The final unit
uses ordering only (`After`); startup checks and the runtime InvocationID watch
enforce the prerequisite without transferring restart jobs. A restarted dependency
requires a fresh explicit navigation launch. Unexpected exit of any required child
is a failure even if that child returned zero.

This ownership loop is not the motion watchdog. ROS health/command deadlines and
the independent motion guard retain responsibility for stopping velocity while
process teardown proceeds. Runtime dependency queries each have a one-second
timeout; fixture cleanup measurements are not physical braking bounds.

## Native systemd acceptance

On 2026-10-28, six cases passed using the same unit builder and foreground owner
as production, with uniquely named disposable service units:

| Case | Required result |
| --- | --- |
| Missing dependencies | Refuse to run children; do not start prerequisites. |
| Demo stop | End the entire navigation fixture cgroup. |
| Localization process failure | End navigation after unexpected dependency loss. |
| Localization restart | End navigation; never automatically start a new invocation. |
| Owned child failure | Stop sibling and detached descendants. |
| Explicit navigation stop | Remove all descendants, leave dependencies unchanged. |

The fixture includes a detached SIGTERM-ignoring child. Process identities come
from the actual cgroup membership and `/proc` start times. All tracked processes
ended. Existing `orbital-demo` and `orbital-localization` stayed active throughout;
no ROS nodes or world were started by this proof. The failed Requisite trial and
its systemd journal are retained alongside the final source-bound result in private
review artifacts. Final source SHA-256 for the shared owner:
`4cd4f5f3b902fae20772d5cedeb99a4a47810bfada313ae9bc105e2d41f895ec`.

To reproduce on Thor, inside tmux `orbital-work`, from the versioned checkout:

```bash
mkdir -p runtime
proof=$(mktemp -d "$PWD/runtime/service-proof.XXXXXX")
cp -R orbital scripts "$proof/"
/usr/bin/python3 "$proof/scripts/check-navigation-service.py"
```

The proof stops only its own unique units. Output and logs remain under the proof
copy's runtime directory. Do not infer live Nav2/guard teardown or stopping distance
from this fixture acceptance; those need the integrated deployment/fault tests.
