#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$ROOT/logs"
# A stopped transient unit can remain loaded briefly while systemd collects it.
# Wait for collection instead of racing a second unit with the same name.
for attempt in {1..50}; do
  state=$(systemctl --user show orbital-demo.service --property=LoadState --value)
  [[ $state == not-found ]] && break
  active=$(systemctl --user show orbital-demo.service --property=ActiveState --value)
  if [[ $active == active || $active == activating ]]; then
    echo 'Demo is already running; stop it explicitly before starting another world.' >&2
    exit 1
  fi
  if [[ $active == failed ]]; then
    # --collect can unload a failed transient unit after the state read.
    if ! systemctl --user reset-failed orbital-demo.service 2>/dev/null; then
      [[ $(systemctl --user show orbital-demo.service --property=LoadState --value) == not-found ]] || {
        echo 'Could not reset the previous failed demo unit.' >&2; exit 1;
      }
    fi
  fi
  sleep .1
done
[[ $(systemctl --user show orbital-demo.service --property=LoadState --value) == not-found ]] || {
  echo 'Previous demo unit has not finished cleanup.' >&2; exit 1;
}
# The user-service cgroup includes Gazebo descendants that create new sessions.
# No enable/boot startup: the demo is an explicitly started transient service.
systemd-run --user --unit=orbital-demo --collect \
  --property="WorkingDirectory=$ROOT" \
  --property=KillMode=control-group --property=TimeoutStopSec=2 \
  --property="StandardOutput=append:$ROOT/logs/session.log" \
  --property="StandardError=append:$ROOT/logs/session.log" \
  /bin/bash "$ROOT/scripts/start-demo.sh"
