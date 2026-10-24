#!/usr/bin/env bash
# Explicitly start localization; never start or restart the simulator for it.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
[[ -n ${TMUX:-} ]] || { echo 'Run inside Thor tmux orbital-work.' >&2; exit 1; }
[[ $ROOT == /home/jensen/.goske/codex/orbital-mechanic ]] || {
  echo 'This launcher is restricted to the Thor simulation workspace.' >&2; exit 1;
}
systemctl --user is-active --quiet orbital-demo.service || {
  echo 'Start the simulator explicitly first.' >&2; exit 1;
}
# Requisite does not pull in/start the simulator; PartOf propagates its stop.
# A normal stop/start of the demo leaves localization stopped until requested.
[[ $(systemctl --user show orbital-localization.service --property=LoadState --value) == not-found ]] || {
  echo 'Existing localization unit must finish cleanup before a new launch.' >&2; exit 1;
}
mkdir -p "$ROOT/logs"
systemd-run --user --unit=orbital-localization --collect \
  --property="WorkingDirectory=$ROOT" \
  --property=Requisite=orbital-demo.service --property=After=orbital-demo.service \
  --property=PartOf=orbital-demo.service \
  --property=KillMode=control-group --property=TimeoutStopSec=2 \
  --property="StandardOutput=append:$ROOT/logs/localization.log" \
  --property="StandardError=append:$ROOT/logs/localization.log" \
  /bin/bash "$ROOT/scripts/start-localization.sh"
