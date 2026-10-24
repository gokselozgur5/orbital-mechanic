#!/usr/bin/env bash
set -euo pipefail
[[ $(uname -s) == Darwin ]] || { echo 'Run this script on macOS'; exit 1; }
target=${1:?Usage: scripts/open-mac.sh user@verified-host}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$ROOT/runtime"
socket="$ROOT/runtime/thor-view.sock"
options=(-o StrictHostKeyChecking=yes)
if [[ -n ${THOR_KNOWN_HOSTS:-} ]]; then options+=(-o "UserKnownHostsFile=$THOR_KNOWN_HOSTS"); fi
if ! ssh "${options[@]}" -S "$socket" -O check "$target" >/dev/null 2>&1; then
  ssh "${options[@]}" -M -S "$socket" -fNT \
    -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
    -L 127.0.0.1:15942:127.0.0.1:5942 \
    -L 127.0.0.1:18765:127.0.0.1:8765 "$target"
fi
open 'vnc://localhost:15942'
app=${FOXGLOVE_APP:-"$ROOT/../.local/apps/foxglove/Foxglove.app"}
if [[ -d "$app" ]]; then
  open -a "$app" --args "--user-data-dir=$ROOT/runtime/foxglove-current" 'foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:18765'
else
  open 'foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:18765'
fi
printf 'Gazebo: VNC localhost:15942\nFoxglove: ws://localhost:18765\n'
