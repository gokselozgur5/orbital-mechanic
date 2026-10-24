#!/usr/bin/env bash
# Mac-local viewer: browser -> WebSocket proxy -> authenticated SSH tunnel -> VNC.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TOOLS="$ROOT/../.local"
proxy="$TOOLS/venvs/orbital-viewer/bin/websockify"
assets="$TOOLS/apps/orbital-novnc"
[[ -x "$proxy" && -f "$assets/vnc.html" ]] || { echo 'Run scripts/setup-gazebo-viewer.sh first'; exit 1; }
mkdir -p "$ROOT/runtime"
if ! tmux has-session -t orbital-viewer 2>/dev/null 2>&1; then
  # Fixed local-only ports; authentication belongs to the verified SSH tunnel.
  tmux new-session -d -s orbital-viewer -c "$ROOT" \
    "exec '$proxy' --web '$assets' 127.0.0.1:15943 127.0.0.1:15942 > '$ROOT/runtime/viewer.log' 2>&1"
fi
for attempt in {1..20}; do
  if curl -fsS http://127.0.0.1:15943/vnc.html >/dev/null 2>&1; then
    open 'http://127.0.0.1:15943/vnc.html?autoconnect=true&resize=scale'
    exit 0
  fi
  sleep .2
done
echo 'Viewer failed to start; inspect runtime/viewer.log' >&2
exit 1
