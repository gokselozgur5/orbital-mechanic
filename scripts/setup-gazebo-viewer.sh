#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TOOLS="$ROOT/../.local"
mkdir -p "$TOOLS/apps" "$TOOLS/venvs"
python3 -m venv "$TOOLS/venvs/orbital-viewer"
"$TOOLS/venvs/orbital-viewer/bin/pip" install websockify==0.13.0
if [[ ! -e "$TOOLS/apps/orbital-novnc" ]]; then
  git clone --depth 1 --branch v1.7.0 https://github.com/novnc/noVNC.git "$TOOLS/apps/orbital-novnc"
fi
[[ $(git -C "$TOOLS/apps/orbital-novnc" rev-parse HEAD) == 63107bd06d9e1f6136ff21aeda8cd62cbf0d433e ]] || { echo 'Unexpected noVNC revision'; exit 1; }
