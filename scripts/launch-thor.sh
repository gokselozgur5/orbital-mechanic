#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$ROOT/logs"
# The user-service cgroup includes Gazebo descendants that create new sessions.
# No enable/boot startup: the demo is an explicitly started transient service.
systemd-run --user --unit=orbital-demo --collect \
  --property="WorkingDirectory=$ROOT" \
  --property=KillMode=control-group --property=TimeoutStopSec=10 \
  --property="StandardOutput=append:$ROOT/logs/session.log" \
  --property="StandardError=append:$ROOT/logs/session.log" \
  /bin/bash "$ROOT/scripts/start-demo.sh"
