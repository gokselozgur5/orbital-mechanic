#!/usr/bin/env bash
# Foreground owner in orbital-inspection.service; dependencies must already run.
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
mkdir -p runtime/ros runtime/tmp
export ROS_LOG_DIR="$ROOT/runtime/ros" TMPDIR="$ROOT/runtime/tmp"
source scripts/ros-env.sh
exec /usr/bin/python3 scripts/inspection-service.py
