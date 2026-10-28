#!/usr/bin/env bash
# Foreground owner inside orbital-navigation.service; no lifecycle activation.
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
mkdir -p runtime/navigation runtime/ros runtime/tmp
export ROS_LOG_DIR="$ROOT/runtime/ros" TMPDIR="$ROOT/runtime/tmp"
source scripts/ros-env.sh
/usr/bin/python3 scripts/check-nav2-compatibility.py > runtime/navigation/compatibility.json
/usr/bin/python3 scripts/navigation-manifest.py > runtime/navigation/source-manifest.json
/usr/bin/python3 scripts/check-map-binding.py --localization-only > runtime/navigation/map-before.json
/usr/bin/python3 scripts/check-navigation-graph.py --phase before > runtime/navigation/graph-before.json
/usr/bin/python3 scripts/navigation-manifest.py --verify runtime/navigation/source-manifest.json \
  --evidence runtime/navigation/graph-before.json > runtime/navigation/source-verification.json
exec /usr/bin/python3 scripts/navigation-service.py
