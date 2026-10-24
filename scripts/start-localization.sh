#!/usr/bin/env bash
# Foreground entrypoint owned by orbital-localization.service.
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"
mkdir -p runtime/localization runtime/ros runtime/tmp
export ROS_LOG_DIR="$ROOT/runtime/ros" TMPDIR="$ROOT/runtime/tmp"
source scripts/ros-env.sh
# Refuse old/missing frame geometry before creating a competing localization tree.
timeout 20 /usr/bin/python3 scripts/check-frames.py > runtime/localization/frame-preflight.json
/usr/bin/python3 scripts/generate-known-map.py > runtime/localization/map-generation.json
map_yaml=$(/usr/bin/python3 -c 'import json; print(json.load(open("runtime/localization/map-generation.json"))["directory"]+"/map.yaml")')
children=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep .2
  for pid in "${children[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT INT TERM
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true autostart:=true use_composition:=False use_respawn:=False \
  "map:=$map_yaml" "params_file:=$ROOT/config/localization.yaml" &
children+=("$!")
# PartOf handles explicit stop; also notice an unexpected demo process exit.
(while systemctl --user is-active --quiet orbital-demo.service; do sleep .2; done; exit 1) &
children+=("$!")
wait -n "${children[@]}"
# An unexpected launch exit is a failure, even when ros2 launch returns zero.
exit 1
