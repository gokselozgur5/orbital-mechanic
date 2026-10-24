#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$ROOT"/{logs,runtime/ros,runtime/gazebo-home,runtime/config,runtime/cache,runtime/data,runtime/tmp}
ulimit -c 0
# Gazebo 8 uses ~/.gz despite the XDG paths; keep the actual data in this project.
if [[ ! -e "$HOME/.gz" && ! -L "$HOME/.gz" ]]; then
  ln -s "$ROOT/runtime/gazebo-home" "$HOME/.gz"
fi
if [[ $(readlink -f "$HOME/.gz") != "$ROOT/runtime/gazebo-home" ]]; then
  echo 'Existing ~/.gz belongs elsewhere; preserve it and resolve the cache location first.' >&2
  exit 1
fi
export TMPDIR="$ROOT/runtime/tmp"
export ROS_LOG_DIR="$ROOT/runtime/ros"
export XDG_CONFIG_HOME="$ROOT/runtime/config" XDG_CACHE_HOME="$ROOT/runtime/cache" XDG_DATA_HOME="$ROOT/runtime/data"
source "$ROOT/scripts/ros-env.sh"
if [[ -f "$ROOT/compat_ws/install/setup.bash" ]]; then source "$ROOT/compat_ws/install/setup.bash"; fi
export DISPLAY=:42 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 QT_QPA_PLATFORM=xcb
children=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do kill -- -"$pid" 2>/dev/null || true; done
  # Bound local grace; systemd owns and terminates detached descendants.
  sleep .2
  for pid in "${children[@]}"; do kill -KILL -- -"$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT INT TERM
setsid Xtigervnc :42 -geometry 1440x900 -depth 24 -localhost yes -rfbport 5942 -SecurityTypes None > "$ROOT/logs/vnc.log" 2>&1 &
children+=("$!")
sleep 2
kill -0 "${children[0]}"
if command -v openbox >/dev/null; then
  setsid openbox > "$ROOT/logs/openbox.log" 2>&1 & children+=("$!")
fi
setsid gz sim -r --render-engine ogre "$ROOT/worlds/maintenance.sdf" > "$ROOT/logs/gazebo.log" 2>&1 & children+=("$!")
setsid ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$ROOT/config/bridge.yaml" > "$ROOT/logs/bridge.log" 2>&1 & children+=("$!")
setsid ros2 run robot_state_publisher robot_state_publisher "$ROOT/config/robot.urdf" --ros-args -p use_sim_time:=true > "$ROOT/logs/robot-state.log" 2>&1 & children+=("$!")
setsid /usr/bin/python3 "$ROOT/scripts/motion-guard.py" --ros-args -p use_sim_time:=true > "$ROOT/logs/motion.log" 2>&1 & children+=("$!")
setsid /usr/bin/python3 "$ROOT/scripts/machine-telemetry.py" --ros-args -p use_sim_time:=true > "$ROOT/logs/machines.log" 2>&1 & children+=("$!")
setsid ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765 > "$ROOT/logs/foxglove.log" 2>&1 & children+=("$!")
printf 'Demo running; VNC localhost:5942, Foxglove localhost:8765\n'
# Any component failure tears down this demonstration instead of leaving a partial stack.
wait -n "${children[@]}"
