#!/usr/bin/env bash
# Optional legacy bridge overlay for the existing Foxglove Studio 1.84 client.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/jazzy/setup.bash
sudo apt-get install -y libasio-dev libwebsocketpp-dev nlohmann-json3-dev ros-jazzy-rosx-introspection python3-colcon-common-extensions
mkdir -p "$ROOT/compat_ws/src"
checkout="$ROOT/compat_ws/src/foxglove_bridge"
if [[ ! -d "$checkout" ]]; then
  git clone --depth 1 --branch 0.8.5 https://github.com/foxglove/ros-foxglove-bridge.git "$checkout"
fi
[[ $(git -C "$checkout" rev-parse HEAD) == 41f96cc6053632a472d9a821989952771b1117f2 ]] || { echo 'Unexpected source revision'; exit 1; }
[[ -z $(git -C "$checkout" status --porcelain) ]] || { echo 'Compatibility source has changes; inspect first'; exit 1; }
cd "$ROOT/compat_ws"
export MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-select foxglove_bridge --allow-overriding foxglove_bridge --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
