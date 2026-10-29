#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/jazzy/setup.bash
cmake -S "$ROOT/native/tf_authority" -B "$ROOT/runtime/tf-authority-build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/runtime/tf-authority-build" --parallel 2
