#!/usr/bin/env bash
# Explicitly invoked system installation; does not change L4T or reboot.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo 'Ubuntu 24.04 required'; exit 1; }
[[ $(dpkg --print-architecture) == arm64 ]] || { echo 'This recipe targets ARM64 Thor'; exit 1; }
free_kb=$(df -Pk "$ROOT" | awk 'NR==2 {print $4}')
(( free_kb > 14 * 1024 * 1024 )) || { echo 'At least 14 GiB free required before installation'; exit 1; }
mkdir -p "$ROOT/setup" "$ROOT/logs"
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  curl -fL https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb -o "$ROOT/setup/ros2-apt-source.deb"
  sudo dpkg -i "$ROOT/setup/ros2-apt-source.deb"
fi
sudo apt-get update
packages=(ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-foxglove-bridge ros-jazzy-gz-ros2-control ros-jazzy-navigation2 ros-jazzy-nav2-bringup tigervnc-standalone-server openbox)
apt-get -s install "${packages[@]}" > "$ROOT/logs/install-preview.log"
if grep -q '^Remv ' "$ROOT/logs/install-preview.log"; then
  echo 'Package removal proposed; inspect logs/install-preview.log before continuing'; exit 1
fi
sudo apt-get install -y "${packages[@]}"
dpkg-query -W -f='${binary:Package}\t${Version}\n' 'ros-jazzy-*' > "$ROOT/logs/ros-package-versions.tsv"
