#!/usr/bin/env bash
# Source this file for project ROS tools; do not execute it in a child shell.
ORBITAL_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=orbital-maintenance
export FASTRTPS_DEFAULT_PROFILES_FILE="$ORBITAL_ROOT/config/fastdds-local.xml"
