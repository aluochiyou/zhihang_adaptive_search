#!/usr/bin/env bash
# PX4 SITL environment setup for V6.7.19 portable
# This script adds PX4_Firmware to ROS_PACKAGE_PATH and sets Gazebo paths

PX4_ROOT="${ZHIHANG_PX4_ROOT:-$HOME/PX4_Firmware}"

if [[ -d "$PX4_ROOT" ]]; then
  # Source PX4 Gazebo setup
  if [[ -f "$PX4_ROOT/Tools/setup_gazebo.bash" ]]; then
    source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
  fi
  
  # Add PX4 to ROS_PACKAGE_PATH
  export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"
fi
