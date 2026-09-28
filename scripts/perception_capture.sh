#!/usr/bin/env bash
# Run inside a sourced ROS workspace. Sensor QoS overrides live in the package.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 || "$1" != "record" && "$1" != "play" ]]; then
  echo "Usage: $0 record NEW_BAG_DIRECTORY [--sim-time] | play BAG_DIRECTORY" >&2
  exit 2
fi
config="$(ros2 pkg prefix ur7e_perception)/share/ur7e_perception/config/bag_qos.yaml"
if [[ "$1" == record ]]; then
  [[ ! -e "$2" ]] || { echo 'Refusing to overwrite existing bag' >&2; exit 2; }
  options=()
  if [[ $# == 3 ]]; then
    [[ "$3" == --sim-time ]] || { echo 'Unknown option' >&2; exit 2; }
    options+=(--use-sim-time)
  fi
  exec ros2 bag record --qos-profile-overrides-path "$config" -o "$2" \
    "${options[@]}" /camera/rgb /camera/depth /camera/camera_info /tf /tf_static
else
  [[ $# == 2 ]] || { echo 'play takes only a bag directory' >&2; exit 2; }
  exec ros2 bag play "$2" --clock --qos-profile-overrides-path "$config"
fi
