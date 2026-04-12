#!/usr/bin/env bash

# Convenience overlay for this workspace.
# Usage:
#   cd ~/night_patrol_ws
#   source setup.bash

_ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ ! -f "$_ws_dir/install/setup.bash" ]; then
  echo "[night_patrol_ws] Missing install/setup.bash. Build first:" >&2
  echo "  cd $_ws_dir && source /opt/ros/humble/setup.bash && colcon build" >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1091
source "$_ws_dir/install/setup.bash"

unset _ws_dir