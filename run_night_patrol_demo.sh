#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Error: /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?" >&2
  exit 1
fi

if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
  echo "Error: $WS_DIR/install/setup.bash not found." >&2
  echo "Build the workspace first:" >&2
  echo "  cd $WS_DIR && source /opt/ros/humble/setup.bash && colcon build" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

exec ros2 launch patrol_bot_gazebo night_patrol_demo.launch.py "$@"