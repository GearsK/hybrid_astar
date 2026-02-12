#!/usr/bin/env bash
# Build and launch Hybrid A* from this clone (Linux with ROS Noetic/Melodic).
# Usage: ./scripts/setup_and_launch.sh [catkin_ws_dir]
# Default catkin_ws_dir: ~/catkin_ws

set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CATKIN_WS="${1:-$HOME/catkin_ws}"

echo "Using repo: $REPO_DIR"
echo "Using catkin workspace: $CATKIN_WS"

mkdir -p "$CATKIN_WS/src"
if [ ! -L "$CATKIN_WS/src/hybrid_astar" ] && [ ! -d "$CATKIN_WS/src/hybrid_astar" ]; then
  ln -sf "$REPO_DIR" "$CATKIN_WS/src/hybrid_astar"
  echo "Linked hybrid_astar into workspace."
fi

# Source ROS (try noetic then melodic)
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
  source /opt/ros/melodic/setup.bash
else
  echo "ROS not found. Install ROS Noetic or Melodic and source setup.bash first."
  exit 1
fi

cd "$CATKIN_WS"
catkin_make
source devel/setup.bash
roslaunch hybrid_astar manual.launch
