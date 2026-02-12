#!/usr/bin/env bash
# Run the Hybrid A* Docker image with RViz (X11 GUI).
# Prerequisites (on macOS):
#   1. Install XQuartz, log out/in, then open XQuartz.
#   2. XQuartz → Preferences → Security → "Allow connections from network clients"; restart XQuartz.
#   3. In this terminal (before running this script):  xhost +localhost

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

docker run --rm -e DISPLAY=host.docker.internal:0 hybrid_astar:noetic \
  bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch hybrid_astar manual.launch"
