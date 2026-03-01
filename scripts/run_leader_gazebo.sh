#!/usr/bin/env bash
# Run the leader controller with Gazebo (TurtleBot3).
# Prerequisites:
#   - ROS (Noetic/Melodic) and this package built in a catkin workspace
#   - Gazebo running with a TurtleBot3 (or similar diff-drive robot)
#   - Hybrid A* running (roslaunch hybrid_astar manual.launch) so /sPath is published
#
# The leader subscribes to /sPath and drives the robot via cmd_vel.
# By default it expects topics: /turtlebot3_leader/cmd_vel, /turtlebot3_leader/odom
# If your Gazebo robot uses /cmd_vel and /odom instead, use:
#   ./run_leader_gazebo.sh --remap

set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMAP=false
for arg in "$@"; do
  case "$arg" in
    --remap) REMAP=true ;;
  esac
done

# Prefer workspace that contains this repo
CATKIN_WS=""
for d in "$REPO_DIR/../.." "$HOME/catkin_ws"; do
  if [ -f "$d/devel/setup.bash" ] && [ -d "$d/src/hybrid_astar" ] || [ -L "$d/src/hybrid_astar" ]; then
    CATKIN_WS="$d"
    break
  fi
done
if [ -z "$CATKIN_WS" ]; then
  echo "Source your catkin workspace first, e.g.:"
  echo "  source /path/to/catkin_ws/devel/setup.bash"
  if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source "$HOME/catkin_ws/devel/setup.bash"
  else
    exit 1
  fi
else
  source "$CATKIN_WS/devel/setup.bash"
fi

LEADER_SCRIPT="$REPO_DIR/src/scripts/leader_v1.2.py"
if [ ! -f "$LEADER_SCRIPT" ]; then
  echo "Not found: $LEADER_SCRIPT"
  exit 1
fi

# Use installed script if available, else run from source
if type rosrun &>/dev/null && rospack find hybrid_astar &>/dev/null; then
  LEADER_CMD="rosrun hybrid_astar leader_v1.2"
else
  LEADER_CMD="python3 $LEADER_SCRIPT"
fi

if [ "$REMAP" = true ]; then
  echo "Running leader with remaps: turtlebot3_leader -> /cmd_vel, /odom"
  $LEADER_CMD \
    /turtlebot3_leader/cmd_vel:=/cmd_vel \
    /turtlebot3_leader/odom:=/odom
else
  echo "Running leader (expects /turtlebot3_leader/cmd_vel, /turtlebot3_leader/odom)"
  echo "If your robot uses /cmd_vel and /odom, run: $0 --remap"
  $LEADER_CMD
fi
