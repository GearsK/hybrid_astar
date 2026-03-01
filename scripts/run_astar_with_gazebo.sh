#!/usr/bin/env bash
# Run Gazebo (gz sim) + Hybrid A* (map, planner, RViz) + leader in one go.
# You get: Gazebo window (3D) and RViz (map + A* path when you set a goal).
#
# Usage:
#   ./run_astar_with_gazebo.sh              # gz sim with empty GUI
#   ./run_astar_with_gazebo.sh shapes.sdf   # gz sim with world file
#   ./run_astar_with_gazebo.sh --no-gz      # skip Gazebo (only ROS stack + leader)
#
# In RViz: use "2D Pose Estimate" for start, "2D Nav Goal" for goal.
# The A* path appears in RViz on /sPath. The leader follows /sPath and publishes
# cmd_vel (by default to /turtlebot3_leader/cmd_vel).

set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_GZ=true
GZ_WORLD=""

for arg in "$@"; do
  case "$arg" in
    --no-gz) RUN_GZ=false ;;
    *)       GZ_WORLD="$arg" ;;
  esac
done

# --- Source catkin workspace ---
CATKIN_WS=""
for d in "$REPO_DIR/../.." "$HOME/catkin_ws"; do
  if [ -f "$d/devel/setup.bash" ] && { [ -d "$d/src/hybrid_astar" ] || [ -L "$d/src/hybrid_astar" ]; }; then
    CATKIN_WS="$d"
    break
  fi
done
if [ -z "$CATKIN_WS" ]; then
  echo "Could not find catkin workspace with hybrid_astar. Source it first, e.g.:"
  echo "  source /path/to/catkin_ws/devel/setup.bash"
  [ -f "$HOME/catkin_ws/devel/setup.bash" ] && source "$HOME/catkin_ws/devel/setup.bash" || exit 1
else
  source "$CATKIN_WS/devel/setup.bash"
fi

# --- Optional: start roscore if not running ---
if ! rostopic list &>/dev/null; then
  echo "Starting roscore in background..."
  roscore &
  ROSTERM_PID=$!
  for i in 1 2 3 4 5 6 7 8 9 10; do
    sleep 1
    rostopic list &>/dev/null && break
  done
  if ! rostopic list &>/dev/null; then
    echo "roscore did not come up in time."
    kill $ROSTERM_PID 2>/dev/null || true
    exit 1
  fi
  echo "roscore is up."
fi

# --- Start gz sim (Gazebo) with GUI ---
if [ "$RUN_GZ" = true ]; then
  if ! command -v gz &>/dev/null; then
    echo "Warning: 'gz' not found. Skipping Gazebo. Install or source Gazebo Sim (e.g. gz-jetty)."
  else
    if [ -n "$GZ_WORLD" ]; then
      echo "Starting gz sim with: $GZ_WORLD"
      gz sim -v 4 $GZ_WORLD -g &
    else
      echo "Starting gz sim (empty GUI)."
      gz sim -v 4 -g &
    fi
    GZ_PID=$!
    sleep 2
  fi
fi

# --- Start Hybrid A* stack (map_server, hybrid_astar, RViz) ---
echo "Starting Hybrid A* (map + planner + RViz)..."
roslaunch hybrid_astar manual.launch &
LAUNCH_PID=$!

# Wait for planner and RViz to be ready
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
  sleep 1
  rostopic list 2>/dev/null | grep -q '/sPath' && break
done
if ! rostopic list 2>/dev/null | grep -q '/sPath'; then
  echo "Warning: /sPath not advertised yet; leader may wait for path."
fi
echo "Hybrid A* and RViz are up. In RViz: set 2D Nav Goal to see the A* path."

# --- Run leader v1.2 (follows /sPath, publishes cmd_vel; initializes bot at path start) ---
LEADER_SCRIPT="$REPO_DIR/src/scripts/leader_v1.2.py"
if [ ! -f "$LEADER_SCRIPT" ]; then
  echo "Leader script not found: $LEADER_SCRIPT"
  wait $LAUNCH_PID
  exit 1
fi

# Prefer python3 when script exists (works in Docker and when leader not installed)
if [ -f "$LEADER_SCRIPT" ]; then
  LEADER_CMD="python3 $LEADER_SCRIPT"
elif type rosrun &>/dev/null && rospack find hybrid_astar &>/dev/null 2>/dev/null; then
  LEADER_CMD="rosrun hybrid_astar leader_v1.2"
else
  LEADER_CMD="python3 $LEADER_SCRIPT"
fi

echo "Starting leader (subscribes to /sPath, publishes to /turtlebot3_leader/cmd_vel)."
echo "Use ./scripts/run_leader_gazebo.sh --remap if your robot uses /cmd_vel and /odom."
$LEADER_CMD
