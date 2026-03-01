#!/usr/bin/env bash
# Run inside Docker: Xvfb + VNC, then Hybrid A* (map, planner, RViz) + leader.
# From host: docker run --rm --platform linux/amd64 -p 5900:5900 hybrid_astar:noetic /catkin_ws/src/hybrid_astar/scripts/run_gazebo_with_astar.sh
# If port 5900 is in use, use another host port, e.g.:  -p 5901:5900  then connect to localhost:5901
# Then connect with a VNC client to the host port (password empty). Set 2D Nav Goal in RViz to see A* path.

set -e
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1

# Start virtual X server
Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!
sleep 2

# Start VNC server
x11vnc -display :99 -forever -nopw -rfbport 5900 -shared -listen 0.0.0.0 -q -bg
sleep 1

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# When USE_SIM_TIME is set (e.g. when also running Gazebo container), use sim time so TF from Gazebo is not TF_OLD_DATA
SIM_TIME_ARG=""
if [ "${USE_SIM_TIME}" = "true" ] || [ "${USE_SIM_TIME}" = "1" ]; then
  SIM_TIME_ARG="use_sim_time:=true"
fi
# Start Hybrid A* (map, planner, RViz) in background so we can run leader in foreground
roslaunch hybrid_astar manual.launch $SIM_TIME_ARG &
LAUNCH_PID=$!

# Wait for planner to advertise /sPath
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do
  sleep 1
  rostopic list 2>/dev/null | grep -q '/sPath' && break
done
echo "Hybrid A* and RViz are up. Connect VNC to localhost:5900 and set 2D Nav Goal."

# Run leader v1.2 (use python3 so it works without rosrun install; initializes bot at path start)
LEADER_SCRIPT="/catkin_ws/src/hybrid_astar/src/scripts/leader_v1.2.py"
if [ ! -f "$LEADER_SCRIPT" ]; then
  echo "Leader script not found: $LEADER_SCRIPT"
  kill $LAUNCH_PID 2>/dev/null || true
  kill $XVFB_PID 2>/dev/null || true
  exit 1
fi
echo "Starting leader (subscribes to /sPath, publishes to /turtlebot3_leader/cmd_vel)."
python3 "$LEADER_SCRIPT" || true

# Cleanup when leader exits
kill $LAUNCH_PID 2>/dev/null || true
kill $XVFB_PID 2>/dev/null || true
