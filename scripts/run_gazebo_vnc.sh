#!/usr/bin/env bash
# Run Gazebo + TurtleBot3 with a virtual display and VNC (no XQuartz needed).
# Start container with: -p 5902:5900 (then connect VNC to localhost:5902).
# Example:
#   docker run -it --rm --name gazebo --network rosnet -p 5902:5900 \
#     -e ROS_MASTER_URI=http://astar:11311 -e ROS_HOSTNAME=gazebo -e TURTLEBOT3_MODEL=burger \
#     hybrid_astar_gazebo:noetic /run_gazebo_vnc.sh
set -e
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1

Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!
sleep 2
x11vnc -display :99 -forever -nopw -rfbport 5900 -shared -listen 0.0.0.0 -q -bg
sleep 1

# Headless: set GAZEBO_HEADLESS=1 to skip gzclient (no VNC window; view robot in RViz only)
if [ "${GAZEBO_HEADLESS}" = "1" ] || [ "${GAZEBO_HEADLESS}" = "true" ]; then
  echo "Gazebo headless: gzserver only. View the robot in RViz (VNC to A* container)."
  exec /entrypoint.sh roslaunch turtlebot3_gazebo turtlebot3_world.launch gui:=false
fi

echo "Gazebo VNC: connect to localhost (port from -p 5902:5900, e.g. localhost:5902). No password."
exec /entrypoint.sh roslaunch turtlebot3_gazebo turtlebot3_world.launch
