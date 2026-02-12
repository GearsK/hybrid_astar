#!/usr/bin/env bash
# Run inside the container: start virtual display (Xvfb), VNC server, then RViz.
# From host: docker run --rm -p 5900:5900 hybrid_astar:noetic /catkin_ws/src/hybrid_astar/scripts/run_rviz_vnc.sh
# Then connect with a VNC client to localhost:5900 (password empty).

set -e
export DISPLAY=:99
# Software GL so OGRE/RViz can create a context on Xvfb (no GPU in container)
export LIBGL_ALWAYS_SOFTWARE=1

# Start virtual X server (resolution 1280x720, 24-bit color)
Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!
sleep 2

# Start VNC server (listen IPv4 only to avoid "Address already in use" on IPv6, quiet output)
x11vnc -display :99 -forever -nopw -rfbport 5900 -shared -listen 0.0.0.0 -q -bg
sleep 1

# Launch ROS with RViz (RViz will use :99)
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
roslaunch hybrid_astar manual.launch

# When roslaunch exits, clean up
kill $XVFB_PID 2>/dev/null || true
