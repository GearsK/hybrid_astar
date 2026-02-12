# Hybrid A* Path Planner - run with ROS Noetic in Docker (e.g. on macOS)
# On Apple Silicon: build with --platform linux/amd64 to use the amd64 image (emulated).
FROM osrf/ros:noetic-desktop-full

# Install OMPL, map_server, and for RViz-over-VNC: virtual display + VNC server
RUN apt-get update && apt-get install -y --no-install-recommends \
    libompl-dev \
    ros-noetic-map-server \
    xvfb \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace and clone / copy package
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Copy this package into the workspace (context is repo root)
COPY . /catkin_ws/src/hybrid_astar

WORKDIR /catkin_ws

# Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Script to run RViz on virtual display and expose via VNC (avoids XQuartz/GLX issues on macOS)
RUN chmod +x /catkin_ws/src/hybrid_astar/scripts/run_rviz_vnc.sh

# Source workspace in bashrc so it's ready when we run
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default: launch without RViz so it runs headless (no display required).
# For RViz GUI: use manual.launch and set DISPLAY (e.g. XQuartz on macOS).
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch hybrid_astar manual_nogui.launch"]
