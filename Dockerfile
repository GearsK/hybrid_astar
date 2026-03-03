# Hybrid A* Path Planner - run with ROS Noetic in Docker (e.g. on macOS)
# On Apple Silicon: build with --platform linux/amd64 to use the amd64 image (emulated).
FROM osrf/ros:noetic-desktop-full

# Install OMPL, map_server, for RViz-over-VNC (xvfb, x11vnc), leader script deps (numpy, gazebo_msgs)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libompl-dev \
    ros-noetic-map-server \
    ros-noetic-gazebo-ros-pkgs \
    python3-numpy \
    xvfb \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace and clone / copy package
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Copy this package into the workspace (context is repo root)
COPY . /catkin_ws/src/hybrid_astar

# Ensure map and image exist so RViz shows the chosen maze (maps/map.yaml image: field)
RUN MAP_YAML=/catkin_ws/src/hybrid_astar/maps/map.yaml && \
    test -f "$MAP_YAML" && \
    MAP_IMG=$(grep '^image:' "$MAP_YAML" | sed 's/^image:[[:space:]]*//') && \
    test -n "$MAP_IMG" && test -f "/catkin_ws/src/hybrid_astar/maps/$MAP_IMG" || \
    (echo "Build error: maps/map.yaml or its image not found. Ensure maps/ contains the image referenced in map.yaml (e.g. map_maze.png)." && exit 1)

WORKDIR /catkin_ws

# Build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Scripts to run with virtual display + VNC (avoids XQuartz/GLX issues on macOS)
# Leader v1.2: init bot at path start (set_model_state) + odom-space fixes
RUN chmod +x /catkin_ws/src/hybrid_astar/scripts/run_rviz_vnc.sh \
    && chmod +x /catkin_ws/src/hybrid_astar/scripts/run_gazebo_with_astar.sh \
    && chmod +x /catkin_ws/src/hybrid_astar/src/scripts/leader_v1.2.py

# Source workspace in bashrc so it's ready when we run
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default: launch without RViz so it runs headless (no display required).
# For RViz GUI: use manual.launch and set DISPLAY (e.g. XQuartz on macOS).
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch hybrid_astar manual_nogui.launch"]
