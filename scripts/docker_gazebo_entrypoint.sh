#!/bin/bash
# Source ROS and catkin workspace so turtlebot3_gazebo and launch files are found.
# Then run the given command (default: roslaunch turtlebot3_gazebo turtlebot3_world.launch).
set -e
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec "$@"
