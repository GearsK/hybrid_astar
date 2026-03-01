#!/usr/bin/env bash
# Start topic relays inside the running Gazebo container so the leader (in the A* container)
# drives the TurtleBot3 in Gazebo:
#   /turtlebot3_leader/cmd_vel -> /cmd_vel   (leader commands -> Gazebo robot)
#   /odom -> /turtlebot3_leader/odom        (Gazebo robot odom -> leader)
# Usage: ./scripts/run_leader_relays_in_gazebo.sh
# Requires: Docker container named "gazebo" is already running (e.g. from Dockerfile_gazebo).
set -e
CONTAINER="${GAZEBO_CONTAINER:-gazebo}"
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "Error: Container '${CONTAINER}' is not running. Start the Gazebo container first."
  exit 1
fi
echo "Starting topic relays in container '${CONTAINER}'..."
docker exec -d "${CONTAINER}" /entrypoint.sh rosrun topic_tools relay /turtlebot3_leader/cmd_vel /cmd_vel
docker exec -d "${CONTAINER}" /entrypoint.sh rosrun topic_tools relay /odom /turtlebot3_leader/odom
echo "Relays started. In RViz (VNC), set a 2D Nav Goal; the TurtleBot3 in Gazebo should follow the path."
