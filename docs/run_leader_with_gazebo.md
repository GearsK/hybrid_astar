# Running the Leader Controller with Gazebo

The `leader.py` node drives a (TurtleBot3-style) robot in Gazebo by following the path published by the Hybrid A* planner (`/sPath`).

## One script: Gazebo + A* + leader

To run **gz sim** (Gazebo Sim) together with the Hybrid A* stack and the leader in one go (A* path visible in RViz):

```bash
# From repo root (with catkin workspace that contains this package)
./scripts/run_astar_with_gazebo.sh
```

- **Gazebo (gz sim)** opens with an empty GUI.
- **RViz** opens with the map; use **2D Pose Estimate** (start) and **2D Nav Goal** (goal). The A* path appears on `/sPath` in RViz.
- **Leader** runs in the foreground and follows `/sPath` (publishes to `/turtlebot3_leader/cmd_vel`).

With a world file (e.g. `shapes.sdf`):

```bash
./scripts/run_astar_with_gazebo.sh shapes.sdf
```

To skip starting Gazebo and only run the ROS stack + leader:

```bash
./scripts/run_astar_with_gazebo.sh --no-gz
```

The A* path is rendered in **RViz** (map + path display). The leader will drive a robot in Gazebo only if that robot’s topics are connected (e.g. via a ROS–Gazebo bridge or Gazebo Classic with TurtleBot3).

### Docker: A* + RViz + leader over VNC

Inside the Docker image (e.g. on macOS), use the script that starts a virtual display and VNC, then the full stack and leader:

```bash
docker run --rm --platform linux/amd64 -p 5900:5900 hybrid_astar:noetic /catkin_ws/src/hybrid_astar/scripts/run_gazebo_with_astar.sh
```

If port 5900 is already in use, use another host port (e.g. `-p 5901:5900`) and connect VNC to that port (e.g. `localhost:5901`). No password. In RViz set **2D Nav Goal** to see the A* path; the leader runs in the container and follows `/sPath`. (The image does not include Gazebo Sim; you see the planner and path in RViz.) If you also run the Gazebo container, use the **two containers** setup below and start the A* container with `-e USE_SIM_TIME=true` and `--network rosnet --name astar` to avoid TF_OLD_DATA warnings.

#### Gazebo container with XQuartz (macOS)

To run the Gazebo + TurtleBot3 container with the GUI on your Mac (XQuartz):

1. **Start XQuartz** (e.g. Applications → Utilities → XQuartz).
2. In your terminal, set the display so `xhost` can connect:  
   `export DISPLAY=:0`
3. Allow Docker to use the display:  
   `xhost +localhost`
4. Run the Gazebo container (see `Dockerfile_gazebo` for the full `docker run` command). Use `-e DISPLAY=host.docker.internal:0` and `-e LIBGL_ALWAYS_SOFTWARE=1` so the Gazebo GUI renders via XQuartz (software OpenGL).

If you see “unable to open display” when running `xhost`, ensure XQuartz is running and `DISPLAY=:0` is set in that terminal.

#### See Gazebo via VNC (recommended; no XQuartz)

To see the Gazebo window in **Real VNC Viewer** (same as for RViz), run the Gazebo container with a virtual display and VNC inside the container. No XQuartz needed.

```bash
docker run -it --rm --name gazebo --network rosnet -p 5902:5900 \
  -e ROS_MASTER_URI=http://astar:11311 \
  -e ROS_HOSTNAME=gazebo \
  -e TURTLEBOT3_MODEL=burger \
  hybrid_astar_gazebo:noetic /run_gazebo_vnc.sh
```

- **VNC:** Connect Real VNC Viewer to **localhost:5902** (no password). You get a second desktop with the Gazebo 3D window (A* + RViz stay on **localhost:5901**).
- Rebuild the image first if you haven't since adding VNC:  
  `docker build -f Dockerfile_gazebo -t hybrid_astar_gazebo:noetic .`
- **If the Gazebo VNC window stays black:** run the container in **headless** mode (gzserver only; view the robot in RViz on 5901): add `-e GAZEBO_HEADLESS=1` and you can omit `-p 5902:5900`. The sim and robot still run; you see them in RViz (RobotModel, TF, path).

#### Relays and restarts

You do **not** need to restart the A* or Gazebo containers after running the relay script. Run `./scripts/run_leader_relays_in_gazebo.sh` once with both containers already running, then set a **2D Nav Goal** in RViz.

#### Two containers (A* + Gazebo): use sim time to avoid TF_OLD_DATA

When you run the **A* container** together with the **Gazebo container**, the A* side must use simulation time (`/clock` from Gazebo); otherwise you get repeated **TF_OLD_DATA** warnings (odom → base_footprint "data from the past"). Start the A* container with **`-e USE_SIM_TIME=true`** and on the same Docker network as Gazebo:

```bash
docker network create rosnet
docker run --rm --platform linux/amd64 --network rosnet --name astar -p 5901:5900 \
  -e USE_SIM_TIME=true \
  hybrid_astar:noetic /catkin_ws/src/hybrid_astar/scripts/run_gazebo_with_astar.sh
```

Then start the **Gazebo container**. Prefer **VNC** (below) so you can see the Gazebo window in Real VNC Viewer; XQuartz often fails with libGL on macOS.

#### Connect the leader to the Gazebo TurtleBot3 (topic relays)

With both containers running (**astar** with A* + RViz + leader, and **gazebo** with TurtleBot3), the leader publishes to `/turtlebot3_leader/cmd_vel` while the Gazebo robot listens on `/cmd_vel` and publishes `/odom`. Run the topic relays so the leader drives the robot in Gazebo:

```bash
./scripts/run_leader_relays_in_gazebo.sh
```

This starts two relay nodes inside the **gazebo** container: `/turtlebot3_leader/cmd_vel` → `/cmd_vel` and `/odom` → `/turtlebot3_leader/odom`. Then in RViz (via VNC), set a **2D Nav Goal**; the TurtleBot3 in the Gazebo window should follow the path.

To run the relays manually instead:

```bash
docker exec -d gazebo /entrypoint.sh rosrun topic_tools relay /turtlebot3_leader/cmd_vel /cmd_vel
docker exec -d gazebo /entrypoint.sh rosrun topic_tools relay /odom /turtlebot3_leader/odom
```

---

## What you need

1. **ROS** (Noetic or Melodic) and this package built in a catkin workspace.
2. **Gazebo** with a TurtleBot3 (or any diff-drive robot that has `cmd_vel` and `odom`).
3. **Hybrid A*** running so that a path is published on `/sPath` (set start/goal in RViz to trigger planning).

## Step-by-step

### 1. Start Gazebo and your robot

Start your Gazebo simulation and spawn the robot (e.g. TurtleBot3). For example, with official TurtleBot3 packages:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

If your launch uses a **different robot name/namespace**, note the topic names (e.g. `/cmd_vel`, `/odom` or `/tb3_0/cmd_vel`, `/tb3_0/odom`). You’ll use them in step 4.

### 2. Start Hybrid A* (map + planner + optional RViz)

In another terminal, from your catkin workspace:

```bash
source devel/setup.bash
roslaunch hybrid_astar manual.launch
```

- In RViz: set **2D Pose Estimate** (start) and **2D Nav Goal** (goal) on the map.  
- The planner will publish the path on `/sPath`.  
- Make sure the map and frame (e.g. `map`) match your Gazebo world if you use a single environment.

### 3. Run the leader node

In a third terminal, from the repo root:

```bash
source /path/to/catkin_ws/devel/setup.bash
./scripts/run_leader_gazebo.sh
```

Or run the script directly:

```bash
source /path/to/catkin_ws/devel/setup.bash
rosrun hybrid_astar leader
```

The leader subscribes to `/sPath` and publishes velocity commands to the robot.

### 4. If your robot topics are not `turtlebot3_leader/*`

By default the leader uses:

- **Publish:** `/turtlebot3_leader/cmd_vel`
- **Subscribe:** `/turtlebot3_leader/odom`

If your Gazebo robot uses `/cmd_vel` and `/odom` instead, run with remapping:

```bash
./scripts/run_leader_gazebo.sh --remap
```

Or manually:

```bash
rosrun hybrid_astar leader /turtlebot3_leader/cmd_vel:=/cmd_vel /turtlebot3_leader/odom:=/odom
```

For other namespaces (e.g. `tb3_0`), remap to the actual topic names:

```bash
rosrun hybrid_astar leader /turtlebot3_leader/cmd_vel:=/tb3_0/cmd_vel /turtlebot3_leader/odom:=/tb3_0/odom
```

## Topic summary

| Topic                 | Type   | Role for leader                |
|-----------------------|--------|---------------------------------|
| `/sPath`              | Path   | Input path to follow (from planner) |
| `/turtlebot3_leader/odom`  | Odometry | Robot pose (input)         |
| `/turtlebot3_leader/cmd_vel` | Twist  | Velocity commands (output)  |
| `/leader/path`        | Path   | Leader’s traveled path (for visualization) |

## Frames and map

- The planner and map use the `map` frame; the leader uses the path poses as given (in the same frame).
- For Gazebo, ensure your sim’s `odom` (and optionally `map`) are consistent with the Hybrid A* map (e.g. same world and scale) so the path and robot pose align.
