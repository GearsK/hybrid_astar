#include "planner.h"

#include <cmath>
#include <algorithm>

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  const double res = map->info.resolution;
  const int ow = map->info.width;
  const int oh = map->info.height;
  const double mapWidthM = ow * res;
  const double mapHeightM = oh * res;

  // Resample to Constants::cellSize (0.1m) when map resolution differs (e.g. TurtleBot 0.05m)
  // so the planner's collision lookup and grid math (which assume 0.1m cells) stay correct.
  if (std::fabs(res - Constants::cellSize) > 1e-6f) {
    const float cellSize = Constants::cellSize;
    const int nw = static_cast<int>(mapWidthM / cellSize + 0.5);
    const int nh = static_cast<int>(mapHeightM / cellSize + 0.5);
    if (nw <= 0 || nh <= 0) {
      std::cout << "setMap: resampled grid would be empty, using map as-is." << std::endl;
      grid = map;
    } else {
      gridResampled = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());
      gridResampled->info.resolution = cellSize;
      gridResampled->info.width = nw;
      gridResampled->info.height = nh;
      gridResampled->info.origin = map->info.origin;
      gridResampled->header = map->header;
      gridResampled->data.resize(static_cast<size_t>(nw * nh), 0);

      for (int j = 0; j < nh; ++j) {
        for (int i = 0; i < nw; ++i) {
          int ox0 = static_cast<int>(i * cellSize / res);
          int oy0 = static_cast<int>(j * cellSize / res);
          int ox1 = static_cast<int>((i + 1) * cellSize / res) - 1;
          int oy1 = static_cast<int>((j + 1) * cellSize / res) - 1;
          ox0 = std::max(0, ox0);
          oy0 = std::max(0, oy0);
          ox1 = std::min(ow - 1, ox1);
          oy1 = std::min(oh - 1, oy1);
          int8_t val = 0;
          for (int oy = oy0; oy <= oy1 && !val; ++oy) {
            for (int ox = ox0; ox <= ox1; ++ox) {
              if (map->data[static_cast<size_t>(oy * ow + ox)]) {
                val = 100;
                break;
              }
            }
          }
          gridResampled->data[static_cast<size_t>(j * nw + i)] = val;
        }
      }
      std::cout << "setMap: resampled " << ow << "x" << oh << " @" << res << "m to "
                << nw << "x" << nh << " @" << cellSize << "m for planner." << std::endl;
      grid = gridResampled;
    }
  } else {
    grid = map;
    gridResampled.reset();
  }

  //update the configuration space with the current map
  configurationSpace.updateGrid(grid);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = grid->info.height;
  int width = grid->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = grid->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    const double maxX = grid->info.width * grid->info.resolution;
    const double maxY = grid->info.height * grid->info.resolution;
    if (start.pose.pose.position.x >= 0 && start.pose.pose.position.x <= maxX &&
        start.pose.pose.position.y >= 0 && start.pose.pose.position.y <= maxY) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  if (!grid) {
    std::cout << "setStart: map not received yet, ignoring initial pose." << std::endl;
    return;
  }
  const double px = initial->pose.pose.position.x;
  const double py = initial->pose.pose.position.y;
  const double maxX = grid->info.width * grid->info.resolution;
  const double maxY = grid->info.height * grid->info.resolution;

  float x = px / Constants::cellSize;
  float y = py / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (px >= 0 && px <= maxX && py >= 0 && py <= maxY) {
    // Reject if start is inside an obstacle (prevents path from starting in walls)
    const int cx = static_cast<int>(px / grid->info.resolution);
    const int cy = static_cast<int>(py / grid->info.resolution);
    if (cx >= 0 && cx < grid->info.width && cy >= 0 && cy < grid->info.height &&
        grid->data[cy * grid->info.width + cx] != 0) {
      std::cout << "invalid start (inside obstacle) x:" << px << " y:" << py << " — place 2D Pose Estimate in free (white) area." << std::endl;
      return;
    }
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start (out of map bounds) x:" << px << " y:" << py << " (map 0-" << maxX << ", 0-" << maxY << ")" << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  if (!grid) {
    std::cout << "setGoal: map not received yet, ignoring goal." << std::endl;
    return;
  }
  // retrieving goal position
  const double px = end->pose.position.x;
  const double py = end->pose.position.y;
  const double maxX = grid->info.width * grid->info.resolution;
  const double maxY = grid->info.height * grid->info.resolution;

  float x = px / Constants::cellSize;
  float y = py / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (px >= 0 && px <= maxX && py >= 0 && py <= maxY) {
    // Reject if goal is inside an obstacle (prevents path into walls)
    const int cx = static_cast<int>(px / grid->info.resolution);
    const int cy = static_cast<int>(py / grid->info.resolution);
    if (cx >= 0 && cx < grid->info.width && cy >= 0 && cy < grid->info.height &&
        grid->data[cy * grid->info.width + cx] != 0) {
      std::cout << "invalid goal (inside obstacle) x:" << px << " y:" << py << " — set 2D Nav Goal in free (white) area." << std::endl;
      return;
    }
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal (out of map bounds) x:" << px << " y:" << py << " (map 0-" << maxX << ", 0-" << maxY << ")" << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
    //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // ___________
    // DEBUG START
    //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    // FIND THE PATH
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram);
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);



    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
