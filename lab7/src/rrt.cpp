// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "lab7/rrt.h"
#include "lab7/geom_helpers.h"
#include "stdio.h"

#define PI 3.1415927

// Destructor of the RRT class
RRT::~RRT()
{
  // Do something in here, free up used memory, print message, etc.
  ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh)
    : nh_(nh), gen((std::random_device())()), tfListener(tfBuffer)
{

  // TODO: Load parameters from yaml file, you could add your own parameters to
  // the rrt_params.yaml file
  nh_.getParam("pose_topic", pose_topic);
  nh_.getParam("scan_topic", scan_topic);
  nh_.getParam("drive_topic", drive_topic);
  nh_.getParam("env_viz", env_viz);
  nh_.getParam("dynamic_viz", dynamic_viz);
  nh_.getParam("static_viz", static_viz);
  nh_.getParam("tree_lines", tree_lines);
  nh_.getParam("map_viz_topic", map_viz_topic);
  nh_.getParam("map_topic", map_topic);

  nh_.getParam("fov", fov);
  nh_.getParam("min_goal_distance", min_goal_distance);
  nh_.getParam("steer_length", steer_length);
  nh_.getParam("lookahead_distance", lookahead_distance);
  nh_.getParam("max_iteration", max_iteration);

  // flags
  nh_.getParam("publish_grid", publish_grid);

  // ROS publishers
  // TODO: create publishers for the the drive topic, and other topics you might
  // need
  drive_pub_ =
      nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1000);
  // mapvis_pub_ = nh_.advertise<visualization_msgs::Marker>(env_viz, 1000);
  // points_pub_ = nh_.advertise<visualization_msgs::Marker>(dynamic_viz, 1000);
  // waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>(static_viz,
  // 1000); lines_pub_ = nh_.advertise<visualization_msgs::Marker>(tree_lines,
  // 1000);
  map_viz_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_viz_topic, 1000);
  point_pub_ = nh_.advertise<visualization_msgs::Marker>("/test_point", 1000);

  // ROS subscribers
  // TODO: create subscribers as you need
  pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
  scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
  auto map_message_ptr =
      ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh_);
  nav_msgs::OccupancyGrid map_message = *map_message_ptr;

  // TODO: create a occupancy grid
  // save empty grid for easy reset during scan callback (grid size [500,200])
  // grid params
  height = map_message.info.height;
  width = map_message.info.width;
  resolution = map_message.info.resolution;
  origin_x = map_message.info.origin.position.x;
  origin_y = map_message.info.origin.position.y;
  top_left_x = origin_x + resolution * width;
  top_left_y = origin_y + resolution * height;

  // char tmp[256];
  // getcwd(tmp, 256);
  // std::cout << tmp << std::endl;
  goals = get_goals("/home/reuben/reuben_ws/src/lab7/waypoints_cleaned.csv");
  // std::cout << goals[0][0] << " " << goals[0][1] << std::endl;

  occupancy_grid_static = unflatten(map_message.data, height, width);

  occupancy_grid = occupancy_grid_static;

  ROS_INFO("Created new RRT Object.");
}

// REFERENCE: https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/#reading-from-csv
// and: https://stackoverflow.com/questions/53142798/reading-csv-file-to-vector-of-doubles
std::vector<std::vector<double>> RRT::get_goals(std::string path)
{
  std::vector<std::vector<double>> goals;
  std::ifstream data(path, std::ifstream::in);
  // std::cout << data.is_open() << std::endl;
  if (data.is_open())
  {
    std::string line;
    std::vector<double> goal;
    while (std::getline(data, line)) // break lines by \n
    {
      std::stringstream ss(line); // create stringstream for current line
      std::string str_point;
      while (getline(ss, str_point, ',')) // break each line by comma
        goal.emplace_back(std::stold(str_point));
      goals.emplace_back(goal);
    }
  }

  return goals;
}

// get individual goalpoint
std::vector<double> RRT::get_goalpoint(bool plot)
{
  std::vector<double> trans_x, trans_y, dist, goal_point;
  std::size_t num_goals = goals.size();
  for (std::size_t i = 0; i < num_goals; i++)
  {
    double x, y, tran_x, tran_y, distance;
    x = goals[i][0];
    y = goals[i][1];
    geometry_msgs::PointStamped point = getTransformedPoint(x, y, localTransformStamped); // transform to local frame
    tran_x = point.point.x;
    tran_y = point.point.y;
    trans_x.emplace_back(tran_x);
    trans_y.emplace_back(tran_y);
    distance = std::sqrt(std::pow(tran_x, 2) + std::pow(tran_y, 2));
    dist.emplace_back(distance);

    bool found = false;
    while (!found)
    {
      std::vector<double>::iterator res = std::min_element(dist.begin(), dist.end());
      int index = std::distance(dist.begin(), res);
      if (trans_x[index] > 0)
      {
        found = true;
        double point_x, point_y;
        point_x = trans_x[index];
        point_y = trans_y[index];
        goal_point.emplace_back(point_x);
        goal_point.emplace_back(point_y);
      }
      else
      {
        dist[index] = 999999; // set to high value so we skip past it next time we find the min
      }
    }
  }

  // if (plot)
  //   publishPoint(goal_point[0], goal_point[1], 0, 255, 0);

  return goal_point;
}

std::vector<int> RRT::flatten(const std::vector<std::vector<int>> &matrix)
{
  std::size_t total_size = 0;
  for (const auto &sub : matrix)
    total_size += sub.size();
  std::vector<int> res;
  res.reserve(total_size);
  for (const auto &sub : matrix)
    res.insert(res.end(), sub.begin(), sub.end());

  return res;
}

// convert 1D array into 2D matrix
std::vector<std::vector<int>> RRT::unflatten(const std::vector<int8_t> &array,
                                             int height, int width)
{
  std::vector<std::vector<int>> res(height, std::vector<int>(width, FREE));
  for (int k = 0; k < array.size(); k++)
  {
    int i = k / width;
    int j = k % width;
    res[i][j] = array[k];
    if ((array[k] != FREE) && (array[k] != OCCUPIED))
    {
      res[i][j] = OCCUPIED; // assume occupied for weird values
    }
  }
  // std::vector<int> grid_coords = get_grid_coords(0, 0);
  // int x = grid_coords[0];
  // int y = grid_coords[1];
  // std::cout<< "x, y = " << x << ", " << y << std::endl;
  // for (int i = 0; i < 100; i++) {
  //     res[x][y+i] = OCCUPIED;
  // }

  return res;
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  // The scan callback, update your occupancy grid here
  // Args:
  //    scan_msg (*LaserScan): pointer to the incoming scan message
  // TODO: update your occupancy grid
  occupancy_grid = occupancy_grid_static;
  double rear_to_lidar = 0.29275; // not sure how to use for now
}

void RRT::publishOccupancy(const std::vector<std::vector<int>> &occupancyGrid)
{
  nav_msgs::OccupancyGrid grid_msg;
  std::vector<int> flattened;
  grid_msg.header.stamp = ros::Time::now();
  grid_msg.header.frame_id = "map";

  // fill in map data
  grid_msg.info.height = height; // measurements in terms of cells
  grid_msg.info.width = width;
  grid_msg.info.resolution = resolution;
  // std::cout << "Pose position x" << last_pose.pose.pose.position.x <<
  // std::endl; grid_msg.info.origin.position.x =
  // last_pose.pose.pose.position.x; grid_msg.info.origin.position.y =
  // last_pose.pose.pose.position.y;
  grid_msg.info.origin.position.x = origin_x;
  grid_msg.info.origin.position.y = origin_y;
  // ?: Not sure if angle required
  grid_msg.info.origin.orientation.w = 1;
  grid_msg.info.origin.orientation.x = 0;
  grid_msg.info.origin.orientation.y = 0;
  grid_msg.info.origin.orientation.z = 0;

  flattened = flatten(occupancy_grid);
  grid_msg.data = std::vector<int8_t>(
      flattened.begin(), flattened.end()); // cast to match message data type

  map_viz_pub_.publish(grid_msg);
}

// void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
void RRT::pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
  // The pose callback when subscribed to particle filter's inferred pose
  // The RRT main loop happens here
  // Args:
  //    pose_msg (*PoseStamped): pointer to the incoming pose message
  // Returns:
  //
  std::cout << "Inside pf callback" << std::endl;
  try
  {
    // acquire transform to convert local frame to global frame
    transformStamped =
        tfBuffer.lookupTransform(global_frame, local_frame, ros::Time(0));
    // also acquire to convert to local frame
    localTransformStamped =
        tfBuffer.lookupTransform(local_frame, global_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  last_pose = *pose_msg; // multiple methods require access to pose
  pose_set = true;
  geometry_msgs::Quaternion q = last_pose.pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  heading_current = tf2::impl::getYaw(quat); // heading_current = yaw
  // tree as std::vector
  std::vector<Node> tree;
  // store the final path
  std::vector<Node> paths;
  // setup root node
  Node start;
  start.x = pose_msg->pose.pose.position.x;
  start.y = pose_msg->pose.pose.position.y;
  start.is_root = true;
  tree.emplace_back(start);

  // get the waypoint
  std::vector<double> goal = get_goalpoint();
  std::cout << "Goalpoint is: " << goal[0] << ", " << goal[1] << std::endl;

  // TODO: fill in the RRT main loop
  for (unsigned int i = 0; i < max_iteration; i++)
  {
    std::cout << "Calling sample" << std::endl;
    std::vector<double> sampled_point = sample();
    unsigned int nearest_point = nearest(tree, sampled_point);
    Node new_node = steer(tree[nearest_point], sampled_point);
    new_node.parent = nearest_point;
    if (!check_collision(tree[nearest_point], new_node))
    {
      tree.emplace_back(new_node);
      if (is_goal(new_node, goal[0], goal[1]))
      {
        paths = find_path(tree, new_node);
        break;
      }
    }
  }

  // Perform pure pursuit
  double x_target, y_target;
  std::size_t path_len = paths.size();
  std::cout << "Path size: " << path_len << std::endl;
  if (path_len == 0)
  {
    // do nothing
  }
  else
  {
    for (std::size_t i = 0; i < path_len; i++)
    {
      std::cout << "Iter..." << i << std::endl;
      double distance = euclidean_distance(paths[path_len - 1 - i].x, paths[path_len - 1 - i].y,
                                           pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
      if (distance >= lookahead_distance)
      {
        x_target = paths[path_len - 1 - i].x;
        y_target = paths[path_len - 1 - i].y;
        break;
      }
    }
    // PP control
    double target_distance = euclidean_distance(x_target, y_target, pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
    double lookahead_angle = std::atan2(y_target - pose_msg->pose.pose.position.y, x_target - pose_msg->pose.pose.position.x);
    double dy = target_distance * std::sin(lookahead_angle - heading_current);
    double angle = 2 * dy / (std::pow(target_distance, 2));
    steer_pure_pursuit(angle);
  }
}

void RRT::steer_pure_pursuit(const double &angle)
{
  double velocity = 0.5;
  ackermann_msgs::AckermannDriveStamped drive_msg = ackermann_msgs::AckermannDriveStamped();
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = "laser";
  drive_msg.drive.steering_angle = angle;
  drive_msg.drive.speed = velocity;
  drive_pub_.publish(drive_msg);
}

std::vector<double> RRT::sample() // WORKS
{
  // This method returns a sampled point from the free space
  // You should restrict so that it only samples a small region
  // of interest around the car's current position
  // Args:
  // Returns:
  //     sampled_point (std::vector<double>): the sampled point in free space

  // sample points locally in front of the car
  double x_bound_top, x_bound_bot, y_bound_left, y_bound_right;
  double sx, sy, gx, gy;
  x_bound_top = 2.5;
  x_bound_bot = 0;
  y_bound_left = 0.8;
  y_bound_right = -0.75;
  std::vector<double> sampled_point;
  std::uniform_real_distribution<> dis_x(x_bound_bot, x_bound_top);
  std::uniform_real_distribution<> dis_y(y_bound_left, y_bound_right);

  sx = dis_x(gen); // sample locally
  sy = dis_y(gen);

  geometry_msgs::PointStamped sampledPoint = getTransformedPoint(sx, sy, transformStamped);
  gx = sampledPoint.point.x;
  gy = sampledPoint.point.y;

  std::cout << "Sampling..." << gx << ", " << gy << std::endl;
  publishPoint(gx, gy, 255, 0, 0); // publish red point for visualisation

  sampled_point.push_back(gx);
  sampled_point.push_back(gy);

  return sampled_point;
}

void RRT::publishPoint(const double &x, const double &y, const int &r, const int &g, const int &b)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Pose pos;
  pos.position.x = x;
  pos.position.y = y;
  pos.position.z = 0;
  pos.orientation.x = 0;
  pos.orientation.y = 0;
  pos.orientation.z = 0;
  pos.orientation.w = 1;
  marker.pose = pos;

  geometry_msgs::Vector3 scale;
  scale.x = 0.4;
  scale.y = 0.4;
  scale.z = 0.4;
  marker.scale = scale;

  std_msgs::ColorRGBA colour;
  colour.r = r;
  colour.g = g;
  colour.b = b;
  colour.a = 1;
  marker.color = colour;

  marker.lifetime.sec = 0;
  marker.frame_locked = true;
  point_pub_.publish(marker);
}

double RRT::euclidean_distance(const double &x1, const double &y1, const double &x2, const double &y2)
{
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point)
{
  // This method returns the nearest node on the tree to the sampled point
  // Args:
  //     tree (std::vector<Node>): the current RRT tree
  //     sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //     nearest_node (int): index of nearest node on the tree

  int nearest_node = 0;
  // TODO: fill in this method
  double sx, sy;
  sx = sampled_point[0];
  sy = sampled_point[1];

  double min_distance = euclidean_distance(tree[0].x, tree[0].y, sx, sy);
  for (std::size_t i; i < tree.size(); i++)
  {
    if (euclidean_distance(tree[i].x, tree[i].y, sx, sy) < min_distance)
    {
      min_distance = euclidean_distance(tree[i].x, tree[i].y, sx, sy);
      nearest_node = i;
    }
  }

  return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point)
{
  // The function steer:(x,y)->z returns a point such that z is “closer”
  // to y than x is. The point z returned by the function steer will be
  // such that z minimizes ||z−y|| while at the same time maintaining
  //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

  // basically, expand the tree towards the sample point (within a max dist)

  // Args:
  //    nearest_node (Node): nearest node on the tree to the sampled point
  //    sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //    new_node (Node): new node created from steering

  Node new_node;
  // TODO: fill in this method
  double euclidean = euclidean_distance(nearest_node.x, nearest_node.y, sampled_point[0], sampled_point[1]);
  // think off as slider between the nearest node and the sampled point
  new_node.x = nearest_node.x + steer_length / euclidean * (sampled_point[0] - nearest_node.x);
  new_node.y = nearest_node.y + steer_length / euclidean * (sampled_point[1] - nearest_node.y);

  return new_node;
}

std::vector<int> RRT::get_grid_coords(double global_x, double global_y)
{
  // This method returns the grid coordinates when given the global coordinates

  // Args:
  //    global_x, global_y: x and y coordinates in global frame accordingly
  // Returns:
  //    vector of grid coordinates (grid_x, grid_y)
  int grid_y = std::floor((global_x - origin_x) /
                          resolution); // the grid visualization is flipped
  int grid_x = std::floor((global_y - origin_y) / resolution);
  return {grid_x, grid_y};
}

bool RRT::check_occupied(int grid_x, int grid_y)
{
  // This method checks if a grid cell is occupied when given the grid
  // coordinates
  return occupancy_grid[grid_x][grid_y] == OCCUPIED;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node)
{
  // This method returns a boolean indicating if the path between the
  // nearest node and the new node created from steering is collision free
  // Args:
  //    nearest_node (Node): nearest node on the tree to the sampled point
  //    new_node (Node): new node created from steering
  // Returns:
  //    collision (bool): true if in collision, false otherwise

  bool collision = false;
  // TODO: fill in this method
  const int NUM_CHECKPOINTS = 1000;
  double checkpoint_x, checkpoint_y;
  std::vector<int> grid_coords(2, 0);
  for (int i = 0; i <= NUM_CHECKPOINTS; i++)
  {
    checkpoint_x = nearest_node.x +
                   (double)i / NUM_CHECKPOINTS * (new_node.x - nearest_node.x);
    checkpoint_y = nearest_node.y +
                   (double)i / NUM_CHECKPOINTS * (new_node.y - nearest_node.y);
    grid_coords = get_grid_coords(checkpoint_x, checkpoint_y);
    if (check_occupied(grid_coords[0], grid_coords[1]))
    {
      collision = true;
      // std::cout << "occupied cell (" << checkpoint_x << ", " << checkpoint_y
      //           << ")" << std::endl;
      // std::cout << "grid coord (" << grid_coords[0] << ", " << grid_coords[1]
      //           << ")" << std::endl;
      break;
    }
  }

  return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y)
{
  // This method checks if the latest node added to the tree is close
  // enough (defined by goal_threshold) to the goal so we can terminate
  // the search and find a path
  // Args:
  //   latest_added_node (Node): latest addition to the tree
  //   goal_x (double): x coordinate of the current goal
  //   goal_y (double): y coordinate of the current goal
  // Returns:
  //   close_enough (bool): true if node close enough to the goal

  bool close_enough = false;
  // TODO: fill in this method
  double distance = std::pow(std::pow(latest_added_node.x - goal_x, 2) +
                                 std::pow(latest_added_node.y - goal_y, 2),
                             0.5);
  if (distance < min_goal_distance)
    close_enough = true;

  return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree,
                                 Node &latest_added_node)
{
  // This method traverses the tree from the node that has been determined
  // as goal
  // Args:
  //   latest_added_node (Node): latest addition to the tree that has been
  //      determined to be close enough to the goal
  // Returns:
  //   path (std::vector<Node>): the vector that represents the order of
  //      of the nodes traversed as the found path

  std::vector<Node> found_path;
  // TODO: fill in this method
  found_path.push_back(latest_added_node);
  Node next_node = tree[latest_added_node.parent];
  while (!next_node.is_root)
  {
    found_path.push_back(next_node);
    next_node = tree[next_node.parent];
  }

  found_path.push_back(tree[0]); // always ends at root

  return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node)
{
  // This method returns the cost associated with a node
  // Args:
  //    tree (std::vector<Node>): the current tree
  //    node (Node): the node the cost is calculated for
  // Returns:
  //    cost (double): the cost value associated with the node

  double cost = 0;
  // TODO: fill in this method

  return cost;
}

double RRT::line_cost(Node &n1, Node &n2)
{
  // This method returns the cost of the straight line path between two nodes
  // Args:
  //    n1 (Node): the Node at one end of the path
  //    n2 (Node): the Node at the other end of the path
  // Returns:
  //    cost (double): the cost value associated with the path

  double cost = 0;
  // TODO: fill in this method

  return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{
  // This method returns the set of Nodes in the neighborhood of a
  // node.
  // Args:
  //   tree (std::vector<Node>): the current tree
  //   node (Node): the node to find the neighborhood for
  // Returns:
  //   neighborhood (std::vector<int>): the index of the nodes in the
  //   neighborhood

  std::vector<int> neighborhood;
  // TODO:: fill in this method
  return neighborhood;
}

double toDegrees(double r) { return r * 180 / PI; }

double toRadians(double d) { return d * PI / 180; }
