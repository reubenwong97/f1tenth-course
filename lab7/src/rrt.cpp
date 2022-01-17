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

  std::cout << "height" << height << "\n";
  std::cout << "width" << width << "\n";
  std::cout << "flattened size" << map_message.data.size() << std::endl;

  occupancy_grid_static = unflatten(map_message.data, height, width);

  occupancy_grid = occupancy_grid_static;

  ROS_INFO("Created new RRT Object.");
}

std::vector<double>
RRT::truncateFOV(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
                 double &fov_min, double &fov_max, double &angle_increment)
{
  double fov_half = toRadians(fov / 2);
  int min_fov_idx = static_cast<int>((-fov_half - scan_msg->angle_min) /
                                     scan_msg->angle_increment);
  int max_fov_idx = static_cast<int>((fov_half - scan_msg->angle_min) /
                                     scan_msg->angle_increment);

  // store for access outside function
  // angles for the min and max scan
  fov_min = scan_msg->angle_min + min_fov_idx * scan_msg->angle_increment;
  fov_max = scan_msg->angle_min + max_fov_idx * scan_msg->angle_increment;
  angle_increment = scan_msg->angle_increment;

  return std::vector<double>(scan_msg->ranges.begin() + min_fov_idx,
                             scan_msg->ranges.begin() + max_fov_idx);
}

/* NOTE: Origin is at the bottom right corner of the map
 */
std::tuple<int, int>
RRT::toGlobalIndex(const double &distance, const double &angle,
                   const geometry_msgs::TransformStamped &transformStamped,
                   const nav_msgs::Odometry &pose_msg)
{
  // get position of car in global frame
  double local_pos_x, local_pos_y, global_pos_x, global_pos_y, end_x, end_y;
  int x_idx, y_idx;
  geometry_msgs::PointStamped car_pos;
  local_pos_x = pose_msg.pose.pose.position.x;
  local_pos_y = pose_msg.pose.pose.position.y;
  car_pos = getTransformedPoint(local_pos_x, local_pos_y, transformStamped);
  global_pos_x = car_pos.point.x;
  global_pos_y = car_pos.point.y;

  std::tuple<int, int> index;

  double rear_to_lidar = 0.29275;
  double x_lidar = global_pos_x + rear_to_lidar * std::cos(heading_current);
  double y_lidar = global_pos_y + rear_to_lidar * std::sin(heading_current);
  double global_angle = angle + heading_current;

  end_x = x_lidar + distance * std::cos(global_angle);
  end_y = y_lidar + distance * std::sin(global_angle);

  std::vector<int> grid_coords = get_grid_coords(end_x, end_y);

  x_idx = grid_coords[0];
  y_idx = grid_coords[1];
  index = std::make_tuple(x_idx, y_idx);

  return index;
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
  try
  {
    // acquire transform to convert local frame to global frame
    transformStamped =
        tfBuffer.lookupTransform(global_frame, local_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  double fov_min = 0, fov_max = 0, angle_increment = 0, angle = toRadians(-90);
  std::vector<double> truncatedRanges =
      truncateFOV(scan_msg, fov_min, fov_max, angle_increment);

  // TODO: update your occupancy grid
  occupancy_grid = occupancy_grid_static;
  double rear_to_lidar = 0.29275; // not sure how to use for now

  if (pose_set)
  {
    for (double range : truncatedRanges)
    {
      // find hit points of each scan
      int x, y;
      std::tuple<int, int> endpoint;
      std::vector<std::tuple<int, int>> coords;
      // endpoint needs to be saved to set those locations as occupied since
      // bresenham frees them
      endpoint = toGlobalIndex(range, angle, transformStamped, last_pose);
      x = std::get<0>(endpoint);
      y = std::get<1>(endpoint);
      angle = angle + angle_increment;

      coords = bresenham(0, 0, x, y);
      for (std::tuple<int, int> coord : coords)
      {
        int row, col;
        row = std::get<0>(coord);
        col = std::get<1>(coord);

        occupancy_grid[row][col] = FREE; // set to empty
      }

      occupancy_grid[x][y] = OCCUPIED; // set to occupied
    }
    publishOccupancy(occupancy_grid);
  }
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

  last_pose = *pose_msg; // multiple methods require access to pose
  pose_set = true;
  // tree as std::vector
  std::vector<Node> tree;

  // TODO: fill in the RRT main loop
  std::vector<double> sampled_points = sample();

  // std::cout << last_pose.pose.pose.position.x << " " << last_pose.pose.pose.position.y << std::endl;
  // ROS_INFO_STREAM("pose has been set");
  // std::cout << "pose has been set" << std::endl;
  // last_posx = pose_msg->pose.pose.position.x;
  // last_posy = pose_msg->pose.pose.position.y;
  // last_orw = pose_msg->pose.pose.orientation.w;
  // last_orx = last_pose->pose.pose.orientation.x;
  // last_ory = last_pose->pose.pose.orientation.y;
  // last_orz = last_pose->pose.pose.orientation.z;

  // compute current heading
  // double siny_cosp = 2.0 * (last_pose.pose.pose.orientation.w *
  // last_pose.pose.pose.orientation.z + last_pose.pose.pose.orientation.x *
  // last_pose.pose.pose.orientation.y); double cosy_cosp = 1.0 - 2.0 *
  // (last_pose.pose.pose.orientation.y * last_pose.pose.pose.orientation.y +
  // last_pose.pose.pose.orientation.z * last_pose.pose.pose.orientation.z);
  // heading_current = std::atan2(siny_cosp, cosy_cosp);

  geometry_msgs::Quaternion q = last_pose.pose.pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  heading_current = tf2::impl::getYaw(quat); // heading_current = yaw
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
      std::cout << "occupied cell (" << checkpoint_x << ", " << checkpoint_y
                << ")" << std::endl;
      std::cout << "grid coord (" << grid_coords[0] << ", " << grid_coords[1]
                << ")" << std::endl;
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
