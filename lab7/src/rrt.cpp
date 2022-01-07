// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "lab7/rrt.h"
#include "lab7/geom_helpers.h"

#define PI 3.1415927

// Destructor of the RRT class
RRT::~RRT()
{
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh) : nh_(nh), gen((std::random_device())())
{

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
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

    // flags
    nh_.getParam("publish_grid", publish_grid);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1000);
    // mapvis_pub_ = nh_.advertise<visualization_msgs::Marker>(env_viz, 1000);
    // points_pub_ = nh_.advertise<visualization_msgs::Marker>(dynamic_viz, 1000);
    // waypoint_pub_ = nh_.advertise<visualization_msgs::Marker>(static_viz, 1000);
    // lines_pub_ = nh_.advertise<visualization_msgs::Marker>(tree_lines, 1000);
    map_viz_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_viz_topic, 1000);

    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    auto map_message_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, nh_);
    nav_msgs::OccupancyGrid map_message = *map_message_ptr;

    // TODO: create a occupancy grid
    // save empty grid for easy reset during scan callback (grid size [500,200])
    // grid params
    height = map_message.info.height;
    width = map_message.info.width;
    resolution = map_message.info.resolution;
    origin_x = map_message.info.origin.position.x;
    origin_y = map_message.info.origin.position.y;

    occupancy_grid_static = unflatten(map_message.data, height, width);

    occupancy_grid = occupancy_grid_static;

    ROS_INFO("Created new RRT Object.");
}

std::vector<double> RRT::truncateFOV(const sensor_msgs::LaserScan::ConstPtr &scan_msg, double &fov_min, double &fov_max, double &angle_increment)
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
std::vector<std::vector<int>> RRT::unflatten(const std::vector<int8_t> &array, int height, int width)
{
    std::vector<std::vector<int>> res(height, std::vector<int>(width, FREE));
    for (int k = 0; k < array.size(); k++) {
        int i = k / width;
        int j = k % width;
        res[i][j] = array[k];
    }

    return res;
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    double fov_min = 0, fov_max = 0, angle_increment = 0, angle = toRadians(-90);
    std::vector<double> truncatedRanges = truncateFOV(scan_msg, fov_min, fov_max, angle_increment);

    // TODO: update your occupancy grid
    occupancy_grid = occupancy_grid_static;
    double rear_to_lidar = 0.29275; // not sure how to use for now

    for (double range : truncatedRanges)
    {
        // find hit points of each scan
        int x, y;
        std::tuple<int, int> endpoint;
        std::vector<std::tuple<int, int>> coords;
        // endpoint needs to be saved to set those locations as occupied since bresenham frees them
        endpoint = toIndex(range, angle, resolution, width);
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

    // if (pose_set)
    publishOccupancy(occupancy_grid);
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
    // std::cout << "Pose position x" << last_pose.pose.pose.position.x << std::endl;
    // grid_msg.info.origin.position.x = last_pose.pose.pose.position.x;
    // grid_msg.info.origin.position.y = last_pose.pose.pose.position.y;
    grid_msg.info.origin.position.x = origin_x;
    grid_msg.info.origin.position.y = origin_y;
    // ?: Not sure if angle required
    // grid_msg.info.origin.orientation.w = last_pose.pose.pose.orientation.w;
    // grid_msg.info.origin.orientation.x = last_pose.pose.pose.orientation.x;
    // grid_msg.info.origin.orientation.y = last_pose.pose.pose.orientation.y;
    // grid_msg.info.origin.orientation.z = last_pose.pose.pose.orientation.z;

    flattened = flatten(occupancy_grid);
    grid_msg.data = std::vector<int8_t>(flattened.begin(), flattened.end()); // cast to match message data type

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

    // tree as std::vector
    std::vector<Node> tree;

    // TODO: fill in the RRT main loop

    // path found as Path message
    // last_pose = *pose_msg;
    // pose_set = true;
    // ROS_INFO_STREAM("pose has been set");
    // std::cout << "pose has been set" << std::endl;
    // last_posx = pose_msg->pose.pose.position.x;
    // last_posy = pose_msg->pose.pose.position.y;
    // last_orw = pose_msg->pose.pose.orientation.w;
    // last_orx = last_pose->pose.pose.orientation.x;
    // last_ory = last_pose->pose.pose.orientation.y;
    // last_orz = last_pose->pose.pose.orientation.z;
}

std::vector<double> RRT::sample()
{
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    return sampled_point;
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

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node)
{
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:double &fov_min, double &fov_max, double &angle_increment of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node)
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
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method
    return neighborhood;
}

double toDegrees(double r) { return r * 180 / PI; }

double toRadians(double d) { return d * PI / 180; }