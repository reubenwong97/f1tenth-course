// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node
{
    double x, y;
    double cost; // only used for RRT*
    int parent;  // index of parent node in the tree vector
    bool is_root = false;
} Node;

class RRT
{
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();

private:
    ros::NodeHandle nh_;

    double fov;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need

    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher drive_pub_;
    ros::Publisher mapvis_pub_;
    ros::Publisher points_pub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher lines_pub_;
    ros::Publisher map_pub_;

    // topics
    std::string pose_topic, scan_topic, drive_topic, env_viz, dynamic_viz, static_viz, tree_lines, map_topic;

    // grid params
    int height, width;
    double resolution;
    // last pose for publishing map
    // nav_msgs::Odometry::Ptr last_pose;

    // flags for debugging;
    bool publish_grid;

    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params
    std::vector<std::vector<int>> occupancy_grid_empty;
    std::vector<std::vector<int>> occupancy_grid;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    // callbacks
    // where rrt actually happens
    // original data type
    // void pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    // new data type
    void pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    std::vector<double> truncateFOV(const sensor_msgs::LaserScan::ConstPtr &scan_msg, double &fov_min, double &fov_max, double &angle_increment);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);
    std::vector<int> flatten(const std::vector<std::vector<int>> &matrix);

    // to consider writing as func
    void publishOccupancy(const std::vector<std::vector<int>> &occupancyGrid);
};

double findDistance(double a, double b, double angle);
double toDegrees(double r);
double toRadians(double d);