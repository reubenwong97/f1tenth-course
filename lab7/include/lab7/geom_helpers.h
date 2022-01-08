#include <vector>
#include <math.h>
#include <algorithm>
#include <tuple>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/convert.h>

std::vector<std::tuple<int, int>> bresenham(int x0, int y0, int x1, int y1);
std::tuple<int, int> toLocalIndex(const double &distance, const double &angle, const double &resolution = 0.05, const int &width = 500);
std::tuple<int, int> toGlobalIndex(const double &distance, const double &angle,
                                   const double &top_left_x, const double &top_left_y,
                                   const geometry_msgs::TransformStamped &transformStamped, const nav_msgs::Odometry &pose_msg);

geometry_msgs::PointStamped getTransformedPoint(const double &pos_x, const double &pos_y, const geometry_msgs::TransformStamped &transformStamped);