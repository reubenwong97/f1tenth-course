#ifndef __FOLLOW_GAP__
#define __FOLLOW_GAP__

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

class FollowGap {
public:
  FollowGap();
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  void preprocessLidar(std::vector<double> &ranges, const double range_min,
                       const double range_max);
  std::vector<double>
  truncateFOV(const sensor_msgs::LaserScan::ConstPtr &scan_msg, double &fov_min,
              double &fov_max, double &angle_increment);
  void findMaxGap(const std::vector<double> &ranges, unsigned int &start_idx,
                  unsigned int &end_idx);
  unsigned int findMax(const std::vector<double> &ranges,
                       const unsigned int &start_idx,
                       const unsigned int &end_idx);

private:
  ros::NodeHandle n;
  ros::Subscriber scanSubscriber;
  ros::Publisher drivePublisher;

  const double bubbleRadius;
  const double fov;
  const unsigned int window;
  std::vector<double> ranges;
  const double maxThresh;
};

double findDistance(double a, double b, double angle);
double toDegrees(double r);
double toRadians(double d);

#endif
