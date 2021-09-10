#include <wong1109_lab4_cpp/follow_gap.h>

#define PI 3.1415927

FollowGap::FollowGap()
    : bubbleRadius(0.5), fov(150.), window(5), maxThresh(2.5) {
  // NOTE: being short-sighted helps deal with turning corners
  n = ros::NodeHandle();
  scanSubscriber = n.subscribe("/scan", 1, &FollowGap::laserScanCallback, this);
  drivePublisher =
      n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);
}

void FollowGap::findMaxGap(const std::vector<double> &ranges,
                           unsigned int &start_idx, unsigned int &end_idx) {
  // both min_idx and max_idx initialised to zeros
  unsigned int count = 0, max_count = 0;
  bool startMinCount = false;
  for (unsigned int i = 0; i < ranges.size(); i++) {
    if (ranges[i] > 0.) {
      ++count;
      if (count > max_count) {
        max_count = count;
        end_idx = i;
        if (startMinCount) {
          start_idx = i;
          startMinCount = false; // set bool so we don't override this
        }
      }
    }
    // case where we hit zeros
    else {
      count = 0;            // reset counter to count right side
      startMinCount = true; // min is not 0, so we need to track
    }
  }
}

unsigned int FollowGap::findMax(const std::vector<double> &ranges,
                                const unsigned int &start_idx,
                                const unsigned int &end_idx) {
  // find max from subset
  auto maxPtr =
      std::max_element(ranges.begin() + start_idx, ranges.begin() + end_idx);
  double maxRange = *maxPtr;

  auto i = maxPtr; // want to store this outside
  for (; i != ranges.end(); ++i) {
    if (*i == maxRange) {
      continue;
    } else {
      break;
    }
  }
  // don't return the very first max, which would slant towards the right,
  // but rather, return the max that is in the middle!
  return maxPtr + (i - maxPtr) / 2 - ranges.begin(); // love this about c++
}

double findDistance(double a, double b, double angle) {
  // just cosine law
  return std::sqrt(a * a + b * b - 2 * a * b * std::cos(angle));
}

double toDegrees(double r) { return r * 180 / PI; }

double toRadians(double d) { return d * PI / 180; }

std::vector<double>
FollowGap::truncateFOV(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
                       double &fov_min, double &fov_max,
                       double &angle_increment) {
  double fov_half = toRadians(fov / 2);
  int min_fov_idx = static_cast<int>((-fov_half - scan_msg->angle_min) /
                                     scan_msg->angle_increment);
  int max_fov_idx = static_cast<int>((fov_half - scan_msg->angle_min) /
                                     scan_msg->angle_increment);

  // store for access outside function
  fov_min = scan_msg->angle_min + min_fov_idx * scan_msg->angle_increment;
  fov_max = scan_msg->angle_min + max_fov_idx * scan_msg->angle_increment;
  angle_increment = scan_msg->angle_increment;

  return std::vector<double>(scan_msg->ranges.begin() + min_fov_idx,
                             scan_msg->ranges.begin() + max_fov_idx);
}

void FollowGap::preprocessLidar(std::vector<double> &ranges,
                                const double range_min,
                                const double range_max) {
  // remove noisy scans with foreach loop
  for (double &r : ranges) {
    if (r < range_min)
      r = range_min;
    else if (r > maxThresh)
      r = maxThresh;
  }
  std::vector<double> temp(ranges.size(),
                           0); // init temp vector of size ranges to 0
  std::deque<double>
      running_avg; // easy structure allowing front and back access

  for (unsigned int i = 0; i < ranges.size(); ++i) {
    running_avg.push_back(ranges[i]);
    if (running_avg.size() > window)
      running_avg.pop_front();

    // note, values at extremities have a smaller window size
    double sum = std::accumulate(running_avg.begin(), running_avg.end(), 0);
    temp[i] = sum / running_avg.size();
  }
  ranges = temp;
}

void FollowGap::laserScanCallback(
    const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  double fov_min = 0, fov_max = 0, angle_increment = 0; // to store values later
  std::vector<double> truncatedRanges =
      truncateFOV(scan_msg, fov_min, fov_max, angle_increment);
  preprocessLidar(truncatedRanges, scan_msg->range_min, scan_msg->range_max);

  // find index and val of closest beam
  auto minPtr =
      std::min_element(truncatedRanges.begin(), truncatedRanges.end());
  const double min_range = *minPtr;
  int min_idx =
      minPtr - truncatedRanges.begin(); // min_idx wrt the subset ranges

  // create 0 bubble
  for (int i = 0; i < truncatedRanges.size(); ++i) {
    if (findDistance(truncatedRanges[i], min_range,
                     angle_increment * std::abs(min_idx - i)) <=
        bubbleRadius) // note must match int type or abs throws error
      truncatedRanges[i] = 0.;
  }

  unsigned int start_idx = 0, end_idx = 0;
  findMaxGap(truncatedRanges, start_idx, end_idx);
  // ROS_INFO("---Start and End Indices---");
  // ROS_INFO("start: %d, end: %d", start_idx, end_idx);
  // ROS_INFO("---------------------------");

  unsigned int target_angle_idx = findMax(truncatedRanges, start_idx, end_idx);
  // ROS_INFO("---Target Index---");
  // ROS_INFO("target: %d", target_angle_idx);
  // ROS_INFO("------------------");

  // ROS_INFO("---Closest Index---");
  // ROS_INFO("min index: %d", min_idx);
  // ROS_INFO("relative pos of min: %lf",
  //          (static_cast<double>(min_idx) / truncatedRanges.size()));
  // ROS_INFO("-------------------");

  double steering_angle = fov_min + target_angle_idx * angle_increment;

  // ROS_INFO("---Steering Angle---");
  // ROS_INFO("angle: %f", steering_angle);
  // ROS_INFO("--------------------");

  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = "laser";
  drive_msg.drive.steering_angle = steering_angle;

  if (toDegrees(std::abs(steering_angle)) > 20.)
    drive_msg.drive.speed = 0.5;
  else if (toDegrees(std::abs(steering_angle)) > 10.)
    drive_msg.drive.speed = 1.5;
  else {
    drive_msg.drive.speed = 3.5;
  }

  drivePublisher.publish(drive_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "follow_gap", ros::InitOption::AnonymousName);
  FollowGap gapFollower;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
