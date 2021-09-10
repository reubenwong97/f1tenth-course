#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from reuben_wong_lab1.msg import scan_range
import numpy as np


class LidarProcessor(object):
    def __init__(self, rate=10):
        """
        Initialises the node and the various subscribers. Also initialises the min_val and max_val
        variables that can be accessed by the various methods.
        """
        rospy.init_node("lidar_processor")
        # self.sub = rospy.Subscriber("/scan", LaserScan, callback=self.callback)
        self.min_pub = rospy.Publisher("/closest_point", Float64, queue_size=10)
        self.max_pub = rospy.Publisher("/farthest_point", Float64, queue_size=10)
        self.sr_pub = rospy.Publisher("/scan_range", scan_range, queue_size=10)

        self.min_val = np.inf
        self.max_val = -np.inf

        self.rate = rospy.Rate(rate)

    def callback(self, msg):
        """
        Callback function used by the subscriber node to process messages from the
        /scan topic.
        """
        self.ranges = msg.ranges
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        self.min_val = np.inf
        self.max_val = -np.inf

        for val in self.ranges:
            if np.isinf(val) or np.isnan(val):
                continue
            if val < self.min_val and val > self.range_min:
                self.min_val = val
            if val > self.max_val and val < self.range_max:
                self.max_val = val

        msg = scan_range()
        if not np.isinf(self.min_val) and not np.isnan(self.min_val):
            msg.closest_point = self.min_val

        if not np.isinf(self.max_val) and not np.isnan(self.max_val):
            msg.farthest_point = self.max_val

        if (not np.isinf(self.min_val) and not np.isnan(self.min_val)) or (
            not np.isinf(self.max_val) and not np.isnan(self.max_val)
        ):
            self.sr_pub.publish(msg)

    def callback2(self, msg):
        self.ranges = msg.ranges
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        self.min_val = np.inf
        self.max_val = -np.inf

        for val in self.ranges:
            if np.isinf(val) or np.isnan(val):
                continue
            if val < self.min_val and val > self.range_min:
                self.min_val = val
            if val > self.max_val and val < self.range_max:
                self.max_val = val

        if not np.isinf(self.min_val) and not np.isnan(self.min_val):
            self.min_pub.publish(self.min_val)

        if not np.isinf(self.max_val) and not np.isnan(self.max_val):
            self.max_pub.publish(self.max_val)

    def process_scan(self):
        """
        Loop that publishes topics /closest_point and /farthest_point as long as the process
        is still running. Checks for inf or nan values.

        To check if working, in one terminal do:
        `rosrun reuben_wong_lab1 lidar_processor.py`
        In another terminal, do:
        `rostopic echo /closest_point`
        To view the subscription connections, run:
        `rosrun rqt_graph rqt_graph`
        """
        self.sub = rospy.Subscriber("/scan", LaserScan, callback=self.callback2)

        rospy.spin()

    def process_scan_range(self):
        """
        Loop that publishes the /scan_range topic. Makes use of the custom message scan_range.msg
        to publish. This is the one that is called in the assignment and what is recorded by the bagfile.
        """
        self.sub = rospy.Subscriber("/scan", LaserScan, callback=self.callback)

        rospy.spin()

        ##################### UNUSED ################################
        # def process_scan_range(self):

    #     """
    #     Loop that publishes the /scan_range topic. Makes use of the custom message scan_range.msg
    #     to publish. This is the one that is called in the assignment and what is recorded by the bagfile.
    #     """
    #     while not rospy.is_shutdown():
    #         # spin() not required, reference: https://get-help.robotigniteacademy.com/t/what-is-rospy-spin-ros-spin-ros-spinonce-and-what-are-they-for/58
    #         msg = scan_range()
    #         if not np.isinf(self.min_val) and not np.isnan(self.min_val):
    #             msg.closest_point = self.min_val

    #         if not np.isinf(self.max_val) and not np.isnan(self.max_val):
    #             msg.farthest_point = self.max_val

    #         if (not np.isinf(self.min_val) and not np.isnan(self.min_val)) or (
    #             not np.isinf(self.max_val) and not np.isnan(self.max_val)
    #         ):
    #             self.sr_pub.publish(msg)

    #         self.rate.sleep()

    # def process_scan(self):
    #     """
    #     Loop that publishes topics /closest_point and /farthest_point as long as the process
    #     is still running. Checks for inf or nan values.

    #     To check if working, in one terminal do:
    #     `rosrun reuben_wong_lab1 lidar_processor.py`
    #     In another terminal, do:
    #     `rostopic echo /closest_point`
    #     To view the subscription connections, run:
    #     `rosrun rqt_graph rqt_graph`
    #     """
    #     while not rospy.is_shutdown():
    #         if not np.isinf(self.min_val) and not np.isnan(self.min_val):
    #             self.min_pub.publish(self.min_val)

    #         if not np.isinf(self.max_val) and not np.isnan(self.max_val):
    #             self.max_pub.publish(self.max_val)

    #         self.rate.sleep()


if __name__ == "__main__":
    processor = LidarProcessor()
    processor.process_scan_range()
