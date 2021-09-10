#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# PID CONTROL PARAMS
# kp = 19.5
# kd = 0.017
# ki = 0.00017

# Experimental Params
kp = 2.6
kd = 0.025
ki = 0.0025

servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0
current_time = 0.0
prev_time = 0.0

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9  # meters
DESIRED_DISTANCE_LEFT = 0.55
LOOK_AHEAD = 0.75
VELOCITY = 2.00  # meters per second
CAR_LENGTH = 0.50  # Traxxas Rally is 20 inches or 0.5 meters
THETA = 1.13


class WallFollow:
    """Implement Wall Following on the car"""

    def __init__(self):
        # Topics & Subs, Pubs
        lidarscan_topic = "/scan"
        drive_topic = "/nav"

        self.lidar_sub = rospy.Subscriber(
            lidarscan_topic, LaserScan, callback=self.lidar_callback
        )
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=1
        )

    def getRange(self, data, angle):
        """
        Rotates (corrects) the angle given where 0 is to the right of the car, into a format that
        we can easily index out the corresponding range value from.

        As a note to self, counter-clockwise is positive...
        """
        corrected_angle = angle - math.radians(90)
        range_index = int((corrected_angle - data.angle_min) / data.angle_increment)

        range = data.ranges[range_index]

        return range, range_index, corrected_angle

    def pid_control(self, error, velocity):
        """
        Function that implements the pid controller.
        """
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_time
        global current_time
        # accumulate integral
        prev_time = current_time
        current_time = rospy.Time.now().to_sec()
        dt = current_time - prev_time
        integral += error

        angle = kp * error + ki * integral + kd * (error - prev_error) / dt

        if abs(math.degrees(angle)) < 10:
            velocity = 1.5
        elif abs(math.degrees(angle)) < 20 and abs(math.degrees(angle)) >= 10:
            velocity = 1.0
        else:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity

        self.drive_pub.publish(drive_msg)

        # update prev_error
        prev_error = error

    def followLeft(self, data, leftDist):
        """
        Function that computes the error when following the left wall.
        """
        # Follow left wall as per the algorithm
        angle_b = 3.14
        angle_a = 2.356

        # collects debug information as well
        range_b, b_i, b_a = self.getRange(data, angle_b)
        range_a, a_i, a_a = self.getRange(data, angle_a)

        alpha = math.atan2(
            (range_a * math.cos(THETA) - range_b),
            (range_a * math.sin(THETA)),
        )
        d_t = range_b * math.cos(alpha)
        d_t1 = d_t + LOOK_AHEAD * math.sin(alpha)

        error = d_t1 - leftDist
        return error

    def lidar_callback(self, data):
        """ """
        error = self.followLeft(
            data, DESIRED_DISTANCE_LEFT
        )  # TODO: replace with error returned by followLeft
        # send error to pid_control
        self.pid_control(error, VELOCITY)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
