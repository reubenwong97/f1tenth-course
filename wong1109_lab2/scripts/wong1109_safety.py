#!/usr/bin/env python
import rospy
import numpy as np

# TODO: import ROS msg types and libraries
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class Safety(object):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        NOTE time to stop (based on kinematics) = speed / deceleration,
             hopefully this is < TTC
        """
        self.speed = 0
        self.TTC_PARAM = 2.5
        rospy.loginfo("TTC_PARAM %.2f" % self.TTC_PARAM)
        self.DECEL = 8.26  # found from f1tenth simulator params.yaml
        # TODO: create ROS subscribers and publishers.
        self.odom_sub = rospy.Subscriber("/odom", Odometry, callback=self.odom_callback)
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, callback=self.scan_callback
        )
        self.brake_bool_pub = rospy.Publisher("/brake_bool", Bool, queue_size=1)
        self.brake_pub = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=1)

    def odom_callback(self, odom_msg):
        """
        Simple callback to update the current speed of the vehicle.
        """
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x  # verified y component always zero

    def scan_callback(self, scan_msg):
        """
        Callback that implements the AEB. Implementation is vectorized for performing all computations
        on all elements of the numpy arrays. Also filters out invalid TTCs based on whether the initial
        range values recorded were outside of the acceptable ranges (range_max and range_min).
        """
        self.range_min = scan_msg.range_min
        self.range_max = scan_msg.range_max
        self.angle_min = scan_msg.angle_min
        self.angle_max = scan_msg.angle_max
        self.increment = scan_msg.angle_increment
        self.ranges = scan_msg.ranges
        self.ranges = np.array(self.ranges)
        self.angles = []

        # TODO: calculate TTC
        for i in range(len(self.ranges)):
            angle_to_head = self.angle_min + i * self.increment
            self.angles.append(angle_to_head)

        self.angles = np.array(self.angles)
        self.time_derivatives = np.maximum(-1 * self.speed * np.cos(self.angles), 0)
        self.ttcs = self.ranges / self.time_derivatives
        # vectorized processing of ttcs
        # filer out values < range_min or > range_max, use ranges for indexing criteria
        self.ttcs = self.ttcs[
            np.logical_or(
                ~(self.ranges < self.range_min), ~(self.ranges > self.range_max)
            )
        ]

        # closest collision
        self.ttc = np.min(self.ttcs)
        # time to stop (based on kinematics) = speed / deceleration, hopefully this is < TTC
        # NOTE: speed can be <0, without taking abs, this condition would not trigger in reverse
        crash_var = self.TTC_PARAM * np.abs(self.speed) / self.DECEL

        # TODO: publish brake message and publish controller bool
        if self.ttc <= crash_var and not np.isinf(self.ttc):
            msg_bool = Bool(data=True)
            msg_drive = AckermannDriveStamped()
            msg_drive.drive.speed = 0.0

            self.brake_bool_pub.publish(msg_bool)
            self.brake_pub.publish(msg_drive)

            rospy.loginfo("E-brake message sent!")
        else:
            msg_bool = Bool(data=False)
            self.brake_bool_pub.publish(msg_bool)


def main():
    rospy.init_node("safety_node")
    sn = Safety()
    rospy.spin()


if __name__ == "__main__":
    main()
