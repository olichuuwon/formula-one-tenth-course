#!/usr/bin/env python
from __future__ import print_function

import math
import sys

import numpy as np
# ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# PID CONTROL PARAMS
kp = 1.0
kd = 0.001
ki = 0.005
servo_offset = 0.0
prev_error = 0.0
error = 0.0
time = 0.0
integral = 0.0

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9  # meters
DESIRED_DISTANCE_LEFT = 0.55  # meters
VELOCITY = 1.5  # meters per second
FUTURE = 0.3
CAR_LENGTH = 0.50  # Traxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ Implement Wall Following on the car
    """

    def __init__(self):
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        odom_topic = '/odom'
        self.speed = 0
        # Subscribe to LIDAR & Publish to drive
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

        # Init time
        self.prev_time = rospy.get_rostime().to_sec()

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd

        # Use kp, ki & kd to implement a PID controller for 
        # Finding angle and velocity

        now_time = rospy.get_rostime().to_sec()
        gap_time = now_time - self.prev_time
        # rospy.loginfo(type(gap_time))
        integral += prev_error * float(gap_time)
        angle = (kp * error + kd * (error - prev_error) / gap_time + ki * integral)
        self.prev_time = now_time
        prev_error = error
        if abs(angle) > 1 / 9 * math.pi:
            velocity = 0.5
        elif abs(angle) > 1 / 18 * math.pi:
            velocity = 1.0
        else:
            velocity = 1.5

        # Takes care of publishing drive_msg ackermann messages
        # rospy.loginfo(str(angle))
        # rospy.loginfo(str(velocity))
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        change_in_angle = data.angle_increment
        ranges_output = data.ranges
        range_isnan = np.isnan(ranges_output)
        range_isinf = np.isinf(ranges_output)
        for index in range(len(ranges_output)):
            if range_isnan[index]:
                ranges_output[index] = 0
            if range_isinf[index]:
                ranges_output[index] = 1
        current_time = rospy.get_rostime().to_sec()

        a_index = int(round((math.pi / 4) / change_in_angle))
        b_index = int(round((math.pi / 2) / change_in_angle))

        a_range = ranges_output[a_index]
        b_range = ranges_output[b_index]

        consta = math.pi / 4
        denominator = b_range * math.sin(consta)
        alpha = math.atan((b_range * math.cos(consta) - a_range) / denominator)
        distance = a_range * math.cos(alpha)

        if self.speed == None:
            future_distance = distance + VELOCITY * FUTURE * math.sin(alpha)
        else:
            future_distance = distance + self.speed * FUTURE * math.sin(alpha)

        error = DESIRED_DISTANCE_RIGHT - future_distance

        # Send error to pid_control
        self.pid_control(error, VELOCITY)


def main(args):
    rospy.init_node("wall_follower", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

