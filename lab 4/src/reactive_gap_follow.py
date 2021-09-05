#!/usr/bin/env python
from __future__ import print_function

import math
import sys

# ROS Imports
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class reactive_follow_gap:
    def __init__(self):
        # Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

    def preprocess_lidar(self, lidar_info):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = lidar_info.ranges
        for index in range(len(proc_ranges)):
            if math.isnan(proc_ranges[index]):
                proc_ranges[index] = 0.0
            elif math.isinf(proc_ranges[index]):
                proc_ranges[index] = 10.0

        i = 0
        window_size = 3
        moving_averages = []
        while i < len(proc_ranges) - window_size + 1:
            this_window = proc_ranges[i: i + window_size]
            window_average = sum(this_window) / window_size
            moving_averages.append(window_average)
            i += 1

        return moving_averages

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(data)
        min_angle = -60 / 180.0 * math.pi
        min_indx = math.floor((min_angle - data.angle_min) / data.angle_increment)
        max_angle = 60 / 180.0 * math.pi
        max_indx = math.ceil((max_angle - data.angle_min) / data.angle_increment)
        needed_range = []
        for index in range(int(min_indx), int(max_indx) + 1):
            needed_range.append(index)

        # Find closest point to LiDAR
        closest_value = 888.888
        closest_index = -1

        for index in needed_range:
            if proc_ranges[index] < closest_value:
                closest_value = proc_ranges[index]
                closest_index = index

        # Eliminate all points inside 'bubble' (set them to zero)
        radius = 180
        for index in range(int(closest_index - radius), int(closest_index + radius + 1)):
            proc_ranges[index] = 0.0

        # Find max length gap
        start = min_indx
        end = min_indx
        current_start = min_indx - 1
        current_length = 0
        max_length = 0
        for index in range(int(min_indx), int(max_indx + 1)):
            if current_start < min_indx:
                if proc_ranges[index] > 0.0:
                    current_start = index
            elif proc_ranges[index] <= 0.0:
                current_length = index - current_start
                if current_length > max_length:
                    max_length = current_length
                    start = current_start
                    end = index - 1
                current_start = min_indx - 1
        if current_start >= min_indx:
            current_length = max_indx + 1 - current_start
            if current_length > max_length:
                max_length = current_length
                start = current_start
                end = max_indx

        # Find the best point in the gap ( furthest )
        current_max = 0.0
        angle = 0.0
        for index in range(int(start), int(end + 1)):
            if proc_ranges[index] > current_max:
                current_max = proc_ranges[index]
                angle = data.angle_min + index * data.angle_increment
        velo = 4.80
        if abs(angle) > 1 / 9 * math.pi:
            velocity = velo * (1.0 / 3)
        elif abs(angle) > 1 / 18 * math.pi:
            velocity = velo * (2.0 / 3)
        else:
            velocity = velo

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
