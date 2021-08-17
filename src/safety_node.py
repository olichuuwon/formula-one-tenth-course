#!/usr/bin/env python
# TODO: import ROS msg types and libraries
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math

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
        """
        self.speed = 0
	self.ttc_limit = 0.60
        # TODO: create ROS subscribers and publishers.
	self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
	self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
	self.acker_msg = rospy.Publisher('brake', AckermannDriveStamped, queue_size=1)
	self.bool_msg = rospy.Publisher('brake_bool', Bool, queue_size=1)
	self.acker_object = AckermannDriveStamped()

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
	self.ttc_list = [8888.8888]
	for i in range(len(scan_msg.ranges)):
		distance = scan_msg.ranges[i]
		angle = scan_msg.angle_min + i * scan_msg.angle_increment
        	range_rate = self.speed * math.cos(angle)
		if range_rate > 0:
			self.ttc_list.append(distance/range_rate)
        # TODO: publish brake message and publish controller bool
	# if brake
	if min(self.ttc_list) < self.ttc_limit:
		self.acker_object.drive.speed = 0.0
		self.acker_msg.publish(self.acker_object)
		self.bool_msg.publish(True)
		print(min(self.ttc_list))
	# if no brake 
	else:
		self.bool_msg.publish(False)

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
