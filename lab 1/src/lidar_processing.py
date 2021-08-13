#!/usr/bin/env python

# imports
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

# declaring publishers
close_pub = rospy.Publisher('closest_point', Float32, queue_size=1)
far_pub = rospy.Publisher('farthest_point', Float32, queue_size=1)

# to reroute data to own publishers
# data.ranges is a list of the ranges detect on lidar
# min max simply gets required info tutorials required
def republish(data):
	close_pub.publish(min(data.ranges))
	far_pub.publish(max(data.ranges))
	
# main
# initialise node, then subscribe to /scan
# then once received some data do a republish
# and allow for spin
def main():
	try:
		rospy.init_node("lidar_node", anonymous=True)
		rospy.Subscriber("/scan", LaserScan, republish)
		rospy.spin()
	except rospy.ROSException:
		pass

if __name__ == '__main__':
	main()
