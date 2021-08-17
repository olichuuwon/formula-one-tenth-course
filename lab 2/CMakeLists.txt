cmake_minimum_required(VERSION 3.0.2)
project(jeslyn_roslab)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/safety_node.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
