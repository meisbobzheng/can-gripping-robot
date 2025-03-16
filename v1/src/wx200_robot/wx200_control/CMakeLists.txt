cmake_minimum_required(VERSION 3.0.2)
project(wx200_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/wx200_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
