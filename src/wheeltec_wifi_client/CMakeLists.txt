cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_wifi_client)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/wifi_command_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
