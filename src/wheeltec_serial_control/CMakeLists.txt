cmake_minimum_required(VERSION 2.8.3)
project(wheeltec_serial_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  tf
)

catkin_package()

# 安装Python脚本
catkin_install_python(PROGRAMS
  scripts/wheeltec_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
