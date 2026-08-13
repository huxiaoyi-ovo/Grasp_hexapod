cmake_minimum_required(VERSION 3.0.2)
project(hexapod_perception)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/perception_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
