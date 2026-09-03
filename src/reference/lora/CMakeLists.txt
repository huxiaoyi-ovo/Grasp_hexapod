cmake_minimum_required(VERSION 3.0.2)
project(lora)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/lora_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
