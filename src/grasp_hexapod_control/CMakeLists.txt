cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_control)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/run_real.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    scripts/approach_mode.py
    scripts/climb_mode.py
    scripts/control.py
    scripts/dock_mode.py
    scripts/kinematics.py
    scripts/utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
