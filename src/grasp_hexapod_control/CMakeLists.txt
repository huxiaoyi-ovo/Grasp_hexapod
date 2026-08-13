cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_control)

find_package(
  catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs rospy sensor_msgs std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/run_real.py
    scripts/run_sim.py
    scripts/tag_perception.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    scripts/approach_mode.py
    scripts/climb_mode.py
    scripts/control.py
    scripts/kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY scripts/utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
)

install(
  FILES
    config/climb.json
    config/climb_compact.json
    config/workspace_bounds.csv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
