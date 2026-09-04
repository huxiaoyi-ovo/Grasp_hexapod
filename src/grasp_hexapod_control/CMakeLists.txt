cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_control)

find_package(
  catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  grasp_hexapod_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs grasp_hexapod_msgs rospy sensor_msgs std_msgs tf tf2_ros trajectory_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/navigation_node.py
    scripts/run_real.py
    scripts/run_sim.py
    scripts/dock_static_tf.py
    scripts/tag_perception.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    scripts/approach_mode.py
    scripts/climb_mode.py
    scripts/control.py
    scripts/dock_mode.py
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
    config/climb_compact.json
    config/climb_rebuild_profile.json
    config/dock_camera.yaml
    config/dock_system.yaml
    config/navigation_rtk_imu.yaml
    config/workspace_bounds.csv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
