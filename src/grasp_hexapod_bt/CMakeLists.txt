cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_bt)

find_package(
  catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  grasp_hexapod_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs grasp_hexapod_msgs
)

# 各脚本同目录安装：bt_mock_world / sim_feedback / run_real_bt 通过同目录
# import hexapod_bt / sensor_health_monitor / bt_mock_world
catkin_install_python(
  PROGRAMS
    scripts/hexapod_bt.py
    scripts/bt_mock_world.py
    scripts/sensor_health_monitor.py
    scripts/remote_control.py
    scripts/sim_feedback.py
    scripts/sim_manual.py
    scripts/run_real_bt.py
    scripts/bt_monitor.py
    scripts/bt_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    config/sensor_health.yaml
    config/real_bt.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY behavior_trees
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
