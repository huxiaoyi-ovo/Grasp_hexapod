cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_bt_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grasp_hexapod_control
  grasp_hexapod_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  roslib
)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs grasp_hexapod_control grasp_hexapod_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros roslib
  DEPENDS EIGEN3
)

include_directories(
  include
  # climb_compact.h 等公共头包含 <nlohmann/json.hpp>，头文件在
  # grasp_hexapod_control/vendor 下（仓库内兄弟包相对路径约定）。
  ${CMAKE_CURRENT_SOURCE_DIR}/../grasp_hexapod_control/vendor
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

# ---------------------------------------------------------------------------
# bt_control_node：面向行为树的高层控制节点（服务切换状态机 + /cmd_vel 步态）。
# 数值核心复用 grasp_hexapod_control 的 grasp_hexapod_control_core 纯计算库，
# 本包只实现 ROS 壳（对原 real_control_node 的服务化改造版）。
# ---------------------------------------------------------------------------
add_executable(bt_control_node
  node/bt_control_main.cpp
  src/bt_control_node.cpp
)
target_link_libraries(bt_control_node
  grasp_hexapod_control_core
  grasp_hexapod_control_ros
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(bt_control_node ${catkin_EXPORTED_TARGETS})

# ---------------------------------------------------------------------------
# remote_control_node：手柄 → 服务调用 + /cmd_vel（原 /joy 直连链路的替代）。
# ---------------------------------------------------------------------------
add_executable(remote_control_node
  node/remote_control_main.cpp
  src/remote_control_node.cpp
)
target_link_libraries(remote_control_node
  ${catkin_LIBRARIES}
)
add_dependencies(remote_control_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  set(GTEST_MAIN_SRC /usr/src/googletest/googletest/src/gtest_main.cc)
  catkin_add_gtest(test_bt_control_routing
    test/test_bt_control_routing.cpp
    src/bt_control_node.cpp
    ../grasp_hexapod_control/src/dock_perception_ros.cpp
    ${GTEST_MAIN_SRC}
  )
  target_link_libraries(test_bt_control_routing grasp_hexapod_control_core ${catkin_LIBRARIES})
  target_compile_definitions(test_bt_control_routing PRIVATE
    GRASP_CONTROL_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../grasp_hexapod_control/config")
endif()

install(TARGETS bt_control_node remote_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
