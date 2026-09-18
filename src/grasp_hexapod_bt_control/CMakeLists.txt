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
# 模式框架公共源（接口契约/注册表/YAML 加载/订阅输入封装）。
# ---------------------------------------------------------------------------
set(MODE_FRAMEWORK_SOURCES
  src/io_inputs.cpp
  src/mode_base.cpp
  src/mode_topic_config.cpp
)

# ---------------------------------------------------------------------------
# 各模式实现（每个模式一对 mode_<name>.h/.cpp，由不同作者独立维护；
# 新增模式 = 新增一对文件 + 在此列表加一行 + mode_topics.yaml 加一节）。
# ---------------------------------------------------------------------------
set(MODE_SOURCES
  src/modes/mode_approach.cpp
  src/modes/mode_climb.cpp
  src/modes/mode_dock.cpp
  src/modes/mode_home.cpp
  src/modes/mode_release.cpp
  src/modes/mode_spin_search.cpp
  src/modes/mode_walk.cpp
)

# ---------------------------------------------------------------------------
# bt_control_node：模式调度器（switch_mode 服务 + HOLD 租约 + 安全环路）。
# 数值核心复用 grasp_hexapod_control 的 grasp_hexapod_control_core 纯计算库；
# dock 感知/状态机走圆环链（grasp_hexapod_ring_perception + ring_core，
# 与 run_real_ring_cpp 同源），本包只实现 ROS 壳与各模式的 ROS 接线。
# ---------------------------------------------------------------------------
add_executable(bt_control_node
  node/bt_control_main.cpp
  src/bt_control_node.cpp
  ${MODE_FRAMEWORK_SOURCES}
  ${MODE_SOURCES}
)
target_link_libraries(bt_control_node
  grasp_hexapod_control_core
  grasp_hexapod_ring_perception
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
    ${MODE_FRAMEWORK_SOURCES}
    ${GTEST_MAIN_SRC}
  )
  target_link_libraries(test_bt_control_routing grasp_hexapod_control_core grasp_hexapod_ring_core ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
  target_compile_definitions(test_bt_control_routing PRIVATE
    GRASP_CONTROL_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../grasp_hexapod_control/config")

  catkin_add_gtest(test_mode_framework
    test/test_mode_framework.cpp
    ${MODE_FRAMEWORK_SOURCES}
    ${GTEST_MAIN_SRC}
  )
  target_link_libraries(test_mode_framework ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS bt_control_node remote_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
