cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_control_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  grasp_hexapod_msgs
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
  roslib
)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grasp_hexapod_control_core
  CATKIN_DEPENDS grasp_hexapod_msgs message_runtime roscpp sensor_msgs std_msgs geometry_msgs tf2_ros tf2 roslib
  DEPENDS EIGEN3
)

include_directories(
  include
  vendor
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# 数值核心库：不依赖 ROS，仅 Eigen + nlohmann/json + yaml-cpp。
# 对应 Python 侧 kinematics.py / utils / control.py / climb_mode.py /
# dock_mode.py(状态机) / approach_mode.py，供节点与 gtest 复用。
# ---------------------------------------------------------------------------
add_library(grasp_hexapod_control_core
  src/math_utils.cpp
  src/kinematics.cpp
  src/config_io.cpp
  src/climb_compact.cpp
  src/approach_mode.cpp
  src/climb_mode.cpp
  src/dock_mode.cpp
  src/control.cpp
)
target_link_libraries(grasp_hexapod_control_core
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(grasp_hexapod_control_core ${catkin_EXPORTED_TARGETS})

# ---------------------------------------------------------------------------
# ROS 壳：对应 run_real.py 的 RosControlNode（话题/服务/参数/状态机/BT仲裁）
# 与 dock_mode.py 的 DockPerception（tf2 查询）。
# ---------------------------------------------------------------------------
add_library(grasp_hexapod_control_ros
  src/dock_perception_ros.cpp
  src/real_control_node.cpp
)
target_link_libraries(grasp_hexapod_control_ros
  grasp_hexapod_control_core
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(grasp_hexapod_control_ros ${catkin_EXPORTED_TARGETS})

# 实机高层控制节点：节点名与 Python 版一致（同一时刻只运行一个实例）。
add_executable(run_real_cpp src/run_real_cpp_main.cpp)
target_link_libraries(run_real_cpp grasp_hexapod_control_ros ${catkin_LIBRARIES})
add_dependencies(run_real_cpp ${catkin_EXPORTED_TARGETS})

# ---------------------------------------------------------------------------
# 测试：金标数值回归（与 Python 实现逐帧对比）+ 运动学/模式纯函数单测。
# 金标数据与配置路径在编译期注入，测试无需 ROS master。
# ---------------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  # 本机 catkin 打包的 gtest 不含 gtest_main，直接补系统源文件。
  set(GTEST_MAIN_SRC /usr/src/googletest/googletest/src/gtest_main.cc)

  set(GRASP_CONTROL_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../grasp_hexapod_control/config")
  set(GRASP_DESCRIPTION_MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../grasp_hexapod_description/meshes")
  set(GRASP_GOLDEN_JSON "${CMAKE_CURRENT_SOURCE_DIR}/test/golden_control_trajectories.json")

  catkin_add_gtest(test_kinematics_golden test/test_kinematics_golden.cpp ${GTEST_MAIN_SRC})
  target_link_libraries(test_kinematics_golden grasp_hexapod_control_core)
  target_compile_definitions(test_kinematics_golden PRIVATE
    GRASP_GOLDEN_JSON="${GRASP_GOLDEN_JSON}"
    GRASP_CONTROL_CONFIG_DIR="${GRASP_CONTROL_CONFIG_DIR}")

  catkin_add_gtest(test_golden_trajectory test/test_golden_trajectory.cpp ${GTEST_MAIN_SRC})
  target_link_libraries(test_golden_trajectory grasp_hexapod_control_core)
  target_compile_definitions(test_golden_trajectory PRIVATE
    GRASP_GOLDEN_JSON="${GRASP_GOLDEN_JSON}"
    GRASP_CONTROL_CONFIG_DIR="${GRASP_CONTROL_CONFIG_DIR}")

  catkin_add_gtest(test_golden_config test/test_golden_config.cpp ${GTEST_MAIN_SRC})
  target_link_libraries(test_golden_config grasp_hexapod_control_core)
  target_compile_definitions(test_golden_config PRIVATE
    GRASP_GOLDEN_JSON="${GRASP_GOLDEN_JSON}"
    GRASP_CONTROL_CONFIG_DIR="${GRASP_CONTROL_CONFIG_DIR}"
    GRASP_DESCRIPTION_MESH_DIR="${GRASP_DESCRIPTION_MESH_DIR}")
endif()

install(TARGETS run_real_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
