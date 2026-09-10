cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_servo)

find_package(catkin REQUIRED COMPONENTS
  grasp_hexapod_msgs
  rospy
  roscpp
  sensor_msgs
  serial
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grasp_hexapod_servo_cpp
  CATKIN_DEPENDS grasp_hexapod_msgs rospy roscpp sensor_msgs serial std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/servo.py
  scripts/servo_trace_replay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/hiwonder_servo_cmd.py
  scripts/hiwonder_servo_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# ---------------------------------------------------------------------------
# C++ 舵机驱动（原 grasp_hexapod_servo_cpp 合并而来）。
# ROS 接口与 scripts/servo.py 完全一致，运行期二选一。
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 驱动核心库：协议控制器 + 夹爪管理 + 单板节点（供可执行文件与单元测试复用）。
add_library(grasp_hexapod_servo_cpp
  src/gripper_manager.cpp
  src/hiwonder_servo_controller.cpp
  src/servo_side_node.cpp
)
target_link_libraries(grasp_hexapod_servo_cpp ${catkin_LIBRARIES})
add_dependencies(grasp_hexapod_servo_cpp ${catkin_EXPORTED_TARGETS})

# 单板舵机驱动节点：每实例管理一块板（9 个舵机、3 条腿，左板含夹爪）。
add_executable(servo_node node/servo_node.cpp)
target_link_libraries(servo_node grasp_hexapod_servo_cpp ${catkin_LIBRARIES})
add_dependencies(servo_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_servo_protocol test/test_servo_protocol.cpp)
  target_link_libraries(test_servo_protocol grasp_hexapod_servo_cpp)
  add_dependencies(test_servo_protocol ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(test_gripper test/test_gripper.cpp)
  target_link_libraries(test_gripper grasp_hexapod_servo_cpp)
  add_dependencies(test_gripper ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS servo_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
