cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_servo_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  grasp_hexapod_msgs
  message_generation
  roscpp
  sensor_msgs
  serial
  std_msgs
)

add_service_files(
  FILES
  GripperCommand.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grasp_hexapod_servo_cpp
  CATKIN_DEPENDS grasp_hexapod_msgs message_runtime roscpp sensor_msgs serial std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 驱动核心库：协议控制器 + 夹爪管理 + 单板节点（供可执行文件与单元测试复用）。
add_library(${PROJECT_NAME}
  src/gripper_manager.cpp
  src/hiwonder_servo_controller.cpp
  src/servo_side_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# 单板舵机驱动节点：每实例管理一块板（9 个舵机、3 条腿，左板含夹爪）。
add_executable(servo_node src/servo_node.cpp)
target_link_libraries(servo_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(servo_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_servo_protocol test/test_servo_protocol.cpp)
  target_link_libraries(test_servo_protocol ${PROJECT_NAME})
  add_dependencies(test_servo_protocol ${catkin_EXPORTED_TARGETS})

  catkin_add_gtest(test_gripper test/test_gripper.cpp)
  target_link_libraries(test_gripper ${PROJECT_NAME})
  add_dependencies(test_gripper ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS servo_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
