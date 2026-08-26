cmake_minimum_required(VERSION 3.0.2)
project(encoder_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encoder_driver
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(encoder_driver
  src/encoder_frame.cpp
)
target_include_directories(encoder_driver PUBLIC include)
add_dependencies(encoder_driver ${catkin_EXPORTED_TARGETS})

add_executable(encoder_node
  src/encoder_node.cpp
)
target_link_libraries(encoder_node encoder_driver ${catkin_LIBRARIES})
add_dependencies(encoder_node ${catkin_EXPORTED_TARGETS})

install(TARGETS encoder_node encoder_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
