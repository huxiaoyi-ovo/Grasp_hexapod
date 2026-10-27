cmake_minimum_required(VERSION 3.0.2)
project(xiaolan_yolo_detection)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs cv_bridge message_generation)
catkin_python_setup()
add_message_files(FILES XiaolanDetection.msg XiaolanTarget.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge message_runtime)
catkin_install_python(
  PROGRAMS
    scripts/xiaolan_yolo_node.py
    scripts/xiaolan_target_3d_node.py
    scripts/xiaolan_visualizer.py
    scripts/xiaolan_image_collector.py
    scripts/prepare_xiaolan_dataset.py
    scripts/build_trt_engine.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch models docs requirements DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
