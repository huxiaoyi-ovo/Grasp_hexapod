cmake_minimum_required(VERSION 3.0.2)
project(grasp_hexapod_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# 自建消息：把 .msg 文件放进 msg/ 后在 FILES 中列出文件名（不带目录）。
# 字段用到其他包的类型（如 std_msgs/Header）时，同步补充 find_package 与
# generate_messages 的 DEPENDENCIES 以及 package.xml 依赖。
add_message_files(
  FILES
  SensorHealth.msg
  SensorHealthArray.msg
  RemoteCmd.msg
  EncoderState.msg
  BtNodeState.msg
  BtStateArray.msg
)

# 自建服务：把 .srv 文件放进 srv/ 后在 FILES 中列出文件名。
add_service_files(
  FILES
  SwitchMode.srv
  GripperAct.srv
  SimInject.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
