#!/bin/bash

# 运行脚本兼容 Ubuntu和ROS 的不同版本，支持在任意目录启动
###########################################
# 获取当前脚本所在的目录，切换到脚本所在目录
current_dir="$(dirname "$(readlink -f "$0")")"
cd "$current_dir"
package_dir="$current_dir/../.."

###########################################
# 检测 Ubuntu 版本和 ROS 版本
version=$(lsb_release -sc) 
case $version in
    "xenial") ROS_DISTRO="kinetic" ;;
    "bionic") ROS_DISTRO="melodic" ;;
    "focal")  ROS_DISTRO="noetic" ;;
    *)
        echo "不支持的 Ubuntu 版本用于 ROS 安装. 退出."
        exit 1
        ;;
esac
echo "检测 Ubuntu 版本: $version"
echo "适配到的 ROS 发行版: $ROS_DISTRO"

###########################################
# 设置本脚本环境变量
source ../env/env.sh

# 运行功能
{
  gnome-terminal -x bash -c "roscore"
}&
sleep 1s
{
  gnome-terminal -x bash -c "rosrun rviz rviz -d ~/handsfree/handsfree_kit_ws/src/handsfree_ros_imu/rviz/handsfree_ros_imu.rviz"
}&
sleep 1s
{
  gnome-terminal -x bash -c "rosbag play -l ~/handsfree/handsfree_kit_ws/src/handsfree_ros_imu/rosbag/all_imu_dataset.bag"
}
###########################################
# 运行说明
# 程序启动后，测试实物-IMU可视化

