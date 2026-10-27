#!/bin/bash

###########################################
apt_install_command="sudo apt-get install -y"

###########################################
# 获取当前脚本所在的目录，切换到脚本所在目录
current_dir="$(dirname "$(readlink -f "$0")")"
cd $current_dir
package_dir="$current_dir"

###########################################
# 检测 Ubuntu 版本和处理系统版本兼容性需求
version=$(lsb_release -sc) 
echo "检测 Ubuntu 版本: $version"

if [ "$version" = "xenial" ]; then   # ubuntu16
    ROS_DISTRO="kinetic"
    echo "适配到的 ROS 发行版: $ROS_DISTRO"

elif [ "$version" = "bionic" ]; then # ubuntu18
    ROS_DISTRO="melodic"
    echo "适配到的 ROS 发行版: $ROS_DISTRO"
    $apt_install_command ros-melodic-rviz-imu-plugin
    pip install pyserial numpy matplotlib

elif [ "$version" = "focal" ]; then  # ubuntu20
    ROS_DISTRO="noetic"
    echo "适配到的 ROS 发行版: $ROS_DISTRO"
    $apt_install_command ros-melodic-rviz-imu-plugin
    pip install pyserial numpy matplotlib

elif [ "$version" = "jammy" ]; then  # ubuntu22
    echo "pass"

elif [ "$version" = "mantic" ]; then # ubuntu24
    echo "pass"

else
    echo "不支持的 Ubuntu 版本, 退出."
    exit 1
fi

###########################################
# 设置本脚本环境变量
source /opt/ros/$ROS_DISTRO/setup.bash

###########################################
# 执行脚本的实际任务
echo "编译软件包......"
cd $package_dir/../..
catkin_make --pkg handsfree_ros_imu

echo "设置udev USB规则......"
cd $package_dir
sudo cp usb_rules/handsfree_ros_imu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb

# 修改 Python 文件权限
chmod 777 ./demo/python/*.py
chmod 777 ./scripts/*.py

###########################################
echo "软件包安装程序运行完毕"
