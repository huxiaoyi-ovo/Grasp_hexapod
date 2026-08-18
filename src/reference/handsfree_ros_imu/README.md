[简体中文]|[English](README_EN.md)  

**[A9 / TB100 IMU 在线教程](https://alidocs.dingtalk.com/i/p/NE0VzgAErrZmJepPRBGvENgrKpvBgmDA)**  
**[B9 / B6 IMU 在线教程](https://alidocs.dingtalk.com/i/p/NE0VzgAErrZmJepPRBGvaMMQ9WDBAmDA)**  

# 1. Taobotics IMU 使用说明（ROS 环境）  

## 1.1 安装  
### 1.1.1 安装依赖  
使用如下指令完成串口驱动的依赖安装  
```sh
pip install pyserial numpy matplotlib # python2 安装
pip3 install pyserial numpy matplotlib  # python3 安装
```

### 1.1.2 编译
handsfree_ros_imu 包主要由python驱动脚本文件、usb挂载点udev规则和launch文件组成  

首先创建一个工作空间（这里工作空间名称可以自定义，这里默认为catkin_ws）  

```bash
cd & mkdir -p catkin_ws/src
```

在我们的在线文档中下载 handsfree_ros_imu.zip ,解压到 ~/catkin_ws/src 中  

在解压后的 handsfree_ros_imu 文件夹中打开终端，运行以下指令进行安装。    

```bash
cd ~/catkin_ws/src/handsfree_ros_imu/
bash auto_install.sh
```

## 1.2 使用  
可使用如下指令将 handsfree_ros_imu 的 ros 设置到.bashrc文件中，便于后期使用，“~/catkin_ws/”是工作空间，用户请根据实际情况修改。  

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 1.2.1 handsfree_imu.launch  

- **功能**：  
   IMU模块驱动程序，解析协议并发布ROS数据。    
- **串口名称**：  
   `/dev/HFRobotIMU`（可通过`~port`参数修改）  
- **发布话题**:  
  - `/handsfree/imu` ([sensor_msgs/Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html))，含姿态的四元数、角速度、线速度数据。    
  - `/handsfree/mag` ([sensor_msgs/MagneticField](https://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html))，磁力计数据。  

- **启动命令**：  

   ```bash
    roslaunch handsfree_ros_imu handsfree_imu.launch imu_type:=a9 port:=/dev/HFRobotIMU
   ```
    imu_type 可选 a9, tb100, b9, b6; 其中 a9, b9, b6 的 port 为 /dev/HFRobotIMU, tb100 的 port 为 /dev/ttyACM0。  

### 1.2.2 handsfree_imu_with_rviz.launch  
- **功能**：  
   启动 IMU 驱动，并使用 Rviz 可视化工具查看 IMU 姿态。 启动参数参考 handsfree_imu.launch  

### 1.2.3 handsfree_imu_with_ui.launch  

- **功能**：  
   启动 IMU 驱动，并使用 UI 查看 IMU 姿态。  启动参数参考 handsfree_imu.launch  

# 2. Taobotics IMU 使用说明（非ROS 环境）  

## 2.1 Python 程序  
使用如下指令完成串口驱动的依赖安装  
```sh
pip install pyserial numpy matplotlib # python2 安装
pip3 install pyserial numpy matplotlib  # python3 安装
```

程序位于 `./demo/python/` 目录下。  
以 a9 为例，Linux 环境下：  
```sh
python python_driver_a9.py --port /dev/ttyUSB0 --baud 921600
```

Windwos 环境下：  
```sh
python python_driver_a9.py --port COM3 --baud 921600。（具体端口号需用户自行确认）  
```

## 2.2 C++ 程序  
程序位于 ` ./demo/c++/` 目录下， 编译及运行方式请查看代码顶部的注释。  
