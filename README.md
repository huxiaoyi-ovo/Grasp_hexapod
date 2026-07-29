# Grasp Hexapod Robot

抓取六足机器人的ROS Noetic工作空间，包含机器人描述、传统运动控制、
Isaac Gym仿真入口和LX-15D舵机驱动。

## 目录

```text
src/
├── grasp_hexapod_description/  # URDF、mesh、RViz/Gazebo
├── grasp_hexapod_control/      # scripts中为全部Python控制代码
├── grasp_hexapod_servo/        # scripts中为LX-15D通信与ROS节点
├── reference/                  # 已验证六足项目，仅作逻辑参考
└── docs/                       # 跨功能包的接口说明
```

功能包内的`scripts/`放Python，未来需要C++时才建立功能包内的`src/`。

## 环境

```bash
sudo apt update
sudo apt install \
  ros-noetic-desktop-full \
  python3-catkin-tools \
  python3-numpy \
  python3-rospkg \
  python3-serial

python3 -m pip install numpy matplotlib pygame
python3 -m pip install -e /path/to/isaacgym/python
```

## Clone与构建

```bash
source /opt/ros/noetic/setup.bash
git clone https://github.com/huxiaoyi-ovo/Grasp_hexapod.git grasp_hexapod_robot
cd grasp_hexapod_robot
git switch test
catkin_make
source devel/setup.bash
```

## 运行

```bash
# RViz模型
roslaunch grasp_hexapod_description display.launch

# Gazebo模型
roslaunch grasp_hexapod_description gazebo.launch

# Isaac Gym手柄控制：只需Isaac Gym的Conda环境，无需source ROS工作空间
/home/hxy/anaconda3/envs/hexapod_rl_env/bin/python \
  src/grasp_hexapod_control/scripts/run_sim.py

# 实机ROS感知接口；舵机闭环尚未接通
roslaunch grasp_hexapod_control run_real.launch

# 已验证的LX-15D节点
roslaunch grasp_hexapod_servo servo_dual_side.launch
```

外部感知话题、坐标系和自动接近接口见
[`src/docs/ROS_INTERFACES.md`](src/docs/ROS_INTERFACES.md)。

## 协作

```bash
git switch test
git pull --ff-only origin test
git switch -c feat/your-feature

git add <changed-files>
git commit -m "Add your feature"
git push -u origin feat/your-feature
```

Pull Request目标分支统一为：

```text
feat/your-feature -> test
```
