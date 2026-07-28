# Grasp Hexapod Robot

抓取六足机器人模型与 Isaac Gym 手写控制器项目。

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- NVIDIA Isaac Gym Preview 4
- Git

安装项目所需的 ROS 和 Python 依赖：

```bash
sudo apt update
sudo apt install \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-rviz \
  ros-noetic-gazebo-ros \
  ros-noetic-urdf

python3 -m pip install numpy matplotlib pygame
```

安装 Isaac Gym。将路径替换为本机 Isaac Gym SDK 的实际位置：

```bash
python3 -m pip install -e /path/to/isaacgym/python
python3 -c "from isaacgym import gymapi; print('Isaac Gym ready')"
```

## Clone 与编译

```bash
source /opt/ros/noetic/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/huxiaoyi-ovo/Grasp_hexapod.git

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

需要长期使用时，可将工作空间环境加入 shell：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 运行

ROS：

```bash
roslaunch 抓取机器人export_urdf.SLDASM display.launch
roslaunch 抓取机器人export_urdf.SLDASM gazebo.launch
```

Isaac Gym：

```bash
cd ~/catkin_ws/src/Grasp_hexapod

# 固定机身检查模型
python3 scripts/view_isaacgym.py --fix-base

# 运行控制器，需要连接手柄
python3 scripts/run_control_isaacgym.py
```

## 协作开发

分支用途：

- `main`：稳定版本。
- `test`：日常集成与测试分支。
- `feat/*`、`fix/*`：个人功能或修复分支。

开始开发：

```bash
cd ~/catkin_ws/src/Grasp_hexapod

git switch test
git pull --ff-only origin test
git switch -c feat/your-feature
```

提交并推送：

```bash
git status
git diff

git add <changed-files>
git commit -m "Add your feature"
git push -u origin feat/your-feature
```

推送后，在 GitHub 创建 Pull Request：

```text
feat/your-feature -> test
```

开发过程中同步远程 `test`：

```bash
git fetch origin
git rebase origin/test
```

合并完成后清理本地分支：

```bash
git switch test
git pull --ff-only origin test
git branch -d feat/your-feature
```

## 提交要求

- 不直接向 `main` 推送开发中的代码。
- 一个分支只处理一个功能或问题。
- 不提交日志、缓存、虚拟环境和本地编辑器配置。
- 修改 URDF、关节方向或控制参数时，在 PR 中写明测试结果。
- 合并前至少运行对应的 ROS 或 Isaac Gym 入口，确认程序能够正常启动。
