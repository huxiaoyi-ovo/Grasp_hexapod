# grasp_hexapod_msgs

保存本工作空间自建的消息（`msg/`）和服务（`srv/`）定义，供控制、感知和驱动等
包统一引用，避免接口定义散落在各功能包里。

## 新增一条消息

1. 在 `msg/` 下新建 `Xxx.msg`，例如：

   ```text
   std_msgs/Header header
   float64 angle_deg
   string state
   ```

2. 在包根目录 `CMakeLists.txt` 的 `add_message_files(FILES ...)` 中列出 `Xxx.msg`；
   字段用到其他包的类型（如 `std_msgs/Header`）时，确认 `find_package`、
   `generate_messages(DEPENDENCIES ...)` 和 `package.xml` 已包含对应包。
3. 在工作空间根目录 `catkin_make` 后即可使用：
   Python `from grasp_hexapod_msgs.msg import Xxx`，
   C++ `#include <grasp_hexapod_msgs/Xxx.h>`。

## 新增一个服务

1. 在 `srv/` 下新建 `Xxx.srv`（`---` 上方为请求、下方为响应）。
2. 在 `CMakeLists.txt` 的 `add_service_files(FILES ...)` 中列出 `Xxx.srv`。
3. 重新 `catkin_make` 后即可使用：
   Python `from grasp_hexapod_msgs.srv import Xxx`，
   C++ `#include <grasp_hexapod_msgs/Xxx.h>`。

## 其他包引用本包

- `package.xml`：`<depend>grasp_hexapod_msgs</depend>`
- `CMakeLists.txt`：`find_package(catkin REQUIRED COMPONENTS ... grasp_hexapod_msgs)`
- Python 直接 import；C++ 头文件在 `grasp_hexapod_msgs/` 下。

## 现有自建接口备忘

- 服务 `GripperCommand.srv`（夹爪开合命令）目前仍定义在
  `grasp_hexapod_servo_cpp/srv/` 并被原包引用，按协作准则暂不迁移；如需迁移到本包，
  需同步修改 `grasp_hexapod_servo_cpp` 的构建与引用，先沟通确认。
- 现有自建话题（`/grasp_hexapod/perception/*`、`/grasp_hexapod/navigation/*` 等）
  暂复用标准消息类型（`String`、`Float64`、`PoseStamped`、`PolygonStamped` 等）；
  后续如需结构化字段，把对应 `.msg` 放进本包即可。
