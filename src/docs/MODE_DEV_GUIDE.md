# 模式开发指南（bt_control_node 模式框架）

本包的控制节点 `bt_control_node` 是一个**模式调度器**：它对外提供与旧版完全
一致的阻塞服务 `/grasp_hexapod/switch_mode` 与 `/grasp_hexapod/hold_motion`
心跳租约，对内把每个模式（home/walk/approach/climb/dock/release/spin_search）
拆成**一对独立文件**（`mode_<name>.h` / `mode_<name>.cpp`），由不同作者分别
维护，互不相干。

## 新增一个模式只要三步

1. **复制模板**：把 `templates/mode_template.h/.cpp` 复制为
   `include/grasp_hexapod_bt_control/modes/mode_<name>.h` 和
   `src/modes/mode_<name>.cpp`，替换占位符（`<name>` 小写模式名、`<Name>` 类
   名前缀、`__NAME__` 注册名字符串），实现 `start/step/checkTerminal`。
   模式类统一放在 `grasp_hexapod_bt_control::modes` 命名空间
   （避免与核心库 ClimbMode/DockMode/ApproachMode 撞名）。
   `.cpp` 末尾的 `REGISTER_MODE("__NAME__", NameMode)` 一行即完成接入——
   调度器启动时自动发现，**不需要改调度器和任何别人的文件**。
2. **加一节 YAML**：在 `config/mode_topics.yaml` 增加 `__NAME__:` 节，写上
   本模式订阅/发布的话题名。模式内用 `topics().get("key")` 读取；缺 key 会
   在启动时报 `mode_topics.yaml[__NAME__.key] is missing`。
3. **CMakeLists 加一行**：把 `src/modes/mode_<name>.cpp` 加进
   `MODE_SOURCES` 列表。

`spin_search` 是现成的占位例子：`mode_spin_search.cpp` 用
`REGISTER_MODE_STUB` 注册，行为树调用会得到
`executor not implemented: spin_search`（与旧版一致）；实现时把它换成
`REGISTER_MODE` 并补齐逻辑即可。

## 接口契约（mode_base.h，冻结）

模式作者只需要 include 两个头：`mode_base.h`（契约）与 `mode_context.h`
（上下文）。**不要 include `bt_control_node.h`。**

```cpp
class MyMode : public ModeBase {
  bool start(const JointAngles& q_cur, std::string& reason);   // 必须
  std::optional<JointAngles> step(const ModeStepInput& in);    // 必须
  std::optional<ModeOutcome> checkTerminal();                  // 必须
  // 可选：pause/resume/stop/onNodeHold/onPreFrame/startFromAnyState
};
```

- `start`：入口门控 + 进入模式（通常 `ctx().setMachineState(Running)`）。
  返回 `false` + `reason` 表示拒绝，reason 原样回给行为树。
  调度器保证非全局模式只在 `Hold` 状态调用 start（WAIT_B 会先被
  "call home first" 拒绝，Running 会先走抢占回正）。
- `step`：每帧推进并返回本帧 18 关节目标（一般调
  `ctx().controller().update(...)`）；`nullopt` 表示本帧不发布。
- `checkTerminal`：每帧终态检测；到终态先做收尾（夹爪、切 Hold）再返回
  `ModeOutcome`。也可以在 start/step 里直接 `finish(结果, 消息)`。
- `wait(timeout)` / `status()`：框架提供，服务线程阻塞等待用，模式不用管。

### 暂停/恢复/停止语义

- `pause(reason)` = **原地冻结**：框架冻结目标发布、不再调用 step；
  需要附加动作的模式（如 climb 的 `holdClimb()`）先调 `ModeBase::pause(reason)`
  再做自己的事。`resume()` 从冻结点继续。
  触发源是行为树 `/grasp_hexapod/hold_motion` 心跳租约（PauseGate）。
- `stop(smooth)`：显式停止钩子。注意：**抢占不会调用 stop**——抢占时框架
  直接把结果置为 `Preempted` 并由调度器执行统一的中止序列
  （`abortActiveMotion`：取消任务 + 中止攀爬/对接 + 平滑回正）。
- `onNodeHold(reason)`：反馈丢失等节点级看门狗；返回 true 表示本模式已
  完全处理（见 DockMode），false 走通用路径。
- `onPreFrame(input)`：每帧前置检查（暂停判断之前），用于输入健康监视
  （见 WalkMode 的 `/cmd_vel` 超时）。
- `startFromAnyState()`：默认 false；仅 home 这类可从任意状态直接进入的
  全局模式覆写为 true。

## ModeContext（模式看到的全部世界）

| 方法 | 用途 |
|---|---|
| `nodeHandle()` | 公共命名空间，建自己的订阅/发布 |
| `privateNodeHandle()` | 私有命名空间，读自己的 `~` 参数 |
| `controller()` | `GraspController` 运动核心（步态/运动学/攀爬/对接原语），直接调用 |
| `inputs()` | 共享输入只读快照：`qCur()/feedbackStamps()/imu()/navigationSnapshot()/navigationMotionSnapshot()` |
| `callGripper("open"/"clamp")` | 夹爪服务同步调用，返回 `<成功, 消息>` |
| `machineState()/setMachineState()` | 机器状态机（WaitB/Resetting/Hold/Running） |
| `abortActiveMotion(reason)` | 中止当前运动并平滑回正（home 与抢占共用） |

六腿关节反馈、IMU、导航位姿订阅由调度器常驻持有（安全看门狗在无模式运行
时也依赖它们；approach 与 climb 共用导航输入），模式经 `inputs()` 只读访问；
其余订阅（如 walk 的 `/cmd_vel`、dock 的 `lock_confirmed`）由各模式自持。

## 模式状态反馈

调度器在 `bt.mode_status`（默认 `/grasp_hexapod/mode_status`，
`grasp_hexapod_msgs/ModeStatus`）发布模式状态：变化即发 + 空闲 1Hz 心跳。
`status ∈ idle/running/paused/success/failed/preempted`，含调用代次 run 与
终态/暂停原因。模式作者零成本——无需发布任何状态。

## 线程契约（必须遵守）

1. `start/step/checkTerminal/pause/resume/stop/onNodeHold/onPreFrame` 全部在
   **120Hz 控制线程**调用，**绝不阻塞**（不 sleep、不等服务、不加长锁）。
   夹爪用 `ctx().callGripper(...)`（与原实现一致的短同步调用）。
2. 订阅回调在 AsyncSpinner 线程：**只做加锁缓存最新消息**，不做计算。
3. `wait/status` 在服务线程；同步由框架保证。
4. `GraspController` 的调用只允许出现在 1 的上下文里。

## 话题名配置（config/mode_topics.yaml）

- 单文件按模式分节，节名 = switch_mode 的模式名；`shared/services/bt` 为
  框架保留节。多人编辑各自节，冲突概率低。
- 节点用 yaml-cpp 直读，路径由 `~topics_config` 指定
  （launch 默认指向本包 `config/mode_topics.yaml`）。
- `shared.joint_feedback/joint_command` 必须是 6 个话题、按腿索引
  `lf,lm,lb,rf,rm,rb` 排序。
- launch 文件不再逐个设置话题参数（旧 `~base_pose_topic` 等 8 个参数已删除；
  `dock_image_topic/dock_camera_info_topic` 为死参数一并清理）。

## 目录结构

```
include/grasp_hexapod_bt_control/
  mode_base.h          # 接口契约（冻结，修改需全队同意）
  mode_context.h       # 统一上下文（MachineState/SharedInputs/ModeContext）
  mode_registry.h      # 注册表 + REGISTER_MODE 宏（单头文件）
  mode_topic_config.h  # mode_topics.yaml 加载器
  io_inputs.h          # NavigationInput/ImuInput/BoolInput 订阅封装
  bt_control_node.h    # 调度器（模式作者不需要看）
  modes/mode_*.h       # 各模式头（每人一对）
src/
  mode_base.cpp  mode_topic_config.cpp  io_inputs.cpp
  bt_control_node.cpp  # 调度器：switch_mode 服务/租约/看门狗/发布
  modes/mode_*.cpp     # 各模式实现 + 文件末尾 REGISTER_MODE 自注册
config/mode_topics.yaml
templates/mode_template.h/.cpp   # 新模式模板（不编译）
```

## 调用链（与旧版对外行为逐字段一致）

```
行为树 RunMode ──> /grasp_hexapod/switch_mode（阻塞）
  └ 调度器：查注册表 → 抢占旧调用(Preempted) → 门控 → mode->start()
    → 控制环每帧 mode->step() / pollTerminal()
    → 服务线程 mode->wait() 返回终态 ←────────────┘
行为树 PauseGate ─> /grasp_hexapod/hold_motion 心跳 ─> mode->pause()/resume()
```

同模式重复调用合并等待；不同模式调用触发抢占（旧调用立即得到
`preempted by <new>`）；WAIT_B 只接受 home；dock 成功后必执行夹爪闭合
（在 DockMode::checkTerminal 内）。

## 测试

- `test/test_bt_control_routing.cpp`：反馈门控/限幅纯函数（旧测试，不变）。
- `test/test_mode_framework.cpp`：注册表、YAML 加载器、ModeLifecycle
  状态机（run 代次/wait/pause/抢占 superseded）。
- 新模式建议给纯逻辑部分写 gtest；e2e 用 `test/fake_hardware_e2e.py`。
