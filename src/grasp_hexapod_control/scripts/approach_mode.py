"""接近模式的目标选择、固定路径和三角步态规划器。

功能：
    根据小蓝位姿和光伏板边界选择左右接近点，执行对齐、直线接近和最终
    停稳；也支持手柄速度输入。两种输入最终共用同一套三角步态。
输入：
    command.shape=(4,)=[向右、向前、向上、偏航]，单位m/s、m/s、m/s、rad/s；
    NavigationState提供pv_map中的六足、小蓝和光伏板边界。
输出：
    更新controller.foot_desired_base.shape=(6,3)，并给出接近任务状态。
结构：
    候选接近点 -> 边界检查与评分 -> 姿态对齐 -> 直线接近 -> 停稳；
    速度指令 -> 支撑/摆动阶段 -> 六足共同支撑 -> 三角组换相。
边界：
    不订阅ROS、不计算关节角、不处理攀爬或视觉对接。左右接近位姿必须
    经过实测后由外部配置，未配置时自动接近不能启动。
"""

from dataclasses import dataclass, field

import numpy as np

from utils import (
    NavigationState,
    circular_path_feasible,
    distance_to_polygon_boundary,
    points_in_polygon,
    transform_points,
    wrap_angle,
    yaw_from_transform,
)


TRIPOD_A_INDICES = np.array([0, 1, 5], dtype=np.int64)  # lb, lf, rm
TRIPOD_B_INDICES = np.array([2, 3, 4], dtype=np.int64)  # lm, rb, rf


@dataclass
class ApproachPlan:
    """当前自动接近目标和执行状态。"""

    active: bool = False
    state: str = "idle"
    target_side: str = ""
    target_pose_pv: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64)
    )
    score: float = np.inf
    minimum_clearance: float = 0.0
    ready_for_climb: bool = False
    failed: bool = False
    reason: str = ""


class ApproachMode:
    """接近模式：接近点决策、固定路径、平地步态和机身升降。"""

    def __init__(self, controller):
        self.controller = controller
        self.dt = controller.dt

        # 自动接近参数只有在左右标准位姿完成测量后才会启用。
        self.xiaolan_from_approach = {}
        self.approach_plan = ApproachPlan()
        self.navigation_state = None
        self.boundary_margin = 0.03
        self.path_sample_spacing = 0.02
        self.auto_linear_speed = 0.12
        self.auto_yaw_rate = 0.8
        self.position_tolerance = 0.03
        self.yaw_tolerance = np.deg2rad(5.0)
        self.robot_safety_radius = np.max(
            np.linalg.norm(controller.foot_init_base[:, :2], axis=1)
        )

        # True表示支撑腿，False表示摆动腿。
        self.gaits = np.zeros(6, dtype=bool)
        self.gaits[TRIPOD_A_INDICES] = True
        self.stance_group_index = 0

        # 每个摆动相固定为0.30 s；当前0.02 m/s实机限速下，
        # 单相机身位移约6 mm，先以小步幅验证平地行走。
        self.phase_duration = 0.30
        self.phase_time = 0.0

        # 共同支撑目标为50 ms，并量化到最接近的整数控制帧：
        # 60 Hz为3帧，30 Hz为2帧，避免频率变化时缓冲时间直接翻倍。
        self.transfer_frames = max(
            1,
            int(round(0.050 / self.dt)),
        )
        self.transfer_duration = self.transfer_frames * self.dt
        self.transfer_time = 0.0
        self.transfer_active = False

        self.step_height = 0.020
        self.phase_command = np.zeros(4, dtype=np.float64)
        self.stop_requested = False

        self.body_height_offset = 0.0
        self.body_height_offset_min = -0.0075
        self.body_height_offset_max = 0.015

        self.gait_started = False
        self.first_step = True

        self.swing_start_base = controller.foot_init_base.copy()
        self.swing_target_base = controller.foot_init_base.copy()
        self.foot_velocity_xy = np.zeros((6, 2), dtype=np.float64)
        self.swing_start_velocity_xy = np.zeros(
            (6, 2),
            dtype=np.float64,
        )
        self.swing_target_velocity_xy = np.zeros(
            (6, 2),
            dtype=np.float64,
        )

    def configure_autonomous_approach(
        self,
        xiaolan_from_left_base,
        xiaolan_from_right_base,
        boundary_margin=0.03,
        linear_speed=0.12,
        yaw_rate=0.8,
    ):
        """设置两侧标准接近位姿和经过实机确认的导航参数。

        xiaolan_from_*_base表示目标base_link在xiaolan_base中的4×4位姿。
        这些值与小蓝尺寸、攀爬动作和RTK天线外参有关，不能在控制器中猜测。
        """

        self.xiaolan_from_approach = {
            "left": np.asarray(
                xiaolan_from_left_base,
                dtype=np.float64,
            ).reshape(4, 4).copy(),
            "right": np.asarray(
                xiaolan_from_right_base,
                dtype=np.float64,
            ).reshape(4, 4).copy(),
        }
        self.boundary_margin = float(boundary_margin)
        self.auto_linear_speed = float(linear_speed)
        self.auto_yaw_rate = float(yaw_rate)

    def plan_autonomous_approach(self, navigation_state):
        """枚举左右接近点，选择边界内代价最低的固定直线路径。"""

        navigation = navigation_state.normalized()
        self.navigation_state = navigation

        if not navigation.valid:
            return self._fail_approach("navigation data is invalid")
        if not navigation.landing_confirmed:
            return self._fail_approach("landing is not confirmed")
        if len(navigation.pv_boundary) < 3:
            return self._fail_approach("pv boundary is unavailable")
        if set(self.xiaolan_from_approach) != {"left", "right"}:
            return self._fail_approach(
                "left/right approach poses are not configured"
            )

        current_xy = navigation.pv_from_base[:2, 3]
        current_yaw = yaw_from_transform(navigation.pv_from_base)
        safety_radius = self.robot_safety_radius + self.boundary_margin
        candidates = []

        for side, xiaolan_from_target in (
            self.xiaolan_from_approach.items()
        ):
            target_pose = (
                navigation.pv_from_xiaolan @ xiaolan_from_target
            )
            target_xy = target_pose[:2, 3]
            feasible, minimum_clearance = circular_path_feasible(
                current_xy,
                target_xy,
                navigation.pv_boundary,
                safety_radius,
                self.path_sample_spacing,
            )
            if not feasible:
                continue

            travel_distance = np.linalg.norm(target_xy - current_xy)
            yaw_change = abs(
                wrap_angle(
                    yaw_from_transform(target_pose) - current_yaw
                )
            )
            # 把转角按机器人安全半径折算为等效弧长，同时奖励更大边界余量。
            score = (
                travel_distance
                + self.robot_safety_radius * yaw_change
                - 0.25 * minimum_clearance
            )
            candidates.append(
                (
                    score,
                    side,
                    target_pose,
                    minimum_clearance,
                )
            )

        if not candidates:
            return self._fail_approach(
                "no approach side has a boundary-safe straight path"
            )

        score, side, target_pose, clearance = min(
            candidates,
            key=lambda candidate: candidate[0],
        )
        self.approach_plan = ApproachPlan(
            active=True,
            state="align",
            target_side=side,
            target_pose_pv=target_pose,
            score=float(score),
            minimum_clearance=float(clearance),
        )
        return self.approach_plan

    def cancel_autonomous_approach(self, reason="cancelled"):
        """取消自动接近，并让当前摆动组三腿正常落地后停止。"""

        self.approach_plan.active = False
        self.approach_plan.failed = False
        self.approach_plan.state = "cancelled"
        self.approach_plan.reason = reason

    def _fail_approach(self, reason):
        """记录不可执行状态；上层据此保持站立或进入SAFE。"""

        self.approach_plan = ApproachPlan(
            state="failed",
            failed=True,
            reason=reason,
        )
        return self.approach_plan

    def _autonomous_command(self, navigation_state):
        """把固定接近路径的当前阶段转换为base_link速度指令。"""

        navigation = navigation_state.normalized()
        self.navigation_state = navigation
        command = np.zeros(4, dtype=np.float64)

        if not navigation.valid:
            self._fail_approach("navigation data became invalid")
            return command
        if not navigation.landing_confirmed:
            self._fail_approach("landing confirmation was lost")
            return command
        if not self.approach_plan.active:
            return command

        target_pose = self.approach_plan.target_pose_pv
        current_xy = navigation.pv_from_base[:2, 3]
        target_xy = target_pose[:2, 3]
        current_yaw = yaw_from_transform(navigation.pv_from_base)
        target_yaw = yaw_from_transform(target_pose)
        yaw_error = wrap_angle(target_yaw - current_yaw)
        position_error_pv = target_xy - current_xy
        position_error = np.linalg.norm(position_error_pv)

        feasible, minimum_clearance = circular_path_feasible(
            current_xy,
            target_xy,
            navigation.pv_boundary,
            self.robot_safety_radius + self.boundary_margin,
            self.path_sample_spacing,
        )
        self.approach_plan.minimum_clearance = minimum_clearance
        if not feasible:
            self._fail_approach("remaining approach path became unsafe")
            return command

        if self.approach_plan.state == "align":
            if abs(yaw_error) > self.yaw_tolerance:
                command[3] = np.clip(
                    1.5 * yaw_error,
                    -self.auto_yaw_rate,
                    self.auto_yaw_rate,
                )
                return command
            self.approach_plan.state = "translate"

        if self.approach_plan.state == "translate":
            # 偏航漂移明显时先停下重新对齐，避免边走边扭影响相机稳定。
            if abs(yaw_error) > 2.0 * self.yaw_tolerance:
                self.approach_plan.state = "align"
                return command

            if position_error > self.position_tolerance:
                speed = min(
                    self.auto_linear_speed,
                    1.5 * position_error,
                )
                velocity_pv = (
                    speed * position_error_pv / position_error
                )
                # pv速度转到当前base_link；+x右、+y前。
                cosine = np.cos(current_yaw)
                sine = np.sin(current_yaw)
                command[:2] = np.array(
                    [[cosine, sine], [-sine, cosine]]
                ) @ velocity_pv
                return command
            self.approach_plan.state = "settle"

        if self.approach_plan.state == "settle":
            # 零指令会让当前摆动组落地；六足停稳后才允许切换CLIMB。
            if not self.gait_started and not self.transfer_active:
                # 半周期冻结会产生少量停车距离，停稳后重新测量并修正，
                # 不能只凭发出零指令前的位置宣布到达。
                if abs(yaw_error) > self.yaw_tolerance:
                    self.approach_plan.state = "align"
                elif position_error > self.position_tolerance:
                    self.approach_plan.state = "translate"
                else:
                    self.approach_plan.active = False
                    self.approach_plan.state = "ready"
                    self.approach_plan.ready_for_climb = True

        return command

    def _commit_candidate(self, candidate_base):
        """提交足端目标；自动接近时额外检查六个落脚点的光伏板边界。"""

        navigation_guard_active = (
            self.approach_plan.active
            or self.approach_plan.failed
        )
        if navigation_guard_active and self.navigation_state is not None:
            if len(self.navigation_state.pv_boundary) < 3:
                self._fail_approach("pv boundary became unavailable")
                return False
            feet_pv = transform_points(
                self.navigation_state.pv_from_base,
                candidate_base,
            )
            inside = points_in_polygon(
                feet_pv[:, :2],
                self.navigation_state.pv_boundary,
            )
            clearance = distance_to_polygon_boundary(
                feet_pv[:, :2],
                self.navigation_state.pv_boundary,
            )
            if not (
                inside.all()
                and np.all(clearance >= self.boundary_margin)
            ):
                self._fail_approach(
                    "planned foot position reaches pv boundary margin"
                )
                return False

        self.controller._commit_workspace_candidate(candidate_base)
        return True

    @staticmethod
    def _smooth_step(phase):
        return 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5

    @staticmethod
    def _quintic_segment(
        start,
        target,
        start_velocity,
        target_velocity,
        duration,
        phase,
    ):
        """五次轨迹：两端位置、速度连续，两端加速度为零。"""
        start_scaled_velocity = start_velocity * duration
        velocity_change = (
            target_velocity - start_velocity
        ) * duration
        residual = target - start - start_scaled_velocity

        coefficient_3 = 10.0 * residual - 4.0 * velocity_change
        coefficient_4 = -15.0 * residual + 7.0 * velocity_change
        coefficient_5 = 6.0 * residual - 3.0 * velocity_change

        position = (
            start
            + start_scaled_velocity * phase
            + coefficient_3 * phase**3
            + coefficient_4 * phase**4
            + coefficient_5 * phase**5
        )
        velocity = (
            start_scaled_velocity
            + 3.0 * coefficient_3 * phase**2
            + 4.0 * coefficient_4 * phase**3
            + 5.0 * coefficient_5 * phase**4
        ) / duration
        return position, velocity

    @staticmethod
    def _stance_velocity(foot_xy, command):
        """地面固定足端在base_link中的刚体相对速度。"""
        velocity_right = command[0]
        velocity_forward = command[1]
        yaw_rate = command[3]

        velocity = np.empty_like(foot_xy)
        velocity[:, 0] = (
            -velocity_right + yaw_rate * foot_xy[:, 1]
        )
        velocity[:, 1] = (
            -velocity_forward - yaw_rate * foot_xy[:, 0]
        )
        return velocity

    def reset(self):
        """回正期间清空三角步态状态，完成后从A组重新起步。"""
        self.stance_group_index = 0
        self.gaits[:] = False
        self.gaits[TRIPOD_A_INDICES] = True

        self.phase_time = 0.0
        self.transfer_time = 0.0
        self.transfer_active = False
        self.phase_command[:] = 0.0
        self.stop_requested = False
        self.gait_started = False
        self.first_step = True
        self.foot_velocity_xy[:] = 0.0
        self.swing_start_velocity_xy[:] = 0.0
        self.swing_target_velocity_xy[:] = 0.0
        self.approach_plan = ApproachPlan()
        self.navigation_state = None

    def finish_reset(self):
        """标准站姿恢复完成后同步模式内部的足端基准。"""
        self.body_height_offset = 0.0
        self.swing_start_base[:] = self.controller.foot_init_base
        self.swing_target_base[:] = self.controller.foot_init_base

    def _begin_step(self, command):
        """开启一个新的三角步态阶段，command=[vx,vy,vz,wz]。"""
        controller = self.controller
        self.phase_time = 0.0
        self.phase_command = np.asarray(
            command,
            dtype=np.float64,
        ).reshape(4).copy()

        swing_indices = np.where(~self.gaits)[0]
        self.swing_start_base = controller.foot_desired_base.copy()
        self.swing_target_base = controller.foot_init_base.copy()
        self.swing_start_velocity_xy = self.foot_velocity_xy.copy()

        home_xy = controller.foot_init_base[swing_indices, :2]
        body_velocity_at_home = -self._stance_velocity(
            home_xy,
            self.phase_command,
        )

        support_duration = (
            self.phase_duration + 2.0 * self.transfer_duration
        )
        self.swing_target_base[swing_indices, :2] += (
            0.5 * support_duration * body_velocity_at_home
        )
        self.swing_target_base[swing_indices, 2] = (
            controller.foot_init_base[swing_indices, 2]
            - self.body_height_offset
        )

        # 越界落点连续压到安全边界，不退回固定工作中心。
        target_feasible = controller._workspace_feasible(
            self.swing_target_base
        )
        if not target_feasible[swing_indices].all():
            projected_target = controller._project_workspace(
                self.swing_target_base
            )
            self.swing_target_base[swing_indices] = projected_target[
                swing_indices
            ]

        # 落地速度衔接下一段支撑速度，减少触地水平擦动。
        self.swing_target_velocity_xy[swing_indices] = (
            self._stance_velocity(
                self.swing_target_base[swing_indices, :2],
                self.phase_command,
            )
        )

    def update(self, command, navigation_state=None):
        """根据手柄或自动接近指令更新六足目标，并返回接近任务状态。"""

        if self.approach_plan.failed:
            command = np.zeros(4, dtype=np.float64)
        elif self.approach_plan.active:
            if navigation_state is None:
                command = np.zeros(4, dtype=np.float64)
                self._fail_approach(
                    "navigation state was not supplied"
                )
            else:
                command = self._autonomous_command(navigation_state)

        self._update_gait(command)
        return self.approach_plan

    def _update_gait(self, command):
        """把本周期速度指令转换为base_link中的六足目标位置。"""

        controller = self.controller
        command = np.asarray(command, dtype=np.float64).reshape(4)

        # 回正由控制器中的关节插值接管。
        if controller.reset_active:
            return

        velocity_right = command[0]
        velocity_forward = command[1]
        velocity_up = command[2]
        yaw_rate = command[3]

        self.body_height_offset += velocity_up * self.dt
        self.body_height_offset = np.clip(
            self.body_height_offset,
            self.body_height_offset_min,
            self.body_height_offset_max,
        )

        ground_z = (
            controller.foot_init_base[:, 2]
            - self.body_height_offset
        )
        candidate_base = controller.foot_desired_base.copy()

        # 单独的vz只调节机身高度，不触发三角换相。
        planar_command = np.array(
            [velocity_right, velocity_forward, yaw_rate],
            dtype=np.float64,
        )
        if not self.gait_started:
            candidate_base[:, 2] = ground_z
            self._commit_candidate(candidate_base)
            self.foot_velocity_xy[:] = 0.0

            if np.linalg.norm(planar_command) < 1e-8:
                return

            self.gait_started = True
            self.stop_requested = False
            self._begin_step(command)
        else:
            # 零指令时先让当前摆动组落地，再停止。
            self.stop_requested = (
                np.linalg.norm(planar_command) < 1e-8
            )

        if self.transfer_active:
            transfer_dt = min(
                self.dt,
                self.transfer_duration - self.transfer_time,
            )
            stance_velocity = self._stance_velocity(
                candidate_base[:, :2],
                self.phase_command,
            )
            candidate_base[:, :2] += stance_velocity * transfer_dt
            candidate_base[:, 2] = ground_z
            self.foot_velocity_xy[:] = stance_velocity

            self.transfer_time += transfer_dt
            self._commit_candidate(candidate_base)

            transfer_finished = (
                self.transfer_time
                >= self.transfer_duration - 1e-12
            )
            if not transfer_finished:
                return

            self.transfer_active = False
            self.transfer_time = 0.0
            self.stance_group_index = 1 - self.stance_group_index
            self.gaits[:] = False
            if self.stance_group_index == 0:
                self.gaits[TRIPOD_A_INDICES] = True
            else:
                self.gaits[TRIPOD_B_INDICES] = True
            self.first_step = False

            if self.stop_requested:
                self.gait_started = False
                self.phase_time = 0.0
                self.foot_velocity_xy[:] = 0.0
                return

            self._begin_step(command)
            return

        phase_dt = min(
            self.dt,
            self.phase_duration - self.phase_time,
        )
        stance_indices = np.where(self.gaits)[0]
        swing_indices = np.where(~self.gaits)[0]

        # 一个phase内冻结平面指令，换相后再采用新方向。
        stance_velocity = self._stance_velocity(
            candidate_base[stance_indices, :2],
            self.phase_command,
        )
        if self.first_step:
            stance_velocity *= 0.5

        candidate_base[stance_indices, :2] += (
            stance_velocity * phase_dt
        )
        candidate_base[stance_indices, 2] = ground_z[stance_indices]
        self.foot_velocity_xy[stance_indices] = stance_velocity

        self.phase_time += phase_dt
        phase = np.clip(
            self.phase_time / self.phase_duration,
            0.0,
            1.0,
        )
        swing_position, swing_velocity = self._quintic_segment(
            self.swing_start_base[swing_indices, :2],
            self.swing_target_base[swing_indices, :2],
            self.swing_start_velocity_xy[swing_indices],
            self.swing_target_velocity_xy[swing_indices],
            self.phase_duration,
            phase,
        )
        candidate_base[swing_indices, :2] = swing_position
        self.foot_velocity_xy[swing_indices] = swing_velocity

        if phase < 0.5:
            lift_height = (
                self.step_height * self._smooth_step(2.0 * phase)
            )
        else:
            lift_height = (
                self.step_height
                * self._smooth_step(2.0 * (1.0 - phase))
            )
        candidate_base[swing_indices, 2] = (
            ground_z[swing_indices] + lift_height
        )

        phase_finished = (
            self.phase_time >= self.phase_duration - 1e-12
        )
        if phase_finished:
            self.swing_target_base[swing_indices, 2] = ground_z[
                swing_indices
            ]
            candidate_base[swing_indices] = self.swing_target_base[
                swing_indices
            ]
            self.foot_velocity_xy[swing_indices] = (
                self.swing_target_velocity_xy[swing_indices]
            )

        self._commit_candidate(candidate_base)

        if phase_finished:
            self.transfer_active = True
            self.transfer_time = 0.0
