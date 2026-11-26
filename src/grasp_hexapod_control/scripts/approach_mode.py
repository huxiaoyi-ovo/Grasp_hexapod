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

from kinematics import JOINT_LOWER, JOINT_UPPER
from utils import (
    NavigationState,
    distance_to_polygon_boundary,
    points_in_polygon,
    polygon_area,
    transform_points,
    sample_segment,
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
    waypoints_pv: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float64)
    )
    waypoint_index: int = 0
    minimum_clearance: float = 0.0
    ready_for_climb: bool = False
    failed: bool = False
    reason: str = ""


class ApproachMode:
    """接近模式：接近点决策、固定路径、平地步态和机身升降。"""

    def __init__(self, controller):
        self.controller = controller
        self.dt = controller.dt

        # 左右入口都必须是相对xiaolan_frame的外部标定量。
        self.xiaolan_from_fixed_approaches = {}
        self.xiaolan_keepout_polygon_xy = np.empty((0, 2), dtype=np.float64)
        self.xiaolan_body_clearance = 0.13
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

        # 摆动腿走关节空间轨迹，步态周期放慢以保证关节速度不超过 3 rad/s。
        # 步距 = v*(T+2τ)：0.14 m/s 约 8.4 cm，落点由工作空间投影兜底。
        self.phase_duration = 0.50
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

        # 摆动全程足端相对支撑地面的最高净空（按 21 个采样点的峰值标定）。
        self.step_height = 0.040
        # 摆动两端关节速度 = alpha × 支撑腿刚体速度对应的关节速度。
        self.swing_boundary_velocity_ratio = 0.5
        # 从静止起步时第一组支撑腿从 home 出发，斜坡时间约为 T+τ 才能让首程
        # 约为半个步距，留在工作空间内。
        self.max_linear_acceleration = 0.25
        self.max_yaw_acceleration = 1.2
        # 合并平移和偏航后，限制标准足端的平面速度预算。
        # 这是实际限速瓶颈：navigation_linear_speed 配 0.20 也被这里砍回限幅值。
        self.max_foot_planar_speed = 0.14
        self.nominal_foot_radius = float(np.mean(
            np.linalg.norm(controller.foot_init_base[:, :2], axis=1)
        ))
        self.requested_command = np.zeros(4, dtype=np.float64)
        # 最近一帧经足端速度预算缩放后的原始指令；落点前瞻按它保持不变外推。
        self.slew_target_command = np.zeros(4, dtype=np.float64)
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

        # 摆动腿关节空间计划（beginStep 规划一次，每帧求值）：
        # 基础五次 Hermite 曲线端点/端速度 + 仅膝关节的抬腿鼓包。
        self.swing_q_start = controller.q_init.copy()
        self.swing_q_end = controller.q_init.copy()
        self.swing_qd_start = np.zeros((6, 3), dtype=np.float64)
        self.swing_qd_end = np.zeros((6, 3), dtype=np.float64)
        # 带符号的膝关节抬腿幅值 s*A（rad）；饱和标志仅供离线验证读取。
        self.swing_lift_amplitude = np.zeros(6, dtype=np.float64)
        self.swing_lift_saturated = False
        # 摆动腿的抬腿幅值受关节速度限制（rad/s），分析峰值取 41 个均匀采样点。
        self.swing_joint_speed_limit = 3.0
        # 离线诊断：基础路径本身已超速、速度限幅无解的腿步数。
        self.speed_governor_infeasible = 0
        # 步行期间六条腿全部走关节覆盖：摆动腿取调度，其余腿取已提交足端的
        # 精确 IK。last_override_q 是上一帧成功提交的六腿关节目标；
        # commit 或 IK 失败时重复发出，保持原目标。
        self.last_override_q = controller.q_init.copy()
        self.last_override_valid = False
        self.swing_override_emitted = np.zeros(6, dtype=bool)

    def configure_fixed_approach(
        self,
        xiaolan_from_base,
        target_side="left",
        boundary_margin=0.03,
        linear_speed=0.12,
        yaw_rate=0.8,
        xiaolan_keepout_polygon_xy_m=(),
        xiaolan_body_clearance_m=0.13,
    ):
        """增加一个固定左右接近位姿和共用导航参数。

        xiaolan_from_base表示目标base_link在xiaolan_base中的4×4位姿。
        仿真与实机均使用该接口；实机来源必须先通过外部标定配置门。
        """

        if target_side not in ("left", "right"):
            raise ValueError("fixed approach target_side must be left or right")
        target_transform = np.asarray(
            xiaolan_from_base,
            dtype=np.float64,
        ).reshape(4, 4)
        if (
            not np.isfinite(target_transform).all()
            or not np.allclose(target_transform[3], [0.0, 0.0, 0.0, 1.0])
            or not np.allclose(
                target_transform[:3, :3].T @ target_transform[:3, :3],
                np.eye(3),
                atol=1e-6,
            )
            or not np.isclose(
                np.linalg.det(target_transform[:3, :3]), 1.0, atol=1e-6
            )
        ):
            raise ValueError("fixed approach pose must be a finite rigid transform")
        self.xiaolan_from_fixed_approaches[target_side] = target_transform.copy()
        self.boundary_margin = float(boundary_margin)
        self.auto_linear_speed = float(linear_speed)
        self.auto_yaw_rate = float(yaw_rate)
        self.xiaolan_keepout_polygon_xy = np.asarray(
            xiaolan_keepout_polygon_xy_m,
            dtype=np.float64,
        ).reshape(-1, 2).copy()
        if (
            len(self.xiaolan_keepout_polygon_xy) not in (0,)
            and len(self.xiaolan_keepout_polygon_xy) < 3
        ):
            raise ValueError("xiaolan keepout polygon must contain at least 3 points")
        if not np.isfinite(self.xiaolan_keepout_polygon_xy).all():
            raise ValueError("xiaolan keepout polygon must be finite")
        if (
            len(self.xiaolan_keepout_polygon_xy) >= 3
            and abs(polygon_area(self.xiaolan_keepout_polygon_xy)) <= 1e-9
        ):
            raise ValueError("xiaolan keepout polygon must enclose area")
        self.xiaolan_body_clearance = float(xiaolan_body_clearance_m)
        if min(self.boundary_margin, self.xiaolan_body_clearance) <= 0.0:
            raise ValueError("approach safety margins must be positive")

    def plan_autonomous_approach(self, navigation_state, target_side=None):
        """在左右入口中选择可直达者；不搜索绕障路线。"""

        navigation = navigation_state.normalized()
        self.navigation_state = navigation

        if not navigation.valid:
            return self._fail_approach("navigation data is invalid")
        if not navigation.landing_confirmed:
            return self._fail_approach("landing is not confirmed")
        if len(navigation.pv_boundary) < 3:
            return self._fail_approach("pv boundary is unavailable")
        if not self.xiaolan_from_fixed_approaches:
            return self._fail_approach(
                "fixed approach poses are not configured"
            )
        if target_side is not None and target_side not in ("left", "right"):
            return self._fail_approach("requested approach side is invalid")
        if (
            target_side is not None
            and target_side not in self.xiaolan_from_fixed_approaches
        ):
            return self._fail_approach(
                f"fixed {target_side} approach pose is not configured"
            )

        current_xy = navigation.pv_from_base[:2, 3]
        current_yaw = yaw_from_transform(navigation.pv_from_base)
        if len(self.xiaolan_keepout_polygon_xy) < 3:
            return self._fail_approach(
                "xiaolan keepout polygon is not configured"
            )
        obstacle_polygon_pv = transform_points(
            navigation.pv_from_xiaolan,
            self.xiaolan_keepout_polygon_xy,
        )
        candidate_sides = (
            [target_side]
            if target_side is not None
            else sorted(self.xiaolan_from_fixed_approaches)
        )
        candidates = []
        for side in candidate_sides:
            target_pose = (
                navigation.pv_from_xiaolan
                @ self.xiaolan_from_fixed_approaches[side]
            )
            target_xy = target_pose[:2, 3]
            target_yaw = yaw_from_transform(target_pose)
            feasible, clearance = self._remaining_route_feasible(
                navigation,
                current_xy,
                current_yaw,
                target_xy.reshape(1, 2),
                target_yaw,
            )
            if not feasible or not self._xiaolan_direct_path_feasible(
                current_xy,
                target_xy,
                obstacle_polygon_pv,
            ):
                continue
            distance = float(np.linalg.norm(target_xy - current_xy))
            yaw_change = float(abs(wrap_angle(target_yaw - current_yaw)))
            candidates.append(
                (distance, yaw_change, side, target_pose, clearance)
            )
        if not candidates:
            requested = "left/right" if target_side is None else target_side
            return self._fail_approach(
                f"no direct {requested} approach satisfies panel/Xiaolan constraints"
            )
        _, _, selected_side, target_pose, clearance = min(
            candidates,
            key=lambda item: (item[0], item[1], item[2]),
        )
        waypoint_array = target_pose[:2, 3].reshape(1, 2)

        self.approach_plan = ApproachPlan(
            active=True,
            state="align",
            target_side=selected_side,
            target_pose_pv=target_pose,
            waypoints_pv=waypoint_array,
            minimum_clearance=float(clearance),
        )
        return self.approach_plan

    def _xiaolan_direct_path_feasible(self, start_xy, target_xy, obstacle):
        """机身中心直线不得进入小蓝轮廓及其机身安全余量。"""

        samples = sample_segment(
            start_xy,
            target_xy,
            self.path_sample_spacing,
        )
        outside = ~points_in_polygon(samples, obstacle)
        clearance = distance_to_polygon_boundary(samples, obstacle)
        return bool(
            outside.all()
            and np.all(clearance >= self.xiaolan_body_clearance)
        )

    def _gait_motion_margin(self):
        """返回一次三角步态周期内标准足端的保守平面运动余量。"""

        support_duration = (
            self.phase_duration + 2.0 * self.transfer_duration
        )
        return 0.5 * support_duration * max(
            self.auto_linear_speed,
            self.robot_safety_radius * self.auto_yaw_rate,
        )

    def _pose_feasible(self, center_xy, yaw, boundary):
        """检查一个标准足印和机身中心的光伏板边界余量。"""

        cosine, sine = np.cos(yaw), np.sin(yaw)
        rotation = np.array(
            [[cosine, -sine], [sine, cosine]],
            dtype=np.float64,
        )
        feet_xy = (
            self.controller.foot_init_base[:, :2] @ rotation.T
            + np.asarray(center_xy, dtype=np.float64).reshape(2)
        )
        checked_points = np.vstack(
            (np.asarray(center_xy, dtype=np.float64).reshape(1, 2), feet_xy)
        )
        clearance = distance_to_polygon_boundary(checked_points, boundary)
        required_margin = self.boundary_margin + self._gait_motion_margin()
        feasible = bool(
            points_in_polygon(checked_points, boundary).all()
            and np.all(clearance >= required_margin)
        )
        return feasible, float(clearance.min())

    def _rotation_feasible(self, center_xy, start_yaw, target_yaw, boundary):
        """按最大5度采样检查原地转向的标准足印。"""

        yaw_change = wrap_angle(target_yaw - start_yaw)
        count = max(2, int(np.ceil(abs(yaw_change) / np.deg2rad(5.0))) + 1)
        minimum_clearance = np.inf
        for yaw in np.linspace(start_yaw, start_yaw + yaw_change, count):
            feasible, clearance = self._pose_feasible(center_xy, yaw, boundary)
            minimum_clearance = min(minimum_clearance, clearance)
            if not feasible:
                return False, float(minimum_clearance)
        return True, float(minimum_clearance)

    def _translation_feasible(self, start_xy, end_xy, yaw, boundary):
        """按最大0.02m采样检查固定偏航直线路段的标准足印。"""

        start_xy = np.asarray(start_xy, dtype=np.float64).reshape(2)
        end_xy = np.asarray(end_xy, dtype=np.float64).reshape(2)
        count = max(
            2,
            int(
                np.ceil(
                    np.linalg.norm(end_xy - start_xy)
                    / self.path_sample_spacing
                )
            )
            + 1,
        )
        minimum_clearance = np.inf
        for center_xy in np.linspace(start_xy, end_xy, count):
            feasible, clearance = self._pose_feasible(center_xy, yaw, boundary)
            minimum_clearance = min(minimum_clearance, clearance)
            if not feasible:
                return False, float(minimum_clearance)
        return True, float(minimum_clearance)

    def _remaining_route_feasible(
        self,
        navigation,
        current_xy,
        current_yaw,
        waypoints_pv,
        target_yaw,
    ):
        """复核当前转向及所有剩余固定路线的标准足印。"""

        boundary = navigation.pv_boundary
        rotation_ok, minimum_clearance = self._rotation_feasible(
            current_xy,
            current_yaw,
            target_yaw,
            boundary,
        )
        if not rotation_ok:
            return False, minimum_clearance

        previous_xy = np.asarray(current_xy, dtype=np.float64).reshape(2)
        for waypoint_xy in np.asarray(
            waypoints_pv,
            dtype=np.float64,
        ).reshape(-1, 2):
            feasible, clearance = self._translation_feasible(
                previous_xy,
                waypoint_xy,
                target_yaw,
                boundary,
            )
            minimum_clearance = min(minimum_clearance, clearance)
            if not feasible:
                return False, float(minimum_clearance)
            previous_xy = waypoint_xy
        return True, float(minimum_clearance)

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
        current_yaw = yaw_from_transform(navigation.pv_from_base)
        target_yaw = yaw_from_transform(target_pose)
        yaw_error = wrap_angle(target_yaw - current_yaw)
        waypoints = self.approach_plan.waypoints_pv[
            self.approach_plan.waypoint_index :
        ]
        feasible, minimum_clearance = self._remaining_route_feasible(
            navigation,
            current_xy,
            current_yaw,
            waypoints,
            target_yaw,
        )
        self.approach_plan.minimum_clearance = minimum_clearance
        if not feasible:
            self._fail_approach("remaining approach path became unsafe")
            return command
        if not self._xiaolan_direct_path_feasible(
            current_xy,
            target_pose[:2, 3],
            transform_points(
                navigation.pv_from_xiaolan,
                self.xiaolan_keepout_polygon_xy,
            ),
        ):
            self._fail_approach(
                "remaining direct route reaches Xiaolan safety envelope"
            )
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
            if (
                self.approach_plan.waypoint_index
                >= len(self.approach_plan.waypoints_pv)
            ):
                self.approach_plan.state = "settle"
                return command
            # 偏航漂移明显时先停下重新对齐，避免边走边扭影响相机稳定。
            if abs(yaw_error) > 2.0 * self.yaw_tolerance:
                self.approach_plan.state = "align"
                return command

            target_xy = self.approach_plan.waypoints_pv[
                self.approach_plan.waypoint_index
            ]
            position_error_pv = target_xy - current_xy
            position_error = np.linalg.norm(position_error_pv)
            if position_error > self.position_tolerance:
                # 比例项 2.5：满速保持到误差 8cm 以内才减速，缩短末段蠕行；过冲风险换来的提速，位置容差只有 3cm，不宜再放大。
                speed = min(
                    self.auto_linear_speed,
                    2.5 * position_error,
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
                elif (
                    self.approach_plan.waypoint_index
                    < len(self.approach_plan.waypoints_pv)
                    and np.linalg.norm(
                        self.approach_plan.waypoints_pv[
                            self.approach_plan.waypoint_index
                        ]
                        - current_xy
                    )
                    > self.position_tolerance
                ):
                    self.approach_plan.state = "translate"
                elif (
                    self.approach_plan.waypoint_index
                    < len(self.approach_plan.waypoints_pv)
                ):
                    self.approach_plan.waypoint_index += 1
                    self.approach_plan.state = "align"
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

        return self.controller._commit_workspace_candidate(candidate_base)

    # ------------------------------------------------------------------
    # 摆动腿关节空间轨迹
    # ------------------------------------------------------------------
    def _base_point_to_hip(self, leg, point_base):
        """单腿足端点从 base_link 转到该腿髋坐标系。"""
        feet = np.tile(np.asarray(point_base, dtype=np.float64), (6, 1))
        return self.controller.kinematic.base_to_hip(feet)[leg]

    def _base_vector_to_hip(self, leg, point_base, vector_base):
        """base_link 中位于 point_base 的速度向量，旋转到该腿髋坐标系。"""
        point_base = np.asarray(point_base, dtype=np.float64)
        return self._base_point_to_hip(
            leg, point_base + vector_base
        ) - self._base_point_to_hip(leg, point_base)

    def _leg_ik(self, leg, target_hip, q_seed):
        """单腿数值逆解：雅可比 Newton 迭代，返回 (关节角, 是否收敛)。"""
        kinematic = self.controller.kinematic
        q = np.array(q_seed, dtype=np.float64)
        identity = np.eye(3)
        for _ in range(50):
            error = target_hip - kinematic.forward_leg(leg, q)
            if np.linalg.norm(error) < 1e-10:
                break
            jacobian = kinematic.jacobian_leg(leg, q)
            q = q + np.linalg.solve(
                jacobian.T @ jacobian + 1e-9 * identity,
                jacobian.T @ error,
            )
        error = target_hip - kinematic.forward_leg(leg, q)
        return q, bool(np.linalg.norm(error) <= 1e-6)

    @staticmethod
    def _hermite_quintic(q0, q1, qd0, qd1, duration, u):
        """关节五次 Hermite：两端位置、速度给定，两端加速度为零。返回 (q, dq/dt)。"""
        scaled_velocity = qd0 * duration
        residual = q1 - q0 - scaled_velocity
        velocity_change = (qd1 - qd0) * duration
        c3 = 10.0 * residual - 4.0 * velocity_change
        c4 = -15.0 * residual + 7.0 * velocity_change
        c5 = 6.0 * residual - 3.0 * velocity_change
        position = (
            q0 + scaled_velocity * u + c3 * u**3 + c4 * u**4 + c5 * u**5
        )
        velocity = (
            scaled_velocity
            + 3.0 * c3 * u**2
            + 4.0 * c4 * u**3
            + 5.0 * c5 * u**4
        ) / duration
        return position, velocity

    def _swing_joint_state(self, leg, u):
        """第 leg 腿摆动相位 u 的关节角和关节速度（膝关节叠加抬腿鼓包）。"""
        q, dq = self._hermite_quintic(
            self.swing_q_start[leg],
            self.swing_q_end[leg],
            self.swing_qd_start[leg],
            self.swing_qd_end[leg],
            self.phase_duration,
            u,
        )
        # b(u)=64u^3(1-u)^3：b(0)=b(1)=0，两端速度、加速度为零，b(0.5)=1。
        bump = 64.0 * u**3 * (1.0 - u) ** 3
        bump_rate = (
            192.0 * u**2 * (1.0 - u) ** 2 * (1.0 - 2.0 * u)
            / self.phase_duration
        )
        q = q.copy()
        dq = dq.copy()
        q[1] += self.swing_lift_amplitude[leg] * bump
        dq[1] += self.swing_lift_amplitude[leg] * bump_rate
        return q, dq

    def _plan_swing(self, swing_indices):
        """规划本相摆动腿关节轨迹；任何腿无解或越限返回 False。"""
        controller = self.controller
        kinematic = controller.kinematic
        alpha = self.swing_boundary_velocity_ratio
        duration = self.phase_duration
        self.swing_lift_saturated = False
        ground_check_u = np.linspace(0.0, 1.0, 21)

        for leg in swing_indices:
            start_base = self.swing_start_base[leg]
            target_base = self.swing_target_base[leg]
            start_hip = self._base_point_to_hip(leg, start_base)
            target_hip = self._base_point_to_hip(leg, target_base)
            q_start, start_ok = self._leg_ik(
                leg, start_hip, controller.q_des[leg]
            )
            q_end, end_ok = self._leg_ik(
                leg, target_hip, controller.q_init[leg]
            )
            if not (start_ok and end_ok):
                return False

            start_velocity = np.append(
                self.swing_start_velocity_xy[leg], 0.0
            )
            target_velocity = np.append(
                self.swing_target_velocity_xy[leg], 0.0
            )
            qd_start = alpha * np.linalg.solve(
                kinematic.jacobian_leg(leg, q_start),
                self._base_vector_to_hip(leg, start_base, start_velocity),
            )
            qd_end = alpha * np.linalg.solve(
                kinematic.jacobian_leg(leg, q_end),
                self._base_vector_to_hip(leg, target_base, target_velocity),
            )
            self.swing_q_start[leg] = q_start
            self.swing_q_end[leg] = q_end
            self.swing_qd_start[leg] = qd_start
            self.swing_qd_end[leg] = qd_end

            # 膝关节抬腿：幅值二分到 21 个均匀采样点上足端最高点
            # 高出落点地面 step_height。
            q_base, _ = self._hermite_quintic(
                q_start,
                q_end,
                qd_start,
                qd_end,
                duration,
                ground_check_u[:, np.newaxis],
            )
            bump = 64.0 * ground_check_u**3 * (1.0 - ground_check_u) ** 3
            q_mid, _ = self._hermite_quintic(
                q_start, q_end, qd_start, qd_end, duration, 0.5
            )
            sign = (
                1.0
                if kinematic.jacobian_leg(leg, q_mid)[2, 1] >= 0.0
                else -1.0
            )
            ground_z_hip = target_hip[2]

            def peak_height(amplitude):
                heights = []
                for q_sample, bump_sample in zip(q_base, bump):
                    q_lift = q_sample.copy()
                    q_lift[1] += sign * amplitude * bump_sample
                    heights.append(
                        kinematic.forward_leg(leg, q_lift)[2] - ground_z_hip
                    )
                return max(heights)

            low, high = 0.0, 1.2
            for _ in range(60):
                middle = 0.5 * (low + high)
                if peak_height(middle) < self.step_height:
                    low = middle
                else:
                    high = middle
            amplitude = 0.5 * (low + high)
            if peak_height(1.2) < self.step_height:
                amplitude = 1.2
                self.swing_lift_saturated = True

            # 速度限幅：幅值取 min(高度所需幅值, 该腿 41 个均匀采样点上
            # 三个关节解析峰值速度不超过限值的最大幅值)。
            speed_u = np.linspace(0.0, 1.0, 41)
            _, base_rate = self._hermite_quintic(
                q_start,
                q_end,
                qd_start,
                qd_end,
                duration,
                speed_u[:, np.newaxis],
            )
            bump_rate = (
                192.0 * speed_u**2 * (1.0 - speed_u) ** 2
                * (1.0 - 2.0 * speed_u) / duration
            )

            def peak_speed(lift_amplitude):
                rate = base_rate.copy()
                rate[:, 1] += sign * lift_amplitude * bump_rate
                return float(np.abs(rate).max())

            limit = self.swing_joint_speed_limit
            if peak_speed(0.0) > limit:
                # 基础路径本身已超速，不能靠减小抬腿解决：保持高度所需幅值。
                self.speed_governor_infeasible += 1
            elif peak_speed(1.2) > limit:
                feasible, infeasible = 0.0, 1.2
                for _ in range(60):
                    middle = 0.5 * (feasible + infeasible)
                    if peak_speed(middle) <= limit:
                        feasible = middle
                    else:
                        infeasible = middle
                amplitude = min(amplitude, feasible)
            self.swing_lift_amplitude[leg] = sign * amplitude

            for u in ground_check_u:
                q, _ = self._swing_joint_state(leg, u)
                if np.any(q < JOINT_LOWER[leg]) or np.any(
                    q > JOINT_UPPER[leg]
                ):
                    return False
        return True

    def _clear_override_state(self):
        """清空关节覆盖状态，并撤销本模式发出的关节覆盖（回正前后调用）。"""
        for leg in np.where(self.swing_override_emitted)[0]:
            self.controller.joint_override[leg] = None
        self.swing_override_emitted[:] = False
        self.last_override_q[:] = 0.0
        self.last_override_valid = False

    def _emit_overrides(self, committed, swing_targets=None):
        """写本帧六条腿的关节覆盖。

        committed 为真时：摆动腿取调度关节角，其余腿对已提交足端做 IK
        （以上一帧覆盖为初值）；任一腿 IK 失败，或 committed 为假，
        重复发出上一帧覆盖（没有上一帧时保持 q_des）。
        """
        controller = self.controller
        if committed:
            feet_hip = controller.kinematic.base_to_hip(
                controller.foot_desired_base
            )
            targets = np.empty((6, 3), dtype=np.float64)
            for leg in range(6):
                if swing_targets is not None and leg in swing_targets:
                    targets[leg] = swing_targets[leg]
                    continue
                seed = (
                    self.last_override_q[leg]
                    if self.last_override_valid
                    else controller.q_des[leg]
                )
                targets[leg], converged = self._leg_ik(
                    leg, feet_hip[leg], seed
                )
                if not converged:
                    break
            else:
                self.last_override_q[:] = targets
                self.last_override_valid = True
        emitted = (
            self.last_override_q
            if self.last_override_valid
            else controller.q_des
        )
        for leg in range(6):
            controller.joint_override[leg] = emitted[leg].copy()
        self.swing_override_emitted[:] = True

    def _abort_step(self):
        """摆动规划失败：不起步，全部保持支撑，按停止路径清零指令。"""
        self.gait_started = False
        self.phase_time = 0.0
        self.requested_command[:] = 0.0
        self.foot_velocity_xy[:] = 0.0

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

    def _scaled_command(self, command):
        """按足端平面速度预算缩放指令（平移和偏航按同一比例）。"""
        command = np.asarray(command, dtype=np.float64).reshape(4).copy()
        foot_speed_budget = (
            np.linalg.norm(command[:2])
            + self.nominal_foot_radius * abs(command[3])
        )
        if foot_speed_budget > self.max_foot_planar_speed:
            scale = self.max_foot_planar_speed / foot_speed_budget
            command[:2] *= scale
            command[3] *= scale
        return command

    def _slew_command(self, requested, target):
        """纯函数：requested 按实机可控加速度向 target 推进一帧。"""
        slewed = np.asarray(requested, dtype=np.float64).copy()
        linear_delta = target[:2] - slewed[:2]
        linear_norm = np.linalg.norm(linear_delta)
        linear_step = self.max_linear_acceleration * self.dt
        if linear_norm > linear_step:
            linear_delta *= linear_step / linear_norm
        slewed[:2] += linear_delta

        yaw_delta = np.clip(
            target[3] - slewed[3],
            -self.max_yaw_acceleration * self.dt,
            self.max_yaw_acceleration * self.dt,
        )
        slewed[3] += yaw_delta
        slewed[2] = target[2]
        return slewed

    def _slew_requested_command(self, command):
        """按实机可控加速度追踪平面/偏航指令。"""
        self.slew_target_command = self._scaled_command(command)
        self.requested_command[:] = self._slew_command(
            self.requested_command, self.slew_target_command
        )

    def _predict_command_profile(self, count):
        """纯函数：从当前 requested_command 起，向保持不变的目标指令逐帧外推。

        返回 (count,4)，第 i 行是 i 帧之后的限加速度指令（第 0 行即当前指令）。
        """
        profile = np.empty((count, 4), dtype=np.float64)
        current = self.requested_command.copy()
        for index in range(count):
            profile[index] = current
            current = self._slew_command(current, self.slew_target_command)
        return profile

    def _update_swing_target(self, swing_indices, frame_offset):
        """按前瞻指令序列更新本相落点和落点边界速度。

        frame_offset：本相第一帧相对当前帧的帧数（起步帧为0，换相帧为1）。
        落地腿的支撑期 = 随后的共同支撑 + 下一摆动相 + 再下一次共同支撑。
        """

        controller = self.controller
        target = controller.foot_init_base.copy()
        home_xy = controller.foot_init_base[swing_indices, :2]
        phase_frames = int(np.ceil(self.phase_duration / self.dt - 1e-9))
        first_stance = frame_offset + phase_frames
        stance_count = (
            self.transfer_frames + phase_frames + self.transfer_frames
        )
        profile = self._predict_command_profile(first_stance + stance_count)
        stance_displacement = np.zeros_like(home_xy)
        for index in range(first_stance, first_stance + stance_count):
            stance_displacement += (
                self._stance_velocity(home_xy, profile[index]) * self.dt
            )
        target[swing_indices, :2] -= 0.5 * stance_displacement
        target[swing_indices, 2] = (
            controller.foot_init_base[swing_indices, 2]
            - self.body_height_offset
        )

        target_feasible = controller._workspace_feasible(target)
        if not target_feasible[swing_indices].all():
            target = controller._project_workspace(target)
        self.swing_target_base[swing_indices] = target[swing_indices]
        self.swing_target_velocity_xy[swing_indices] = (
            self._stance_velocity(
                self.swing_target_base[swing_indices, :2],
                profile[first_stance],
            )
        )

    def reset(self):
        """回正期间清空三角步态状态，完成后从A组重新起步。"""
        self.stance_group_index = 0
        self.gaits[:] = False
        self.gaits[TRIPOD_A_INDICES] = True

        self.phase_time = 0.0
        self.transfer_time = 0.0
        self.transfer_active = False
        self.requested_command[:] = 0.0
        self.slew_target_command[:] = 0.0
        self.stop_requested = False
        self.gait_started = False
        self.first_step = True
        self.foot_velocity_xy[:] = 0.0
        self.swing_start_velocity_xy[:] = 0.0
        self.swing_target_velocity_xy[:] = 0.0
        self._clear_override_state()
        self.approach_plan = ApproachPlan()
        self.navigation_state = None

    def finish_reset(self):
        """标准站姿恢复完成后同步模式内部的足端基准。"""
        self._clear_override_state()
        self.body_height_offset = 0.0
        self.swing_start_base[:] = self.controller.foot_init_base
        self.swing_target_base[:] = self.controller.foot_init_base

    def _begin_step(self, frame_offset):
        """开启一个新的三角步态阶段；摆动规划失败返回False且不起步。"""
        controller = self.controller
        self.phase_time = 0.0

        swing_indices = np.where(~self.gaits)[0]
        self.swing_start_base = controller.foot_desired_base.copy()
        self.swing_target_base = controller.foot_init_base.copy()
        self.swing_start_velocity_xy = self.foot_velocity_xy.copy()
        self._update_swing_target(swing_indices, frame_offset)
        if not self._plan_swing(swing_indices):
            return False
        return True

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
        # 控制器每帧开头已清空 joint_override；本帧的发出标记重新计。
        self.swing_override_emitted[:] = False

        # 回正由控制器中的关节插值接管。
        if controller.reset_active:
            return

        # 每帧过滤操作者请求；本相使用的命令只会在下一次换相时锁存。
        self._slew_requested_command(command)

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
            committed = self._commit_candidate(candidate_base)
            self._emit_overrides(committed)
            if not committed:
                return
            self.foot_velocity_xy[:] = 0.0

            if np.linalg.norm(planar_command) < 1e-8:
                return

            self.gait_started = True
            self.stop_requested = False
            if not self._begin_step(0):
                self._abort_step()
                return
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
                self.requested_command,
            )
            candidate_base[:, :2] += stance_velocity * transfer_dt
            candidate_base[:, 2] = ground_z
            velocity_before = self.foot_velocity_xy.copy()
            self.foot_velocity_xy[:] = stance_velocity

            committed = self._commit_candidate(candidate_base)
            self._emit_overrides(committed)
            if not committed:
                self.foot_velocity_xy[:] = velocity_before
                return
            self.transfer_time += transfer_dt

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

            # 零指令且限加速度指令已降到零才停；否则继续迈步，
            # 前瞻落点会让最后几步自然缩短。
            if (
                self.stop_requested
                and np.linalg.norm(self.requested_command[:2]) < 1e-6
                and abs(self.requested_command[3]) < 1e-6
            ):
                self.gait_started = False
                self.phase_time = 0.0
                self.requested_command[:] = 0.0
                self.foot_velocity_xy[:] = 0.0
                return

            if not self._begin_step(1):
                self._abort_step()
                return
            return

        phase_dt = min(
            self.dt,
            self.phase_duration - self.phase_time,
        )
        stance_indices = np.where(self.gaits)[0]
        swing_indices = np.where(~self.gaits)[0]

        stance_velocity = self._stance_velocity(
            candidate_base[stance_indices, :2],
            self.requested_command,
        )
        candidate_base[stance_indices, :2] += (
            stance_velocity * phase_dt
        )
        candidate_base[stance_indices, 2] = ground_z[stance_indices]
        velocity_before = self.foot_velocity_xy.copy()
        self.foot_velocity_xy[stance_indices] = stance_velocity

        next_phase_time = self.phase_time + phase_dt
        phase = np.clip(
            next_phase_time / self.phase_duration,
            0.0,
            1.0,
        )
        phase_finished = (
            next_phase_time >= self.phase_duration - 1e-12
        )
        # 摆动腿走关节空间轨迹：候选足端取规划关节角的 FK，使边界、
        # 工作空间和碰撞检查看到真实的规划足端。
        swing_joint_targets = {}
        swing_hip_feet = np.zeros((6, 3), dtype=np.float64)
        swing_hip_velocity = np.zeros((6, 3), dtype=np.float64)
        for leg in swing_indices:
            if phase_finished:
                swing_joint_targets[leg] = self.swing_q_end[leg].copy()
                continue
            q_leg, dq_leg = self._swing_joint_state(leg, phase)
            swing_joint_targets[leg] = q_leg
            swing_hip_feet[leg] = controller.kinematic.forward_leg(
                leg, q_leg
            )
            swing_hip_velocity[leg] = (
                controller.kinematic.jacobian_leg(leg, q_leg) @ dq_leg
            )
        if not phase_finished:
            swing_feet_base = controller.kinematic.hip_to_base(
                swing_hip_feet
            )
            swing_feet_moved = controller.kinematic.hip_to_base(
                swing_hip_feet + swing_hip_velocity
            )
            for leg in swing_indices:
                candidate_base[leg] = swing_feet_base[leg]
                self.foot_velocity_xy[leg] = (
                    swing_feet_moved[leg, :2] - swing_feet_base[leg, :2]
                )
        else:
            endpoint = self.swing_target_base[swing_indices].copy()
            endpoint[:, 2] = ground_z[swing_indices]
            candidate_base[swing_indices] = endpoint
            self.foot_velocity_xy[swing_indices] = (
                self.swing_target_velocity_xy[swing_indices]
            )

        committed = self._commit_candidate(candidate_base)
        # 控制器每帧清空覆盖：摆动腿取调度，支撑腿取已提交足端的精确 IK；
        # commit 失败时重复上一帧目标以保持足端。
        self._emit_overrides(committed, swing_joint_targets)
        if not committed:
            self.foot_velocity_xy[:] = velocity_before
            return
        self.phase_time = next_phase_time

        if phase_finished:
            self.swing_target_base[swing_indices, 2] = ground_z[
                swing_indices
            ]
            self.transfer_active = True
            self.transfer_time = 0.0
