"""六足总控制器与公共安全执行层。

功能：
    管理APPROACH和CLIMB两种模式，维护公共足端目标，执行工作空间、
    足端/连杆碰撞检查，并用DLS将笛卡尔足端误差转换为关节目标。
输入：
    q_cur.shape=(6,3)，单位rad；当前模式指令或观测数据。
输出：
    q_des.shape=(6,3)，顺序为lb、lf、lm、rb、rf、rm，每腿thigh、knee、ankle。
结构：
    模式目标生成 -> 足端可行性检查 -> 坐标转换 -> DLS -> 关节碰撞保护。
边界：
    不直接调用Isaac Gym、ROS或舵机SDK；仿真和实机共用本控制逻辑。
"""

import numpy as np

from approach_mode import ApproachMode
from climb_mode import ClimbMode
from kinematics import (
    FOOT_RADIUS,
    JOINT_LOWER,
    JOINT_UPPER,
    JOINT_VELOCITY_LIMIT,
    Q_STAND,
    GraspKinematic,
)
from utils import (
    package_config_path,
    transform_points,
    wrap_angle,
    yaw_from_transform,
)


# 当前简化URDF碰撞盒对应的控制器碰撞模型。
LINK_COLLISION_RADII = np.array(
    [
        np.hypot(0.025 / 2.0, 0.025 / 2.0),
        np.hypot(0.022 / 2.0, 0.022 / 2.0),
        np.hypot(0.022 / 2.0, 0.022 / 2.0),
    ],
    dtype=np.float64,
)
COLLISION_MARGIN = 0.003
BODY_COLLISION_RADIUS = 0.097
BODY_COLLISION_Z_MIN = 0.0
BODY_COLLISION_Z_MAX = 0.121
MIN_FOOT_CLEARANCE = 2.0 * FOOT_RADIUS + COLLISION_MARGIN

WORKSPACE_BOUNDARY_PATH = package_config_path("workspace_bounds.csv")
WORKSPACE_BETA_LIMIT = np.deg2rad(30.0)
WORKSPACE_NUMERICAL_TOLERANCE = 1e-9


class MissionStateMachine:
    """调度自动接近、攀爬和对接交接状态。"""

    IDLE, APPROACH, PREPARE_CLIMB, CLIMB, DOCK, FAILED = (
        "IDLE",
        "APPROACH",
        "PREPARE_CLIMB",
        "CLIMB",
        "DOCK",
        "FAILED",
    )

    def __init__(self, controller):
        self.controller = controller
        self.state = self.IDLE
        self.reason = ""
        self.climb_config = None
        self.climb_start_stage_index = 0
        self.climb_end_stage_index = None
        self.approach_elapsed_s = 0.0
        self.approach_timeout_s = 60.0
        self.prepare_elapsed_s = 0.0
        self.prepare_settle_s = 0.0
        self.last_prepare_joint_error_rad = np.inf
        self.last_prepare_world_foot_error_m = np.inf
        self.last_prepare_position_error_m = np.inf
        self.last_prepare_yaw_error_rad = np.inf
        self.approach_target_pose_pv = None
        self.approach_position_tolerance_m = 0.01
        self.approach_yaw_tolerance_rad = np.deg2rad(1.0)
        self.prepare_retry_count = 0
        self.max_prepare_retries = 3
        self.posture_prepared = False
        self.prepare_previous_pose_pv = None
        self.last_prepare_linear_speed_m_s = np.inf
        self.last_prepare_angular_speed_rad_s = np.inf
        self.prepare_linear_speed_limit_m_s = 0.01
        self.prepare_angular_speed_limit_rad_s = np.deg2rad(1.0)
        self.final_position_error_m = np.inf
        self.final_orientation_error_rad = np.inf
        self.final_world_foot_error_m = np.inf
        self.final_position_tolerance_m = 0.03
        self.final_orientation_tolerance_rad = np.deg2rad(5.0)
        self.final_world_foot_tolerance_m = 0.03

    def start(
        self,
        navigation_state,
        climb_config,
        start_stage_index=0,
        end_stage_index=None,
        approach_timeout_s=60.0,
        approach_position_tolerance_m=0.01,
        approach_yaw_tolerance_rad=np.deg2rad(1.0),
    ):
        """规划自动接近并保存后续攀爬任务。"""

        if self.state in (
            self.APPROACH,
            self.PREPARE_CLIMB,
            self.CLIMB,
        ):
            raise ValueError("full mission is already running")
        self.controller.climb_mode._validate_config(climb_config)
        self.controller.set_mode(self.controller.APPROACH)
        self.controller.reset_active = False
        self.climb_config = climb_config
        self.climb_start_stage_index = int(start_stage_index)
        self.climb_end_stage_index = (
            len(climb_config["stages"]) - 1
            if end_stage_index is None
            else int(end_stage_index)
        )
        self.approach_elapsed_s = 0.0
        self.approach_timeout_s = float(approach_timeout_s)
        if self.approach_timeout_s <= 0.0:
            raise ValueError("approach_timeout_s must be positive")
        self.approach_position_tolerance_m = float(
            approach_position_tolerance_m
        )
        self.approach_yaw_tolerance_rad = float(
            approach_yaw_tolerance_rad
        )
        if (
            self.approach_position_tolerance_m <= 0.0
            or self.approach_yaw_tolerance_rad <= 0.0
        ):
            raise ValueError("full mission approach tolerances must be positive")
        self.controller.approach_mode.position_tolerance = (
            self.approach_position_tolerance_m
        )
        self.controller.approach_mode.yaw_tolerance = (
            self.approach_yaw_tolerance_rad
        )
        self.reason = ""
        self.prepare_elapsed_s = 0.0
        self.prepare_settle_s = 0.0
        self.last_prepare_joint_error_rad = np.inf
        self.last_prepare_world_foot_error_m = np.inf
        self.last_prepare_position_error_m = np.inf
        self.last_prepare_yaw_error_rad = np.inf
        self.prepare_retry_count = 0
        self.posture_prepared = False
        self.prepare_previous_pose_pv = None
        self.last_prepare_linear_speed_m_s = np.inf
        self.last_prepare_angular_speed_rad_s = np.inf
        self.final_position_error_m = np.inf
        self.final_orientation_error_rad = np.inf
        self.final_world_foot_error_m = np.inf

        result = self.controller.start_autonomous_approach(
            navigation_state
        )
        if result.failed:
            self.state = self.FAILED
            self.reason = result.reason
        else:
            self.state = self.APPROACH
            self.approach_target_pose_pv = (
                result.target_pose_pv.copy()
            )
        return result

    def cancel(self, reason="cancelled"):
        """取消自动任务并回到空闲调度状态。"""

        if self.state == self.APPROACH:
            self.controller.approach_mode.cancel_autonomous_approach(
                reason
            )
        self.state = self.IDLE
        self.reason = reason
        self.climb_config = None

    def update(self, q_cur, navigation_state):
        """推进当前任务阶段；成功门槛来自各子模式。"""

        command = np.zeros(4, dtype=np.float64)
        if self.state == self.APPROACH:
            self.approach_elapsed_s += self.controller.dt
            result = self.controller.approach_mode.update(
                command,
                navigation_state,
            )
            if result.failed:
                self.state = self.FAILED
                self.reason = result.reason
            elif self.approach_elapsed_s >= self.approach_timeout_s:
                self.controller.approach_mode.cancel_autonomous_approach(
                    "approach timeout"
                )
                self.state = self.FAILED
                self.reason = "approach timeout"
            elif result.ready_for_climb:
                self.approach_target_pose_pv = (
                    result.target_pose_pv.copy()
                )
                self.prepare_elapsed_s = 0.0
                self.prepare_settle_s = 0.0
                if not self.posture_prepared:
                    self.controller.prepare_climb_entry(q_cur)
                    self.posture_prepared = True
                self.prepare_previous_pose_pv = (
                    navigation_state.normalized().pv_from_base
                )
                self.state = self.PREPARE_CLIMB
            return result

        if self.state == self.PREPARE_CLIMB:
            self.prepare_elapsed_s += self.controller.dt
            self.last_prepare_joint_error_rad = float(
                np.max(np.abs(np.asarray(q_cur) - self.controller.q_init))
            )
            gate = self.climb_config["settle_gate"]
            if not self.controller.reset_active:
                navigation = navigation_state.normalized()
                if self.prepare_previous_pose_pv is not None:
                    self.last_prepare_linear_speed_m_s = float(
                        np.linalg.norm(
                            navigation.pv_from_base[:3, 3]
                            - self.prepare_previous_pose_pv[:3, 3]
                        )
                        / self.controller.dt
                    )
                    relative_rotation = (
                        self.prepare_previous_pose_pv[:3, :3].T
                        @ navigation.pv_from_base[:3, :3]
                    )
                    rotation_cosine = np.clip(
                        (np.trace(relative_rotation) - 1.0) / 2.0,
                        -1.0,
                        1.0,
                    )
                    self.last_prepare_angular_speed_rad_s = float(
                        np.arccos(rotation_cosine) / self.controller.dt
                    )
                self.prepare_previous_pose_pv = (
                    navigation.pv_from_base.copy()
                )
                position_error = float(
                    np.linalg.norm(
                        navigation.pv_from_base[:2, 3]
                        - self.approach_target_pose_pv[:2, 3]
                    )
                )
                yaw_error = abs(
                    wrap_angle(
                        yaw_from_transform(self.approach_target_pose_pv)
                        - yaw_from_transform(navigation.pv_from_base)
                    )
                )
                self.last_prepare_position_error_m = position_error
                self.last_prepare_yaw_error_rad = yaw_error
                pose_ready = bool(
                    navigation.valid
                    and position_error
                    <= self.approach_position_tolerance_m
                    and yaw_error <= self.approach_yaw_tolerance_rad
                )
                feet_base = self.controller.kinematic.hip_to_base(
                    self.controller.kinematic.forward(q_cur)
                )
                feet_world = transform_points(
                    navigation.pv_from_base,
                    feet_base,
                )
                target_anchors = np.asarray(
                    self.climb_config["p0"]["anchors_world_m"],
                    dtype=np.float64,
                )
                self.last_prepare_world_foot_error_m = float(
                    np.max(
                        np.linalg.norm(
                            feet_world - target_anchors,
                            axis=1,
                        )
                    )
                )
                if not pose_ready:
                    if self.prepare_retry_count >= self.max_prepare_retries:
                        self.state = self.FAILED
                        self.reason = (
                            "climb entry pose changed after posture preparation: "
                            "position_error_m={:.6g} yaw_error_deg={:.6g} "
                            "world_foot_error_m={:.6g}".format(
                                position_error,
                                np.degrees(yaw_error),
                                self.last_prepare_world_foot_error_m,
                            )
                        )
                        return None
                    self.prepare_retry_count += 1
                    result = self.controller.start_autonomous_approach(
                        navigation
                    )
                    if result.failed:
                        self.state = self.FAILED
                        self.reason = result.reason
                    else:
                        self.state = self.APPROACH
                        self.approach_elapsed_s = 0.0
                        self.approach_target_pose_pv = (
                            result.target_pose_pv.copy()
                        )
                    return result
                if (
                    self.last_prepare_joint_error_rad
                    <= gate["entry_max_joint_error_rad"]
                    and self.last_prepare_world_foot_error_m
                    <= gate["max_foot_target_error_m"]
                    and self.last_prepare_linear_speed_m_s
                    <= self.prepare_linear_speed_limit_m_s
                    and self.last_prepare_angular_speed_rad_s
                    <= self.prepare_angular_speed_limit_rad_s
                ):
                    self.prepare_settle_s += self.controller.dt
                else:
                    self.prepare_settle_s = 0.0
                if self.prepare_settle_s >= gate["persistence_s"]:
                    self.controller.enter_climb(
                        q_cur,
                        self.climb_config,
                        self.climb_start_stage_index,
                        self.climb_end_stage_index,
                    )
                    self.state = self.CLIMB
            if (
                self.state == self.PREPARE_CLIMB
                and self.prepare_elapsed_s
                >= self.controller.reset_duration + 8.0
            ):
                self.state = self.FAILED
                self.reason = (
                    "climb entry posture timeout: joint_error_rad="
                    + "{:.6g}".format(self.last_prepare_joint_error_rad)
                )
            return None

        if self.state == self.CLIMB:
            result = self.controller.climb_mode.update(command, q_cur)
            if self.controller.climb_mode.state == ClimbMode.DONE:
                if self._climb_terminal_ready(q_cur, navigation_state):
                    # 当前仓库尚无可执行的视觉对接器；DOCK先作为终端保持与
                    # 后续DockMode的明确接管点，不能解释为已经完成物理对接。
                    self.controller.set_mode(self.controller.DOCK)
                    self.state = self.DOCK
                else:
                    self.state = self.FAILED
                    self.reason = (
                        "climb terminal observation gate failed: "
                        "position_error_m={:.6g} orientation_error_deg={:.6g} "
                        "world_foot_error_m={:.6g}".format(
                            self.final_position_error_m,
                            np.degrees(self.final_orientation_error_rad),
                            self.final_world_foot_error_m,
                        )
                    )
            elif self.controller.climb_mode.state == ClimbMode.FAILED:
                self.state = self.FAILED
                self.reason = self.controller.climb_mode.failure_reason
            return result

        return None

    def _climb_terminal_ready(self, q_cur, navigation_state):
        """用RTK/IMU位姿与关节FK检查C30几何终态。"""

        if navigation_state is None:
            return False
        navigation = navigation_state.normalized()
        if not navigation.valid:
            return False
        final_stage = self.climb_config["stages"][
            self.climb_end_stage_index
        ]
        target_pose = ClimbMode._world_from_base(
            np.asarray(final_stage["pose_end"], dtype=np.float64)
        )
        actual_pose = navigation.pv_from_base
        self.final_position_error_m = float(
            np.linalg.norm(actual_pose[:3, 3] - target_pose[:3, 3])
        )
        relative_rotation = target_pose[:3, :3].T @ actual_pose[:3, :3]
        rotation_cosine = np.clip(
            (np.trace(relative_rotation) - 1.0) / 2.0,
            -1.0,
            1.0,
        )
        self.final_orientation_error_rad = float(
            np.arccos(rotation_cosine)
        )
        feet_base = self.controller.kinematic.hip_to_base(
            self.controller.kinematic.forward(q_cur)
        )
        feet_world = transform_points(actual_pose, feet_base)
        target_feet_world = np.asarray(
            final_stage["anchor_knots"][-1],
            dtype=np.float64,
        )
        self.final_world_foot_error_m = float(
            np.max(
                np.linalg.norm(
                    feet_world - target_feet_world,
                    axis=1,
                )
            )
        )
        return bool(
            self.final_position_error_m <= self.final_position_tolerance_m
            and self.final_orientation_error_rad
            <= self.final_orientation_tolerance_rad
            and self.final_world_foot_error_m
            <= self.final_world_foot_tolerance_m
        )


class GraspController:
    """六足控制器
    速度指令
        -> 足端步态规划（base_link坐标系）
        -> 转换到各腿hip坐标系
        -> 阻尼雅可比
        -> q_des"""

    APPROACH = "approach"
    CLIMB = "climb"
    DOCK = "dock"

    def __init__(self, dt, enable_link_collision_check=False):
        self.dt = dt
        self.enable_link_collision_check = bool(
            enable_link_collision_check
        )
        self.kinematic = GraspKinematic()
        self.workspace_boundary = np.loadtxt(
            WORKSPACE_BOUNDARY_PATH,
            delimiter=",",
        )

        self.q_init = Q_STAND.copy()  # shape == (6, 3)
        self.q_des = self.q_init.copy()  # shape == (6, 3)

        self.foot_init_hip = self.kinematic.forward(self.q_init)  # shape == (6, 3)
        self.foot_init_base = self.kinematic.hip_to_base(self.foot_init_hip)

        self.foot_current_hip = self.foot_init_hip.copy()  # shape == (6, 3)

        self.foot_desired_base = self.foot_init_base.copy()
        self.foot_desired_base_prev = self.foot_desired_base.copy()  # 前馈差分用
        self.enable_workspace_check = False  # 越界足端直接提交, 不投影
        self.foot_desired_hip = self.foot_init_hip.copy()  # shape == (6, 3)

        self.base_height_at_stand = (
            FOOT_RADIUS - np.mean(self.foot_init_base[:, 2])
        )

        # A使能和B复位都用五次曲线平滑回到标准站姿。
        # 2 s五次曲线让整机从任意当前姿态平滑回到Q_STAND。
        self.reset_duration = 2.0
        self.reset_time = 0.0
        self.reset_active = False
        self.reset_start_q = self.q_init.copy()

        self.last_link_collision_free = np.ones(6, dtype=bool)
        # Observation-only per-update telemetry; it never feeds control.
        self.last_update_velocity_limit_clip_count = 0
        self.last_update_collision_guard_hold_count = 0

        self.approach_mode = ApproachMode(self)
        self.climb_mode = ClimbMode(self)
        self.dock_mode = None
        self.mission = MissionStateMachine(self)
        self.mode = self.APPROACH
        self.last_mode_result = None

        if not self._workspace_feasible(self.foot_init_base).all():
            raise ValueError("Q_STAND is outside the safe workspace")
        if not self._link_collision_free(self.q_init).all():
            raise ValueError("Q_STAND contains a link collision")

    @staticmethod
    def _smooth_step(phase):
        return 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5

    def _workspace_feasible(self, foot_positions_base):
        """判断六个候选足端是否位于离线生成的安全工作空间。"""

        foot_positions_hip = self.kinematic.base_to_hip(
            foot_positions_base
        )
        rho = np.linalg.norm(foot_positions_hip[:, :2], axis=1)
        beta = np.arctan2(
            foot_positions_hip[:, 1],
            foot_positions_hip[:, 0],
        )
        z = foot_positions_hip[:, 2]

        boundary_z = self.workspace_boundary[:, 0]
        rho_min = np.interp(
            z,
            boundary_z,
            self.workspace_boundary[:, 1],
        )
        rho_max = np.interp(
            z,
            boundary_z,
            self.workspace_boundary[:, 2],
        )

        return (
            (z >= boundary_z[0] - WORKSPACE_NUMERICAL_TOLERANCE)
            & (z <= boundary_z[-1] + WORKSPACE_NUMERICAL_TOLERANCE)
            & (rho >= rho_min - WORKSPACE_NUMERICAL_TOLERANCE)
            & (rho <= rho_max + WORKSPACE_NUMERICAL_TOLERANCE)
            & (
                np.abs(beta)
                <= WORKSPACE_BETA_LIMIT + WORKSPACE_NUMERICAL_TOLERANCE
            )
        )

    def _project_workspace(self, foot_positions_base):
        """把候选足端连续投影到安全工作空间边界内。"""

        foot_positions_hip = self.kinematic.base_to_hip(
            foot_positions_base
        )
        boundary_z = self.workspace_boundary[:, 0]

        z = np.clip(
            foot_positions_hip[:, 2],
            boundary_z[0],
            boundary_z[-1],
        )
        beta = np.clip(
            np.arctan2(
                foot_positions_hip[:, 1],
                foot_positions_hip[:, 0],
            ),
            -WORKSPACE_BETA_LIMIT,
            WORKSPACE_BETA_LIMIT,
        )
        rho = np.linalg.norm(foot_positions_hip[:, :2], axis=1)
        rho_min = np.interp(
            z,
            boundary_z,
            self.workspace_boundary[:, 1],
        )
        rho_max = np.interp(
            z,
            boundary_z,
            self.workspace_boundary[:, 2],
        )
        rho = np.clip(rho, rho_min, rho_max)

        foot_positions_hip[:, 0] = rho * np.cos(beta)
        foot_positions_hip[:, 1] = rho * np.sin(beta)
        foot_positions_hip[:, 2] = z
        return self.kinematic.hip_to_base(foot_positions_hip)

    @staticmethod
    def _segment_distance(a_start, a_end, b_start, b_end):
        """计算两条三维有限线段之间的最短距离。"""

        direction_a = a_end - a_start
        direction_b = b_end - b_start
        start_delta = a_start - b_start

        length_a_sq = np.dot(direction_a, direction_a)
        length_b_sq = np.dot(direction_b, direction_b)
        direction_dot = np.dot(direction_a, direction_b)
        a_start_dot = np.dot(direction_a, start_delta)
        b_start_dot = np.dot(direction_b, start_delta)

        denominator = (
            length_a_sq * length_b_sq
            - direction_dot**2
        )
        if denominator > 1e-12:
            scale_a = np.clip(
                (
                    direction_dot * b_start_dot
                    - a_start_dot * length_b_sq
                )
                / denominator,
                0.0,
                1.0,
            )
        else:
            scale_a = 0.0

        scale_b = (
            direction_dot * scale_a + b_start_dot
        ) / length_b_sq

        # 如果B的最近点落在线段外，把它固定在端点，
        # 再重新求A线段上的最近点。
        if scale_b < 0.0:
            scale_b = 0.0
            scale_a = np.clip(
                -a_start_dot / length_a_sq,
                0.0,
                1.0,
            )
        elif scale_b > 1.0:
            scale_b = 1.0
            scale_a = np.clip(
                (direction_dot - a_start_dot) / length_a_sq,
                0.0,
                1.0,
            )

        closest_a = a_start + scale_a * direction_a
        closest_b = b_start + scale_b * direction_b
        return np.linalg.norm(closest_a - closest_b)

    def _foot_collision_free(self, foot_positions_base):
        """规划层足端球碰撞检查，对应expert.py::_CollideCheck。"""

        foot_positions_base = np.asarray(
            foot_positions_base,
            dtype=np.float64,
        ).reshape(6, 3)
        collision_free = np.ones(6, dtype=bool)

        for first_leg in range(6):
            for second_leg in range(first_leg + 1, 6):
                distance = np.linalg.norm(
                    foot_positions_base[first_leg]
                    - foot_positions_base[second_leg]
                )
                if distance < MIN_FOOT_CLEARANCE:
                    collision_free[first_leg] = False
                    collision_free[second_leg] = False

        return collision_free

    def _link_collision_free(self, joint_angles):
        """用URDF碰撞盒等效胶囊检查整机连杆自碰撞。"""

        points = self.kinematic.link_points_base(joint_angles)
        collision_free = np.ones(6, dtype=bool)

        # 同一条腿只检查不相邻的thigh与ankle。
        # 相邻连杆在关节处本来就相接，不能作为碰撞处理。
        self_clearance = (
            LINK_COLLISION_RADII[0]
            + LINK_COLLISION_RADII[2]
            + COLLISION_MARGIN
        )
        for leg_index in range(6):
            distance = self._segment_distance(
                points[leg_index, 0],
                points[leg_index, 1],
                points[leg_index, 2],
                points[leg_index, 3],
            )
            if distance < self_clearance:
                collision_free[leg_index] = False

        # 六条腿两两检查，共15组；每组检查3×3个胶囊组合。
        for first_leg in range(6):
            for second_leg in range(first_leg + 1, 6):
                pair_collision = False

                for first_link in range(3):
                    for second_link in range(3):
                        distance = self._segment_distance(
                            points[first_leg, first_link],
                            points[first_leg, first_link + 1],
                            points[second_leg, second_link],
                            points[second_leg, second_link + 1],
                        )
                        clearance = (
                            LINK_COLLISION_RADII[first_link]
                            + LINK_COLLISION_RADII[second_link]
                            + COLLISION_MARGIN
                        )
                        if distance < clearance:
                            pair_collision = True
                            break

                    if pair_collision:
                        break

                if pair_collision:
                    collision_free[first_leg] = False
                    collision_free[second_leg] = False

        # thigh根部从机身内部安装，因此只检查knee和ankle。
        body_start = np.array(
            [0.0, 0.0, BODY_COLLISION_Z_MIN],
            dtype=np.float64,
        )
        body_end = np.array(
            [0.0, 0.0, BODY_COLLISION_Z_MAX],
            dtype=np.float64,
        )
        for leg_index in range(6):
            for link_index in (1, 2):
                distance = self._segment_distance(
                    points[leg_index, link_index],
                    points[leg_index, link_index + 1],
                    body_start,
                    body_end,
                )
                clearance = (
                    BODY_COLLISION_RADIUS
                    + LINK_COLLISION_RADII[link_index]
                    + COLLISION_MARGIN
                )
                if distance < clearance:
                    collision_free[leg_index] = False

        return collision_free

    def collision_guard(self, q_candidate, q_current):
        """候选整机姿态全部安全才提交，否则整体保持当前姿态。"""

        q_candidate = np.asarray(
            q_candidate,
            dtype=np.float64,
        ).reshape(6, 3)
        q_current = np.asarray(
            q_current,
            dtype=np.float64,
        ).reshape(6, 3)
        if not self.enable_link_collision_check:
            # 仅绕过耗时的整机连杆胶囊碰撞检查。候选姿态在到达这里前
            # 仍经过工作空间、足端间距和关节限位约束。
            self.last_link_collision_free[:] = True
            return q_candidate

        self.last_link_collision_free = self._link_collision_free(
            q_candidate
        )
        if self.last_link_collision_free.all():
            return q_candidate
        return q_current.copy()

    def _sync_actual_feet(self, q_cur):
        """用当前反馈同步模式交接的足端参考。"""

        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6, 3)
        feet_base = self.kinematic.hip_to_base(self.kinematic.forward(q_cur))
        self.foot_desired_base[:] = feet_base
        self.foot_desired_base_prev[:] = feet_base
        self.q_des = q_cur.copy()
        return feet_base

    def _safe_direct_joint_target(self, q_candidate, q_cur):
        """检查DockMode给出的关节目标，不安全时保持反馈姿态。"""

        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6, 3)
        try:
            q_candidate = np.asarray(q_candidate, dtype=np.float64).reshape(6, 3)
        except (TypeError, ValueError):
            return q_cur.copy(), "dock target shape is invalid"
        if not np.isfinite(q_candidate).all():
            return q_cur.copy(), "dock target is non-finite"
        if (q_candidate < JOINT_LOWER).any() or (q_candidate > JOINT_UPPER).any():
            return q_cur.copy(), "dock target exceeds joint limits"
        if np.any(np.abs(q_candidate - q_cur) > JOINT_VELOCITY_LIMIT * self.dt):
            return q_cur.copy(), "dock target exceeds per-cycle joint speed limit"
        accepted = self.collision_guard(q_candidate, q_cur)
        if not np.array_equal(accepted, q_candidate):
            return q_cur.copy(), "dock target rejected by link collision guard"
        return accepted, ""

    def _commit_workspace_candidate(self, candidate_base):
        """连续投影并提交足端候选，碰撞时才保持旧目标。"""

        candidate_base = np.asarray(
            candidate_base,
            dtype=np.float64,
        ).reshape(6, 3)

        # 可行候选直接使用；只有越界时才做连续投影，
        # 避免每帧重复执行无意义的坐标裁剪。
        safe_candidate = candidate_base
        if (
            self.enable_workspace_check
            and not self._workspace_feasible(candidate_base).all()
        ):
            safe_candidate = self._project_workspace(candidate_base)

        if self._foot_collision_free(safe_candidate).all():
            self.foot_desired_base[:] = safe_candidate

    def reset_to_stand(self, q_cur):
        """从当前关节角平滑回到标准站姿。"""
        # B是全局恢复动作，必须退出攀爬状态再执行站立轨迹。
        self.mission.cancel("reset to stand")
        self.abort_climb()
        self.exit_dock(q_cur)
        self.set_mode(self.APPROACH)
        self.prepare_climb_entry(q_cur)

    def prepare_climb_entry(self, q_cur):
        """从当前关节角平滑整理到compact要求的标准站姿。"""

        self.reset_start_q = np.asarray(
            q_cur,
            dtype=np.float64,
        ).reshape(6, 3).copy()
        self.reset_time = 0.0
        self.reset_active = True

        self.approach_mode.reset()

    def start_autonomous_approach(self, navigation_state):
        """在APPROACH模式下选择安全接近点并启动固定路径。"""

        if self.mode != self.APPROACH:
            raise ValueError(
                "Autonomous approach requires APPROACH mode"
            )
        return self.approach_mode.plan_autonomous_approach(
            navigation_state
        )

    def start_full_mission(
        self,
        navigation_state,
        climb_config,
        start_stage_index=0,
        end_stage_index=None,
        approach_timeout_s=60.0,
        approach_position_tolerance_m=0.01,
        approach_yaw_tolerance_rad=np.deg2rad(1.0),
    ):
        """启动APPROACH到CLIMB再到DOCK的自动调度。"""

        return self.mission.start(
            navigation_state,
            climb_config,
            start_stage_index,
            end_stage_index,
            approach_timeout_s,
            approach_position_tolerance_m,
            approach_yaw_tolerance_rad,
        )

    def set_mode(self, mode):
        """切换唯一活动模式。"""

        if mode not in (self.APPROACH, self.CLIMB, self.DOCK):
            raise ValueError(f"Unknown control mode: {mode}")
        self.mode = mode

    def enter_climb(
        self,
        q_cur,
        config=None,
        start_stage_index=0,
        end_stage_index=None,
        hardware_execution=False,
    ):
        """通过唯一入口进入仅仿真的 compact 攀爬。"""
        self.reset_active = False
        self.exit_dock(q_cur)
        self.set_mode(self.CLIMB)
        self.climb_mode.enter(
            q_cur,
            config,
            start_stage_index=start_stage_index,
            end_stage_index=end_stage_index,
            hardware_execution=hardware_execution,
        )

    def attach_dock_mode(self, dock_mode):
        """附接已由ROS入口配置的DockMode。"""

        if dock_mode is None:
            raise ValueError("dock_mode is required")
        self.dock_mode = dock_mode

    def enter_dock(self, q_cur):
        """同步反馈足端并把DockMode设为唯一活动模式。"""

        if self.dock_mode is None:
            raise RuntimeError("DockMode is not attached")
        if self.climb_mode.state == ClimbMode.RUNNING:
            raise ValueError("DockMode cannot enter while climb is running")
        self.abort_climb()
        feet_base = self._sync_actual_feet(q_cur)
        self.set_mode(self.DOCK)
        self.dock_mode.enter(feet_base)

    def exit_dock(self, q_cur):
        """结束DockMode并以实际足端作为下一模式入口。"""

        if self.dock_mode is not None and self.dock_mode.active:
            self.dock_mode.exit()
        self._sync_actual_feet(q_cur)

    def replay_climb_prefix(
        self,
        q_cur,
        config,
        end_stage_index,
        max_ticks,
    ):
        """用同一DLS控制链连续回放 compact 前缀并返回关节快照。"""

        self.enter_climb(q_cur, config, 0, end_stage_index)
        command = np.zeros(4, dtype=np.float64)
        for _ in range(max_ticks):
            if self.climb_mode.state != ClimbMode.RUNNING:
                break
            q_cur = self.update(q_cur, command)
        if self.climb_mode.state != ClimbMode.DONE:
            raise RuntimeError(
                "compact CPU prefix replay failed: "
                + self.climb_mode.failure_reason
            )
        return q_cur

    def hold_climb(self):
        if self.mode == self.CLIMB:
            self.climb_mode.hold()

    def resume_climb(self):
        if self.mode == self.CLIMB:
            self.climb_mode.resume()

    def abort_climb(self):
        """停止当前攀爬会话；B复位后只能重新通过入口开始。"""

        self.climb_mode.hold()
        self.climb_mode.state = ClimbMode.IDLE

    def update(
        self,
        q_cur,
        command,
        navigation_state=None,
        dock_robot_state=None,
    ):
        """执行当前任务模式并输出本周期关节目标。"""
        if self.mission.state != self.mission.IDLE:
            self.last_mode_result = self.mission.update(
                q_cur,
                navigation_state,
            )
        elif self.mode == self.APPROACH:
            self.last_mode_result = self.approach_mode.update(
                command,
                navigation_state,
            )
        elif self.mode == self.CLIMB:
            self.last_mode_result = self.climb_mode.update(command, q_cur)
        elif self.mode == self.DOCK:
            if self.dock_mode is None:
                raise RuntimeError("DockMode is not attached")
            self.last_mode_result = self.dock_mode.update(dock_robot_state)
            target = self.last_mode_result.joint_positions
            if target is None:
                self.q_des = np.asarray(q_cur, dtype=np.float64).reshape(6, 3).copy()
                return self.q_des
            self.q_des, reason = self._safe_direct_joint_target(target, q_cur)
            if reason:
                self.dock_mode.fail_execution(reason)
                self.last_mode_result = self.dock_mode.update(dock_robot_state)
            return self.q_des
        else:
            raise ValueError(f"Unknown control mode: {self.mode}")

        return self.cal_joint_poses(q_cur)

    def cal_joint_poses(self, q_cur):
        """根据足端目标计算下一周期关节目标"""

        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6, 3)
        self.last_update_velocity_limit_clip_count = 0
        self.last_update_collision_guard_hold_count = 0

        # 每个控制周期更新。
        self.foot_current_hip = self.kinematic.forward(q_cur)

        if self.reset_active:
            next_reset_time = min(
                self.reset_time + self.dt,
                self.reset_duration,
            )
            phase = next_reset_time / self.reset_duration
            if phase >= 1.0 - 1e-12:
                phase = 1.0
            blend = self._smooth_step(phase)

            # 五次曲线在起点和终点的速度、加速度都为零，
            # 因而B回正和A使能准备不会产生瞬时关节目标跳变。
            q_scheduled = (
                (1.0 - blend) * self.reset_start_q
                + blend * self.q_init
            )
            self.reset_time = next_reset_time
            self.q_des = q_scheduled

            # 回正轨迹结束即进入HOLD；HOLD仍持续发送站姿目标。
            # 不用承重关节的伺服稳态误差阻塞整机状态机。
            if phase >= 1.0:
                self.reset_active = False
                self.foot_desired_base[:] = self.foot_init_base
                self.foot_desired_hip[:] = self.foot_init_hip
                self.approach_mode.finish_reset()

            return self.q_des

        #步态规划在base_link工作，统一转换一下坐标系
        self.foot_desired_hip = self.kinematic.base_to_hip(
            self.foot_desired_base
        )

        position_error = (self.foot_desired_hip - self.foot_current_hip)

        damped_inverse = (self.kinematic.damped_inverse_jacobian(q_cur))
        # 每条腿。
        joint_correction = (
            damped_inverse @ position_error[..., np.newaxis]
        ).squeeze(-1)

        # 前馈: 足端目标速度(差分)转关节速度, 消除P控制追赶滞后
        vel_ff = (
            self.foot_desired_base - self.foot_desired_base_prev
        ) / self.dt
        self.foot_desired_base_prev = self.foot_desired_base.copy()
        joint_ff = (
            damped_inverse @ vel_ff[..., np.newaxis]
        ).squeeze(-1)

        q_candidate = np.clip(
            q_cur
            + 16.0 * joint_correction * self.dt
            + joint_ff * self.dt,
            JOINT_LOWER,
            JOINT_UPPER,
        )

        # 舵机能力兜底: 单帧关节增量限幅
        step = JOINT_VELOCITY_LIMIT * self.dt
        self.last_update_velocity_limit_clip_count = int(np.count_nonzero(
            np.abs(q_candidate - q_cur) > step
        ))
        q_candidate = np.clip(q_candidate, q_cur - step, q_cur + step)

        # 关节目标下发前检查完整连杆胶囊；
        # 一旦候选姿态碰撞，整机本周期保持当前位置。
        accepted = self.collision_guard(q_candidate, q_cur)
        self.last_update_collision_guard_hold_count = int(
            not np.array_equal(accepted, q_candidate)
        )
        self.q_des = accepted

        return self.q_des
