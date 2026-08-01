"""六足总控制器与公共安全执行层。

功能：
    管理APPROACH、CLIMB、DOCK三种模式，维护公共足端目标，执行工作空间、
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

from pathlib import Path

import numpy as np

from approach_mode import ApproachMode
from climb_mode import ClimbMode
from dock_mode import DockMode
from kinematics import (
    FOOT_RADIUS,
    JOINT_LOWER,
    JOINT_UPPER,
    Q_STAND,
    GraspKinematic,
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

LOCAL_WORKSPACE_BOUNDARY_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "workspace_bounds.csv"
)
if LOCAL_WORKSPACE_BOUNDARY_PATH.is_file():
    WORKSPACE_BOUNDARY_PATH = LOCAL_WORKSPACE_BOUNDARY_PATH
else:
    import rospkg

    WORKSPACE_BOUNDARY_PATH = (
        Path(rospkg.RosPack().get_path("grasp_hexapod_control"))
        / "config"
        / "workspace_bounds.csv"
    )
WORKSPACE_BETA_LIMIT = np.deg2rad(30.0)
WORKSPACE_NUMERICAL_TOLERANCE = 1e-9


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

    def __init__(self, dt, enable_link_collision_check=True):
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

        self.approach_mode = ApproachMode(self)
        self.climb_mode = ClimbMode(self)
        self.dock_mode = DockMode(self)
        self.mode = self.APPROACH
        self.last_mode_result = None
        self.dock_target_accepted = True
        self.dock_reject_reason = ""

        if not self._workspace_feasible(self.foot_init_base).all():
            raise ValueError("Q_STAND is outside the safe workspace")
        if not self._link_collision_free(self.q_init).all():
            raise ValueError("Q_STAND contains a link collision")

    @staticmethod
    def _smooth_step(phase):
        return(10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5)

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

    def _commit_workspace_candidate(self, candidate_base):
        """连续投影并提交足端候选，碰撞时才保持旧目标。"""

        candidate_base = np.asarray(
            candidate_base,
            dtype=np.float64,
        ).reshape(6, 3)

        # 可行候选直接使用；只有越界时才做连续投影，
        # 避免每帧重复执行无意义的坐标裁剪。
        safe_candidate = candidate_base
        if not self._workspace_feasible(candidate_base).all():
            safe_candidate = self._project_workspace(candidate_base)

        if self._foot_collision_free(safe_candidate).all():
            self.foot_desired_base[:] = safe_candidate

    def _commit_dock_candidate(self, candidate_base):
        """严格接受或拒绝固定足端候选，不允许投影破坏刚体关系。"""

        candidate_base = np.asarray(
            candidate_base,
            dtype=np.float64,
        ).reshape(6, 3)
        if not self._workspace_feasible(candidate_base).all():
            return False, "dock target is outside workspace"
        if not self._foot_collision_free(candidate_base).all():
            return False, "dock target contains foot collision"

        self.foot_desired_base[:] = candidate_base
        return True, ""

    def reset_to_stand(self, q_cur):
        """从当前关节角平滑回到标准站姿。"""
        # B是全局恢复动作，必须退出攀爬/对接状态再执行站立轨迹。
        self.set_mode(self.APPROACH)
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

    def set_mode(self, mode):
        """切换唯一活动模式，并完成DOCK进入/退出处理。"""

        if mode not in (self.APPROACH, self.CLIMB, self.DOCK):
            raise ValueError(f"Unknown control mode: {mode}")
        if mode == self.mode:
            return

        if self.mode == self.DOCK:
            self.dock_mode.exit()
        if mode == self.DOCK:
            self.dock_mode.enter(self.foot_desired_base)
            self.dock_target_accepted = True
            self.dock_reject_reason = ""
        self.mode = mode

    def update(self, q_cur, command, navigation_state=None):
        """执行当前任务模式并输出本周期关节目标。"""
        dock_previous_target = None
        dock_candidate_submitted = False

        if self.mode == self.APPROACH:
            self.last_mode_result = self.approach_mode.update(
                command,
                navigation_state,
            )
        elif self.mode == self.CLIMB:
            self.last_mode_result = self.climb_mode.update(command)
        elif self.mode == self.DOCK:
            if not self.dock_mode.active:
                raise RuntimeError("Enter DOCK with controller.set_mode()")

            self.last_mode_result = self.dock_mode.update(
                command,
                self.dock_target_accepted,
                self.dock_reject_reason,
            )
            self.dock_target_accepted = True
            self.dock_reject_reason = ""

            if (
                self.last_mode_result.active
                and not self.last_mode_result.failed
            ):
                dock_previous_target = self.foot_desired_base.copy()
                dock_candidate_submitted = True
                (
                    self.dock_target_accepted,
                    self.dock_reject_reason,
                ) = self._commit_dock_candidate(
                    self.last_mode_result.foot_positions_base
                )
        else:
            raise ValueError(f"Unknown control mode: {self.mode}")

        q_des = self.cal_joint_poses(q_cur)
        if (
            dock_candidate_submitted
            and self.dock_target_accepted
            and not self.last_link_collision_free.all()
        ):
            self.foot_desired_base[:] = dock_previous_target
            self.dock_target_accepted = False
            self.dock_reject_reason = "dock target contains link collision"
        return q_des

    def cal_joint_poses(self, q_cur):
        """根据足端目标计算下一周期关节目标"""

        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6,3)

        #每个控制周期更新
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
        #每条腿
        joint_correction = (
            damped_inverse @ position_error[..., np.newaxis]        
        ).squeeze(-1)

        q_candidate = np.clip(
            q_cur + 32.0 * joint_correction * self.dt,
            JOINT_LOWER,
            JOINT_UPPER,
        )

        # 关节目标下发前检查完整连杆胶囊；
        # 一旦候选姿态碰撞，整机本周期保持当前位置。
        self.q_des = self.collision_guard(q_candidate, q_cur)

        return self.q_des
