"""Simulation-only compact 攀爬轨迹执行器。

只播放固定阶段并更新机身/足端参考；不把时间或运动学误差解释为真实接触、
承载或稳定性，也不授权实机攀爬。
"""

import json
import time

import numpy as np

from kinematics import JOINT_NAMES, LEG_NAMES, Q_STAND
from utils import package_config_path


COMPACT_SCHEMA = "SIMULATION_ONLY_CLIMB_COMPACT_V2"


class ClimbMode:
    """播放固定 compact 攀爬阶段的仿真执行器。

    它只用于 Isaac 预览，不表示实机状态或接触。
    """

    IDLE, RUNNING, HOLD, DONE, FAILED = (
        "IDLE",
        "RUNNING",
        "HOLD",
        "DONE",
        "FAILED",
    )

    def __init__(self, controller):
        """保存控制器并初始化攀爬状态。

        参数:
            controller: 提供运动学、关节目标和时间步长的总控制器。
        """

        self.controller = controller
        self.config = None
        self.state = self.IDLE
        self.phase = None
        self.phase_time = 0.0
        self.stage_elapsed_time = 0.0
        self.settle_time = 0.0
        self.failure_reason = ""
        self.anchors_world = None
        self.base_pose = None
        self.stage_index = None
        self.end_stage_index = None
        self.stage_names = ()
        self.last_tracking_error_rad = 0.0
        self.last_foot_target_error_m = 0.0
        self.last_foot_errors_m = np.zeros(6, dtype=np.float64)
        self.last_actual_foot_base_m = np.zeros((6, 3), dtype=np.float64)
        self.last_desired_foot_base_m = np.zeros((6, 3), dtype=np.float64)
        self.last_foot_error_xyz_m = np.zeros((6, 3), dtype=np.float64)
        self.last_diagnostic_phase_time_s = 0.0
        self.last_diagnostic_stage_elapsed_s = 0.0
        self.last_diagnostic_stage_name = ""
        self.last_diagnostic_stage_duration_s = 0.0
        self.last_diagnostic_active_legs = ()
        self.last_worst_foot_leg = LEG_NAMES[0]
        self.last_worst_foot_error_m = 0.0
        self.last_feet_over_gate = ()
        self.last_joint_tracking_errors_rad = np.zeros((6, 3), dtype=np.float64)
        self.last_worst_joint_leg = LEG_NAMES[0]
        self.last_worst_joint_name = JOINT_NAMES[0]
        self.last_worst_joint_error_rad = 0.0
        self.last_worst_joint_q_current_rad = 0.0
        self.last_worst_joint_q_des_rad = 0.0
        self.last_joints_over_tracking_gate = ()
        self.last_settled = False
        self.last_phase_hold = False
        self.last_collision_guard_hold = False
        self.hardware_execution = False
        self._hardware_stage_last_monotonic = None

    @staticmethod
    def _world_from_base(base):
        """根据机身位姿生成世界坐标变换矩阵。"""

        x, y, z, roll, pitch = np.asarray(base, dtype=np.float64)
        cosine_r, sine_r = np.cos(roll), np.sin(roll)
        cosine_p, sine_p = np.cos(pitch), np.sin(pitch)
        rotation_x = np.array(
            (
                (1.0, 0.0, 0.0),
                (0.0, cosine_r, -sine_r),
                (0.0, sine_r, cosine_r),
            )
        )
        rotation_y = np.array(
            (
                (cosine_p, 0.0, sine_p),
                (0.0, 1.0, 0.0),
                (-sine_p, 0.0, cosine_p),
            )
        )
        output = np.eye(4, dtype=np.float64)
        output[:3, :3] = rotation_y @ rotation_x
        output[:3, 3] = (x, y, z)
        return output

    @staticmethod
    def _smoothstep(value):
        """把 0 到 1 的进度平滑插值。"""

        value = float(np.clip(value, 0.0, 1.0))
        return value ** 3 * (10.0 - 15.0 * value + 6.0 * value ** 2)

    @staticmethod
    def _array(config, path, shape):
        """读取并检查 compact 配置中的数组字段。"""

        value = config
        for key in path:
            value = value[key]
        result = np.asarray(value, dtype=np.float64)
        if result.shape != shape or not np.all(np.isfinite(result)):
            raise ValueError(
                "invalid compact field: " + ".".join(map(str, path))
            )
        return result

    def _load_config(self):
        """读取 compact 攀爬配置。"""

        path = package_config_path("climb_compact.json")
        with path.open() as file:
            return json.load(file)

    def terminal_joints(self, config=None):
        """返回配置中明确保存的攀爬末帧关节姿态。"""

        config = self._load_config() if config is None else config
        self._validate_config(config)
        return self._array(config, ("terminal_q_rad",), (6, 3)).copy()

    def _validate_config(self, config):
        """检查 compact 攀爬配置。"""

        if (
            not isinstance(config, dict)
            or config.get("schema") != COMPACT_SCHEMA
            or config.get("simulation_only") is not True
            or config.get("simulation_candidate_only") is not True
        ):
            raise ValueError("invalid simulation-only compact climb config")
        self._array(config, ("xiaolan_translation",), (3,))
        p0_q = self._array(config, ("p0", "q_rad"), (6, 3))
        self._array(config, ("terminal_q_rad",), (6, 3))
        if not np.array_equal(p0_q, Q_STAND):
            raise ValueError("compact P0 must be Q_STAND")
        p0_base = self._array(config, ("p0", "base"), (4,))
        self._array(config, ("p0", "anchors_world_m"), (6, 3))
        stages = config.get("stages")
        if (
            not isinstance(stages, list)
            or not stages
            or config.get("stage_count") != len(stages)
        ):
            raise ValueError("compact stage list is invalid")
        previous_pose = np.array((*p0_base[:3], 0.0, p0_base[3]))
        previous_anchors = self._array(
            config, ("p0", "anchors_world_m"), (6, 3)
        )
        names = []
        for index, stage in enumerate(stages):
            if not isinstance(stage, dict) or not isinstance(stage.get("name"), str):
                raise ValueError("compact stage is invalid")
            names.append(stage["name"])
            pose_start = self._array(
                config, ("stages", index, "pose_start"), (5,)
            )
            pose_end = self._array(
                config, ("stages", index, "pose_end"), (5,)
            )
            knots = np.asarray(stage.get("anchor_knots"), dtype=np.float64)
            durations = np.asarray(
                stage.get("segment_durations_s"), dtype=np.float64
            )
            active = stage.get("active_legs")
            anchor_curve = stage.get("anchor_curve")
            active_base_knots = np.asarray(
                stage.get("active_base_knots_m", []), dtype=np.float64
            )
            base_piecewise_curve = bool(
                anchor_curve == "piecewise_base_quintic"
                and isinstance(active, list)
                and active
                and active_base_knots.shape
                == (len(knots), len(active), 3)
                and np.all(np.isfinite(active_base_knots))
            )
            relative_height = stage.get("relative_swing_height_m")
            relative_curve = bool(
                anchor_curve == "relative_base_high_step"
                and knots.shape[0] == 2
                and durations.shape == (1,)
                and isinstance(relative_height, (int, float))
                and np.isfinite(relative_height)
                and relative_height > 0.0
                and active
            )
            if (
                knots.ndim != 3
                or knots.shape[1:] != (6, 3)
                or len(knots) < 2
                or not np.all(np.isfinite(knots))
                or durations.shape != (len(knots) - 1,)
                or not np.all(np.isfinite(durations))
                or np.any(durations <= 0.0)
                or not isinstance(active, list)
                or any(
                    not isinstance(leg, int) or leg < 0 or leg >= 6
                    for leg in active
                )
                or len(set(active)) != len(active)
                or stage.get("pose_curve") != "quintic_full_stage"
                or not (
                    (anchor_curve == "piecewise_quintic" and not active)
                    or base_piecewise_curve
                    or relative_curve
                )
                or not isinstance(stage.get("settle_s"), (int, float))
                or stage["settle_s"] <= 0.0
            ):
                raise ValueError("invalid compact stage fields: " + stage["name"])
            if (
                not np.allclose(pose_start, previous_pose, rtol=0.0, atol=1e-12)
                or not np.allclose(
                    knots[0], previous_anchors, rtol=0.0, atol=1e-12
                )
            ):
                raise ValueError("compact stage boundary mismatch: " + stage["name"])
            if base_piecewise_curve:
                active_index = np.asarray(active, dtype=np.int64)
                for endpoint, endpoint_pose, knot_index in (
                    (knots[0], pose_start, 0),
                    (knots[-1], pose_end, -1),
                ):
                    endpoint_world = (
                        np.column_stack(
                            (
                                active_base_knots[knot_index],
                                np.ones(len(active)),
                            )
                        )
                        @ self._world_from_base(endpoint_pose).T
                    )[:, :3]
                    if not np.allclose(
                        endpoint_world,
                        endpoint[active_index],
                        rtol=0.0,
                        atol=1e-9,
                    ):
                        raise ValueError(
                            "base-relative endpoint mismatch: "
                            + stage["name"]
                        )
            previous_pose = pose_end
            previous_anchors = knots[-1]
        if len(set(names)) != len(names):
            raise ValueError("compact stage names must be unique")
        gate = config["settle_gate"]
        gate_values = (
            gate["max_joint_tracking_error_rad"],
            gate["max_foot_target_error_m"],
            gate["entry_max_joint_error_rad"],
            gate["persistence_s"],
            gate["timeout_s"],
        )
        preview_time_only = gate.get("preview_time_only_stage_advance")
        diagnostic_only = gate.get("tracking_errors_diagnostic_only")
        if (
            not gate.get("command_tracking_only_not_contact_proof")
            or not isinstance(preview_time_only, bool)
            or not isinstance(diagnostic_only, bool)
            or preview_time_only != diagnostic_only
            or min(gate_values) <= 0.0
        ):
            raise ValueError("invalid simulation-only settle gate")

    def enter(
        self,
        q_current,
        config=None,
        start_stage_index=0,
        end_stage_index=None,
        hardware_execution=False,
    ):
        """进入 compact 攀爬预览。

        参数:
            q_current: 当前关节角，形状为 `(6, 3)`，单位 rad。
            config: 可选的 compact 配置；未提供时从文件读取。
            start_stage_index: 绝对 compact 阶段下标，闭区间起点。
            end_stage_index: 绝对 compact 阶段下标，闭区间终点。
            hardware_execution: 实机执行时必须用反馈稳定门限推进阶段。
        """

        config = self._load_config() if config is None else config
        self._validate_config(config)
        q_current = np.asarray(q_current, dtype=np.float64).reshape(6, 3)
        if not np.all(np.isfinite(q_current)):
            raise ValueError("q_current must be finite")
        stage_count = len(config["stages"])
        if not isinstance(start_stage_index, int) or not 0 <= start_stage_index < stage_count:
            raise ValueError("compact start stage index is out of range")
        if end_stage_index is None:
            end_stage_index = stage_count - 1
        if (
            not isinstance(end_stage_index, int)
            or not start_stage_index <= end_stage_index < stage_count
        ):
            raise ValueError("compact end stage index is out of range")
        if start_stage_index == 0:
            entry_error = np.max(
                np.abs(q_current - self._array(config, ("p0", "q_rad"), (6, 3)))
            )
            if entry_error > config["settle_gate"]["entry_max_joint_error_rad"]:
                raise ValueError("compact entry joint error exceeds simulation gate")
        self.config = config
        self.hardware_execution = bool(hardware_execution)
        self.stage_index = start_stage_index
        self.end_stage_index = end_stage_index
        self.stage_names = tuple(stage["name"] for stage in config["stages"])
        self.phase = self.stage_names[self.stage_index]
        self.phase_time = 0.0
        self.stage_elapsed_time = 0.0
        self.settle_time = 0.0
        self.failure_reason = ""
        self.state = self.RUNNING
        self._hardware_stage_last_monotonic = (
            time.monotonic()
            if (
                self.hardware_execution
                and self.controller.climb_timeout_uses_wall_time
            )
            else None
        )
        if start_stage_index == 0:
            self.anchors_world = self._array(
                config, ("p0", "anchors_world_m"), (6, 3)
            ).copy()
            p0_base = self._array(config, ("p0", "base"), (4,))
            self.base_pose = np.array((*p0_base[:3], 0.0, p0_base[3]))
        else:
            stage = config["stages"][start_stage_index]
            self.base_pose = self._array(
                config, ("stages", start_stage_index, "pose_start"), (5,)
            ).copy()
            self.anchors_world = np.asarray(
                stage["anchor_knots"], dtype=np.float64
            )[0].copy()
        self._apply_reference(self.base_pose, self.anchors_world, sync_previous=True)
        self.controller.q_des = q_current.copy()
        self._update_tracking_diagnostics(q_current)
        if start_stage_index != 0 and (
            self.last_foot_target_error_m
            > config["settle_gate"]["max_foot_target_error_m"]
        ):
            raise ValueError(
                "compact mid-stage entry foot target error exceeds simulation gate"
            )

    def hold(self):
        """暂停正在执行的攀爬预览。"""

        if self.state == self.RUNNING:
            self.state = self.HOLD
            self._hardware_stage_last_monotonic = None

    def resume(self):
        """继续已暂停的攀爬预览。"""

        if self.state == self.HOLD:
            self.state = self.RUNNING
            if (
                self.hardware_execution
                and self.controller.climb_timeout_uses_wall_time
            ):
                self._hardware_stage_last_monotonic = time.monotonic()

    def _update_hardware_stage_elapsed_time(self):
        """按执行后端累计阶段耗时；实机墙钟，Isaac控制帧时间。"""

        if not self.controller.climb_timeout_uses_wall_time:
            self.stage_elapsed_time += self.controller.dt
            return

        now = time.monotonic()
        previous = self._hardware_stage_last_monotonic
        self._hardware_stage_last_monotonic = now
        if previous is not None:
            self.stage_elapsed_time += max(0.0, now - previous)

    def _apply_reference(self, base, anchors, sync_previous=False):
        """把机身和足端参考写入控制器。"""

        self.base_pose = np.asarray(base, dtype=np.float64).copy()
        self.anchors_world = np.asarray(anchors, dtype=np.float64).copy()
        inverse = np.linalg.inv(self._world_from_base(self.base_pose))
        desired = np.column_stack((self.anchors_world, np.ones(6))) @ inverse.T
        self.controller.foot_desired_base[:] = desired[:, :3]
        if sync_previous:
            self.controller.foot_desired_base_prev[:] = desired[:, :3]

    def _stage_reference(self):
        """计算当前阶段的机身和足端参考。"""

        stage = self.config["stages"][self.stage_index]
        pose_start = np.asarray(stage["pose_start"], dtype=np.float64)
        pose_end = np.asarray(stage["pose_end"], dtype=np.float64)
        knots = np.asarray(stage["anchor_knots"], dtype=np.float64)
        durations = stage["segment_durations_s"]
        total = float(sum(durations))
        phase = float(np.clip(self.phase_time / total, 0.0, 1.0))
        pose_weight = self._smoothstep(phase)
        pose = pose_start * (1.0 - pose_weight) + pose_end * pose_weight
        if stage["anchor_curve"] == "relative_base_high_step":
            anchors = self._relative_base_high_step(
                stage,
                pose,
                pose_weight,
                phase,
            )
        elif stage["anchor_curve"] == "piecewise_base_quintic":
            anchors = self._piecewise_base(stage, pose)
        else:
            anchors = self._piecewise(knots, durations)
        if self.phase_time >= total:
            pose = pose_end.copy()
            anchors = knots[-1].copy()
        return pose, anchors, total

    def _relative_base_high_step(self, stage, pose, pose_weight, phase):
        """按 Approach 同款剖面生成机身相对高抬腿曲线。"""

        knots = np.asarray(stage["anchor_knots"], dtype=np.float64)
        pose_start = np.asarray(stage["pose_start"], dtype=np.float64)
        pose_end = np.asarray(stage["pose_end"], dtype=np.float64)
        start_inverse = np.linalg.inv(self._world_from_base(pose_start))
        end_inverse = np.linalg.inv(self._world_from_base(pose_end))
        start_base = np.column_stack((knots[0], np.ones(6))) @ start_inverse.T
        end_base = np.column_stack((knots[1], np.ones(6))) @ end_inverse.T
        desired_base = (
            (1.0 - pose_weight) * start_base[:, :3]
            + pose_weight * end_base[:, :3]
        )
        # 六次单峰曲线的两端速度、加速度均为零；避免把 Approach
        # 30 mm 梯形抬腿中的分段速度拐点放大到攀爬高步。
        lift_weight = 64.0 * phase**3 * (1.0 - phase) ** 3
        active = np.asarray(stage["active_legs"], dtype=np.int64)
        desired_base[active, 2] += (
            stage["relative_swing_height_m"] * lift_weight
        )
        current_world = (
            np.column_stack((desired_base, np.ones(6)))
            @ self._world_from_base(pose).T
        )
        anchors = knots[0].copy()
        anchors[active] = current_world[active, :3]
        return anchors

    def _piecewise_base(self, stage, pose):
        """在base_link中插值活动腿，固定支撑腿保持世界锚点。"""

        knots = np.asarray(stage["anchor_knots"], dtype=np.float64)
        durations = stage["segment_durations_s"]
        active = np.asarray(stage["active_legs"], dtype=np.int64)
        active_base = self._piecewise(
            np.asarray(stage["active_base_knots_m"], dtype=np.float64),
            durations,
        )
        current_world = (
            np.column_stack((active_base, np.ones(len(active))))
            @ self._world_from_base(pose).T
        )
        anchors = self._piecewise(knots, durations)
        anchors[active] = current_world[:, :3]
        return anchors

    def _piecewise(self, knots, durations):
        """按当前阶段时间插值足端锚点。"""

        elapsed = self.phase_time
        for index, duration in enumerate(durations):
            if elapsed <= duration:
                weight = self._smoothstep(elapsed / duration)
                return (
                    knots[index] * (1.0 - weight)
                    + knots[index + 1] * weight
                )
            elapsed -= duration
        return knots[-1].copy()

    def _advance_stage(self):
        """切换到下一阶段或结束预览。"""

        if self.stage_index == self.end_stage_index:
            self.state = self.DONE
            self._hardware_stage_last_monotonic = None
            return
        self.stage_index += 1
        self.phase = self.stage_names[self.stage_index]
        self.phase_time = 0.0
        self.stage_elapsed_time = 0.0
        self.settle_time = 0.0
        if (
            self.hardware_execution
            and self.controller.climb_timeout_uses_wall_time
        ):
            self._hardware_stage_last_monotonic = time.monotonic()

    def _update_tracking_diagnostics(self, q_current):
        """更新关节和足端目标误差。"""

        q_current = np.asarray(q_current, dtype=np.float64).reshape(6, 3)
        if not np.all(np.isfinite(q_current)):
            raise ValueError("q_current must be finite")
        q_des = np.asarray(
            self.controller.q_des, dtype=np.float64
        ).reshape(6, 3)
        joint_errors = np.abs(q_current - q_des)
        tracking_error = float(np.max(joint_errors))
        actual_base = self.controller.kinematic.hip_to_base(
            self.controller.kinematic.forward(q_current)
        )
        desired_base = np.asarray(
            self.controller.foot_desired_base, dtype=np.float64
        ).reshape(6, 3)
        foot_error_xyz = actual_base - desired_base
        foot_errors = np.linalg.norm(
            foot_error_xyz, axis=1
        )
        foot_error = float(np.max(foot_errors))
        if not np.isfinite(tracking_error) or not np.isfinite(foot_error):
            raise ValueError("compact tracking diagnostics must be finite")
        gate = self.config["settle_gate"]
        worst_foot_index = int(np.argmax(foot_errors))
        worst_joint_index = np.unravel_index(
            int(np.argmax(joint_errors)), joint_errors.shape
        )
        feet_over_gate = np.flatnonzero(
            foot_errors > gate["max_foot_target_error_m"]
        )
        joints_over_gate = np.argwhere(
            joint_errors > gate["max_joint_tracking_error_rad"]
        )
        self.last_tracking_error_rad = tracking_error
        self.last_foot_target_error_m = foot_error
        self.last_foot_errors_m = foot_errors.copy()
        self.last_actual_foot_base_m = actual_base.copy()
        self.last_desired_foot_base_m = desired_base.copy()
        self.last_foot_error_xyz_m = foot_error_xyz.copy()
        self.last_diagnostic_phase_time_s = self.phase_time
        self.last_diagnostic_stage_elapsed_s = self.stage_elapsed_time
        stage = self.config["stages"][self.stage_index]
        self.last_diagnostic_stage_name = stage["name"]
        self.last_diagnostic_stage_duration_s = float(
            sum(stage["segment_durations_s"])
        )
        self.last_diagnostic_active_legs = tuple(stage["active_legs"])
        self.last_worst_foot_leg = LEG_NAMES[worst_foot_index]
        self.last_worst_foot_error_m = float(foot_errors[worst_foot_index])
        self.last_feet_over_gate = tuple(
            (LEG_NAMES[index], float(foot_errors[index]))
            for index in feet_over_gate
        )
        self.last_joint_tracking_errors_rad = joint_errors.copy()
        self.last_worst_joint_leg = LEG_NAMES[worst_joint_index[0]]
        self.last_worst_joint_name = JOINT_NAMES[worst_joint_index[1]]
        self.last_worst_joint_error_rad = float(joint_errors[worst_joint_index])
        self.last_worst_joint_q_current_rad = float(q_current[worst_joint_index])
        self.last_worst_joint_q_des_rad = float(q_des[worst_joint_index])
        self.last_joints_over_tracking_gate = tuple(
            (
                LEG_NAMES[leg_index] + "_" + JOINT_NAMES[joint_index],
                float(joint_errors[leg_index, joint_index]),
            )
            for leg_index, joint_index in joints_over_gate
        )
        # 关节角误差和足端 FK 误差来自同一组回读，并且会随姿态受到
        # 不同 Jacobian 放大。实机负载下前者存在稳定偏差时，不能再把
        # 它作为与足端任务误差并列的完成条件；足端误差才是阶段推进的
        # 任务量，关节误差仅保留给日志/故障诊断。
        self.last_settled = bool(
            foot_error <= gate["max_foot_target_error_m"]
        )

    @staticmethod
    def _format_named_errors(errors):
        """格式化超过既有门限的完整腿/关节列表。"""

        return ",".join(
            "{}={:.9g}".format(name, value) for name, value in errors
        ) or "none"

    @staticmethod
    def _format_xyz(vector):
        """稳定格式化 base_link 中的 xyz 向量。"""

        return "({:.9g},{:.9g},{:.9g})".format(*vector)

    def _format_foot_vectors(self, leg_indices):
        """格式化指定腿的 actual/desired/signed-FK-error base_link 向量。"""

        values = []
        for leg_index in leg_indices:
            values.append(
                "{}[actual_base_xyz_m={},desired_base_xyz_m={},error_xyz_m={}]".format(
                    LEG_NAMES[leg_index],
                    self._format_xyz(self.last_actual_foot_base_m[leg_index]),
                    self._format_xyz(self.last_desired_foot_base_m[leg_index]),
                    self._format_xyz(self.last_foot_error_xyz_m[leg_index]),
                )
            )
        return ";".join(values) or "none"

    def tracking_diagnostic_summary(self):
        """返回实机暂停/失败可直接定位的反馈误差摘要。"""

        gate = self.config["settle_gate"]
        worst_motor = (
            self.last_worst_joint_leg + "_" + self.last_worst_joint_name
        )
        return (
            "worst_foot={} foot_target_error_m={:.9g} foot_gate_m={:.9g} "
            "feet_over_gate={} feet_over_gate_base_link_xyz={} "
            "worst_motor={} tracking_error_rad={:.9g} "
            "q_cur_rad={:.9g} q_des_rad={:.9g} motors_over_{:.9g}rad={}"
        ).format(
            self.last_worst_foot_leg,
            self.last_worst_foot_error_m,
            gate["max_foot_target_error_m"],
            self._format_named_errors(self.last_feet_over_gate),
            self._format_foot_vectors(
                tuple(LEG_NAMES.index(name) for name, _ in self.last_feet_over_gate)
            ),
            worst_motor,
            self.last_worst_joint_error_rad,
            self.last_worst_joint_q_current_rad,
            self.last_worst_joint_q_des_rad,
            gate["max_joint_tracking_error_rad"],
            self._format_named_errors(self.last_joints_over_tracking_gate),
        )

    def active_leg_diagnostic_summary(self):
        """返回当前阶段活动腿在诊断采样时的 base_link 规划/实际对照。"""

        active = self.last_diagnostic_active_legs
        return (
            "base_link diagnostic_stage={} diagnostic_phase_time_s={:.9g} "
            "stage_duration_s={:.9g} diagnostic_stage_elapsed_s={:.9g} active_legs={} "
            "active_leg_base_link_xyz={}"
        ).format(
            self.last_diagnostic_stage_name,
            self.last_diagnostic_phase_time_s,
            self.last_diagnostic_stage_duration_s,
            self.last_diagnostic_stage_elapsed_s,
            ",".join(LEG_NAMES[index] for index in active) or "none",
            self._format_foot_vectors(active),
        )

    def _tracking_failure_reason(self):
        """生成阶段超时的诊断性失败原因，不把其解释为接触证明。"""

        return (
            self.phase
            + ": TRACKING_OR_KINEMATIC_TARGET_TIMEOUT_NOT_CONTACT_PROOF "
            + self.tracking_diagnostic_summary()
        )

    def update(self, command, q_current):
        """推进当前攀爬预览阶段。

        参数:
            command: 控制循环传入的命令，当前不参与攀爬插值。
            q_current: 当前关节角，形状为 `(6, 3)`，单位 rad。

        返回:
            无返回值。
        """

        del command
        if self.state == self.IDLE:
            return None
        if self.state == self.HOLD:
            self._update_tracking_diagnostics(q_current)
            return None
        if self.state == self.FAILED:
            self._update_tracking_diagnostics(q_current)
            return None

        if self.hardware_execution:
            gate = self.config["settle_gate"]
            stage = self.config["stages"][self.stage_index]
            duration = float(sum(stage["segment_durations_s"]))
            self._update_hardware_stage_elapsed_time()

            # 先检查上一控制帧已经下发的目标。实机跟不上或公共碰撞守卫
            # 保持时冻结当前轨迹相位，继续追踪同一目标，避免摆动腿尚未
            # 到达最高点，规划时钟就进入平移或下降。
            self._update_tracking_diagnostics(q_current)
            collision_hold = bool(getattr(
                self.controller,
                "last_update_collision_guard_hold_count",
                0,
            ))
            self.last_collision_guard_hold = collision_hold
            if self.phase_time < duration:
                boundaries = np.cumsum(stage["segment_durations_s"])
                internal = boundaries[:-1]
                at_checkpoint = bool(np.any(np.isclose(
                    self.phase_time, internal, rtol=0.0, atol=1e-12
                )))
                checkpoint_foot_hold = at_checkpoint and not self.last_settled
                self.last_phase_hold = bool(
                    collision_hold or checkpoint_foot_hold
                )
                if not self.last_phase_hold:
                    next_boundary = boundaries[
                        np.searchsorted(boundaries, self.phase_time, side="right")
                    ]
                    self.phase_time = min(
                        float(next_boundary),
                        self.phase_time + self.controller.dt,
                    )
                base, anchors, _ = self._stage_reference()
                self._apply_reference(base, anchors)
                if self.stage_elapsed_time >= duration + gate["timeout_s"]:
                    self.state = self.FAILED
                    self.failure_reason = self._tracking_failure_reason()
                return None

            # 到达终点后的下一帧才开始累计稳定时间，避免把抵达终点前
            # 一帧对旧目标的误差错误计入完成门。
            base, anchors, _ = self._stage_reference()
            self._apply_reference(base, anchors)
            self.last_phase_hold = bool(
                not self.last_settled or collision_hold
            )
            self.settle_time = (
                self.settle_time + self.controller.dt
                if self.last_settled and not collision_hold
                else 0.0
            )
            settle_required = max(
                float(stage["settle_s"]),
                float(gate["persistence_s"]),
            )
            if self.settle_time >= settle_required:
                self._advance_stage()
            elif self.stage_elapsed_time >= duration + gate["timeout_s"]:
                self.state = self.FAILED
                self.failure_reason = self._tracking_failure_reason()
            return None

        base, anchors, duration = self._stage_reference()
        self._apply_reference(base, anchors)
        self._update_tracking_diagnostics(q_current)
        if self.state == self.DONE:
            return None
        if self.phase_time < duration:
            self.phase_time = min(duration, self.phase_time + self.controller.dt)
            return None
        gate = self.config["settle_gate"]
        preview_time_only = (
            gate["preview_time_only_stage_advance"]
            and not self.hardware_execution
        )
        if preview_time_only:
            self.settle_time += self.controller.dt
        else:
            self.settle_time = (
                self.settle_time + self.controller.dt
                if self.last_settled
                else 0.0
            )
        settle_required = self.config["stages"][self.stage_index]["settle_s"]
        if self.settle_time >= settle_required:
            self._advance_stage()
        elif (
            not preview_time_only
            and self.phase_time >= duration + gate["timeout_s"]
        ):
            self.state = self.FAILED
            self.failure_reason = (
                self.phase
                + ": TRACKING_OR_KINEMATIC_TARGET_TIMEOUT_NOT_CONTACT_PROOF "
                + "tracking_error_rad={:.9g} foot_target_error_m={:.9g}".format(
                    self.last_tracking_error_rad,
                    self.last_foot_target_error_m,
                )
            )
        else:
            self.phase_time += self.controller.dt
        return None
