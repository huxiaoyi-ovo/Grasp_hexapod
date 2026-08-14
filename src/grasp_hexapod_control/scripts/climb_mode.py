"""Simulation-only compact 攀爬轨迹执行器。

只播放固定阶段并更新机身/足端参考；不把时间或运动学误差解释为真实接触、
承载或稳定性，也不授权实机攀爬。
"""

import json

import numpy as np

from kinematics import Q_STAND
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
        self.settle_time = 0.0
        self.failure_reason = ""
        self.anchors_world = None
        self.base_pose = None
        self.stage_index = None
        self.end_stage_index = None
        self.stage_names = ()
        self.last_tracking_error_rad = 0.0
        self.last_foot_target_error_m = 0.0
        self.last_settled = False
        self.hardware_execution = False

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
                    anchor_curve == "piecewise_quintic"
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
        self.settle_time = 0.0
        self.failure_reason = ""
        self.state = self.RUNNING
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

    def resume(self):
        """继续已暂停的攀爬预览。"""

        if self.state == self.HOLD:
            self.state = self.RUNNING

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
            return
        self.stage_index += 1
        self.phase = self.stage_names[self.stage_index]
        self.phase_time = 0.0
        self.settle_time = 0.0

    def _update_tracking_diagnostics(self, q_current):
        """更新关节和足端目标误差。"""

        q_current = np.asarray(q_current, dtype=np.float64).reshape(6, 3)
        if not np.all(np.isfinite(q_current)):
            raise ValueError("q_current must be finite")
        tracking_error = float(
            np.max(np.abs(q_current - np.asarray(self.controller.q_des)))
        )
        actual_base = self.controller.kinematic.hip_to_base(
            self.controller.kinematic.forward(q_current)
        )
        foot_error = float(
            np.max(
                np.linalg.norm(
                    actual_base - self.controller.foot_desired_base, axis=1
                )
            )
        )
        if not np.isfinite(tracking_error) or not np.isfinite(foot_error):
            raise ValueError("compact tracking diagnostics must be finite")
        gate = self.config["settle_gate"]
        self.last_tracking_error_rad = tracking_error
        self.last_foot_target_error_m = foot_error
        self.last_settled = bool(
            tracking_error <= gate["max_joint_tracking_error_rad"]
            and foot_error <= gate["max_foot_target_error_m"]
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
