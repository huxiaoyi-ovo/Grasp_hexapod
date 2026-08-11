"""抓取六足的几何参数、坐标变换和纯运动学计算。

功能：
    定义URDF对应的髋位置、关节轴符号和连杆几何，提供FK、连杆关键点、
    Jacobian、DLS阻尼逆以及hip/base_link坐标转换。
输入：
    关节角通常为shape=(6,3)、单位rad；足端位置为shape=(6,3)、单位m。
输出：
    足端/连杆位置、shape=(6,3,3)的Jacobian与DLS逆矩阵、坐标变换结果。
结构：
    几何常量 -> 齐次变换函数 -> GraspKinematic六足批量计算。
边界：
    只包含确定性数学模型，不包含步态、任务状态机、仿真或硬件通信。
"""

import numpy as np


LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")
JOINT_NAMES = ("thigh", "knee", "ankle")

# 六个thigh_joint在base_link中的位置和固定yaw。
HIP_XYZ = np.array(
    [
        [-0.04250, -0.07361, 0.02525],
        [-0.04250,  0.07361, 0.02525],
        [-0.08500,  0.00000, 0.02525],
        [ 0.04250, -0.07361, 0.02525],
        [ 0.04250,  0.07361, 0.02525],
        [ 0.08500,  0.00000, 0.02525],
    ],
    dtype=np.float64,
)
HIP_YAW = np.array(
    [
        -2.0944,
         2.0944,
        -3.1416,
        -1.0472,
         1.0472,
         0.0,
    ],
    dtype=np.float64,
)

# 实际几何旋转角 = JOINT_AXIS_SIGNS * Isaac/实机关节角。
JOINT_AXIS_SIGNS = np.array(
    [
        [-1.0, -1.0, -1.0],
        [-1.0, -1.0, -1.0],
        [-1.0, -1.0, -1.0],
        [-1.0,  1.0,  1.0],
        [-1.0,  1.0,  1.0],
        [-1.0,  1.0,  1.0],
    ],
    dtype=np.float64,
)

FOOT_OFFSET_ANKLE = np.array(
    [0.11660, -0.08015, -0.00025],
    dtype=np.float64,
)
THIGH_TO_KNEE_LENGTH = 0.05236
KNEE_TO_ANKLE_LENGTH = 0.07745
FOOT_RADIUS = 0.0065

# 标准工作中心保持108.971 mm外展半径，同时让末段严格竖直。
STAND_KNEE_ANGLE = np.deg2rad(43.035535181)
STAND_ANKLE_ANGLE = np.deg2rad(-98.531183330)
STAND_GEOMETRIC_ANGLES = np.tile(
    np.array(
        [0.0, STAND_KNEE_ANGLE, STAND_ANKLE_ANGLE],
        dtype=np.float64,
    ),
    (6, 1),
)
Q_STAND = STAND_GEOMETRIC_ANGLES / JOINT_AXIS_SIGNS

# 控制器顺序的URDF机械限位；公共控制层和Isaac Gym使用同一组边界。
JOINT_LOWER = np.tile(
    np.array([-2.094, -2.094, -2.094], dtype=np.float64),
    (6, 1),
)
JOINT_UPPER = np.tile(
    np.array([2.094, 2.094, 2.094], dtype=np.float64),
    (6, 1),
)
JOINT_LOWER[:, 0] = [-0.698, -1.571, -0.698, -1.571, -0.698, -0.698]
JOINT_UPPER[:, 0] = [1.571, 0.698, 0.698, 0.698, 1.571, 0.698]
# LX-15D空载6.5~10.5 rad/s(0.16s/60°@5V ~ 0.10s/60°@6V)；0.4m/s需7.65，取6V档76%。
JOINT_VELOCITY_LIMIT = np.full((6, 3), 8.0, dtype=np.float64)

# M1A 估计惯性/几何模型，来自 hexapod_isaacgym_view.urdf。它们是模型估计，
# 不是实机测量真值。连杆顺序为 thigh/knee/ankle/foot；腿顺序采用
# LEG_NAMES 的稳定控制器顺序。位置单位 m，质量单位 kg。
ESTIMATED_MODEL_LINK_NAMES = ("thigh", "knee", "ankle", "foot")
ESTIMATED_BASE_MASS = np.float64(3.0)
ESTIMATED_BASE_COM = np.array(
    [0.00048381, 0.0032191, 0.050559],
    dtype=np.float64,
)
ESTIMATED_LEG_LINK_MASSES = np.array(
    [0.071, 0.079, 0.092, 0.005],
    dtype=np.float64,
)
ESTIMATED_LEG_LINK_COMS = np.array(
    [
        [0.019013, 0.00032996, 0.00086404],
        [0.024, 0.00029496, 0.00084694],
        [0.037761, -0.014128, 0.0005094],
        [0.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)
ESTIMATED_TOTAL_MASS = np.float64(4.482)


def translation(x, y, z):
    """生成平移齐次变换矩阵。"""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = [x, y, z]
    return transform


def rotation_x(angle):
    """生成绕局部X轴旋转的齐次变换矩阵。"""
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = [
        [1.0, 0.0, 0.0],
        [0.0, cos_angle, -sin_angle],
        [0.0, sin_angle, cos_angle],
    ]
    return transform


def rotation_z(angle):
    """生成绕局部Z轴旋转的齐次变换矩阵。"""
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = [
        [cos_angle, -sin_angle, 0.0],
        [sin_angle, cos_angle, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return transform


class GraspKinematic:
    """抓取六足纯运动学模型，不包含步态和仿真接口。"""

    def __init__(self):
        self.base_from_hip = np.stack(
            [
                translation(*HIP_XYZ[leg_index])
                @ rotation_z(HIP_YAW[leg_index])
                for leg_index in range(6)
            ]
        )
        self.hip_from_base = np.linalg.inv(self.base_from_hip)

        self.thigh_to_knee_origin = (
            translation(THIGH_TO_KNEE_LENGTH, 0.0, 0.0)
            @ rotation_x(1.5708)
        )
        self.knee_to_ankle_origin = translation(
            KNEE_TO_ANKLE_LENGTH,
            0.0,
            0.00025,
        )
        self.ankle_to_foot = translation(*FOOT_OFFSET_ANKLE)

    def forward_leg(self, leg_index, joint_angles):
        """计算一条腿足端球心在该腿髋坐标系中的位置。"""
        q_thigh, q_knee, q_ankle = joint_angles
        hip_from_foot = rotation_z(
            JOINT_AXIS_SIGNS[leg_index, 0] * q_thigh
        )
        hip_from_foot = (
            hip_from_foot
            @ self.thigh_to_knee_origin
            @ rotation_z(JOINT_AXIS_SIGNS[leg_index, 1] * q_knee)
            @ self.knee_to_ankle_origin
            @ rotation_z(JOINT_AXIS_SIGNS[leg_index, 2] * q_ankle)
            @ self.ankle_to_foot
        )
        return hip_from_foot[:3, 3]

    def forward(self, joint_angles):
        """计算六个足端在各自髋坐标系中的位置，shape=(6,3)。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        return np.stack(
            [
                self.forward_leg(leg_index, joint_angles[leg_index])
                for leg_index in range(6)
            ]
        )

    def link_points_base(self, joint_angles):
        """返回六条腿的髋、膝、踝、足端点，shape=(6,4,3)。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        points = np.empty((6, 4, 3), dtype=np.float64)

        for leg_index in range(6):
            theta = JOINT_AXIS_SIGNS[leg_index] * joint_angles[leg_index]
            transform = self.base_from_hip[leg_index].copy()
            points[leg_index, 0] = transform[:3, 3]

            transform = (
                transform
                @ rotation_z(theta[0])
                @ self.thigh_to_knee_origin
            )
            points[leg_index, 1] = transform[:3, 3]

            transform = (
                transform
                @ rotation_z(theta[1])
                @ self.knee_to_ankle_origin
            )
            points[leg_index, 2] = transform[:3, 3]

            transform = (
                transform
                @ rotation_z(theta[2])
                @ self.ankle_to_foot
            )
            points[leg_index, 3] = transform[:3, 3]

        return points

    def jacobian_leg(self, leg_index, joint_angles):
        """计算一条腿足端位置对三个关节角的3×3雅可比。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(3)
        theta = JOINT_AXIS_SIGNS[leg_index] * joint_angles

        joint_origins = np.empty((3, 3), dtype=np.float64)
        joint_axes = np.empty((3, 3), dtype=np.float64)

        transform = np.eye(4, dtype=np.float64)
        joint_origins[0] = transform[:3, 3]
        joint_axes[0] = transform[:3, 2]

        transform = (
            transform
            @ rotation_z(theta[0])
            @ self.thigh_to_knee_origin
        )
        joint_origins[1] = transform[:3, 3]
        joint_axes[1] = transform[:3, 2]

        transform = (
            transform
            @ rotation_z(theta[1])
            @ self.knee_to_ankle_origin
        )
        joint_origins[2] = transform[:3, 3]
        joint_axes[2] = transform[:3, 2]

        transform = (
            transform
            @ rotation_z(theta[2])
            @ self.ankle_to_foot
        )
        foot_position = transform[:3, 3]

        jacobian = np.empty((3, 3), dtype=np.float64)
        for joint_index in range(3):
            jacobian[:, joint_index] = (
                JOINT_AXIS_SIGNS[leg_index, joint_index]
                * np.cross(
                    joint_axes[joint_index],
                    foot_position - joint_origins[joint_index],
                )
            )
        return jacobian

    def jacobian(self, joint_angles):
        """计算六条腿的雅可比，shape=(6,3,3)。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        return np.stack(
            [
                self.jacobian_leg(leg_index, joint_angles[leg_index])
                for leg_index in range(6)
            ]
        )

    def damped_inverse_jacobian(self, joint_angles, damping=0.01):
        """计算DLS阻尼雅可比逆，shape=(6,3,3)。"""
        jacobians = self.jacobian(joint_angles)
        identity = np.eye(3, dtype=np.float64)
        return np.stack(
            [
                jacobian.T
                @ np.linalg.inv(
                    jacobian @ jacobian.T
                    + damping**2 * identity
                )
                for jacobian in jacobians
            ]
        )

    def hip_to_base(self, foot_positions_hip):
        """将六个足端从各自髋坐标系转换到base_link。"""
        foot_positions_hip = np.asarray(
            foot_positions_hip,
            dtype=np.float64,
        ).reshape(6, 3)
        foot_positions_base = np.empty((6, 3), dtype=np.float64)

        for leg_index in range(6):
            foot_homogeneous = np.append(
                foot_positions_hip[leg_index],
                1.0,
            )
            foot_positions_base[leg_index] = (
                self.base_from_hip[leg_index] @ foot_homogeneous
            )[:3]

        return foot_positions_base

    def base_to_hip(self, foot_positions_base):
        """将六个足端从base_link转换到各自髋坐标系。"""
        foot_positions_base = np.asarray(
            foot_positions_base,
            dtype=np.float64,
        ).reshape(6, 3)
        foot_positions_hip = np.empty((6, 3), dtype=np.float64)

        for leg_index in range(6):
            foot_homogeneous = np.append(
                foot_positions_base[leg_index],
                1.0,
            )
            foot_positions_hip[leg_index] = (
                self.hip_from_base[leg_index] @ foot_homogeneous
            )[:3]

        return foot_positions_hip

    def forward_base(self, joint_angles):
        """计算六个足端球心在base_link中的位置。"""
        return self.hip_to_base(self.forward(joint_angles))

    def link_transforms_base(self, joint_angles):
        """返回 base 到各连杆的变换，shape=(6,4,4,4)。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        transforms = np.empty((6, 4, 4, 4), dtype=np.float64)

        for leg_index in range(6):
            theta = JOINT_AXIS_SIGNS[leg_index] * joint_angles[leg_index]
            thigh = self.base_from_hip[leg_index] @ rotation_z(theta[0])
            knee = thigh @ self.thigh_to_knee_origin @ rotation_z(theta[1])
            ankle = (
                knee
                @ self.knee_to_ankle_origin
                @ rotation_z(theta[2])
            )
            foot = ankle @ self.ankle_to_foot
            transforms[leg_index, 0] = thigh
            transforms[leg_index, 1] = knee
            transforms[leg_index, 2] = ankle
            transforms[leg_index, 3] = foot

        return transforms

    def link_com_positions_base(self, joint_angles):
        """返回模型估计的连杆 COM，shape=(6,4,3)，坐标在 base_link、单位 m。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        transforms = self.link_transforms_base(joint_angles)
        com_positions = np.empty((6, 4, 3), dtype=np.float64)

        for leg_index in range(6):
            for link_index in range(4):
                local_com = np.append(
                    ESTIMATED_LEG_LINK_COMS[link_index],
                    1.0,
                )
                com_positions[leg_index, link_index] = (
                    transforms[leg_index, link_index] @ local_com
                )[:3]

        return com_positions

    def center_of_mass_base(self, joint_angles):
        """返回模型估计的整机 COM，shape=(3,)，坐标在 base_link、单位 m。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        weighted_com = ESTIMATED_BASE_MASS * ESTIMATED_BASE_COM
        link_coms = self.link_com_positions_base(joint_angles)
        weighted_com += np.sum(
            link_coms * ESTIMATED_LEG_LINK_MASSES[np.newaxis, :, np.newaxis],
            axis=(0, 1),
        )
        return weighted_com / ESTIMATED_TOTAL_MASS

    def terminal_axes_base(self, joint_angles):
        """返回归一化 ankle 到足端轴，shape=(6,3)，坐标在 base_link。非接触/承载证据。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        transforms = self.link_transforms_base(joint_angles)
        vectors = transforms[:, 3, :3, 3] - transforms[:, 2, :3, 3]
        norms = np.linalg.norm(vectors, axis=1)
        if not np.all(np.isfinite(vectors)) or np.any(norms <= 0.0):
            raise ValueError("terminal axis has zero or non-finite length")
        return vectors / norms[:, np.newaxis]

    def jacobian_min_singular_values(self, joint_angles):
        """返回解析足端雅可比的每腿最小奇异值，shape=(6,)，单位 m/rad。"""
        singular_values = np.linalg.svd(
            self.jacobian(joint_angles),
            compute_uv=False,
        )
        return np.min(singular_values, axis=1)

    def joint_limit_margins(self, joint_angles):
        """返回每关节 min(q-lower, upper-q)，shape=(6,3)，单位 rad；越限时为负。"""
        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        return np.minimum(
            joint_angles - JOINT_LOWER,
            JOINT_UPPER - joint_angles,
        )
