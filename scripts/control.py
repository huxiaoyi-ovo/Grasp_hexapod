from pathlib import Path

import numpy as np


LEG_NAMES = ("lb", "lf", "lm", "rb", "rf", "rm")  # left/right, front/middle/back
JOINT_NAMES = ("thigh", "knee", "ankle")
TRIPOD_A_INDICES = np.array([0, 1, 5], dtype=np.int64)  # lb, lf, rm
TRIPOD_B_INDICES = np.array([2, 3, 4], dtype=np.int64)  # lm, rb, rf
CONTROL_DOF_NAMES = tuple(
    f"{leg_name}_{joint_name}_joint"
    for leg_name in LEG_NAMES
    for joint_name in JOINT_NAMES
)
#将控制关节的名称映射到Isaac Gym中的索引
def build_dof_indices(isaac_dof_names)-> np.ndarray:
    '''建立控制器顺序到Isaac Gym顺序的索引映射，
    output: control_flat[i] = isaac_values[control_to_isaac_indices[i]]'''

    #将名称转换成isaac数组索引  ，如 lb_thigh:0
    isaac_index_by_name = {
        name: index
        for index, name in enumerate(isaac_dof_names)
    }

    missing_names = [
        name
        for name in CONTROL_DOF_NAMES
        if name not in isaac_index_by_name  
    ]
    if missing_names:
        raise ValueError(f"Missing DOF names in Isaac Gym: {missing_names}")
    # 第i个元素表示控制器第i个关节位于Isaac数组的哪个位置
    return np.array(
        [isaac_index_by_name[name] for name in CONTROL_DOF_NAMES],
        dtype=np.int64,
    )

def isaac_to_control(isaac_values, dof_indices):
    '''将Isaac Gym的一维18关节数组转换成Expert内部的(6, 3)。

    输入：
        isaac_values.shape == (18,)

    输出：
        control_values.shape == (6, 3)
        第0维顺序：lb, lf, lm, rb, rf, rm
        第1维顺序：thigh, knee, ankle'''
    

    control_flat = np.asarray(isaac_values)[dof_indices]
    return control_flat.reshape(6, 3)

def control_to_isaac(control_values, dof_indices):
    '''将Expert内部的(6, 3)数组转换成Isaac Gym的一维18关节数组。

    后续用于：
        q_des -> gym.set_actor_dof_position_targets()'''
    
    #将控制器的(6, 3)数组展平为一维数组
    control_flat = np.asarray(control_values).reshape(18)

    isaac_values = np.empty(18, dtype=np.float32)
    isaac_values[dof_indices] = control_flat
    return isaac_values

#urdf几何参数
HIP_XYZ = np.array(
    [
        [-0.04250, -0.07361, 0.02525],  # lb
        [-0.04250,  0.07361, 0.02525],  # lf
        [-0.08500,  0.00000, 0.02525],  # lm
        [ 0.04250, -0.07361, 0.02525],  # rb
        [ 0.04250,  0.07361, 0.02525],  # rf
        [ 0.08500,  0.00000, 0.02525],  # rm
    ],
    dtype=np.float64,   
)

#六个thigh_joint相对base_link的固定yaw
HIP_YAW = np.array(
    [
        -2.0944,  # lb：-120°
         2.0944,  # lf： 120°
        -3.1416,  # lm：-180°
        -1.0472,  # rb： -60°
         1.0472,  # rf：  60°
         0.0,     # rm：   0°
    ],
    dtype=np.float64,
)

# URDF中所有关节轴都沿局部Z轴，但正负方向不同
# 实际旋转角 = axis_sign * Isaac Gym关节角
JOINT_AXIS_SIGNS = np.array(
    [
        [-1.0, -1.0, -1.0],  # lb
        [-1.0, -1.0, -1.0],  # lf
        [-1.0, -1.0, -1.0],  # lm
        [-1.0,  1.0,  1.0],  # rb
        [-1.0,  1.0,  1.0],  # rf
        [-1.0,  1.0,  1.0],  # rm
    ],
    dtype=np.float64,
)

# foot_link球心相对ankle_link的完整固定偏移
# 不能用单独的l3代替，因为它同时具有X、Y、Z分量
FOOT_OFFSET_ANKLE = np.array(
    [0.11660, -0.08015, -0.00025],
    dtype=np.float64,
)

THIGH_TO_KNEE_LENGTH = 0.05236
KNEE_TO_ANKLE_LENGTH = 0.07745
FOOT_RADIUS = 0.0065

# URDF碰撞盒的横截面半对角线，用胶囊完整包住旋转后的盒体。
LINK_COLLISION_RADII = np.array(
    [
        np.hypot(0.025 / 2.0, 0.025 / 2.0),  # thigh
        np.hypot(0.022 / 2.0, 0.022 / 2.0),  # knee
        np.hypot(0.022 / 2.0, 0.022 / 2.0),  # ankle
    ],
    dtype=np.float64,
)
COLLISION_MARGIN = 0.003
BODY_COLLISION_RADIUS = 0.097
BODY_COLLISION_Z_MIN = 0.0
BODY_COLLISION_Z_MAX = 0.121
MIN_FOOT_CLEARANCE = 2.0 * FOOT_RADIUS + COLLISION_MARGIN

# 离线工作空间分析生成的z-rho安全边界。
WORKSPACE_BOUNDARY_PATH = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "workspace_bounds.csv"
)

# 离线采样使用±35°，在线再向内保留5°角向余量。
WORKSPACE_BETA_LIMIT = np.deg2rad(30.0)
WORKSPACE_NUMERICAL_TOLERANCE = 1e-9

# 标准工作中心保持108.971 mm外展半径，同时让末段严格竖直。
# 现有连杆几何决定了该姿态的机身高度约为69.885 mm。
STAND_KNEE_ANGLE = np.deg2rad(43.035535181)
STAND_ANKLE_ANGLE = np.deg2rad(-98.531183330)
STAND_GEOMETRIC_ANGLES = np.tile(
    np.array(
        [0.0, STAND_KNEE_ANGLE, STAND_ANKLE_ANGLE],
        dtype=np.float64,
    ),
    (6,1),#六条腿使用相同的局部几何姿态
)
Q_STAND = (
    STAND_GEOMETRIC_ANGLES / JOINT_AXIS_SIGNS
)

def translation(x,y,z):
    """生成平移矩阵"""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = [x, y, z]
    return transform    

def rotation_x(angle):
    '''
    生成绕局部X轴旋转的齐次变换矩阵
    '''
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
    '''
    生成绕局部Z轴旋转的齐次变换矩阵
    '''
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    transform = np.eye(4, dtype=np.float64)

    transform[:3, :3] = [
        [cos_angle, -sin_angle, 0.0],
        [sin_angle, cos_angle, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return transform    

#kinematic
class GraspKinematic:
    def __init__(self):
        # 每条腿的髋坐标系到base_link坐标系变换
        self.base_from_hip = np.stack(
            [
                translation(*HIP_XYZ[leg_index]) @ rotation_z(HIP_YAW[leg_index])
                for leg_index in range(6)
            ]   
        )

    #反变换用于把base_link中的目标足端转换回髋坐标系
        self.hip_from_base = np.linalg.inv(self.base_from_hip)
        # thigh_link到knee_joint的固定变换
        # URDF：
        # xyz="0.05236 0 0"
        # rpy="1.5708 0 0"
        self.thigh_to_knee_origin = (
            translation(THIGH_TO_KNEE_LENGTH, 0.0, 0.0)
            @ rotation_x(1.5708)
        )

        # knee_link到ankle_joint的固定变换
        # URDF：
        # xyz="0.07745 0 0.00025"
        self.knee_to_ankle_origin = translation(
            KNEE_TO_ANKLE_LENGTH,
            0.0,
            0.00025,
        )

         # ankle_link到foot_link的固定变换
        # 保留完整XYZ偏移，不将其简化成直线l3
        self.ankle_to_foot = translation(
            FOOT_OFFSET_ANKLE[0],
            FOOT_OFFSET_ANKLE[1],
            FOOT_OFFSET_ANKLE[2],
        )

    def forward_leg(self, leg_index, joint_angles):
        """计算一条腿的足端球心位置。

        输入：
            leg_index：LEG_NAMES中的腿索引
            joint_angles：[thigh, knee, ankle]

        输出：
            foot_position_hip：[x, y, z]
            坐标系：该腿thigh_joint的髋坐标系"""

        q_thigh, q_knee, q_ankle = joint_angles
        hip_from_foot = rotation_z(JOINT_AXIS_SIGNS[leg_index, 0] * q_thigh)
        hip_from_foot = (
            hip_from_foot
            @ self.thigh_to_knee_origin
            @ rotation_z(JOINT_AXIS_SIGNS[leg_index, 1] * q_knee)
            @ self.knee_to_ankle_origin
            @ rotation_z(JOINT_AXIS_SIGNS[leg_index, 2] * q_ankle)
            @ self.ankle_to_foot
        )
        foot_position_hip = hip_from_foot[:3, 3]
        return foot_position_hip
    
    def forward(self, joint_angles):
        """
        计算六个足端在各自髋坐标系中的位置。

        输入：
            joint_angles.shape == (6, 3)

        输出：
            foot_positions_hip.shape == (6, 3)
        """
        joint_angles = np.asarray(joint_angles, dtype=np.float64).reshape(6, 3)
        foot_positions_hip = np.stack(
            [
                self.forward_leg(leg_index, joint_angles[leg_index])
                for leg_index in range(6)
            ]
        )
        return foot_positions_hip

    def link_points_base(self, joint_angles):
        """返回六条腿的髋、膝、踝、足端点，shape=(6, 4, 3)。"""

        joint_angles = np.asarray(
            joint_angles,
            dtype=np.float64,
        ).reshape(6, 3)
        points = np.empty((6, 4, 3), dtype=np.float64)

        for leg_index in range(6):
            theta = (
                JOINT_AXIS_SIGNS[leg_index]
                * joint_angles[leg_index]
            )
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
        """计算一条腿的雅克比
        输出：
            jacobian.shape == (3, 3)
            行：足端[x, y, z]
            列：关节[thigh, knee, ankle]

        满足：
            foot_velocity_hip = jacobian @ joint_velocity"""
        joint_angles = np.asarray(
            joint_angles, dtype=np.float64  
        ).reshape(3)

        # theta是URDF中真正发生的旋转角。
        
        theta = (JOINT_AXIS_SIGNS[leg_index]* joint_angles)
        joint_origins = np.empty((3, 3), dtype=np.float64)
        joint_axes = np.empty((3, 3), dtype=np.float64)
        #下面分别计算thigh、knee、ankle的关节原点和旋转轴在髋坐标系中的位置
        transform = np.eye(4, dtype=np.float64)
        joint_origins[0] = transform[:3, 3]
        joint_axes[0] = transform[:3, 2]    

        transform = (transform @ rotation_z(theta[0]) @ self.thigh_to_knee_origin)
        joint_origins[1] = transform[:3, 3]
        joint_axes[1] = transform[:3, 2]    

        transform = (transform @ rotation_z(theta[1]) @ self.knee_to_ankle_origin)  
        joint_origins[2] = transform[:3, 3] 
        joint_axes[2] = transform[:3, 2]    

        #计算足端位置
        transform = (transform @ rotation_z(theta[2]) @ self.ankle_to_foot)
        foot_position = transform[:3, 3]    

        jacobian = np.empty((3, 3), dtype=np.float64)
        for joint_index in range(3):
            # 对旋转角theta_i求导
            jacobian[:, joint_index] = (JOINT_AXIS_SIGNS[leg_index, joint_index] * np.cross(
                joint_axes[joint_index],
                foot_position - joint_origins[joint_index],
            ))
        return jacobian

    def jacobian(self, joint_angles):
        joint_angles = np.asarray(joint_angles, dtype=np.float64).reshape(6, 3)
        jacobians = np.stack(
            [
                self.jacobian_leg(leg_index, joint_angles[leg_index])
                for leg_index in range(6)
            ]
        )   
        return jacobians  # shape == (6, 3, 3)  

    def damped_inverse_jacobian(self, joint_angles, damping=0.01): 
        """计算阻尼雅克比逆"""
        jacobians = self.jacobian(joint_angles)  # shape == (6, 3, 3)
        identity = np.eye(3, dtype=np.float64)
        damped_inverse = np.stack(
            [
                jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + damping**2 * identity)
                for jacobian in jacobians
            ]
        )  # shape == (6, 3, 3)
        return damped_inverse


        
    def hip_to_base(self, foot_positions_hip):
        """将六个足端位置从各自髋坐标系转换到base_link坐标系。

        """
        foot_positions_hip = np.asarray(foot_positions_hip, dtype=np.float64).reshape(6, 3)
        foot_positions_base = np.empty((6, 3), dtype=np.float64)
        for leg_index in range(6):

            #补齐齐次坐标1，使得旋转和平移可以用矩阵乘法表示
            foot_homogeneous = np.append(foot_positions_hip[leg_index], 1.0)
            foot_positions_base[leg_index] = (
                self.base_from_hip[leg_index] @ foot_homogeneous
            )[:3]

        return foot_positions_base
    
    def base_to_hip(self, foot_positions_base):
        foot_positions_base = np.asarray(foot_positions_base, dtype=np.float64).reshape(6, 3)
        foot_positions_hip = np.empty((6, 3), dtype=np.float64)
        for leg_index in range(6):
            foot_homogeneous = np.append(foot_positions_base[leg_index], 1.0)
            foot_positions_hip[leg_index] = (
                self.hip_from_base[leg_index] @ foot_homogeneous
            )[:3]
        return foot_positions_hip
    
    def forward_base(self, joint_angles):
        """
        直接计算六个足端球心在base_link坐标系中的位置。
        """
        foot_positions_hip = self.forward(joint_angles)
        return self.hip_to_base(foot_positions_hip)
                                                                                    
class GraspController:
    """六足控制器
    速度指令
        -> 足端步态规划（base_link坐标系）
        -> 转换到各腿hip坐标系
        -> 阻尼雅可比
        -> q_des"""

    def __init__(self, dt):
        self.dt = dt
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

        #True表示支撑腿，False表示摆动腿
        #初始三条支撑腿
        self.gaits = np.zeros(6, dtype=bool)
        self.gaits[TRIPOD_A_INDICES] = True  # lb, lf，rm支撑

        self.stance_group_index = 0
        #三角步态
        # 0.20 m/s满杆时，0.30 s摆动阶段前进约60 mm。
        self.phase_duration = 0.30
        self.phase_time = 0.0
        # 三角组切换前保留3个控制周期的六足共同支撑。
        self.transfer_duration = 3.0 * self.dt
        self.transfer_time = 0.0
        self.transfer_active = False
        # 平地抬脚20 mm，接近机械边界时由工作空间投影连续修正。
        self.step_height = 0.020
        self.phase_command = np.zeros(4, dtype=np.float64)
        self.stop_requested = False
        #增加z方向
        self.body_height_offset = 0.0
        # 继续降低机身会使ankle接近±120°机械边界。
        self.body_height_offset_min = -0.0075
        self.body_height_offset_max = 0.015

        self.gait_started = False
        self.first_step = True

        # A使能和B复位都先用五次曲线平滑回到标准站姿。
        self.reset_duration = 0.8
        self.reset_time = 0.0
        self.reset_active = False
        self.reset_start_q = self.q_init.copy()

        # 每次换相时记录摆动足起点和落点。
        self.swing_start_base = self.foot_init_base.copy()
        self.swing_target_base = self.foot_init_base.copy()
        self.foot_velocity_xy = np.zeros((6, 2), dtype=np.float64)
        self.swing_start_velocity_xy = np.zeros((6, 2), dtype=np.float64)
        self.swing_target_velocity_xy = np.zeros((6, 2), dtype=np.float64)
        self.last_link_collision_free = np.ones(6, dtype=bool)

        if not self._workspace_feasible(self.foot_init_base).all():
            raise ValueError("Q_STAND is outside the safe workspace")
        if not self._link_collision_free(self.q_init).all():
            raise ValueError("Q_STAND contains a link collision")

    @staticmethod
    def _smooth_step(phase):
        return(10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5)

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

    def reset_to_stand(self, q_cur):
        """从当前关节角平滑回到标准站姿。"""
        self.reset_start_q = np.asarray(
            q_cur,
            dtype=np.float64,
        ).reshape(6, 3).copy()
        self.reset_time = 0.0
        self.reset_active = True

        # 回正期间停止步态状态机，完成后从A组重新起步。
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

    def _begin_step(self, command):
        """开启一个新的三角步态阶段command = [vx, vy, vz, wz]"""
        self.phase_time = 0.0
        self.phase_command = np.asarray(
            command,
            dtype=np.float64,
        ).reshape(4).copy()

        swing_indices = np.where(~self.gaits)[0]
        # 摆动轨迹必须从当前足端目标连续开始，
        self.swing_start_base = self.foot_desired_base.copy()
        self.swing_target_base = self.foot_init_base.copy()
        self.swing_start_velocity_xy = self.foot_velocity_xy.copy()

        home_xy = self.foot_init_base[swing_indices, :2]
        body_velocity_at_home = -self._stance_velocity(
            home_xy,
            self.phase_command,
        )

        # 一条腿的完整支撑时间还包括落地和离地两次共同支撑，
        # 因此用完整支撑位移的一半确定对称落脚点。
        support_duration = (
            self.phase_duration + 2.0 * self.transfer_duration
        )
        self.swing_target_base[
            swing_indices,
            :2,
        ] += (
            0.5
            * support_duration
            * body_velocity_at_home
        )
        # 当前机身升高多少，地面相对base_link就向下多少。
        self.swing_target_base[
            swing_indices,
            2,
        ] = (
            self.foot_init_base[swing_indices, 2]
            - self.body_height_offset
        )

        # 不再把落点投影回固定半径圆。否则前腿的前向落点会被压回
        # 工作中心附近，而支撑阶段仍完整后移，造成支撑末端严重内夹。
        # 保留笛卡尔落点后，满杆时完整支撑区间约为±40 mm，
        # 让末段在一个步态周期内的向外、向内倾角基本对称。

        # 超出工作空间时连续压到边界，不再把整条腿硬退回标准落点。
        target_feasible = self._workspace_feasible(
            self.swing_target_base
        )
        if not target_feasible[swing_indices].all():
            projected_target = self._project_workspace(
                self.swing_target_base
            )
            self.swing_target_base[swing_indices] = projected_target[
                swing_indices
            ]

        # 落地瞬间就使用下一段支撑速度，使足端世界速度接近零，
        # 避免接触地面时发生水平擦动。
        self.swing_target_velocity_xy[swing_indices] = (
            self._stance_velocity(
                self.swing_target_base[swing_indices, :2],
                self.phase_command,
            )
        )

    def update_gait(self, command):
        """先规划候选足端，通过工作空间检查后再提交。"""

        command = np.asarray(command, dtype=np.float64).reshape(4)

        # 回正由cal_joint_poses中的关节插值接管，期间不更新足端步态。
        if self.reset_active:
            return

        velocity_right = command[0]
        velocity_forward = command[1]
        velocity_up = command[2]
        yaw_rate = command[3]

        self.body_height_offset += (
            velocity_up * self.dt
        )

        # 只允许经过当前工作空间验证的机身升降范围。
        self.body_height_offset = np.clip(
            self.body_height_offset,
            self.body_height_offset_min,
            self.body_height_offset_max,
        )
        # 六足接触同一平地时，其地面基准高度一致：
        ground_z = (
            self.foot_init_base[:, 2]
            - self.body_height_offset
        )
        candidate_base = self.foot_desired_base.copy()

        # 是否存在需要启动步态的平面运动。
        # 单独的vz只调整机身高度，不应该触发三角步态。
        planar_command = np.array(
            [
                velocity_right,
                velocity_forward,
                yaw_rate,
            ],
            dtype=np.float64,
        )
        if not self.gait_started:
            # 站立升降模式下，六足水平位置不变，
            # 只有相对base_link的z同时变化。
            candidate_base[:, 2] = ground_z
            self._commit_workspace_candidate(candidate_base)
            self.foot_velocity_xy[:] = 0.0

            if np.linalg.norm(planar_command) < 1e-8:
                return

            self.gait_started = True
            self.stop_requested = False
            self._begin_step(command)
        else:
            # 松开平面摇杆后，先完成当前摆动再停止，
            # 避免三条摆动腿被停在空中。
            self.stop_requested = (
                np.linalg.norm(planar_command) < 1e-8
            )

        # 摆动足落地后，六足共同支撑一小段时间。
        # 六个足端都执行同一个刚体相对速度，不会互相拖拽。
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
            self._commit_workspace_candidate(candidate_base)

            transfer_finished = (
                self.transfer_time
                >= self.transfer_duration - 1e-12
            )
            if not transfer_finished:
                return

            # 共同支撑结束后，原摆动组成为新支撑组，
            # 原支撑组在下一阶段开始摆动。
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

        phase_dt = min(self.dt, self.phase_duration - self.phase_time)

        stance_indices = np.where(self.gaits)[0]
        swing_indices = np.where(~self.gaits)[0]

        # 一个phase内部使用开始时冻结的平面指令。
        # 摇杆的新方向在下一次换相后生效，避免支撑腿和摆动腿
        # 在同一phase内执行两个不同方向。
        # 支撑足在base_link中执行机身刚体速度的反向运动。
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

        # 水平摆动轨迹同时匹配离地和落地速度：
        # 离地承接旧支撑速度，落地衔接新的支撑速度。
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

        # 摆动足的额外抬高量。
        if phase < 0.5:
            lift_height = (
                self.step_height * self._smooth_step(2.0 * phase)
            )
        else:
            lift_height = (
                self.step_height * self._smooth_step(2.0 * (1.0 - phase))
            )
        candidate_base[swing_indices, 2] = (
            ground_z[swing_indices] + lift_height
        )

        phase_finished = (
            self.phase_time >= self.phase_duration - 1e-12
        )
        if phase_finished:
            # 如果换相期间机身高度也发生变化，
            # 用最新ground_z更新摆动落点，避免使用阶段开始时的旧高度。
            self.swing_target_base[swing_indices, 2] = ground_z[
                swing_indices
            ]
            candidate_base[swing_indices] = self.swing_target_base[
                swing_indices
            ]
            self.foot_velocity_xy[swing_indices] = (
                self.swing_target_velocity_xy[swing_indices]
            )

        # 对应expert.py：
        # next_B_e_des -> _FeasiCheck -> B_e_des。
        self._commit_workspace_candidate(candidate_base)

        if phase_finished:
            self.transfer_active = True
            self.transfer_time = 0.0

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
            q_candidate = (
                (1.0 - blend) * self.reset_start_q
                + blend * self.q_init
            )
            self.last_link_collision_free = (
                self._link_collision_free(q_candidate)
            )

            # 回正是整机同步动作；任何腿不安全时本周期整体暂停。
            if not self.last_link_collision_free.all():
                self.q_des = q_cur.copy()
                return self.q_des

            self.reset_time = next_reset_time
            self.q_des = q_candidate

            if phase >= 1.0:
                self.reset_active = False
                self.body_height_offset = 0.0
                self.foot_desired_base[:] = self.foot_init_base
                self.foot_desired_hip[:] = self.foot_init_hip
                self.swing_start_base[:] = self.foot_init_base
                self.swing_target_base[:] = self.foot_init_base

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

        q_candidate = q_cur + 32.0 * joint_correction * self.dt

        # 关节目标下发前检查完整连杆胶囊；
        # 一旦候选姿态碰撞，整机本周期保持当前位置。
        self.q_des = self.collision_guard(q_candidate, q_cur)

        return self.q_des
