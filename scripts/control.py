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
FOOT_OFFSET_PLANAR_LENGTH = np.linalg.norm(FOOT_OFFSET_ANKLE[:2])
FOOT_OFFSET_PLANAR_ANGLE = np.arctan2(FOOT_OFFSET_ANKLE[1], FOOT_OFFSET_ANKLE[0])
REFERENCE_INIT_GEOMETRIC_ANGLES = np.array(
    [
        [0.5, 0.67, -2.2],  # lb
        [-0.5, 0.67, -2.2],  # lf
        [ 0.0, 0.67, -2.2],  # lm
        [-0.5, 0.67, -2.2],  # rb
        [ 0.5, 0.67, -2.2],  # rf
        [ 0.0, 0.67, -2.2],  # rm
    ],
    dtype=np.float64,
)
GRASP_INIT_GEOMETRIC_ANGLES = (
    REFERENCE_INIT_GEOMETRIC_ANGLES.copy()
)

# expert的左右腿基坐标系与抓取机器人的六个径向髋坐标系不同，
# 因此先对齐thigh在base_link中的绝对方向，再换算成本机髋系转角。
REFERENCE_THIGH_YAW_BASE = np.array(
    [
        -np.pi + 0.5,  # lb
         np.pi - 0.5,  # lf
        -np.pi,        # lm
        -0.5,          # rb
         0.5,          # rf
         0.0,          # rm
    ],
    dtype=np.float64,
)
thigh_yaw_difference = REFERENCE_THIGH_YAW_BASE - HIP_YAW
GRASP_INIT_GEOMETRIC_ANGLES[:, 0] = np.arctan2(
    np.sin(thigh_yaw_difference),
    np.cos(thigh_yaw_difference),
)

GRASP_INIT_GEOMETRIC_ANGLES[:, 2] -= (
    FOOT_OFFSET_PLANAR_ANGLE
)


# 标准几何角与Isaac Gym关节坐标的关系：
#     theta = JOINT_AXIS_SIGNS * q
#
# 因此：
#     q = theta / JOINT_AXIS_SIGNS
# Q_INIT是整个项目唯一的初始关节姿态来源。

Q_INIT = (
    GRASP_INIT_GEOMETRIC_ANGLES
    / JOINT_AXIS_SIGNS
)

##Q_INIT = np.zeros((6,3),dtype=np.float64)#测试零位

# foot_link使用半径6.5 mm的球
# 正运动学算到球心；平地接触点还要考虑这个半径
FOOT_RADIUS = 0.0065

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
            translation(0.05236, 0.0, 0.0)
            @ rotation_x(1.5708)
        )

        # knee_link到ankle_joint的固定变换
        # URDF：
        # xyz="0.07745 0 0.00025"
        self.knee_to_ankle_origin = translation(
            0.07745,
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
    """六足控制器"""

    def __init__(self, dt):
        self.dt = dt
        self.kinematic = GraspKinematic()

        self.q_init = Q_INIT.copy()  # shape == (6, 3)
        self.q_des = self.q_init.copy()  # shape == (6, 3)

        self.foot_init_hip = self.kinematic.forward(self.q_init)  # shape == (6, 3)
        self.foot_current_hip = self.foot_init_hip.copy()  # shape == (6, 3)    
        self.foot_desired_hip = self.foot_init_hip.copy()  # shape == (6, 3)

        #True表示支撑腿，False表示摆动腿
        #初始三条支撑腿
        self.gaits = np.zeros(6, dtype=bool)
        self.gaits[TRIPOD_A_INDICES] = True  # lb, lf，rm

        self.stance_group_index = 0
        self.swing_reach_point = np.zeros(6, dtype=bool)

    def cal_joint_poses(self, q_cur, q_dot_cur):

        q_cur = np.asarray(q_cur, dtype=np.float64).reshape(6,3)
        q_dot_cur = np.asarray(q_dot_cur, dtype=np.float64).reshape(6,3)

        #每个控制周期更新
        self.foot_current_hip = self.kinematic.forward(q_cur)

        position_error = (self.foot_desired_hip - self.foot_current_hip)

        damped_inverse = (self.kinematic.damped_inverse_jacobian(q_cur))
        joint_correction = (
            damped_inverse @ position_error[..., np.newaxis]        
        ).squeeze(-1)

        self.q_des = (q_cur + 32.0*joint_correction*self.dt)

        return self.q_des






