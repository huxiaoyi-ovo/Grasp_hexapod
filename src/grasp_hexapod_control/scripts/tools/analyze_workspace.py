#!/usr/bin/env python3
"""六足安全工作空间的离线采样、边界生成和可视化工具。

功能：
    从当前仿真URDF读取关节限位，采样六腿可达空间，筛选站立步态分支、
    奇异性和角度余量，并生成在线控制器使用的z-rho安全边界。
输入：
    urdf/hexapod_isaacgym_view.urdf、kinematics.py几何模型和离线采样参数。
输出：
    config/workspace_bounds.csv、统计信息以及六足工作空间三维可视化。
结构：
    读取限位 -> 关节采样 -> FK/奇异值筛选 -> 边界提取 -> 保存与绘图。
边界：
    仅离线运行，不进入run_sim.py或run_real.py的实时控制循环。
"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import rospkg


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

from kinematics import (
    FOOT_RADIUS,
    HIP_XYZ,
    JOINT_AXIS_SIGNS,
    LEG_NAMES,
    Q_STAND,
    THIGH_TO_KNEE_LENGTH,
    GraspKinematic,
)
from utils import CONTROL_DOF_NAMES, transform_points


# 使用当前仿真 URDF 的关节限位。
ROS_PACKAGES = rospkg.RosPack()
URDF_PATH = (
    Path(ROS_PACKAGES.get_path("grasp_hexapod_description"))
    / "urdf"
    / "hexapod_isaacgym_view.urdf"
)
WORKSPACE_BOUNDARY_PATH = (
    Path(ROS_PACKAGES.get_path("grasp_hexapod_control"))
    / "config"
    / "workspace_bounds.csv"
)

SAMPLES_PER_LEG = 12000
JOINT_MARGIN = np.deg2rad(5.0)       # 不贴着舵机机械限位工作
MIN_SIGMA = 0.01                     # 雅可比最小奇异值，单位m/rad
GAIT_BETA_LIMIT = np.deg2rad(35.0)   # 共同向外扇区：±40°限位减5°余量
MIN_GAIT_RHO = THIGH_TO_KNEE_LENGTH + FOOT_RADIUS
MAX_GAIT_Z = -FOOT_RADIUS            # 足球心保持在髋平面下方
WORKSPACE_RHO_MARGIN = 0.002         # 点云包络再向内保留2 mm

LEG_COLORS = (
    "#2563EB", "#06B6D4", "#14B8A6",
    "#F97316", "#EF4444", "#A855F7",
)
BODY_ORDER = np.array([5, 4, 1, 2, 0, 3], dtype=np.int64)


def read_joint_limits():
    """按控制器顺序读取 URDF 关节限位。

    返回:
        六条腿的关节上下限，shape 为 `(6, 3, 2)`。
    """

    root = ET.parse(URDF_PATH).getroot()
    limits = []

    for joint_name in CONTROL_DOF_NAMES:
        joint = root.find(f".//joint[@name='{joint_name}']")
        limit = joint.find("limit")
        limits.append(
            [float(limit.attrib["lower"]), float(limit.attrib["upper"])]
        )

    return np.asarray(limits, dtype=np.float64).reshape(6, 3, 2)


def sample_leg_workspace(kinematic, leg_index, limits, rng):
    """采样一条腿在站立分支上的可用工作空间。

    参数:
        kinematic: 运动学模型。
        leg_index: 腿索引。
        limits: 三个关节的上下限。
        rng: 随机数生成器。

    返回:
        可用足端位置、对应奇异值和可达样本数。
    """

    lower = limits[:, 0] + JOINT_MARGIN
    upper = limits[:, 1] - JOINT_MARGIN
    q_samples = rng.uniform(lower, upper, size=(SAMPLES_PER_LEG, 3))

    # 复用控制器的正运动学和雅可比。
    positions = np.stack(
        [kinematic.forward_leg(leg_index, q) for q in q_samples]
    )
    jacobians = np.stack(
        [kinematic.jacobian_leg(leg_index, q) for q in q_samples]
    )
    sigma_min = np.linalg.svd(jacobians, compute_uv=False)[:, -1]
    reachable = sigma_min >= MIN_SIGMA

    # 只保留与 Q_STAND 弯曲方向相同的分支。
    theta = JOINT_AXIS_SIGNS[leg_index] * q_samples
    theta_stand = JOINT_AXIS_SIGNS[leg_index] * Q_STAND[leg_index]
    same_branch = (
        (theta[:, 1] * theta_stand[1] > 0.0)
        & (theta[:, 2] * theta_stand[2] > 0.0)
    )

    rho = np.linalg.norm(positions[:, :2], axis=1)
    beta = np.arctan2(positions[:, 1], positions[:, 0])
    gait_candidate = (
        reachable
        & same_branch
        & (rho >= MIN_GAIT_RHO)
        & (np.abs(beta) <= GAIT_BETA_LIMIT)
        & (positions[:, 2] <= MAX_GAIT_Z)
    )

    return (
        positions[gait_candidate],
        sigma_min[gait_candidate],
        reachable.sum(),
    )


def get_leg_chain_base(kinematic, leg_index):
    """获取站立时一条腿各关节在 `base_link` 中的位置。

    参数:
        kinematic: 运动学模型。
        leg_index: 腿索引。

    返回:
        髋、膝、踝和足端球心的位置。
    """

    return kinematic.link_points_base(Q_STAND)[leg_index]


def workspace_envelope(workspaces_hip, bin_count=55):
    """计算不同高度上的径向工作空间边界。

    参数:
        workspaces_hip: 各腿在髋关节坐标中的足端样本。
        bin_count: 高度分箱数量。

    返回:
        高度、最小半径和最大半径。
    """

    points = np.concatenate(workspaces_hip)
    rho = np.linalg.norm(points[:, :2], axis=1)
    z = points[:, 2]
    edges = np.linspace(z.min(), z.max(), bin_count + 1)

    z_values, rho_min, rho_max = [], [], []
    for lower, upper in zip(edges[:-1], edges[1:]):
        mask = (z >= lower) & (z < upper)
        if mask.sum() < 20:
            continue

        # 用分位数去掉少量随机采样的离群点。
        z_values.append(0.5 * (lower + upper))
        rho_min.append(np.percentile(rho[mask], 1.0))
        rho_max.append(np.percentile(rho[mask], 99.0))

    return np.asarray(z_values), np.asarray(rho_min), np.asarray(rho_max)


def save_workspace_boundary(workspaces_hip):
    """保存在线检查使用的高度和半径边界。

    参数:
        workspaces_hip: 各腿在髋关节坐标中的足端样本。

    返回:
        无。
    """

    z, rho_min, rho_max = workspace_envelope(workspaces_hip)
    rho_min += WORKSPACE_RHO_MARGIN
    rho_max -= WORKSPACE_RHO_MARGIN

    # 收缩后没有可用宽度的高度不写入文件。
    valid = rho_min < rho_max
    boundary = np.column_stack((z[valid], rho_min[valid], rho_max[valid]))

    np.savetxt(
        WORKSPACE_BOUNDARY_PATH,
        boundary,
        delimiter=",",
        header="z_hip_m,rho_min_m,rho_max_m",
        comments="# ",
        fmt="%.8f",
    )
    print(f"workspace boundary: {WORKSPACE_BOUNDARY_PATH}")
    print(f"boundary rows: {len(boundary)}")


def set_equal_3d_axes(axis, points):
    """让三维图的三个坐标轴按相同比例显示。

    参数:
        axis: 三维绘图坐标轴。
        points: 用于确定显示范围的点。

    返回:
        无。
    """

    center = 0.5 * (points.min(axis=0) + points.max(axis=0))
    half_range = 0.52 * np.ptp(points, axis=0).max()
    axis.set_xlim(center[0] - half_range, center[0] + half_range)
    axis.set_ylim(center[1] - half_range, center[1] + half_range)
    axis.set_zlim(center[2] - half_range, center[2] + half_range)
    # 较新的 Matplotlib 支持设置三维绘图区比例。
    if hasattr(axis, "set_box_aspect"):
        axis.set_box_aspect((1.0, 1.0, 1.0))


def print_summary(name, points, sigma_min, reachable_count):
    """打印一条腿的工作空间统计信息。

    参数:
        name: 腿名称。
        points: 可用足端位置。
        sigma_min: 对应的最小奇异值。
        reachable_count: 可达样本数。

    返回:
        无。
    """

    rho = np.linalg.norm(points[:, :2], axis=1)
    beta = np.rad2deg(np.arctan2(points[:, 1], points[:, 0]))
    print(
        f"{name}: reachable={reachable_count:5d}, "
        f"gait={len(points):4d}/{SAMPLES_PER_LEG}, "
        f"rho=[{rho.min():.3f}, {rho.max():.3f}] m, "
        f"beta=[{beta.min():.1f}, {beta.max():.1f}] deg, "
        f"z=[{points[:, 2].min():.3f}, {points[:, 2].max():.3f}] m, "
        f"sigma>={sigma_min.min():.3f}"
    )


def visualize_workspace(kinematic, workspaces_hip):
    """绘制整机工作空间、俯视图和径向边界图。

    参数:
        kinematic: 运动学模型。
        workspaces_hip: 各腿在髋关节坐标中的足端样本。

    返回:
        无。
    """

    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.titleweight": "bold",
            "text.color": "#1E293B",
            "axes.labelcolor": "#334155",
        }
    )
    figure = plt.figure(figsize=(16, 9), facecolor="#EEF2F7")
    grid = figure.add_gridspec(
        2, 2, width_ratios=(1.45, 1.0), hspace=0.28, wspace=0.18
    )
    axis_3d = figure.add_subplot(grid[:, 0], projection="3d")
    axis_top = figure.add_subplot(grid[0, 1])
    axis_rz = figure.add_subplot(grid[1, 1])

    for axis in (axis_3d, axis_top, axis_rz):
        axis.set_facecolor("#F8FAFC")

    workspaces_base = []
    for leg_index, (name, color, points_hip) in enumerate(
        zip(LEG_NAMES, LEG_COLORS, workspaces_hip)
    ):
        points_base = transform_points(
            kinematic.base_from_hip[leg_index],
            points_hip,
        )
        workspaces_base.append(points_base)
        chain = get_leg_chain_base(kinematic, leg_index)

        # 点云表示工作空间，实线表示站立姿态的连杆。
        axis_3d.scatter(
            *points_base.T,
            s=1.4,
            alpha=0.055,
            color=color,
            depthshade=False,
        )
        axis_3d.plot(
            *chain.T,
            color=color,
            linewidth=3.3,
            solid_capstyle="round",
            label=name,
        )
        axis_3d.scatter(
            *chain.T,
            s=(24, 31, 31, 55),
            facecolors="white",
            edgecolors=color,
            linewidths=1.7,
            depthshade=False,
        )

        axis_top.scatter(
            points_base[:, 0],
            points_base[:, 1],
            s=1.2,
            alpha=0.06,
            color=color,
        )
        axis_top.plot(chain[:, 0], chain[:, 1], color=color, linewidth=2.2)
        axis_top.scatter(
            chain[-1, 0],
            chain[-1, 1],
            s=42,
            facecolors="white",
            edgecolors=color,
            linewidths=1.6,
            zorder=5,
        )
        axis_top.text(
            chain[-1, 0],
            chain[-1, 1],
            f"  {name}",
            color=color,
            weight="bold",
        )

    # 用髋关节位置画出机身轮廓。
    body = HIP_XYZ[BODY_ORDER]
    axis_3d.add_collection3d(
        Poly3DCollection(
            [body],
            facecolor="#334155",
            edgecolor="#0F172A",
            linewidth=1.7,
            alpha=0.88,
        )
    )
    axis_top.fill(
        body[:, 0],
        body[:, 1],
        facecolor="#334155",
        edgecolor="#0F172A",
        linewidth=1.5,
        alpha=0.88,
        zorder=4,
    )
    axis_3d.scatter(
        0.0, 0.0, 0.0,
        color="#FACC15",
        edgecolor="#0F172A",
        s=55,
        depthshade=False,
    )
    axis_top.scatter(
        0.0, 0.0,
        color="#FACC15",
        edgecolor="#0F172A",
        s=45,
        zorder=6,
    )

    # 足端球的最低点确定站立时的地面高度。
    stand_feet = kinematic.forward_base(Q_STAND)
    ground_z = stand_feet[:, 2].mean() - FOOT_RADIUS
    ground_x, ground_y = np.meshgrid(
        np.linspace(-0.32, 0.32, 9),
        np.linspace(-0.32, 0.32, 9),
    )
    axis_3d.plot_surface(
        ground_x,
        ground_y,
        np.full_like(ground_x, ground_z),
        color="#94A3B8",
        edgecolor="#CBD5E1",
        linewidth=0.35,
        alpha=0.13,
        shade=False,
    )

    # 显示项目统一使用的坐标方向。
    arrow = 0.065
    for vector, color, label in (
        ((arrow, 0.0, 0.0), "#DC2626", "+x"),
        ((0.0, arrow, 0.0), "#16A34A", "+y"),
        ((0.0, 0.0, arrow), "#2563EB", "+z"),
    ):
        axis_3d.quiver(
            0.0, 0.0, 0.0,
            *vector,
            color=color,
            linewidth=2.2,
            arrow_length_ratio=0.18,
        )
        axis_3d.text(*vector, label, color=color)

    all_base = np.concatenate(workspaces_base)
    set_equal_3d_axes(axis_3d, all_base)
    axis_3d.set(
        xlabel="base x / m",
        ylabel="base y / m",
        zlabel="base z / m",
        title=(
            "Grasp Hexapod — Gait Workspace\n"
            "Q_STAND branch + joint margin + singularity reserve"
        ),
    )
    axis_3d.view_init(elev=27, azim=-52)
    axis_3d.legend(loc="upper left", ncol=2, framealpha=0.92)

    # 俯视图显示各腿工作空间是否重叠。
    margin = 0.02
    axis_top.set_xlim(all_base[:, 0].min() - margin, all_base[:, 0].max() + margin)
    axis_top.set_ylim(all_base[:, 1].min() - margin, all_base[:, 1].max() + margin)
    axis_top.set_aspect("equal")
    axis_top.set(
        xlabel="+x right / m",
        ylabel="+y forward / m",
        title="Top View — Six Outward Sectors",
    )

    # 径向图突出不同高度下的伸展范围。
    all_hip = np.concatenate(workspaces_hip)
    all_rho = np.linalg.norm(all_hip[:, :2], axis=1)
    axis_rz.scatter(
        all_rho,
        all_hip[:, 2],
        s=2.0,
        color="#64748B",
        alpha=0.08,
        label="feasible samples",
    )
    z, rho_min, rho_max = workspace_envelope(workspaces_hip)
    axis_rz.fill_betweenx(
        z,
        rho_min,
        rho_max,
        color="#60A5FA",
        alpha=0.24,
        label="sampled envelope",
    )
    axis_rz.plot(rho_min, z, color="#2563EB", linewidth=1.8)
    axis_rz.plot(rho_max, z, color="#2563EB", linewidth=1.8)

    stand_hip = kinematic.forward(Q_STAND)
    stand_rho = np.linalg.norm(stand_hip[:, :2], axis=1).mean()
    axis_rz.scatter(
        stand_rho,
        stand_hip[:, 2].mean(),
        marker="*",
        s=145,
        color="#FACC15",
        edgecolor="#0F172A",
        zorder=6,
        label="Q_STAND",
    )
    axis_rz.set(
        xlabel=r"radial distance $\rho=\sqrt{x^2+y^2}$ / m",
        ylabel="hip-frame z / m",
        title=r"Local Side View — $\rho$-$z$ Envelope",
    )
    axis_rz.legend(framealpha=0.92)

    for axis in (axis_top, axis_rz):
        axis.grid(color="#CBD5E1", linewidth=0.7, alpha=0.55)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    figure.suptitle(
        "Offline Workspace Analysis",
        fontsize=17,
        weight="bold",
        y=0.985,
    )
    figure.text(
        0.5,
        0.012,
        "Kinematic gait candidates only — exact link collision and "
        "support stability are checked separately.",
        ha="center",
        color="#64748B",
        fontsize=9,
    )
    return figure


def main():
    print(f"URDF: {URDF_PATH}")
    print(
        f"joint margin={np.rad2deg(JOINT_MARGIN):.1f} deg, "
        f"sigma limit={MIN_SIGMA:.3f} m/rad, "
        f"beta limit=+/-{np.rad2deg(GAIT_BETA_LIMIT):.1f} deg"
    )

    kinematic = GraspKinematic()
    limits = read_joint_limits()
    rng = np.random.default_rng(7)
    workspaces_hip = []

    for leg_index, name in enumerate(LEG_NAMES):
        points, sigma_min, reachable_count = sample_leg_workspace(
            kinematic,
            leg_index,
            limits[leg_index],
            rng,
        )
        workspaces_hip.append(points)
        print_summary(name, points, sigma_min, reachable_count)

    save_workspace_boundary(workspaces_hip)
    visualize_workspace(kinematic, workspaces_hip)
    plt.show()


if __name__ == "__main__":
    main()
