# -*- coding: utf-8 -*-
"""
集中式硬件配置层 (Configuration)
LoaderConfig 汇总所有机械臂几何参数、电气映射与运动约束，
作为整个系统的唯一配置真相来源 (Single Source of Truth)。
"""

from __future__ import annotations
from dataclasses import dataclass, field

from .models import Pose, JointAngles


@dataclass
class LoaderConfig:
    """Flux Loader (SCARA) 机械臂集中式硬件配置。

    所有字段均为可变属性，支持在运行时通过工厂方法或直接赋值覆盖。
    默认值对齐 MKS Base V1.6 + mechanical_structure.md 的设计规范。
    """

    # ------------------------------------------------------------------
    # SCARA 几何参数 (mm)
    # ------------------------------------------------------------------
    l1: float = 300.0           # 大臂长度 (Joint 1 → Joint 2)
    l2: float = 300.0           # 小臂长度 (Joint 2 → End Effector)
    elbow_dir: int = -1         # 手肘方向: -1 左手系 / +1 右手系，对齐 Configuration.h SCARA_ELBOW_DIR

    # ------------------------------------------------------------------
    # 机械零位（大臂 90°、小臂 0° 伸直指向 +Y 轴，Z 轴安全高度 80 mm）
    # ------------------------------------------------------------------
    home_pose: Pose = field(
        default_factory=lambda: Pose(x=0.0, y=600.0, z=80.0, r=0.0)
    )
    home_angles: JointAngles = field(
        default_factory=lambda: JointAngles(theta=90.0, psi=0.0)
    )

    # ------------------------------------------------------------------
    # 运动约束
    # ------------------------------------------------------------------
    default_feedrate: float = 3000.0    # 默认笛卡尔进给率 (mm/min)
    joint_jog_feedrate: float = 6000.0  # 关节点动默认进给率 (deg/min)
    joint_theta_jog_feedrate: float = 6000.0  # 大臂点动进给率 (deg/min) = 100 deg/s (基准速度50°/s的2倍)
    joint_psi_jog_feedrate: float = 15000.0   # 小臂点动进给率 (deg/min) = 250 deg/s (基准速度50°/s的5倍)
    z_min_mm: float = 0.0               # Z 轴最低安全高度 (mm)
    z_max_mm: float = 100.0             # Z 轴最高行程 (mm)
    z_servo_angle_at_min: float = 270.0 # Z = z_min_mm (最低工作位) 对应的舵机物理角度 (°)
    z_servo_angle_at_max: float = 0.0   # Z = z_max_mm (最高安全位) 对应的舵机物理角度 (°)
    z_servo_angle_max: float = 270.0    # 舵机满行程物理转角 (兼容旧字段)

    # 奇异点安全裕量：IK 求解时目标点与极限可达半径的最小距离 (mm)
    # 避免在 l1==l2 时近原点区域 (两臂完全重叠) 产生数值奇异 (#1 评审修正)
    ik_singularity_margin_mm: float = 10.0

    # ------------------------------------------------------------------
    # 舵机通道 (M280 Pxx 编号，对应 MKS Base 板载 Servo 引脚)
    # ------------------------------------------------------------------
    z_servo_id: int = 0          # Z 轴升降舵机 (Servo 0, A11/D65)
    gripper1_id: int = 1         # 夹爪 1 — 芦笋头端 (Servo 1, D11)
    gripper2_id: int = 2         # 夹爪 2 — 芦笋尾端 (Servo 2, D12)
    gripper_open_angle: int = 0  # 夹爪打开角度 (°)
    gripper_close_angle: int = 90  # 夹爪闭合角度 (°)

    # ------------------------------------------------------------------
    # 串口通信参数
    # ------------------------------------------------------------------
    default_baudrate: int = 115200
    serial_timeout: float = 1.0   # 单次 readline 超时 (s)
    connect_wait_s: float = 2.5   # DTR 复位后等待 Bootloader 启动时间 (s)

    # ------------------------------------------------------------------
    # 预设点位持久化路径
    # ------------------------------------------------------------------
    presets_file: str = "~/.flux_loader/presets.json"  # #4 评审修正：明确 JSON 后端
