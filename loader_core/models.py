# -*- coding: utf-8 -*-
"""
领域模型层 (Domain Models)
定义 Pose、JointAngles、LimitSwitchStatus 三个强类型不可变值对象。
所有模型均使用 frozen dataclass，保证线程安全与可哈希性。
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Dict


@dataclass(frozen=True)
class Pose:
    """笛卡尔空间绝对位姿（不可变值对象）。

    Attributes:
        x: X 轴坐标 (mm)
        y: Y 轴坐标 (mm)
        z: Z 轴坐标 (mm)，对应 Z 轴线性舵机高度
        r: 末端旋转轴角度 (°)，Marlin 内部映射为 E (Extruder) 轴
        f: 运动进给率 (mm/min)，None 表示沿用上下文默认值
    """
    x: float
    y: float
    z: float
    r: float = 0.0
    f: Optional[float] = None

    def with_offset(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        dr: float = 0.0,
    ) -> "Pose":
        """返回在当前位姿基础上叠加偏移量后的新 Pose（原对象不变）。"""
        return Pose(
            x=self.x + dx,
            y=self.y + dy,
            z=self.z + dz,
            r=self.r + dr,
            f=self.f,
        )

    def __str__(self) -> str:
        f_str = f"  F={self.f:.0f}" if self.f is not None else ""
        return f"Pose(X={self.x:.2f}, Y={self.y:.2f}, Z={self.z:.2f}, R={self.r:.2f}{f_str})"


@dataclass(frozen=True)
class JointAngles:
    """关节空间角度（度），不可变值对象。

    Attributes:
        theta: 大臂与 +X 轴正方向夹角 (°)，逆时针为正
        psi:   小臂相对大臂末端的折叠偏角 (°)，逆时针为正
    """
    theta: float  # 大臂 (Joint 1)
    psi: float    # 小臂 (Joint 2)

    def __str__(self) -> str:
        return f"JointAngles(θ={self.theta:.2f}°, ψ={self.psi:.2f}°)"


@dataclass(frozen=True)
class LimitSwitchStatus:
    """各轴限位开关触发状态（不可变值对象）。

    Attributes:
        x_min: X 轴最小限位是否触发
        y_min: Y 轴最小限位是否触发
        z_min: Z 轴最小限位是否触发
        raw:   M119 原始回显行字典（key 为开关名，value 为 "TRIGGERED" / "open"）
    """
    x_min: bool = False
    y_min: bool = False
    z_min: bool = False
    raw: Dict[str, str] = None  # type: ignore[assignment]

    def __post_init__(self):
        # frozen dataclass 需要通过 object.__setattr__ 设置可变默认值
        if self.raw is None:
            object.__setattr__(self, "raw", {})

    def any_triggered(self) -> bool:
        """任意限位被触发时返回 True。"""
        return self.x_min or self.y_min or self.z_min

    def __str__(self) -> str:
        return (
            f"LimitSwitch(x_min={'TRIGGERED' if self.x_min else 'open'}, "
            f"y_min={'TRIGGERED' if self.y_min else 'open'}, "
            f"z_min={'TRIGGERED' if self.z_min else 'open'})"
        )
