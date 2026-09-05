# -*- coding: utf-8 -*-
"""
SCARA 运动学数学引擎 (Pure Math — No Hardware Dependencies)
ScaraKinematics 是纯计算类，零串口/硬件依赖，可独立进行单元测试。

正运动学 (FK): JointAngles → (X, Y)
逆运动学 (IK): (X, Y)    → JointAngles

评审修正 #1：IK 奇异点检查增加安全裕量，防止 L1==L2 时近原点区域数值奇异。
"""

from __future__ import annotations
import math
from typing import Tuple

from .models import JointAngles


class ReachabilityError(ValueError):
    """目标点超出工作空间可达域时抛出。"""


class ScaraKinematics:
    """SCARA 平面二连杆正逆运动学解算器。

    Args:
        l1: 大臂长度 (mm)
        l2: 小臂长度 (mm)
        elbow_dir: 手肘方向，-1 左手 / +1 右手（对齐 Marlin SCARA_ELBOW_DIR）
        singularity_margin: 与极限可达半径的最小安全距离 (mm)，
                            防止 L1==L2 时原点附近产生数值奇异 [评审 #1]
    """

    def __init__(
        self,
        l1: float = 300.0,
        l2: float = 300.0,
        elbow_dir: int = -1,
        singularity_margin: float = 10.0,
    ) -> None:
        if l1 <= 0 or l2 <= 0:
            raise ValueError(f"臂长必须为正数，当前 l1={l1}, l2={l2}")
        self.l1 = l1
        self.l2 = l2
        self.elbow_dir = elbow_dir
        self.singularity_margin = singularity_margin

        # 预计算可达域边界（含裕量）
        self._max_reach = l1 + l2 - singularity_margin
        self._min_reach = abs(l1 - l2) + singularity_margin

    # ------------------------------------------------------------------
    # 正运动学 FK
    # ------------------------------------------------------------------
    def forward(self, angles: JointAngles) -> Tuple[float, float]:
        """正运动学：关节角 → 笛卡尔坐标。

        Args:
            angles: JointAngles(theta, psi)，均为度数

        Returns:
            (x, y) 末端笛卡尔坐标 (mm)
        """
        a = math.radians(angles.theta)
        b = math.radians(angles.theta + angles.psi)
        x = self.l1 * math.cos(a) + self.l2 * math.cos(b)
        y = self.l1 * math.sin(a) + self.l2 * math.sin(b)
        return x, y

    # ------------------------------------------------------------------
    # 逆运动学 IK
    # ------------------------------------------------------------------
    def inverse(self, x: float, y: float) -> JointAngles:
        """逆运动学：笛卡尔坐标 → 关节角。

        带可达域校验与奇异点安全裕量检查 [评审 #1]。

        Args:
            x: 目标 X 坐标 (mm)
            y: 目标 Y 坐标 (mm)

        Returns:
            JointAngles(theta, psi)，均为度数

        Raises:
            ReachabilityError: 目标点超出安全可达域时抛出，含详细提示
        """
        reach = math.hypot(x, y)

        if reach > self._max_reach:
            raise ReachabilityError(
                f"目标点 ({x:.1f}, {y:.1f}) 超出最大可达半径 "
                f"{self._max_reach:.1f} mm（臂展 {self.l1 + self.l2:.1f} mm，"
                f"含 {self.singularity_margin:.1f} mm 奇异点安全裕量）"
            )
        if reach < self._min_reach:
            raise ReachabilityError(
                f"目标点 ({x:.1f}, {y:.1f}) 进入奇异区域，"
                f"距原点 {reach:.1f} mm < 最小安全半径 {self._min_reach:.1f} mm "
                f"（|L1-L2|={abs(self.l1 - self.l2):.1f} mm，"
                f"含 {self.singularity_margin:.1f} mm 安全裕量）"
            )

        hypot2 = x * x + y * y
        cos_psi = (hypot2 - (self.l1 ** 2 + self.l2 ** 2)) / (2.0 * self.l1 * self.l2)
        # 数值截断，防止浮点误差导致 acos 域外
        cos_psi = max(-1.0, min(1.0, cos_psi))
        psi = math.acos(cos_psi) * self.elbow_dir

        k1 = self.l1 + self.l2 * cos_psi
        k2 = self.l2 * math.sin(psi)
        theta = math.atan2(k1 * y - k2 * x, k1 * x + k2 * y)

        return JointAngles(
            theta=math.degrees(theta),
            psi=math.degrees(psi),
        )

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------
    def is_reachable(self, x: float, y: float) -> bool:
        """判断目标点是否在安全可达域内（不抛出异常）。"""
        reach = math.hypot(x, y)
        return self._min_reach <= reach <= self._max_reach

    def workspace_info(self) -> str:
        """返回工作空间描述字符串，用于日志与诊断。"""
        return (
            f"SCARA 工作空间: L1={self.l1}mm L2={self.l2}mm "
            f"手肘={'左手' if self.elbow_dir == -1 else '右手'} "
            f"可达半径 [{self._min_reach:.1f}, {self._max_reach:.1f}] mm "
            f"(奇异点裕量 ±{self.singularity_margin:.1f} mm)"
        )
