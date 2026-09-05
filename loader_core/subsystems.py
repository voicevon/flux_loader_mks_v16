# -*- coding: utf-8 -*-
"""
硬件子系统层 (Hardware Subsystems)
管理末端执行器相关的硬件子系统：Z 轴升降舵机与夹爪舵机。

评审 #6：set_z_height 同时发送 G1 Z + M280 P0 两条指令，
保留双重机制的原因：
  - G1 Z：确保 Marlin 内部坐标系与物理位置同步（防止下次绝对坐标移动跳变）。
  - M280 P0：直接驱动 PWM 精准定位舵机角度（G1 Z 在 SCARA 固件中
    有时因 Z 轴规划缓冲导致实际到位偏慢，M280 可作精确补偿）。
两条指令均需保留，不可只保留其中一条。
"""

from __future__ import annotations
import logging
from typing import Optional

from .comm import MarlinProtocolHandler
from .config import LoaderConfig

logger = logging.getLogger(__name__)


class GripperSubsystem:
    """夹爪与 Z 轴升降舵机子系统。

    负责 Servo 0 (Z 轴升降)、Servo 1/2 (夹爪) 的控制。
    所有角度换算与 G-code 生成封装在此类内，外部以物理量（mm、开/闭状态）交互。
    """

    def __init__(self, handler: MarlinProtocolHandler, config: LoaderConfig) -> None:
        self._h = handler
        self._cfg = config
        # 记录当前 Z 高度（内存状态，由 set_z_height 维护）
        self._current_z_mm: float = config.home_pose.z

    # ------------------------------------------------------------------
    # Z 轴升降
    # ------------------------------------------------------------------
    def set_z_height(self, z_mm: float) -> None:
        """设置 Z 轴高度 (0 ~ z_max_mm mm)。

        双重指令机制（保留原因详见模块文档 [评审 #6]）：
          1. G1 Z<val>：同步 Marlin 坐标系
          2. M280 P<id> S<angle>：直接精准驱动 PWM 舵机

        Args:
            z_mm: 目标高度 (mm)，自动限幅到 [z_min_mm, z_max_mm]
        """
        z_clamped = max(self._cfg.z_min_mm, min(self._cfg.z_max_mm, z_mm))
        z_span = self._cfg.z_max_mm - self._cfg.z_min_mm
        ratio = (z_clamped - self._cfg.z_min_mm) / z_span if z_span > 0 else 0.0
        servo_angle = self._cfg.z_servo_angle_at_min + ratio * (
            self._cfg.z_servo_angle_at_max - self._cfg.z_servo_angle_at_min
        )

        logger.info(
            "Z 轴升降: 目标 %.1f mm -> 限幅后 %.1f mm (舵机角度 %.1f°)",
            z_mm, z_clamped, servo_angle,
        )

        # 1. 精准驱动 PWM 舵机执行物理升降 (Servo 0，杜绝向步进驱动发脉冲)
        self._h.send_and_wait(
            f"M280 P{self._cfg.z_servo_id} S{servo_angle:.0f}",
            timeout=5.0,
        )
        # 2. 纯坐标状态标记 (G92 不会驱动任何步进电机，仅同步 Marlin 内部坐标计数)
        self._h.send_and_wait(
            f"G92 Z{z_clamped:.2f}",
            timeout=5.0,
        )

        self._current_z_mm = z_clamped

    @property
    def current_z_mm(self) -> float:
        """当前 Z 轴高度内存状态 (mm)。"""
        return self._current_z_mm

    # ------------------------------------------------------------------
    # 夹爪控制
    # ------------------------------------------------------------------
    def set_gripper(self, gripper_id: int, open_state: bool) -> None:
        """控制单个夹爪舵机。

        Args:
            gripper_id: 1=芦笋头端 (Servo 1), 2=芦笋尾端 (Servo 2)
            open_state: True=打开(0°), False=闭合(90°)
        """
        angle = self._cfg.gripper_open_angle if open_state else self._cfg.gripper_close_angle
        state_str = f"打开({self._cfg.gripper_open_angle}°)" if open_state else f"闭合({self._cfg.gripper_close_angle}°)"
        logger.info("夹爪 %d %s -> M280 P%d S%d", gripper_id, state_str, gripper_id, angle)
        self._h.send_and_wait(f"M280 P{gripper_id} S{angle}", timeout=3.0)

    def set_both_grippers(self, open_state: bool) -> None:
        """同时控制两个夹爪（先 1 后 2）。

        Args:
            open_state: True=双爪打开, False=双爪闭合
        """
        self.set_gripper(self._cfg.gripper1_id, open_state)
        self.set_gripper(self._cfg.gripper2_id, open_state)
