# -*- coding: utf-8 -*-
"""
点动控制器 (Jog Controller)
将按键事件映射为 ScaraRobot 点动指令，封装步长管理逻辑。
完整保留原 loader_cli.py 中的 W/S/A/D/U/J/Q/E/O/L/I/K 按键映射。
"""

from __future__ import annotations

import logging
from typing import Dict, Optional

from .robot import ScaraRobot

logger = logging.getLogger(__name__)


class StepProfile:
    """步长档位配置（线性轴 mm，旋转轴/关节轴 °）。"""
    PRESETS: Dict[str, tuple] = {
        "1": (1.0, 1.0),    # 精细步长
        "2": (10.0, 5.0),   # 默认步长
        "3": (50.0, 15.0),  # 粗调步长
    }


class JogController:
    """点动步长与键位调度器。

    支持的按键（大小写不敏感）：
      笛卡尔点动:
        W/S  -> Y 轴 +/-    A/D  -> X 轴 -/+
        U/J  -> Z 轴 +/-    Q/E  -> R 轴 +/-
      关节点动 (用于排查电机物理转向):
        O/L  -> 大臂 Theta +/-
        I/K  -> 小臂 Psi   +/-
      步长切换:
        1    -> 1mm / 1°
        2    -> 10mm / 5°
        3    -> 50mm / 15°
    """

    def __init__(self, robot: ScaraRobot) -> None:
        self._robot = robot
        self.step_linear_mm: float = 10.0   # 线性轴步长 (mm)
        self.step_rot_deg: float = 5.0      # 旋转/关节步长 (°)

    def set_step_profile(self, key: str) -> bool:
        """切换步长档位 ('1'/'2'/'3')。

        Returns:
            True 表示切换成功
        """
        if key not in StepProfile.PRESETS:
            return False
        linear, rot = StepProfile.PRESETS[key]
        self.step_linear_mm = linear
        self.step_rot_deg = rot
        logger.info("步长更新: 线性 %.1f mm / 旋转 %.1f°", linear, rot)
        return True

    def handle_key(self, key: str) -> bool:
        """分发单次按键事件。

        Args:
            key: 单字符按键（大小写不敏感）

        Returns:
            True 表示该按键已处理，False 表示未识别
        """
        k = key.strip().lower()
        robot = self._robot

        # 步长切换
        if k in StepProfile.PRESETS:
            return self.set_step_profile(k)

        # 笛卡尔点动
        if k == "w":
            logger.info("点动: Y +%.1f mm", self.step_linear_mm)
            robot.jog_cartesian("y", +self.step_linear_mm)
        elif k == "s":
            logger.info("点动: Y -%.1f mm", self.step_linear_mm)
            robot.jog_cartesian("y", -self.step_linear_mm)
        elif k == "d":
            logger.info("点动: X +%.1f mm", self.step_linear_mm)
            robot.jog_cartesian("x", +self.step_linear_mm)
        elif k == "a":
            logger.info("点动: X -%.1f mm", self.step_linear_mm)
            robot.jog_cartesian("x", -self.step_linear_mm)
        elif k == "u":
            logger.info("点动: Z +%.1f mm", self.step_linear_mm)
            robot.jog_cartesian("z", +self.step_linear_mm)
        elif k == "j":
            logger.info("点动: Z -%.1f mm", self.step_linear_mm)
            robot.jog_cartesian("z", -self.step_linear_mm)
        elif k == "q":
            logger.info("点动: R +%.1f°", self.step_rot_deg)
            robot.jog_cartesian("r", +self.step_rot_deg)
        elif k == "e":
            logger.info("点动: R -%.1f°", self.step_rot_deg)
            robot.jog_cartesian("r", -self.step_rot_deg)

        # 关节点动
        elif k == "o":
            logger.info(
                "关节点动: 大臂(Theta) +%.1f° -> 俯视期望逆时针(CCW)转动", self.step_rot_deg
            )
            robot.jog_joint("theta", +self.step_rot_deg)
        elif k == "l":
            logger.info(
                "关节点动: 大臂(Theta) -%.1f° -> 俯视期望顺时针(CW)转动", self.step_rot_deg
            )
            robot.jog_joint("theta", -self.step_rot_deg)
        elif k == "i":
            logger.info(
                "关节点动: 小臂(Psi) +%.1f° -> 俯视期望逆时针(CCW)偏折", self.step_rot_deg
            )
            robot.jog_joint("psi", +self.step_rot_deg)
        elif k == "k":
            logger.info(
                "关节点动: 小臂(Psi) -%.1f° -> 俯视期望顺时针(CW)偏折", self.step_rot_deg
            )
            robot.jog_joint("psi", -self.step_rot_deg)
        else:
            return False

        return True

    @property
    def step_info(self) -> str:
        """返回当前步长描述字符串（用于 UI 显示）。"""
        return f"线性: {self.step_linear_mm} mm  |  关节/旋转: {self.step_rot_deg}°"
