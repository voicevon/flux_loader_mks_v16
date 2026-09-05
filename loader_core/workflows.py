# -*- coding: utf-8 -*-
"""
作业流程宏层 (Workflow Macros)
将抓取-搬运-释放时序逻辑抽离为独立作业类，支持参数化工作点。

评审 #9 修正：工作点坐标（pick_pose / drop_pose）通过构造参数传入，
不在类内部硬编码，支持不同工位灵活配置。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Optional

from .models import Pose
from .robot import ScaraRobot

logger = logging.getLogger(__name__)

# 默认工作点位（保持与原 loader_cli.py 一致，可通过构造参数覆盖）
_DEFAULT_PICK_POSE = Pose(x=150.0, y=350.0, z=15.0, r=0.0)
_DEFAULT_SAFE_Z = 80.0
_DEFAULT_DROP_POSE = Pose(x=-150.0, y=350.0, z=80.0, r=30.0)


@dataclass
class PickAndPlaceConfig:
    """抓取-搬运工作流参数（评审 #9：替代硬编码工作点）。

    Attributes:
        pick_pose:      抓取点位姿（XY 为上方位置，Z 为下探抓取高度）
        drop_pose:      释放点位姿
        safe_z_mm:      安全提升高度 (mm)，XY 移动时 Z 先升至此高度
        grip_delay_s:   夹爪闭合后等待时间 (s)，确保物料夹紧
        release_delay_s:夹爪打开后等待时间 (s)
        lift_delay_s:   提升动作后稳定等待时间 (s)
    """
    pick_pose: Pose = field(default_factory=lambda: _DEFAULT_PICK_POSE)
    drop_pose: Pose = field(default_factory=lambda: _DEFAULT_DROP_POSE)
    safe_z_mm: float = _DEFAULT_SAFE_Z
    grip_delay_s: float = 0.6
    release_delay_s: float = 0.5
    lift_delay_s: float = 0.4


class PickAndPlaceWorkflow:
    """芦笋抓取-搬运-释放单次循环工作流。

    使用方式::
        cfg = PickAndPlaceConfig(
            pick_pose=Pose(x=150, y=350, z=15, r=0),
            drop_pose=Pose(x=-150, y=350, z=80, r=30),
        )
        workflow = PickAndPlaceWorkflow(robot, cfg)
        workflow.run()
    """

    def __init__(
        self,
        robot: ScaraRobot,
        config: Optional[PickAndPlaceConfig] = None,
    ) -> None:
        self._robot = robot
        self._cfg = config or PickAndPlaceConfig()

    def run(self) -> bool:
        """执行完整的单次抓取-搬运-释放循环。

        Returns:
            True 表示完整执行成功，False 表示被中断或失败。
        """
        cfg = self._cfg
        robot = self._robot

        logger.info("=" * 50)
        logger.info("开始执行【芦笋单次搬运节拍宏测试】")
        logger.info("=" * 50)
        t_start = time.monotonic()

        try:
            # 步骤 1: 提升至安全高度并打开双夹爪
            logger.info("[步骤 1/7] 提升 Z 至安全高度 %.1f mm，打开夹爪...", cfg.safe_z_mm)
            robot.set_z_height(cfg.safe_z_mm)
            robot.gripper.set_both_grippers(open_state=True)
            time.sleep(cfg.release_delay_s)

            # 步骤 2: 快速移动至抓取工位上方 (Z 保持安全高度)
            pick_xy = Pose(
                x=cfg.pick_pose.x,
                y=cfg.pick_pose.y,
                z=cfg.safe_z_mm,
                r=cfg.pick_pose.r,
                f=3500.0,
            )
            logger.info("[步骤 2/7] 移动至抓取工位上方: X=%.1f Y=%.1f", pick_xy.x, pick_xy.y)
            robot.move_to_pose(pick_xy)

            # 步骤 3: 下探至抓取高度
            logger.info("[步骤 3/7] 下探至物料高度 Z=%.1f mm...", cfg.pick_pose.z)
            robot.set_z_height(cfg.pick_pose.z)
            time.sleep(0.3)

            # 步骤 4: 双爪闭合抓紧
            logger.info("[步骤 4/7] 双爪闭合抓紧物料...")
            robot.gripper.set_both_grippers(open_state=False)
            time.sleep(cfg.grip_delay_s)

            # 步骤 5: 提升至安全高度
            logger.info("[步骤 5/7] 提起物料至安全高度 %.1f mm...", cfg.safe_z_mm)
            robot.set_z_height(cfg.safe_z_mm)
            time.sleep(cfg.lift_delay_s)

            # 步骤 6: 搬运至落料工位
            drop_xy = Pose(
                x=cfg.drop_pose.x,
                y=cfg.drop_pose.y,
                z=cfg.safe_z_mm,
                r=cfg.drop_pose.r,
                f=3500.0,
            )
            logger.info("[步骤 6/7] 搬运至分发入口: X=%.1f Y=%.1f R=%.1f", drop_xy.x, drop_xy.y, drop_xy.r)
            robot.move_to_pose(drop_xy)
            time.sleep(0.3)

            # 步骤 7: 打开夹爪释放物料
            logger.info("[步骤 7/7] 打开夹爪释放芦笋...")
            robot.gripper.set_both_grippers(open_state=True)
            time.sleep(cfg.release_delay_s)

            elapsed = time.monotonic() - t_start
            logger.info("=" * 50)
            logger.info("单次搬运循环完成！总耗时: %.2f 秒", elapsed)
            logger.info("=" * 50)
            return True

        except KeyboardInterrupt:
            logger.warning("用户中断了搬运宏测试！")
            return False
        except Exception as exc:
            logger.error("搬运宏执行异常: %s", exc, exc_info=True)
            return False
