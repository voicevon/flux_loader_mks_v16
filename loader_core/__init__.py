# -*- coding: utf-8 -*-
"""
loader_core — Flux Loader SCARA 机械臂核心控制库

对外公开 API（外部视觉算法或自动化脚本直接导入）：

    from loader_core import ScaraRobot, LoaderConfig, Pose, JointAngles
    from loader_core.comm import SerialTransceiver, MarlinProtocolHandler

内部实现模块（不对外暴露）：
    MarlinProtocolParser, GripperSubsystem, StepProfile
"""

from .models import JointAngles, LimitSwitchStatus, Pose
from .config import LoaderConfig
from .kinematics import ScaraKinematics, ReachabilityError
from .comm import (
    ITransceiver,
    SerialTransceiver,
    MockTransceiver,
    MarlinProtocolHandler,
)
from .subsystems import GripperSubsystem
from .robot import ScaraRobot
from .workflows import PickAndPlaceWorkflow, PickAndPlaceConfig
from .jog import JogController

__all__ = [
    # 领域模型
    "Pose",
    "JointAngles",
    "LimitSwitchStatus",
    # 配置
    "LoaderConfig",
    # 运动学
    "ScaraKinematics",
    "ReachabilityError",
    # 通信
    "ITransceiver",
    "SerialTransceiver",
    "MockTransceiver",
    "MarlinProtocolHandler",
    # 子系统
    "GripperSubsystem",
    # 核心实体
    "ScaraRobot",
    # 工作流
    "PickAndPlaceWorkflow",
    "PickAndPlaceConfig",
    # 交互
    "JogController",
]
