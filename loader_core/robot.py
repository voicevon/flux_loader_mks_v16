# -*- coding: utf-8 -*-
"""
机械臂核心领域实体 (ScaraRobot Aggregate Root)

ScaraRobot 是整个系统的核心聚合根，组装：
  - MarlinProtocolHandler（通信与握手）
  - ScaraKinematics（运动学计算）
  - GripperSubsystem（末端执行器）

评审修正：
  #3: 每次 move_* 成功后必须调用 M114 回读刷新 current_pose/current_angles，
      不依赖客户端推算值，防止点动累计漂移。
  #5: home() 内部完整包含 G28 + G92 时序，外部调用者无需手动补 G92。
  #7: 全部使用 logging，不使用 print()。
"""

from __future__ import annotations

import logging
from typing import Optional

from .comm import MarlinProtocolHandler, MarlinProtocolParser
from .config import LoaderConfig
from .kinematics import ReachabilityError, ScaraKinematics
from .models import JointAngles, LimitSwitchStatus, Pose
from .subsystems import GripperSubsystem

logger = logging.getLogger(__name__)

# G28 回零最长等待时间 (s)，SCARA 双臂回零行程较长
_HOME_TIMEOUT = 60.0
# 普通 G1 运动最长等待时间 (s)
_MOVE_TIMEOUT = 20.0
# M114 查询超时 (s)
_QUERY_TIMEOUT = 5.0


class ScaraRobot:
    """SCARA 机械臂核心领域实体（聚合根）。

    对外提供高层级机器人控制接口，内部封装通信、运动学与子系统细节。
    可被 CLI 层、工作流宏层、以及未来视觉算法直接导入使用。

    Example::
        from loader_core import ScaraRobot, LoaderConfig
        from loader_core.comm import SerialTransceiver, MarlinProtocolHandler

        tx = SerialTransceiver()
        handler = MarlinProtocolHandler(tx)
        robot = ScaraRobot(handler, LoaderConfig())
        robot.connect("COM11")
        robot.home()
        robot.move_to_pose(Pose(x=150, y=350, z=80, r=0))
    """

    def __init__(
        self,
        handler: MarlinProtocolHandler,
        config: Optional[LoaderConfig] = None,
        kinematics: Optional[ScaraKinematics] = None,
    ) -> None:
        self._h = handler
        self._cfg = config or LoaderConfig()
        self._kin = kinematics or ScaraKinematics(
            l1=self._cfg.l1,
            l2=self._cfg.l2,
            elbow_dir=self._cfg.elbow_dir,
            singularity_margin=self._cfg.ik_singularity_margin_mm,
        )
        self.gripper = GripperSubsystem(handler, self._cfg)

        # 内存状态（通过 M114 同步刷新，不依赖推算）
        self.current_pose: Pose = self._cfg.home_pose
        self.current_angles: JointAngles = self._cfg.home_angles

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------
    def connect(self, port: str) -> bool:
        """建立串口连接并完成初始化基准同步。"""
        ok = self._h.transceiver.connect(
            port=port,
            baudrate=self._cfg.default_baudrate,
            timeout=self._cfg.serial_timeout,
            wait_s=self._cfg.connect_wait_s,
        )
        if not ok:
            return False
        self._initialize_reference()
        return True

    def disconnect(self) -> None:
        """安全断开串口连接。"""
        self._h.transceiver.disconnect()

    def is_connected(self) -> bool:
        return self._h.transceiver.is_connected()

    # ------------------------------------------------------------------
    # 初始化（内部）
    # ------------------------------------------------------------------
    def _initialize_reference(self) -> None:
        """复位后执行基准坐标同步。

        时序：
          1. G92 — 将当前物理位置声明为机械零点坐标（不移动电机）
          2. M302 P1 — 允许冷挤出，使末端 E 轴旋转步进电机无需加热即可受控
          3. M114 — 回读当前坐标，同步内存状态
        """
        cfg = self._cfg
        hp = cfg.home_pose
        logger.info("发送基准坐标同步 G92 (X=%.2f Y=%.2f Z=%.2f E=%.2f)", hp.x, hp.y, hp.z, hp.r)
        self._h.send_and_wait(
            f"G92 X{hp.x:.2f} Y{hp.y:.2f} Z{hp.z:.2f} E{hp.r:.2f}",
            timeout=_QUERY_TIMEOUT,
        )
        logger.info("允许冷挤出 M302 P1")
        self._h.send_and_wait("M302 P1", timeout=_QUERY_TIMEOUT)
        self._sync_state()
        logger.info("初始化完成: %s | %s", self.current_pose, self.current_angles)

    # ------------------------------------------------------------------
    # 状态同步（内部，评审 #3）
    # ------------------------------------------------------------------
    def _sync_state(self) -> None:
        """发送 M114 并将回显结果同步至 current_pose / current_angles。

        [评审 #3] 每次 move_* 操作后调用此方法，确保内存状态与固件一致，
        不依赖客户端推算，防止点动累积漂移。
        若 M114 回显中无 SCARA 关节角行，则通过 IK 估算补偿。
        """
        lines = self._h.send_and_wait("M114", timeout=_QUERY_TIMEOUT)
        pose, angles = MarlinProtocolParser.parse_m114(
            lines,
            fallback_pose=self.current_pose,
            fallback_angles=self.current_angles,
        )
        if pose:
            self.current_pose = pose
        if angles:
            self.current_angles = angles
        elif pose:
            # M114 无 SCARA 关节角行时，通过 IK 估算
            try:
                self.current_angles = self._kin.inverse(pose.x, pose.y)
                logger.debug("关节角通过 IK 估算: %s", self.current_angles)
            except ReachabilityError:
                logger.warning("IK 估算失败，保留上次关节角状态。")

    # ------------------------------------------------------------------
    # 高层级运动接口
    # ------------------------------------------------------------------
    def home(self) -> bool:
        """执行 G28 三轴回零，回零后完成 G92 基准对齐。

        [评审 #5] G28 + G92 时序完整封装在 home() 内部，
        外部调用者无需手动补 G92，防止遗漏导致坐标漂移。

        Returns:
            True 表示成功
        """
        logger.info("执行 G28 三轴回零 (X/Y/R)，请确认行程无障碍...")
        lines = self._h.send_and_wait("G28", timeout=_HOME_TIMEOUT)
        for line in lines:
            logger.debug("G28 >> %s", line)

        # 回零后将固件坐标对齐机械零点（G28 结束位置即机械零点）
        hp = self._cfg.home_pose
        logger.info("G28 完成，发送 G92 对齐机械零点坐标...")
        self._h.send_and_wait(
            f"G92 X{hp.x:.2f} Y{hp.y:.2f} Z{hp.z:.2f} E{hp.r:.2f}",
            timeout=_QUERY_TIMEOUT,
        )
        self._sync_state()
        logger.info("回零完成: %s", self.current_pose)
        return True

    def move_to_pose(self, pose: Pose) -> bool:
        """笛卡尔空间绝对移动 (G1 X Y Z E F)。

        Args:
            pose: 目标位姿（Pose 对象）

        Returns:
            True 表示指令发送成功
        """
        feedrate = pose.f if pose.f is not None else self._cfg.default_feedrate
        cmd = (
            f"G1 X{pose.x:.2f} Y{pose.y:.2f} Z{pose.z:.2f} "
            f"E{pose.r:.2f} F{feedrate:.0f}"
        )
        logger.info("笛卡尔移动: %s", cmd)
        self._h.send_and_wait(cmd, timeout=_MOVE_TIMEOUT)
        self._sync_state()  # [评审 #3] 移动后必须回读同步
        return True

    def move_joint_direct(self, angles: JointAngles, feedrate: Optional[float] = None) -> bool:
        """关节空间直接移动 (G6)：绕过逆运动学解算，直接按电机角度驱动。

        Args:
            angles:   目标关节角 (JointAngles)
            feedrate: 进给率，None 则使用默认值

        Returns:
            True 表示指令发送成功
        """
        f = feedrate if feedrate is not None else self._cfg.joint_jog_feedrate
        cmd = f"G6 T{angles.theta:.2f} P{angles.psi:.2f} F{f:.0f}"
        logger.info("关节直接驱动 (G6): %s", cmd)
        self._h.send_and_wait(cmd, timeout=_MOVE_TIMEOUT)
        # 立即更新上位机内存角度与位姿，确保连续点动不被旧状态拦截
        self.current_angles = angles
        try:
            x, y = self._kin.forward(angles)
            self.current_pose = Pose(x=x, y=y, z=self.current_pose.z, r=self.current_pose.r)
        except Exception as e:
            logger.warning("正解更新位姿失败: %s", e)
        self._sync_state()
        return True

    def move_to_angles(self, angles: JointAngles, feedrate: Optional[float] = None) -> bool:
        """关节空间移动：正运动学解算后发送 G1。

        Args:
            angles:   目标关节角 (JointAngles)
            feedrate: 进给率，None 则使用默认值

        Returns:
            True 表示指令发送成功
        """
        x, y = self._kin.forward(angles)
        f = feedrate if feedrate is not None else self._cfg.default_feedrate
        logger.info("关节移动: %s -> 正解 (X=%.2f, Y=%.2f)", angles, x, y)
        pose = Pose(x=x, y=y, z=self.current_pose.z, r=self.current_pose.r, f=f)
        return self.move_to_pose(pose)

    def jog_cartesian(self, axis: str, delta: float) -> None:
        """笛卡尔单轴增量点动。

        Args:
            axis:  轴名称，'x' / 'y' / 'z' / 'r'（大小写不敏感）
            delta: 移动增量 (mm 或 °)
        """
        p = self.current_pose
        axis = axis.lower()
        if axis == "x":
            new_pose = Pose(x=p.x + delta, y=p.y, z=p.z, r=p.r)
        elif axis == "y":
            new_pose = Pose(x=p.x, y=p.y + delta, z=p.z, r=p.r)
        elif axis == "z":
            # Z 轴点动通过 GripperSubsystem 管理（双重指令）
            self.gripper.set_z_height(p.z + delta)
            self._sync_state()
            return
        elif axis == "r":
            new_pose = Pose(x=p.x, y=p.y, z=p.z, r=p.r + delta)
        else:
            raise ValueError(f"不支持的轴名称: '{axis}'，有效值: x/y/z/r")
        self.move_to_pose(new_pose)

    def jog_joint(self, joint: str, delta_deg: float) -> None:
        """关节独立增量点动（直接发送 G6 绕过逆解算，纯关节空间控制）。

        大臂与小臂分别采用独立设定的高速进给率（大臂600°/s，小臂1500°/s）。

        Args:
            joint:     关节名称，'theta' (大臂) / 'psi' (小臂)（大小写不敏感）
            delta_deg: 角度增量 (°)
        """
        a = self.current_angles
        joint = joint.lower()
        if joint == "theta":
            new_angles = JointAngles(theta=a.theta + delta_deg, psi=a.psi)
            feedrate = self._cfg.joint_theta_jog_feedrate
        elif joint == "psi":
            new_angles = JointAngles(theta=a.theta, psi=a.psi + delta_deg)
            feedrate = self._cfg.joint_psi_jog_feedrate
        else:
            raise ValueError(f"不支持的关节名称: '{joint}'，有效值: theta/psi")
        self.move_joint_direct(new_angles, feedrate=feedrate)

    def set_z_height(self, z_mm: float) -> None:
        """设置 Z 轴高度（代理到 GripperSubsystem）。"""
        self.gripper.set_z_height(z_mm)
        self._sync_state()

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------
    def get_limit_status(self) -> LimitSwitchStatus:
        """发送 M119 并返回结构化限位开关状态。"""
        lines = self._h.send_and_wait("M119", timeout=_QUERY_TIMEOUT)
        status = MarlinProtocolParser.parse_m119(lines)
        logger.info("限位状态: %s", status)
        return status

    def refresh_state(self) -> None:
        """手动触发 M114 状态刷新（对应菜单 [p] 刷新坐标）。"""
        self._sync_state()
        logger.info("坐标已刷新: %s | %s", self.current_pose, self.current_angles)

    # ------------------------------------------------------------------
    # 系统维护
    # ------------------------------------------------------------------
    def set_coordinate_origin(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0, r: float = 0.0
    ) -> None:
        """强制将当前物理位置设定为指定坐标（G92，不移动电机）。"""
        cmd = f"G92 X{x:.2f} Y{y:.2f} Z{z:.2f} E{r:.2f}"
        logger.info("G92 重设坐标原点: %s", cmd)
        self._h.send_and_wait(cmd, timeout=_QUERY_TIMEOUT)
        self._sync_state()

    def disable_steppers(self) -> None:
        """释放步进电机使能 (M84)，允许手动轻推关节。"""
        logger.info("发送 M84 释放电机使能")
        self._h.send_and_wait("M84", timeout=_QUERY_TIMEOUT)

    def send_raw(self, cmd: str) -> list:
        """透传原生 G-code 指令（用于 G-code 终端模式）。"""
        return self._h.send_and_wait(cmd, timeout=_MOVE_TIMEOUT)

    @property
    def config(self) -> LoaderConfig:
        return self._cfg

    @property
    def kinematics(self) -> ScaraKinematics:
        return self._kin
