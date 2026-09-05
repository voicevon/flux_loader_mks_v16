# -*- coding: utf-8 -*-
"""
通信与协议解析层 (Communication & Protocol)

评审 #2 修正：严格分离传输接口与 Marlin 协议语义：
  - ITransceiver    : 纯传输抽象接口，仅定义字节级 send/readline，
                      不包含任何 Marlin ok/协议判断逻辑。
  - SerialTransceiver: 基于 pyserial 的物理串口实现。
  - MockTransceiver  : 单元测试用仿真收发器，不依赖真实串口。
  - MarlinProtocolHandler: 包装 ITransceiver，承担 ok 握手等待、
                            超时控制，与具体传输方式解耦。
  - MarlinProtocolParser : 纯静态文本解析器，解析 M114/M119 回显。
"""

from __future__ import annotations

import logging
import re
import time
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

from .models import JointAngles, LimitSwitchStatus, Pose

logger = logging.getLogger(__name__)


# ==============================================================================
# 1. 纯传输抽象接口 ITransceiver
#    职责：字节级收发，不含任何 Marlin 协议语义。
# ==============================================================================
class ITransceiver(ABC):
    """纯传输层抽象接口。

    实现类只负责"把字节发出去、把字节读回来"，
    不得包含任何 Marlin ok/协议握手判断逻辑。[评审 #2]
    """

    @abstractmethod
    def connect(self, port: str, baudrate: int, **kwargs) -> bool:
        """建立物理连接。成功返回 True。"""

    @abstractmethod
    def disconnect(self) -> None:
        """断开物理连接。"""

    @abstractmethod
    def is_connected(self) -> bool:
        """返回当前连接状态。"""

    @abstractmethod
    def send_line(self, line: str) -> None:
        """向设备发送一行文本（实现类负责追加 \\n 并 flush）。"""

    @abstractmethod
    def readline(self, timeout: float = 1.0) -> str:
        """从设备读取一行文本（不含 \\n），超时返回空字符串。"""

    @abstractmethod
    def flush_buffers(self) -> None:
        """清空收发缓冲区（用于复位后状态刷新）。"""


# ==============================================================================
# 2. 物理串口实现 SerialTransceiver
# ==============================================================================
class SerialTransceiver(ITransceiver):
    """基于 pyserial 的物理串口收发器。

    包含 DTR 复位等待、自动缓冲清空，不含任何协议解析。
    """

    def __init__(self) -> None:
        self._ser = None  # serial.Serial 实例

    def connect(self, port: str, baudrate: int = 115200, timeout: float = 1.0, wait_s: float = 2.5) -> bool:
        try:
            import serial
            logger.info("正在连接串口 %s (波特率: %d)...", port, baudrate)
            self._ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout,
            )
            # DTR 触发 ATmega2560 复位，等待 Bootloader 启动完成
            logger.info("等待控制板初始化 (%.1f 秒)...", wait_s)
            time.sleep(wait_s)
            self.flush_buffers()
            logger.info("串口 %s 连接成功。", port)
            return True
        except Exception as exc:
            logger.error("串口 %s 连接失败: %s", port, exc)
            self._ser = None
            return False

    def disconnect(self) -> None:
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        logger.info("串口已断开。")

    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def send_line(self, line: str) -> None:
        if not self.is_connected():
            raise ConnectionError("串口未连接，无法发送指令。")
        try:
            import serial
            data = (line.strip() + "\n").encode("utf-8")
            self._ser.write(data)
            self._ser.flush()
            logger.debug(">> %s", line.strip())
        except Exception as exc:
            logger.error("发送失败: %s", exc)
            self.disconnect()
            raise

    def readline(self, timeout: float = 1.0) -> str:
        if not self.is_connected():
            return ""
        try:
            line = self._ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                logger.debug("<< %s", line)
            return line
        except Exception as exc:
            logger.error("读取失败: %s", exc)
            self.disconnect()
            return ""

    def flush_buffers(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

    @staticmethod
    def list_ports() -> List[str]:
        """列出系统所有可用串口名称。"""
        try:
            import serial.tools.list_ports
            return [p.device for p in serial.tools.list_ports.comports()]
        except ImportError:
            return []


# ==============================================================================
# 3. 仿真收发器 MockTransceiver（单元测试用）
# ==============================================================================
class MockTransceiver(ITransceiver):
    """仿真收发器：不依赖真实串口，用于单元测试与离线开发。

    支持预设回显序列，按调用顺序返回。
    """

    def __init__(self, responses: Optional[List[str]] = None) -> None:
        self._connected = False
        self._sent_lines: List[str] = []
        # 预设回显队列；若为空则默认回复 "ok"
        self._responses: List[str] = list(responses or [])

    def connect(self, port: str = "MOCK", baudrate: int = 115200, **kwargs) -> bool:
        self._connected = True
        logger.info("[Mock] 仿真连接已建立 (port=%s)", port)
        return True

    def disconnect(self) -> None:
        self._connected = False
        logger.info("[Mock] 仿真连接已断开。")

    def is_connected(self) -> bool:
        return self._connected

    def send_line(self, line: str) -> None:
        self._sent_lines.append(line.strip())
        logger.debug("[Mock] >> %s", line.strip())

    def readline(self, timeout: float = 1.0) -> str:
        if self._responses:
            return self._responses.pop(0)
        return "ok"

    def flush_buffers(self) -> None:
        pass

    @property
    def sent_lines(self) -> List[str]:
        """返回已发送的所有指令行（测试断言用）。"""
        return list(self._sent_lines)


# ==============================================================================
# 4. Marlin 协议处理器 MarlinProtocolHandler
#    职责：包装 ITransceiver，承担 ok 握手等待与超时控制。
#    [评审 #2] ok 等待属于 Marlin 协议语义，不放在 ITransceiver 中。
# ==============================================================================
class MarlinProtocolHandler:
    """Marlin 协议处理器：包装纯传输层，提供 send_and_wait 语义。

    将 ok 握手判定逻辑集中在此处，ITransceiver 实现类无需知道 Marlin 协议。
    """

    def __init__(self, transceiver: ITransceiver) -> None:
        self._tx = transceiver

    @property
    def transceiver(self) -> ITransceiver:
        return self._tx

    def send_and_wait(
        self,
        cmd: str,
        timeout: float = 10.0,
        wait_ok: bool = True,
    ) -> List[str]:
        """发送一条 G-code 指令并收集回显行，直到出现 'ok' 或超时。

        Args:
            cmd:      G-code 指令字符串（可不含换行）
            timeout:  最长等待时间 (s)
            wait_ok:  False 时发送后立即返回，不等待 ok

        Returns:
            回显行列表（含 ok 行）
        """
        if not self._tx.is_connected():
            logger.warning("设备未连接，跳过指令: %s", cmd)
            return []

        logger.info(">> %s", cmd.strip())
        self._tx.send_line(cmd)
        if not wait_ok:
            return []

        lines: List[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._tx.readline(timeout=min(1.0, deadline - time.monotonic()))
            if not line:
                continue
            lines.append(line)
            logger.debug("<< %s", line)
            if line.startswith("ok"):
                logger.info("<< %s", line)
                break
        else:
            logger.warning("指令 '%s' 等待 ok 超时 (%.1fs)", cmd.strip(), timeout)

        return lines


# ==============================================================================
# 5. Marlin 回显文本解析器 MarlinProtocolParser（纯静态工具）
# ==============================================================================
class MarlinProtocolParser:
    """Marlin 串口回显文本解析器（纯静态方法，无状态）。

    所有方法均为 @staticmethod，可直接调用，无需实例化。
    """

    # 正则：M114 回显中的笛卡尔坐标
    _RE_X = re.compile(r"X:([-+]?\d*\.?\d+)")
    _RE_Y = re.compile(r"Y:([-+]?\d*\.?\d+)")
    _RE_Z = re.compile(r"Z:([-+]?\d*\.?\d+)")
    # Marlin SCARA 固件将 R 轴（末端旋转步进）映射为 E (Extruder) 轴
    _RE_R = re.compile(r"[RE]:([-+]?\d*\.?\d+)")
    # SCARA Theta / Psi 或 Psi+Theta 行
    _RE_SCARA = re.compile(
        r"SCARA Theta:\s*([-+]?\d*\.?\d+)\s+(Psi\+Theta|Psi):\s*([-+]?\d*\.?\d+)"
    )
    # M119 限位状态行
    _RE_LIMIT = re.compile(r"(\w[\w_ ]+):\s*(TRIGGERED|open)", re.IGNORECASE)

    @classmethod
    def parse_m114(
        cls,
        lines: List[str],
        fallback_pose: Optional[Pose] = None,
        fallback_angles: Optional[JointAngles] = None,
    ) -> Tuple[Optional[Pose], Optional[JointAngles]]:
        """解析 M114 回显，提取笛卡尔位姿与关节角。

        Args:
            lines:           M114 回显行列表
            fallback_pose:   若解析失败时返回的默认 Pose
            fallback_angles: 若解析失败时返回的默认 JointAngles

        Returns:
            (Pose | None, JointAngles | None)
        """
        x = y = z = r = None
        theta = psi = None

        for line in lines:
            if x is None:
                m = cls._RE_X.search(line)
                if m:
                    x = float(m.group(1))
            if y is None:
                m = cls._RE_Y.search(line)
                if m:
                    y = float(m.group(1))
            if z is None:
                m = cls._RE_Z.search(line)
                if m:
                    z = float(m.group(1))
            if r is None:
                m = cls._RE_R.search(line)
                if m:
                    r = float(m.group(1))

            m_scara = cls._RE_SCARA.search(line)
            if m_scara:
                theta = float(m_scara.group(1))
                psi_label = m_scara.group(2)
                psi_val = float(m_scara.group(3))
                if "Theta" in psi_label:
                    psi = psi_val - theta  # 固件若报告 Psi+Theta，需减去 Theta
                else:
                    psi = psi_val          # 固件标准输出即为 Psi 本身

        pose: Optional[Pose] = None
        if x is not None and y is not None and z is not None:
            default_r = fallback_pose.r if fallback_pose is not None else 0.0
            pose = Pose(x=x, y=y, z=z, r=r if r is not None else default_r)
        else:
            pose = fallback_pose

        angles: Optional[JointAngles] = None
        if theta is not None and psi is not None:
            angles = JointAngles(theta=theta, psi=psi)
        else:
            angles = fallback_angles

        return pose, angles

    @classmethod
    def parse_m119(cls, lines: List[str]) -> LimitSwitchStatus:
        """解析 M119 回显，提取各轴限位开关状态。

        Args:
            lines: M119 回显行列表

        Returns:
            LimitSwitchStatus 对象
        """
        raw: Dict[str, str] = {}
        for line in lines:
            m = cls._RE_LIMIT.search(line)
            if m:
                key = m.group(1).strip().lower()
                val = m.group(2).strip()
                raw[key] = val

        def is_triggered(key: str) -> bool:
            return raw.get(key, "").upper() == "TRIGGERED"

        return LimitSwitchStatus(
            x_min=is_triggered("x_min"),
            y_min=is_triggered("y_min"),
            z_min=is_triggered("z_min"),
            raw=raw,
        )
