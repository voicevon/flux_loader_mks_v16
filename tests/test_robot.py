# -*- coding: utf-8 -*-
"""
Robot 逻辑层单元测试 (tests/test_robot.py)
验证 ScaraRobot 核心动作、R 轴独立驱动 (G6)、点动与坐标同步。
"""

from __future__ import annotations
import unittest
from typing import List

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from loader_core.models import Pose, JointAngles
from loader_core.config import LoaderConfig
from loader_core.comm import MarlinProtocolHandler
from loader_core.robot import ScaraRobot


class MockRecordingTransceiver:
    """记录所发送所有指令的 Mock 收发器。"""

    def __init__(self) -> None:
        self.sent_commands: List[str] = []
        self._connected = True
        self._mock_responses: List[str] = []

    def connect(self, *args, **kwargs) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def send_line(self, line: str) -> None:
        self.sent_commands.append(line.strip())

    def readline(self, timeout: float = 1.0) -> str:
        if self._mock_responses:
            return self._mock_responses.pop(0)
        return "ok"

    def flush_buffers(self) -> None:
        pass


class TestScaraRobotRDirect(unittest.TestCase):
    """验证 R 轴独立驱动 (G6) 与点动逻辑。"""

    def setUp(self) -> None:
        self.tx = MockRecordingTransceiver()
        self.handler = MarlinProtocolHandler(self.tx)
        self.config = LoaderConfig()
        self.robot = ScaraRobot(self.handler, self.config)

    def test_move_r_direct_sends_g6(self):
        """move_r_direct 应发送 G6 E<r> F<feedrate>，不发 G1。"""
        self.robot.current_pose = Pose(x=0.0, y=600.0, z=80.0, r=0.0)
        self.tx.sent_commands.clear()

        # 模拟 M114 返回最新 E 轴
        self.tx._mock_responses = [
            "X:0.00 Y:600.00 Z:80.00 E:15.00",
            "ok",
        ]

        ret = self.robot.move_r_direct(15.0)
        self.assertTrue(ret)

        # 检查是否包含 G6 E15.00
        g6_cmds = [c for c in self.tx.sent_commands if c.startswith("G6 E15.00")]
        self.assertTrue(len(g6_cmds) > 0, f"实际发送指令: {self.tx.sent_commands}")
        self.assertNotIn("G1", " ".join(self.tx.sent_commands))
        self.assertAlmostEqual(self.robot.current_pose.r, 15.0)

    def test_jog_cartesian_r_bypasses_g1(self):
        """jog_cartesian('r', +10) 应直接调用 G6 且避开奇异点 G1。"""
        self.robot.current_pose = Pose(x=0.0, y=600.0, z=80.0, r=20.0)
        self.tx.sent_commands.clear()

        self.tx._mock_responses = [
            "X:0.00 Y:600.00 Z:80.00 E:30.00",
            "ok",
        ]

        self.robot.jog_cartesian("r", 10.0)

        # 期望目标为 20 + 10 = 30
        g6_cmds = [c for c in self.tx.sent_commands if c.startswith("G6 E30.00")]
        self.assertTrue(len(g6_cmds) > 0, f"实际发送指令: {self.tx.sent_commands}")
        self.assertNotIn("G1", " ".join(self.tx.sent_commands))
        self.assertAlmostEqual(self.robot.current_pose.r, 30.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
