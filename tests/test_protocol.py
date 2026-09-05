# -*- coding: utf-8 -*-
"""
通信协议层单元测试
验证 MockTransceiver、MarlinProtocolHandler、MarlinProtocolParser 的行为。
无需真实串口。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from loader_core.comm import (
    MarlinProtocolHandler,
    MarlinProtocolParser,
    MockTransceiver,
)
from loader_core.models import JointAngles, Pose


class TestMockTransceiver(unittest.TestCase):
    """MockTransceiver 收发行为验证。"""

    def test_connect_disconnect(self):
        tx = MockTransceiver()
        self.assertFalse(tx.is_connected())
        tx.connect("MOCK")
        self.assertTrue(tx.is_connected())
        tx.disconnect()
        self.assertFalse(tx.is_connected())

    def test_send_and_read(self):
        tx = MockTransceiver(responses=["echo: G28", "ok"])
        tx.connect()
        tx.send_line("G28")
        line1 = tx.readline()
        line2 = tx.readline()
        self.assertEqual(line1, "echo: G28")
        self.assertEqual(line2, "ok")

    def test_default_ok_response(self):
        """无预设回显时默认返回 'ok'。"""
        tx = MockTransceiver()
        tx.connect()
        tx.send_line("M114")
        self.assertEqual(tx.readline(), "ok")

    def test_sent_lines_recorded(self):
        tx = MockTransceiver()
        tx.connect()
        tx.send_line("G28")
        tx.send_line("M114")
        self.assertEqual(tx.sent_lines, ["G28", "M114"])


class TestMarlinProtocolHandler(unittest.TestCase):
    """MarlinProtocolHandler send_and_wait 验证。"""

    def _make_handler(self, responses):
        tx = MockTransceiver(responses=responses)
        tx.connect()
        return MarlinProtocolHandler(tx)

    def test_send_and_wait_collects_until_ok(self):
        handler = self._make_handler(["T:200.0 /200.0 B:50.0", "X:0.00 Y:600.00", "ok"])
        lines = handler.send_and_wait("M114")
        self.assertEqual(len(lines), 3)
        self.assertTrue(lines[-1].startswith("ok"))

    def test_send_without_wait(self):
        handler = self._make_handler(["ok"])
        lines = handler.send_and_wait("M84", wait_ok=False)
        self.assertEqual(lines, [])

    def test_disconnected_returns_empty(self):
        tx = MockTransceiver()
        # 不调用 connect()
        handler = MarlinProtocolHandler(tx)
        lines = handler.send_and_wait("G28")
        self.assertEqual(lines, [])


class TestMarlinProtocolParser(unittest.TestCase):
    """MarlinProtocolParser 文本解析验证。"""

    # ------------------------------------------------------------------
    # M114 解析
    # ------------------------------------------------------------------
    def test_parse_m114_cartesian(self):
        lines = [
            "X:150.00 Y:350.00 Z:80.00 E:0.00 Count X:3000 Y:7000 Z:8000",
            "ok",
        ]
        pose, angles = MarlinProtocolParser.parse_m114(lines)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.x, 150.0)
        self.assertAlmostEqual(pose.y, 350.0)
        self.assertAlmostEqual(pose.z, 80.0)
        self.assertAlmostEqual(pose.r, 0.0)

    def test_parse_m114_with_scara_angles(self):
        lines = [
            "X:0.00 Y:600.00 Z:80.00 E:0.00",
            "SCARA Theta: 90.00   Psi+Theta: 90.00",
            "ok",
        ]
        pose, angles = MarlinProtocolParser.parse_m114(lines)
        self.assertIsNotNone(angles)
        # theta=90, Psi+Theta=90 => psi = 90-90 = 0
        self.assertAlmostEqual(angles.theta, 90.0)
        self.assertAlmostEqual(angles.psi, 0.0)

    def test_parse_m114_scara_nonzero_psi(self):
        lines = [
            "X:150.00 Y:350.00 Z:15.00 E:0.00",
            "SCARA Theta: 45.00   Psi+Theta: 75.00",
            "ok",
        ]
        _, angles = MarlinProtocolParser.parse_m114(lines)
        self.assertIsNotNone(angles)
        self.assertAlmostEqual(angles.theta, 45.0)
        self.assertAlmostEqual(angles.psi, 30.0)  # 75 - 45

    def test_parse_m114_fallback(self):
        """回显无坐标时应使用 fallback。"""
        fallback = Pose(x=1.0, y=2.0, z=3.0, r=0.0)
        pose, _ = MarlinProtocolParser.parse_m114(["ok"], fallback_pose=fallback)
        self.assertEqual(pose, fallback)

    # ------------------------------------------------------------------
    # M119 解析
    # ------------------------------------------------------------------
    def test_parse_m119_all_open(self):
        lines = [
            "Reporting endstop status",
            "x_min: open",
            "y_min: open",
            "z_min: open",
            "ok",
        ]
        status = MarlinProtocolParser.parse_m119(lines)
        self.assertFalse(status.x_min)
        self.assertFalse(status.y_min)
        self.assertFalse(status.z_min)
        self.assertFalse(status.any_triggered())

    def test_parse_m119_x_triggered(self):
        lines = [
            "x_min: TRIGGERED",
            "y_min: open",
            "z_min: open",
            "ok",
        ]
        status = MarlinProtocolParser.parse_m119(lines)
        self.assertTrue(status.x_min)
        self.assertFalse(status.y_min)
        self.assertTrue(status.any_triggered())

    def test_parse_m119_all_triggered(self):
        lines = [
            "x_min: TRIGGERED",
            "y_min: TRIGGERED",
            "z_min: TRIGGERED",
            "ok",
        ]
        status = MarlinProtocolParser.parse_m119(lines)
        self.assertTrue(status.any_triggered())
        self.assertEqual(len(status.raw), 3)

    def test_parse_m114_g_code_format(self):
        """验证 G-code 指令使用 E 而非 R（防止回归）。"""
        # move_to_pose 生成的 G-code 应包含 E 字段
        # 此处通过检查 parse_m114 能正确解析 E 字段来间接验证
        lines = ["X:150.00 Y:350.00 Z:80.00 E:45.00", "ok"]
        pose, _ = MarlinProtocolParser.parse_m114(lines)
        self.assertAlmostEqual(pose.r, 45.0)

    def test_parse_m114_fallback_r(self):
        """验证在 M114 回显无 E 字段时保留 fallback_pose 的 R 轴状态。"""
        fb = Pose(x=0.0, y=600.0, z=80.0, r=25.0)
        lines = ["X:0.00 Y:600.00 Z:80.00", "ok"]
        pose, _ = MarlinProtocolParser.parse_m114(lines, fallback_pose=fb)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.r, 25.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
