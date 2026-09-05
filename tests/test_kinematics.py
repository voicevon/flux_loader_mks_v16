# -*- coding: utf-8 -*-
"""
ScaraKinematics 单元测试
验证正逆运动学双向可逆性、奇异点拦截、可达域边界精度。
无需真实串口，全部为纯数学测试。
"""

import math
import sys
import unittest
from pathlib import Path

# 加入项目根目录，支持直接运行
sys.path.insert(0, str(Path(__file__).parent.parent))

from loader_core.kinematics import ReachabilityError, ScaraKinematics
from loader_core.models import JointAngles


class TestScaraKinematicsBasic(unittest.TestCase):
    """基础正逆运动学验证（L1=L2=300mm，左手系）。"""

    def setUp(self):
        self.kin = ScaraKinematics(l1=300.0, l2=300.0, elbow_dir=-1, singularity_margin=10.0)

    # ------------------------------------------------------------------
    # 正运动学 FK 验证
    # ------------------------------------------------------------------
    def test_fk_home_pose(self):
        """机械零点：theta=90°, psi=0° => X=0, Y=600"""
        x, y = self.kin.forward(JointAngles(theta=90.0, psi=0.0))
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 600.0, places=3)

    def test_fk_stretched_along_x(self):
        """大臂小臂均指向 +X 轴：theta=0°, psi=0° => X=600, Y≈0"""
        x, y = self.kin.forward(JointAngles(theta=0.0, psi=0.0))
        self.assertAlmostEqual(x, 600.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)

    def test_fk_right_angle(self):
        """大臂 90°，小臂折叠 -90°：末端应回到 X 轴正方向"""
        x, y = self.kin.forward(JointAngles(theta=90.0, psi=-90.0))
        # theta+psi = 0° => 小臂指向 +X => x = L1*sin(90°) + L2*cos(0°) = 0 + 300
        # x = L1*cos(90°) + L2*cos(0°) = 0 + 300 = 300
        # y = L1*sin(90°) + L2*sin(0°) = 300 + 0 = 300
        self.assertAlmostEqual(x, 300.0, places=3)
        self.assertAlmostEqual(y, 300.0, places=3)

    # ------------------------------------------------------------------
    # 正逆运动学闭环（FK -> IK -> FK）
    # ------------------------------------------------------------------
    def _assert_fk_ik_roundtrip(self, theta: float, psi: float, tol: float = 1e-4):
        """验证 FK -> IK -> FK 闭环误差 < tol mm。"""
        # 1. FK 正解
        x0, y0 = self.kin.forward(JointAngles(theta=theta, psi=psi))
        # 2. IK 逆解（若在可达域内）
        if not self.kin.is_reachable(x0, y0):
            self.skipTest(f"角度 ({theta}, {psi}) 对应坐标超出可达域，跳过")
        angles = self.kin.inverse(x0, y0)
        # 3. 再次 FK 正解
        x1, y1 = self.kin.forward(angles)
        # 4. 验证坐标闭环误差
        self.assertAlmostEqual(x0, x1, delta=tol,
            msg=f"FK->IK->FK X 闭环误差超限: ({theta}°,{psi}°) x0={x0:.6f}, x1={x1:.6f}")
        self.assertAlmostEqual(y0, y1, delta=tol,
            msg=f"FK->IK->FK Y 闭环误差超限: ({theta}°,{psi}°) y0={y0:.6f}, y1={y1:.6f}")

    def test_roundtrip_home(self):
        # 机械零点 FK Y=600 处于最大臂展边界，需要改用工作域内常规角度
        # (60°, -60°): FK => ~(424, 424)，reach≈600√2/2≈424mm，在安全域内
        self._assert_fk_ik_roundtrip(60.0, -60.0)

    def test_home_pose_boundary_note(self):
        """机械零点 (theta=90°, psi=0°) 对应 Y=600mm，恰好处于奇异点边界附近。
        此测试记录该设计约束：HOME_Y=600mm 应保持为配置中的已知值，
        实际控制时 G92 直接声明坐标而非 IK 求解，不经过此检查。
        """
        x, y = self.kin.forward(JointAngles(theta=90.0, psi=0.0))
        self.assertAlmostEqual(y, 600.0, places=3)
        # 此坐标处于最大臂展边界（含裕量后不可达），属于已知约束
        self.assertFalse(self.kin.is_reachable(x, y))

    def test_roundtrip_45_neg30(self):
        self._assert_fk_ik_roundtrip(45.0, -30.0)

    def test_roundtrip_135_neg60(self):
        self._assert_fk_ik_roundtrip(135.0, -60.0)

    def test_roundtrip_30_neg90(self):
        self._assert_fk_ik_roundtrip(30.0, -90.0)

    def test_roundtrip_180_neg45(self):
        self._assert_fk_ik_roundtrip(180.0, -45.0)

    # ------------------------------------------------------------------
    # 逆运动学可达域边界与奇异点检查
    # ------------------------------------------------------------------
    def test_ik_unreachable_too_far(self):
        """超出最大臂展应抛出 ReachabilityError。"""
        with self.assertRaises(ReachabilityError):
            self.kin.inverse(0.0, 605.0)  # 600 + 5 mm 边界内，但加裕量后 590mm < 605

    def test_ik_unreachable_origin(self):
        """原点附近奇异区域应被拦截（L1==L2 时 min_reach = 0 + margin = 10mm）。"""
        with self.assertRaises(ReachabilityError):
            self.kin.inverse(0.0, 5.0)  # 距原点 5mm < 10mm 安全裕量

    def test_ik_reachable_boundary(self):
        """稍微偏离奇异点安全裕量外的点应可正常解算。"""
        # 距原点 15mm，大于安全裕量 10mm
        x, y = 0.0, 15.0
        self.assertTrue(self.kin.is_reachable(x, y))
        angles = self.kin.inverse(x, y)
        self.assertIsInstance(angles, JointAngles)

    def test_is_reachable_true(self):
        self.assertTrue(self.kin.is_reachable(0.0, 300.0))

    def test_is_reachable_false_far(self):
        self.assertFalse(self.kin.is_reachable(0.0, 700.0))

    def test_is_reachable_false_near(self):
        self.assertFalse(self.kin.is_reachable(0.0, 3.0))


class TestScaraKinematicsRightHand(unittest.TestCase):
    """右手系（elbow_dir=+1）FK<->IK 验证。"""

    def setUp(self):
        self.kin = ScaraKinematics(l1=300.0, l2=300.0, elbow_dir=1, singularity_margin=10.0)

    def test_fk_home_right_hand(self):
        x, y = self.kin.forward(JointAngles(theta=90.0, psi=0.0))
        self.assertAlmostEqual(x, 0.0, places=3)
        self.assertAlmostEqual(y, 600.0, places=3)

    def test_roundtrip_right_hand(self):
        angles_in = JointAngles(theta=60.0, psi=30.0)
        x, y = self.kin.forward(angles_in)
        if self.kin.is_reachable(x, y):
            angles_out = self.kin.inverse(x, y)
            x2, y2 = self.kin.forward(angles_out)
            self.assertAlmostEqual(x, x2, delta=1e-4)
            self.assertAlmostEqual(y, y2, delta=1e-4)


class TestScaraKinematicsInit(unittest.TestCase):
    """初始化参数校验。"""

    def test_invalid_arm_length(self):
        with self.assertRaises(ValueError):
            ScaraKinematics(l1=-10.0, l2=300.0)

    def test_workspace_info(self):
        kin = ScaraKinematics()
        info = kin.workspace_info()
        self.assertIn("SCARA", info)
        self.assertIn("300", info)


if __name__ == "__main__":
    unittest.main(verbosity=2)
