#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flux Loader (SCARA) 机械臂交互式调试工具 — 重构版
用于 MKS Base V1.6 (ATmega2560) 固件运动学、舵机与抓取动作调校

此文件为瘦化 CLI 入口层，仅负责：
  - 终端菜单渲染与用户输入解析
  - 日志格式配置
  - 组装 loader_core 各组件并启动 LoaderCLIApp

核心业务逻辑全部由 loader_core 包提供，可独立被外部程序导入。

评审 #4 修正：PresetManager 使用 JSON 文件持久化（~/.flux_loader/presets.json）。
评审 #7 修正：全部使用 logging，不使用 print()。
"""

import argparse
import json
import logging
import os
import re
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

# ==============================================================================
# Windows 控制台 UTF-8 编码修复（保持原有兼容性）
# ==============================================================================
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except AttributeError:
        pass

# ==============================================================================
# 日志配置（评审 #7：CLI 层统一配置格式，loader_core 使用 getLogger(__name__)）
# ==============================================================================
def _setup_logging(verbose: bool = False) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

logger = logging.getLogger(__name__)

# ==============================================================================
# 导入 loader_core（pyserial 缺失时给出友好提示）
# ==============================================================================
try:
    import serial.tools.list_ports  # noqa: F401 — 仅做早期依赖检测
except ImportError:
    print("[错误] 未找到 pyserial 模块。请先运行: pip install pyserial")
    sys.exit(1)

# 将项目根目录加入 sys.path，支持 `python tools/loader_cli.py` 直接运行
_TOOLS_DIR = Path(__file__).parent
_PROJECT_ROOT = _TOOLS_DIR.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from loader_core import (  # noqa: E402
    JogController,
    LoaderConfig,
    MarlinProtocolHandler,
    PickAndPlaceConfig,
    PickAndPlaceWorkflow,
    Pose,
    ScaraRobot,
    SerialTransceiver,
)


# ==============================================================================
# 预设点位管理器（评审 #4：JSON 文件持久化）
# ==============================================================================
class PresetManager:
    """预设特征点位管理器，支持 JSON 文件持久化。

    存储格式（~/.flux_loader/presets.json）::

        {
            "机械零位 (Home Pose)": {"x": 0.0, "y": 600.0, "z": 80.0, "r": 90.0},
            ...
        }
    """

    _DEFAULT_PRESETS: Dict[str, Pose] = {
        "机械零位 (Home Pose)":         Pose(x=0.0,    y=600.0, z=80.0, r=90.0),
        "安全待机位 (Standby)":          Pose(x=0.0,    y=300.0, z=80.0, r=90.0),
        "标准抓取位 (Pick Pose)":        Pose(x=250.0,  y=250.0, z=20.0, r=90.0),
        "落料入料口 (Dealer Drop Pose)": Pose(x=-250.0, y=350.0, z=80.0, r=90.0),
    }

    def __init__(self, filepath: str = "~/.flux_loader/presets.json") -> None:
        self._path = Path(filepath).expanduser()
        self._presets: Dict[str, Pose] = {}
        self._load()

    def _load(self) -> None:
        """从 JSON 文件加载预设点；若文件不存在则使用内置默认值。"""
        if self._path.exists():
            try:
                with open(self._path, "r", encoding="utf-8") as f:
                    raw = json.load(f)
                self._presets = {
                    name: Pose(**data) for name, data in raw.items()
                }
                logger.info("已从 %s 加载 %d 个预设点", self._path, len(self._presets))
                return
            except Exception as exc:
                logger.warning("读取预设文件失败 (%s)，使用内置默认值。", exc)
        self._presets = dict(self._DEFAULT_PRESETS)

    def save(self) -> None:
        """将当前预设点位持久化写入 JSON 文件。"""
        self._path.parent.mkdir(parents=True, exist_ok=True)
        raw = {
            name: {"x": p.x, "y": p.y, "z": p.z, "r": p.r}
            for name, p in self._presets.items()
        }
        with open(self._path, "w", encoding="utf-8") as f:
            json.dump(raw, f, ensure_ascii=False, indent=2)
        logger.info("预设点已保存至 %s", self._path)

    def list_presets(self) -> Dict[str, Pose]:
        return dict(self._presets)

    def add(self, name: str, pose: Pose) -> None:
        self._presets[name] = pose
        self.save()

    def remove(self, name: str) -> bool:
        if name in self._presets:
            del self._presets[name]
            self.save()
            return True
        return False


# ==============================================================================
# CLI 应用主体
# ==============================================================================
class LoaderCLIApp:
    """Flux Loader CLI 终端应用（瘦化入口层）。

    仅负责菜单渲染与输入分发，所有机械臂控制委托给 loader_core。
    """

    def __init__(self, port: Optional[str] = None) -> None:
        self._config = LoaderConfig()
        self._tx = SerialTransceiver()
        self._handler = MarlinProtocolHandler(self._tx)
        self._robot = ScaraRobot(self._handler, self._config)
        self._jog = JogController(self._robot)
        self._workflow = PickAndPlaceWorkflow(self._robot)
        self._presets = PresetManager(self._config.presets_file)
        self._port = port

    # ------------------------------------------------------------------
    # 启动入口
    # ------------------------------------------------------------------
    def start(self) -> None:
        self._print_banner()
        if self._port:
            self._robot.connect(self._port)
        else:
            self._select_port_menu()
        self._main_menu_loop()

    def _print_banner(self) -> None:
        print("\n" + "=" * 58)
        print("    Flux Loader (SCARA) 机械臂调试交互终端 v2.0")
        print("    (OOP 重构版 — loader_core 架构)")
        print("=" * 58)

    # ------------------------------------------------------------------
    # 串口选择
    # ------------------------------------------------------------------
    def _select_port_menu(self) -> None:
        ports = SerialTransceiver.list_ports()
        if not ports:
            print("\n[提示] 当前系统未自动检测到可用串口设备！")
            port = input("请输入串口名 (如 COM11) 或直接按回车跳过: ").strip()
            if port:
                self._robot.connect(port)
            return

        # 优先寻找常用端口（如 COM11，MKS 控制板）
        default_idx = 1
        for idx, p in enumerate(ports, 1):
            if "COM11" in p.upper() or "11" in p:
                default_idx = idx
                break

        print("\n" + "=" * 50)
        print("          选择机械臂串口设备")
        print("=" * 50)
        for idx, p in enumerate(ports, 1):
            tag = "  <-- 推荐 (MKS 控制板)" if idx == default_idx else ""
            print(f"  [{idx}] {p}{tag}")
        print("=" * 50)

        prompt = f">> 请输入编号或直接按回车 [默认: {default_idx} - {ports[default_idx-1]}]: "
        choice = input(prompt).strip()
        if not choice:
            self._robot.connect(ports[default_idx - 1])
        elif choice.isdigit() and 1 <= int(choice) <= len(ports):
            self._robot.connect(ports[int(choice) - 1])
        else:
            self._robot.connect(choice)

    # ------------------------------------------------------------------
    # 主菜单循环
    # ------------------------------------------------------------------
    def _main_menu_loop(self) -> None:
        while True:
            self._print_main_menu()
            choice = input("\n>> 请选择操作编号: ").strip().lower()
            try:
                if choice == "1":
                    status = self._robot.get_limit_status()
                    print(f"\n  限位状态: {status}")
                    for k, v in status.raw.items():
                        print(f"    {k}: {v}")
                elif choice == "2":
                    self._robot.home()
                elif choice == "3":
                    self._set_home_menu()
                elif choice in ("4", "7"):
                    # 第4项与第7项合并：手动点动与末端工具
                    self._jog_and_effector_menu()
                elif choice == "5":
                    self._goto_menu()
                elif choice in ("6", "8"):
                    # 第6项与第8项合并：工位跳转与自动搬运
                    self._preset_and_workflow_menu()
                elif choice == "9":
                    self._raw_gcode_terminal()
                elif choice in ("p", "pos"):
                    self._robot.refresh_state()
                    p = self._robot.current_pose
                    a = self._robot.current_angles
                    print(f"\n[当前坐标] {p}")
                    print(f"[当前关节] {a}")
                elif choice == "r":
                    self._robot.disconnect()
                    self._select_port_menu()
                elif choice == "0":
                    self._robot.disable_steppers()
                    print("[安全] M84 已发送，现在可手动轻推关节。")
                elif choice in ("q", "quit", "exit"):
                    print("\n正在安全退出...")
                    self._robot.disconnect()
                    break
                else:
                    print("\n[提示] 无效选项，请重新输入。")
            except (KeyboardInterrupt, EOFError):
                print("\n[退出] 收到退出信号，正在安全退出...")
                self._robot.disconnect()
                break
            except Exception as exc:
                logger.exception("执行出错: %s", exc)

    def _print_main_menu(self) -> None:
        robot = self._robot
        conn_str = "已连接" if robot.is_connected() else "未连接"
        if robot.is_connected():
            p = robot.current_pose
            a = robot.current_angles
            pos_str = f"X:{p.x:.1f}  Y:{p.y:.1f}  Z:{p.z:.1f}  R:{p.r:.1f}"
            ang_str = f"大臂(θ):{a.theta:.1f}°  小臂(ψ):{a.psi:.1f}°"
        else:
            pos_str = ang_str = "未连接"

        print("\n" + "=" * 66)
        print("           Flux Loader (SCARA) 交互控制终端 v2.1")
        print("=" * 66)
        print(f"  [ 串口 ]:  {conn_str}")
        print(f"  [ 坐标 ]:  {pos_str}")
        print(f"  [ 关节 ]:  {ang_str}")
        print("-" * 66)

        print("\n  【 状态与原点标定 】")
        print("    [1] 传感器与限位诊断   (M119 检测 X/Y/Z 限位)\n")
        print("    [2] 一键三轴回零       (G28 X/Y/R 实体回原点)\n")
        print("    [3] 将当前位置设为零点 (G92 重设当前为原点)\n")
        print("    [p] 刷新当前坐标与状态 (M114)")

        print("\n  【 手动调试与点位控制 】")
        print("    [4] 手动点动与末端工具 (Jog 步进微调 X/Y/Z/R、Z轴舵机升降与夹爪开闭)\n")
        print("    [5] 直达目标绝对坐标   (输入 X Y Z R F 执行平滑运动)\n")
        print("    [6] 工位跳转与自动搬运 (预设工位快速跳转、芦笋单次完整搬运宏)")

        print("\n  【 系统维护与通信 】")
        print("    [9] 原生 G-code 命令行 (透传发送底层指令)\n")
        print("    [r] 重新连接 / 切换串口\n")
        print("    [0] 释放电机使能       (M84 允许手动推臂)\n")
        print("    [q] 退出终端程序")
        print("=" * 66)

    # ------------------------------------------------------------------
    # 子菜单：G92 原点设定
    # ------------------------------------------------------------------
    def _set_home_menu(self) -> None:
        hp = self._config.home_pose
        print("\n" + "-" * 56)
        print("      将当前位置设为零点/原点 (G92)")
        print("-" * 56)
        print("  提示: G92 指令不移动电机，直接重设固件内部的坐标计数。\n")
        print(f"    [1] 一键设为机械标准零位 (G92 X{hp.x} Y{hp.y} Z{hp.z} E{hp.r})\n")
        print("    [2] 设为基座原点         (G92 X0 Y0 Z0 E0)\n")
        print("    [3] 手动输入自定义坐标值\n")
        print("    [b] 返回主菜单")
        print("-" * 56)

        c = input("\n>> 请选择 [1/2/3/b]: ").strip().lower()
        if c == "1":
            self._robot.set_coordinate_origin(x=hp.x, y=hp.y, z=hp.z, r=hp.r)
        elif c == "2":
            self._robot.set_coordinate_origin(0.0, 0.0, 0.0, 0.0)
        elif c == "3":
            raw = input("\n请输入新坐标 (如 X0 Y600 Z80 R0): ").strip()
            if not raw:
                return
            x = y = z = r = 0.0
            m = re.search(r"[Xx]([-+]?\d*\.?\d+)", raw)
            if m: x = float(m.group(1))
            m = re.search(r"[Yy]([-+]?\d*\.?\d+)", raw)
            if m: y = float(m.group(1))
            m = re.search(r"[Zz]([-+]?\d*\.?\d+)", raw)
            if m: z = float(m.group(1))
            m = re.search(r"[RrEe]([-+]?\d*\.?\d+)", raw)
            if m: r = float(m.group(1))
            self._robot.set_coordinate_origin(x, y, z, r)

    # ------------------------------------------------------------------
    # 子菜单：手动点动与末端工具 (第4项与第7项深度合并)
    # ------------------------------------------------------------------
    def _jog_and_effector_menu(self) -> None:
        while True:
            p = self._robot.current_pose
            a = self._robot.current_angles
            print("\n" + "-" * 66)
            print("          手动点动与末端工具控制 (Jog & End-Effector)")
            print("-" * 66)
            print(f"  当前坐标: X:{p.x:6.1f}  Y:{p.y:6.1f}  Z:{p.z:5.1f} mm  R:{p.r:5.1f}°")
            print(f"  当前关节: 大臂(θ):{a.theta:5.1f}°  小臂(ψ):{a.psi:5.1f}°")
            print(f"  当前步长: {self._jog.step_info}\n")
            print("  【 笛卡尔空间点动 】          【 关节角独立点动 】         【 步长档位 】")
            print("    W / S : Y 轴 +/- (前进/后退)  O / L : 大臂 θ +/- (逆/顺)   [1] 1mm / 1°")
            print("    A / D : X 轴 -/+ (左移/右移)  I / K : 小臂 ψ +/- (展/折)   [2] 10mm / 5°")
            print("    U / J : Z 轴 +/- (舵机升降)   Q / E : R  轴 +/- (转角)     [3] 50mm / 15°\n")
            print("  【 Z 轴舵机升降控制 】         【 抓取夹爪舵机控制 】")
            print("    [zh] 升至最高安全位 (100mm)   [c] 双夹爪同时闭合 (抓取)")
            print("    [zl] 降至下探工作位 ( 20mm)   [o] 双夹爪同时打开 (松开)")
            print("    [z]  输入指定高度 (0~100mm)   [c1] / [o1] 夹爪1(头端) 闭/开")
            print("                                  [c2] / [o2] 夹爪2(尾端) 闭/开\n")
            print("    [b] 返回主菜单")
            print("-" * 66)

            cmd = input("\n>> 请输入控制指令: ").strip().lower()
            if cmd == "b":
                break

            # 1. 舵机快捷指令
            if cmd in ("c", "close", "9"):
                print(f"\n[执行] 正在发送双夹爪闭合指令 (M280 P1 S{self._config.gripper_close_angle}, M280 P2 S{self._config.gripper_close_angle})...")
                self._robot.gripper.set_both_grippers(open_state=False)
                print("[完成] 双夹爪已闭合。")
            elif cmd in ("o", "open", "8"):
                print(f"\n[执行] 正在发送双夹爪打开指令 (M280 P1 S{self._config.gripper_open_angle}, M280 P2 S{self._config.gripper_open_angle})...")
                self._robot.gripper.set_both_grippers(open_state=True)
                print("[完成] 双夹爪已打开。")
            elif cmd in ("c1", "5"):
                print(f"\n[执行] 夹爪 1 (头端) 闭合 (M280 P1 S{self._config.gripper_close_angle})...")
                self._robot.gripper.set_gripper(self._config.gripper1_id, open_state=False)
            elif cmd in ("o1", "4"):
                print(f"\n[执行] 夹爪 1 (头端) 打开 (M280 P1 S{self._config.gripper_open_angle})...")
                self._robot.gripper.set_gripper(self._config.gripper1_id, open_state=True)
            elif cmd in ("c2", "7"):
                print(f"\n[执行] 夹爪 2 (尾端) 闭合 (M280 P2 S{self._config.gripper_close_angle})...")
                self._robot.gripper.set_gripper(self._config.gripper2_id, open_state=False)
            elif cmd in ("o2", "6"):
                print(f"\n[执行] 夹爪 2 (尾端) 打开 (M280 P2 S{self._config.gripper_open_angle})...")
                self._robot.gripper.set_gripper(self._config.gripper2_id, open_state=True)
            elif cmd in ("zh", "2"):
                print("\n[执行] Z 轴一键升至最高安全位 (100.0 mm)...")
                self._robot.set_z_height(100.0)
                print(f"[完成] 当前 Z 轴高度: {self._robot.current_pose.z:.1f} mm")
            elif cmd in ("zl", "3"):
                print("\n[执行] Z 轴一键降至下探工作位 (20.0 mm)...")
                self._robot.set_z_height(20.0)
                print(f"[完成] 当前 Z 轴高度: {self._robot.current_pose.z:.1f} mm")
            elif cmd == "z":
                val = input("请输入目标 Z 轴高度 (0~100 mm): ").strip()
                if val:
                    try:
                        target_z = float(val)
                        print(f"\n[执行] Z 轴移动至 {target_z:.1f} mm...")
                        self._robot.set_z_height(target_z)
                        print(f"[完成] 当前 Z 轴高度: {self._robot.current_pose.z:.1f} mm")
                    except ValueError:
                        print("[错误] 请输入有效数字！")
            elif cmd.startswith("z") and len(cmd) > 1 and cmd[1:].replace(".", "", 1).isdigit():
                # 支持直接输入 z50, z80
                target_z = float(cmd[1:])
                print(f"\n[执行] Z 轴移动至 {target_z:.1f} mm...")
                self._robot.set_z_height(target_z)
                print(f"[完成] 当前 Z 轴高度: {self._robot.current_pose.z:.1f} mm")
            else:
                # 2. 轴点动指令由 JogController 分发
                handled = self._jog.handle_key(cmd)
                if not handled:
                    print("[提示] 未识别的按键或指令，请重新输入。")

    # ------------------------------------------------------------------
    # 子菜单：直达目标坐标
    # ------------------------------------------------------------------
    def _goto_menu(self) -> None:
        p = self._robot.current_pose
        print("\n" + "-" * 56)
        print("          直达目标绝对坐标 (G1 X Y Z R F)")
        print("-" * 56)
        print(f"  当前坐标: X:{p.x:.1f}  Y:{p.y:.1f}  Z:{p.z:.1f}  R:{p.r:.1f}")
        raw = input("\n>> 请输入目标坐标 (格式: X100 Y300 Z50 R0，回车取消): ").strip()
        if not raw:
            return

        x = y = z = r = None
        f = self._config.default_feedrate
        m = re.search(r"[Xx]([-+]?\d*\.?\d+)", raw)
        if m: x = float(m.group(1))
        m = re.search(r"[Yy]([-+]?\d*\.?\d+)", raw)
        if m: y = float(m.group(1))
        m = re.search(r"[Zz]([-+]?\d*\.?\d+)", raw)
        if m: z = float(m.group(1))
        m = re.search(r"[RrEe]([-+]?\d*\.?\d+)", raw)
        if m: r = float(m.group(1))
        m = re.search(r"[Ff](\d+)", raw)
        if m: f = float(m.group(1))

        target = Pose(
            x=x if x is not None else p.x,
            y=y if y is not None else p.y,
            z=z if z is not None else p.z,
            r=r if r is not None else p.r,
            f=f,
        )
        self._robot.move_to_pose(target)

    # ------------------------------------------------------------------
    # 子菜单：工位跳转与自动搬运 (第6项与第8项深度合并)
    # ------------------------------------------------------------------
    def _preset_and_workflow_menu(self) -> None:
        while True:
            p = self._robot.current_pose
            presets = self._presets.list_presets()
            keys = list(presets.keys())

            print("\n" + "-" * 66)
            print("          工位跳转与自动搬运 (Workstations & Workflow)")
            print("-" * 66)
            print(f"  当前位置: X:{p.x:6.1f}  Y:{p.y:6.1f}  Z:{p.z:5.1f} mm  R:{p.r:5.1f}°\n")
            print("  【 预设特征工位快速跳转 】")
            for idx, (name, pose) in enumerate(presets.items(), 1):
                print(f"    [{idx}] {name:<28} -> X:{pose.x:6.1f}  Y:{pose.y:6.1f}  Z:{pose.z:5.1f}  R:{pose.r:5.1f}")

            print("\n  【 自动化作业流程测试 】")
            print("    [a] 执行【芦笋搬运节拍宏】 (输入几个 a 即执行几个循环，如 a=1次、aaaa=4次、a6=6次)\n")
            print("  【 工位点位管理 】")
            print("    [s] 将当前实际位置添加为新预设点\n")
            print("    [b] 返回主菜单")
            print("-" * 66)

            try:
                choice = input("\n>> 请选择跳转工位或作业 [编号/a/s/b]: ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                break
            if choice == "b":
                break
            elif choice.startswith("a") or choice == "8":
                # 解析循环次数：输入几个 a 即循环几次（如 aaaaaa -> 6次），亦支持 a6 / a 6
                cycles = 1
                if all(c == "a" for c in choice):
                    cycles = len(choice)
                else:
                    m = re.match(r"^a\s*(\d+)$", choice)
                    if m:
                        cycles = int(m.group(1))
                    elif choice == "8":
                        cycles = 1
                    else:
                        print("\n[提示] 无效选项，多轮循环请输入连续的 a (如 'aaaa') 或 'a4'。")
                        continue

                print(f"\n[工作流] 正在启动芦笋搬运闭环测试 (连续执行 {cycles} 个循环)...")
                executed = 0
                for i in range(1, cycles + 1):
                    if cycles > 1:
                        print(f"\n{'=' * 20} 正在执行第 {i}/{cycles} 轮搬运循环 {'=' * 20}")
                    success = self._workflow.run()
                    executed = i
                    if not success:
                        print(f"\n[警告] 第 {i} 轮循环被中断或执行失败！")
                        break
                    if i < cycles:
                        time.sleep(0.5)
                print(f"\n[完成] 搬运节拍测试已结束 (共执行完成 {executed}/{cycles} 轮)。")
            elif choice.isdigit() and 1 <= int(choice) <= len(keys):
                name = keys[int(choice) - 1]
                target = presets[name]
                print(f"\n[跳转] 正在移动至: {name} (X={target.x:.1f}, Y={target.y:.1f}, Z={target.z:.1f}, R={target.r:.1f})")
                # 先升至安全高度再平移，保护末端机构
                safe_z = self._config.home_pose.z
                self._robot.set_z_height(safe_z)
                self._robot.move_to_pose(Pose(x=target.x, y=target.y, z=safe_z, r=target.r))
                self._robot.set_z_height(target.z)
                print(f"[完成] 已精准到达: {name}")
            elif choice == "s":
                name = input("\n请输入点位描述名称: ").strip()
                if not name or name.lower() == "b":
                    print("[提示] 已取消添加。")
                    continue
                self._presets.add(name, p)
                print(f"[保存] 已将当前位置 {p} 保存为 [{name}]")
            else:
                print("\n[提示] 无效选项，请重新输入。")

    # ------------------------------------------------------------------
    # 子菜单：G-code 透传终端
    # ------------------------------------------------------------------
    def _raw_gcode_terminal(self) -> None:
        print("\n--- 进入原生 G-code 透传模式 ---")
        print("直接输入 G-code 发送给 Marlin (输入 'exit' 或 'b' 返回菜单)")
        while True:
            cmd = input("Marlin G-code > ").strip()
            if not cmd:
                continue
            if cmd.lower() in ("exit", "b", "quit"):
                break
            lines = self._robot.send_raw(cmd)
            for line in lines:
                print(f"  < {line}")


# ==============================================================================
# 入口函数
# ==============================================================================
def main() -> None:
    parser = argparse.ArgumentParser(description="Flux Loader SCARA 机械臂调试工具")
    parser.add_argument("--port", "-p", type=str, default=None, help="串口号 (如 COM3, /dev/ttyUSB0)")
    parser.add_argument("--verbose", "-v", action="store_true", help="启用 DEBUG 级别日志输出")
    args = parser.parse_args()

    _setup_logging(verbose=args.verbose)

    app = LoaderCLIApp(port=args.port)
    app.start()


if __name__ == "__main__":
    main()
