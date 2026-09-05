#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flux Loader (SCARA) 机械臂交互式调试工具
用于 MKS Base V1.6 (ATmega2560) 固件运动学、舵机与抓取动作调校

功能包含：
- 串口自动扫描与连接管理 (115200 bps)
- 限位诊断 (M119)
- 一键三轴回零 (G28)
- 笛卡尔坐标点动微调 (Jogging)
- 目标坐标直达 (G1 X Y Z R F)
- 末端工具控制 (Z 轴舵机映射 0~100mm, 夹爪 1/2 独立与协同开闭)
- 完整抓取搬运节拍宏测试 (Pick & Place Macro)
- 原生 G-code 命令行透传终端
"""

import sys
import time
import re
import argparse
from typing import Optional, Tuple, Dict, List

# 确保在 Windows 控制台环境下输出 UTF-8 中文不乱码
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except AttributeError:
        pass

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[错误] 未找到 pyserial 模块。请先运行: pip install pyserial")
    sys.exit(1)


# ==============================================================================
# 常量与宏配置（对齐 mechanical_structure.md 与 requirements.md）
# ==============================================================================
DEFAULT_BAUDRATE = 115200

# Z 轴舵机映射参数 (Servo 0, A11/D65)
Z_MM_MIN = 0.0
Z_MM_MAX = 100.0
Z_SERVO_ANGLE_MIN = 0.0
Z_SERVO_ANGLE_MAX = 270.0

# 夹爪舵机配置
GRIPPER1_PIN = 1     # Servo 1, D11 (芦笋头端)
GRIPPER2_PIN = 2     # Servo 2, D12 (芦笋尾端)
GRIPPER_OPEN_ANGLE = 0
GRIPPER_CLOSE_ANGLE = 90

# 默认进给率 (mm/min)
DEFAULT_FEEDRATE = 3000


# ==============================================================================
# Marlin 串口通信客户端
# ==============================================================================
class MarlinSerialClient:
    def __init__(self, port: Optional[str] = None, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.current_pos = {"X": 0.0, "Y": 0.0, "Z": 0.0, "R": 0.0}

    @staticmethod
    def list_ports() -> List[str]:
        """列出系统中所有可用的串口名称"""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self, port: Optional[str] = None) -> bool:
        """建立串口连接并等待 Marlin 初始化握手"""
        if port:
            self.port = port
        if not self.port:
            print("[错误] 未指定串口。")
            return False

        try:
            print(f"[串口] 正在连接 {self.port} (波特率: {self.baudrate})...")
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            # DTR 触发 ATmega2560 复位，等待其 Bootloader 启动完成
            print("[串口] 等待控制板初始化 (约 2~3 秒)...")
            time.sleep(2.5)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # 发送回车测试握手
            lines = self.send_command("")
            print(f"[串口] 成功连接至 {self.port}！")
            # 尝试获取一次当前坐标
            self.update_position()
            return True
        except serial.SerialException as e:
            print(f"[错误] 打开串口 {self.port} 失败: {e}")
            self.ser = None
            return False

    def disconnect(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        print("[串口] 已断开连接。")

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send_command(self, cmd: str, wait_ok: bool = True, timeout: float = 10.0) -> List[str]:
        """
        向 Marlin 发送单条 G-code，并等待其回复 'ok'。
        返回该指令产生的所有回显文本行列表。
        """
        if not self.is_connected():
            print("[警告] 串口未连接，无法发送指令。")
            return []

        clean_cmd = cmd.strip()
        data = (clean_cmd + "\n").encode("utf-8")
        try:
            self.ser.write(data)
            self.ser.flush()
        except serial.SerialException as e:
            print(f"[通信异常] 发送失败: {e}")
            self.disconnect()
            return []

        if not wait_ok:
            return []

        response_lines = []
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException as e:
                print(f"[通信异常] 读取失败: {e}")
                self.disconnect()
                break

            if not line:
                continue

            response_lines.append(line)
            # Marlin 的标准应答行以 'ok' 开头
            if line.startswith("ok"):
                break

        return response_lines

    def update_position(self) -> Dict[str, float]:
        """发送 M114 并解析当前坐标"""
        lines = self.send_command("M114")
        for line in lines:
            # 常见格式: X:0.00 Y:0.00 Z:0.00 E:0.00 或 R:0.00
            m_x = re.search(r"X:([-+]?\d*\.?\d+)", line)
            m_y = re.search(r"Y:([-+]?\d*\.?\d+)", line)
            m_z = re.search(r"Z:([-+]?\d*\.?\d+)", line)
            m_r = re.search(r"[RE]:([-+]?\d*\.?\d+)", line)

            if m_x:
                self.current_pos["X"] = float(m_x.group(1))
            if m_y:
                self.current_pos["Y"] = float(m_y.group(1))
            if m_z:
                self.current_pos["Z"] = float(m_z.group(1))
            if m_r:
                self.current_pos["R"] = float(m_r.group(1))

        return self.current_pos


# ==============================================================================
# SCARA 动作与工具管理业务层
# ==============================================================================
class LoaderController:
    def __init__(self, client: MarlinSerialClient):
        self.client = client

    def get_limit_status(self) -> List[str]:
        """发送 M119 获取限位开关状态"""
        print("\n--- 发送 M119 限位开关诊断 ---")
        lines = self.client.send_command("M119")
        for l in lines:
            if "min" in l.lower() or "max" in l.lower() or "x" in l.lower():
                print(f"  > {l}")
        return lines

    def home_all(self) -> bool:
        """执行 G28 三轴一键回零"""
        print("\n--- 执行 G28 三轴回零 (X, Y, R) ---")
        print("[提示] 机械臂各轴将向限位方向运动，请确认行程无障碍！")
        lines = self.client.send_command("G28", timeout=30.0)
        for l in lines:
            print(f"  > {l}")
        self.client.update_position()
        print(f"[完成] 回零结束，当前坐标: {self.client.current_pos}")
        return True

    def set_current_as_home(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, r: float = 0.0) -> bool:
        """
        使用 G92 强制将当前物理位置设定为零点或指定坐标（不移动电机）
        Marlin 中标准指令: G92 X<pos> Y<pos> Z<pos> E<pos>
        """
        print(f"\n--- 发送 G92 将当前位置标记为原点/指定坐标 (X={x}, Y={y}, Z={z}, R={r}) ---")
        cmd = f"G92 X{x:.2f} Y{y:.2f} Z{z:.2f} E{r:.2f}"
        lines = self.client.send_command(cmd)
        for l in lines:
            print(f"  > {l}")
        self.client.update_position()
        print(f"[完成] 坐标原点已重设！当前坐标更新为: {self.client.current_pos}")
        return True

    def move_to(self, x: Optional[float] = None, y: Optional[float] = None,
                z: Optional[float] = None, r: Optional[float] = None,
                feedrate: float = DEFAULT_FEEDRATE) -> bool:
        """
        发送 G1 直角坐标移动指令
        """
        cmd_parts = ["G1"]
        if x is not None:
            cmd_parts.append(f"X{x:.2f}")
        if y is not None:
            cmd_parts.append(f"Y{y:.2f}")
        if z is not None:
            # 限制 Z 范围在 0~100mm
            z_clamped = max(Z_MM_MIN, min(Z_MM_MAX, z))
            cmd_parts.append(f"Z{z_clamped:.2f}")
        if r is not None:
            cmd_parts.append(f"R{r:.2f}")
        cmd_parts.append(f"F{feedrate:.0f}")

        cmd_str = " ".join(cmd_parts)
        print(f"[运动指令] >> {cmd_str}")
        lines = self.client.send_command(cmd_str, timeout=20.0)
        for l in lines:
            print(f"  > {l}")
        self.client.update_position()
        return True

    def set_z_height(self, z_mm: float):
        """
        设置 Z 轴高度 (0~100mm)
        同时支持向固件发 G1 Z 坐标以及直接发 M280 P0 舵机角度
        """
        z_clamped = max(Z_MM_MIN, min(Z_MM_MAX, z_mm))
        servo_angle = (z_clamped / Z_MM_MAX) * Z_SERVO_ANGLE_MAX
        print(f"[Z 轴升降] 目标高度: {z_clamped:.1f} mm (对应舵机角度: {servo_angle:.1f}°)")
        
        # 1. 尝试使用 G1 Z 控制
        self.move_to(z=z_clamped)
        # 2. 补充 M280 P0 确保舵机精准定位
        self.client.send_command(f"M280 P0 S{servo_angle:.0f}")

    def control_gripper(self, gripper_id: int, open_state: bool):
        """
        控制夹爪舵机
        gripper_id: 1 (头端, D11), 2 (尾端, D12), 3 (双爪同时)
        open_state: True 为打开(0°)，False 为闭合抓紧(90°)
        """
        angle = GRIPPER_OPEN_ANGLE if open_state else GRIPPER_CLOSE_ANGLE
        state_str = "打开 (0°)" if open_state else "闭合 (90°)"

        if gripper_id in (1, 3):
            print(f"[夹爪 1 (头端)] 执行: {state_str} (M280 P1 S{angle})")
            self.client.send_command(f"M280 P1 S{angle}")
        if gripper_id in (2, 3):
            print(f"[夹爪 2 (尾端)] 执行: {state_str} (M280 P2 S{angle})")
            self.client.send_command(f"M280 P2 S{angle}")

    def disable_steppers(self):
        """发送 M84 释放步进电机使能"""
        print("\n[安全] 发送 M84 释放电机使能 (现在可手动轻推关节)")
        lines = self.client.send_command("M84")
        for l in lines:
            print(f"  > {l}")

    def run_pick_and_place_macro(self):
        """
        执行标准抓取-搬运-释放循环测试宏
        时序流程：
        1. 提升 Z 到安全高度 (80mm)
        2. 打开双夹爪
        3. 移动到待抓取 XY 位 (X=150, Y=350, R=0)
        4. 下探至抓取高度 (Z=15mm)
        5. 闭合双夹爪抓紧
        6. 提升至安全高度 (Z=80mm)
        7. 旋转平移至出料工位 (X=-150, Y=350, R=45)
        8. 打开夹爪释放芦笋
        9. 返回初始待机高度
        """
        print("\n==============================================")
        print("  开始执行【芦笋单次搬运节拍宏测试】")
        print("==============================================")
        t_start = time.time()

        try:
            # 步骤 1: 提升至安全位并打开夹爪
            print("\n[步骤 1/7] 提升 Z 至安全高度 (80mm) 并打开夹爪...")
            self.set_z_height(80.0)
            self.control_gripper(3, open_state=True)
            time.sleep(0.5)

            # 步骤 2: 移至抓取工位上方
            pick_x, pick_y, pick_r = 150.0, 350.0, 0.0
            print(f"\n[步骤 2/7] 快速移动至抓取工位 XY: ({pick_x}, {pick_y}), R={pick_r}°...")
            self.move_to(x=pick_x, y=pick_y, r=pick_r, feedrate=3500)
            time.sleep(0.3)

            # 步骤 3: 下探
            print("\n[步骤 3/7] 下探至物料高度 (Z=15mm)...")
            self.set_z_height(15.0)
            time.sleep(0.5)

            # 步骤 4: 抓紧
            print("\n[步骤 4/7] 双爪闭合抓紧物料...")
            self.control_gripper(3, open_state=False)
            time.sleep(0.6)

            # 步骤 5: 抬起
            print("\n[步骤 5/7] 提起物料至安全高度 (80mm)...")
            self.set_z_height(80.0)
            time.sleep(0.4)

            # 步骤 6: 移动至分发流水线入口
            drop_x, drop_y, drop_r = -150.0, 350.0, 30.0
            print(f"\n[步骤 6/7] 搬运移动至分发入口 XY: ({drop_x}, {drop_y}), R={drop_r}°...")
            self.move_to(x=drop_x, y=drop_y, r=drop_r, feedrate=3500)
            time.sleep(0.3)

            # 步骤 7: 释放
            print("\n[步骤 7/7] 开启夹爪释放芦笋...")
            self.control_gripper(3, open_state=True)
            time.sleep(0.5)

            total_elapsed = time.time() - t_start
            print("\n==============================================")
            print(f"  [完成] 单次搬运循环测试结束！总耗时: {total_elapsed:.2f} 秒")
            print("==============================================")
        except KeyboardInterrupt:
            print("\n[中断] 用户打断了搬运宏测试！")


# ==============================================================================
# CLI 交互终端菜单主逻辑
# ==============================================================================
class LoaderCLI:
    def __init__(self, port: Optional[str] = None):
        self.client = MarlinSerialClient(port=port)
        self.loader = LoaderController(self.client)
        self.jog_step_mm = 10.0
        self.jog_step_deg = 5.0

    def start(self):
        print("\n=======================================================")
        print("    Flux Loader (SCARA) 机械臂调试交互终端 v1.0")
        print("=======================================================")

        # 如果没有指定端口，列出端口供用户挑选
        if not self.client.port:
            self._select_port_menu()
        else:
            self.client.connect()

        # 进入主循环
        self._main_menu_loop()

    def _select_port_menu(self):
        ports = MarlinSerialClient.list_ports()
        if not ports:
            print("[提示] 当前系统未检测到可用串口设备！")
            custom_port = input("请输入串口名 (如 COM3) 或直接按回车退出: ").strip()
            if custom_port:
                self.client.connect(custom_port)
            else:
                return
        else:
            print("\n检测到以下串口设备：")
            for idx, p in enumerate(ports, 1):
                print(f"  [{idx}] {p}")
            choice = input(f"请选择串口编号 (1~{len(ports)}) 或直接输入端口名: ").strip()
            if choice.isdigit() and 1 <= int(choice) <= len(ports):
                self.client.connect(ports[int(choice) - 1])
            elif choice:
                self.client.connect(choice)
            else:
                self.client.connect(ports[0])

    def _main_menu_loop(self):
        while True:
            # 状态刷新
            pos_str = "未连接"
            if self.client.is_connected():
                p = self.client.current_pos
                pos_str = f"X:{p['X']:.1f}  Y:{p['Y']:.1f}  Z:{p['Z']:.1f}  R:{p['R']:.1f}"

            print("\n" + "=" * 60)
            print(f"  [串口状态: {self.client.port if self.client.is_connected() else '未连接'}]")
            print(f"  [当前坐标: {pos_str}]")
            print("=" * 60)
            print("  [1] 传感器与限位诊断 (M119 检测 X/Y/Z 限位)")
            print("  [2] 一键三轴回零 (G28 X/Y/R 回原点)")
            print("  [3] 将当前位置设为零点 (G92 重设坐标/标记已回零)")
            print("  [4] 笛卡尔/关节点动微调 (Jog Mode: 步进微调)")
            print("  [5] 直达目标绝对坐标 (输入 X Y Z R F 执行平滑运动)")
            print("  [6] 预设特征点位快速跳转 (原点/待机位/抓取位/释放位)")
            print("  [7] 末端工具与夹爪测试 (Z轴舵机高度、夹爪1/2独立及协同)")
            print("  [8] 芦笋搬运循环宏测试 (完整抓取-提升-移载-释放闭环)")
            print("  [9] 原生 G-code 命令行透传 (手动输入任意指令)")
            print("  [p] 刷新当前坐标与状态 (M114)")
            print("  [r] 重新连接/切换串口")
            print("  [0] 释放电机使能 (M84)")
            print("  [q] 退出终端")
            print("=" * 60)

            choice = input("请选择操作编号: ").strip().lower()

            try:
                if choice == "1":
                    self.loader.get_limit_status()
                elif choice == "2":
                    self.loader.home_all()
                elif choice == "3":
                    self._set_home_menu()
                elif choice == "4":
                    self._jog_menu()
                elif choice == "5":
                    self._goto_menu()
                elif choice == "6":
                    self._preset_positions_menu()
                elif choice == "7":
                    self._end_effector_menu()
                elif choice == "8":
                    self.loader.run_pick_and_place_macro()
                elif choice == "9":
                    self._raw_gcode_terminal()
                elif choice in ("p", "pos"):
                    self.client.update_position()
                    print(f"[当前坐标] {self.client.current_pos}")
                elif choice == "r":
                    self.client.disconnect()
                    self._select_port_menu()
                elif choice == "0":
                    self.loader.disable_steppers()
                elif choice in ("q", "quit", "exit"):
                    print("正在退出...")
                    self.client.disconnect()
                    break
                else:
                    print("[提示] 无效选项，请重新输入。")
            except KeyboardInterrupt:
                print("\n[中断] 操作已取消。")
            except Exception as e:
                print(f"[异常] 执行出错: {e}")

    # --------------------------------------------------------------------------
    # 子菜单：将当前位置设为原点/指定零点 (G92)
    # --------------------------------------------------------------------------
    def _set_home_menu(self):
        print("\n--- 将当前位置设为零点/原点 (G92) ---")
        print("说明: G92 指令不移动任何电机，仅直接重设单片机内部当前坐标值。")
        print("  [1] 一键设为绝对零点 (G92 X0 Y0 Z0 E0)")
        print("  [2] 设为机械标准零位 (G92 X0 Y600 Z80 E0 - 大小臂伸直指向+Y)")
        print("  [3] 手动输入自定义坐标")
        print("  [b] 取消并返回")

        c = input("请选择 [1/2/3/b]: ").strip().lower()
        if c == "1":
            self.loader.set_current_as_home(x=0.0, y=0.0, z=0.0, r=0.0)
        elif c == "2":
            # 根据机械设计文档，大臂90°小臂0°顺向伸直时，两臂合计600mm
            self.loader.set_current_as_home(x=0.0, y=600.0, z=80.0, r=0.0)
        elif c == "3":
            raw = input("请输入新坐标 (如 X0 Y100 Z0 R0): ").strip()
            if not raw:
                return
            x, y, z, r = 0.0, 0.0, 0.0, 0.0
            m_x = re.search(r"[Xx]([-+]?\d*\.?\d+)", raw)
            m_y = re.search(r"[Yy]([-+]?\d*\.?\d+)", raw)
            m_z = re.search(r"[Zz]([-+]?\d*\.?\d+)", raw)
            m_r = re.search(r"[RrEe]([-+]?\d*\.?\d+)", raw)
            if m_x: x = float(m_x.group(1))
            if m_y: y = float(m_y.group(1))
            if m_z: z = float(m_z.group(1))
            if m_r: r = float(m_r.group(1))
            self.loader.set_current_as_home(x=x, y=y, z=z, r=r)

    # --------------------------------------------------------------------------
    # 子菜单：点动微调
    # --------------------------------------------------------------------------
    def _jog_menu(self):
        while True:
            p = self.client.current_pos
            print("\n--- 笛卡尔/关节点动微调 (Jog Mode) ---")
            print(f"当前坐标: X:{p['X']:.1f}, Y:{p['Y']:.1f}, Z:{p['Z']:.1f}, R:{p['R']:.1f}")
            print(f"当前步长: 线性轴={self.jog_step_mm} mm, 旋转轴={self.jog_step_deg}°")
            print("快捷指令:")
            print("  [w/s]: X 轴 +/-        [a/d]: Y 轴 +/-")
            print("  [u/j]: Z 轴 +/- (升降)  [q/e]: R 轴 +/- (旋转)")
            print("  [1/2/3]: 切换线性步长 (1mm / 10mm / 50mm)")
            print("  [b]: 返回主菜单")

            cmd = input("请输入点动键: ").strip().lower()
            if cmd == "b":
                break
            elif cmd == "1":
                self.jog_step_mm = 1.0
                self.jog_step_deg = 1.0
                print(f"[步长更新] 步长设为 1mm / 1°")
            elif cmd == "2":
                self.jog_step_mm = 10.0
                self.jog_step_deg = 5.0
                print(f"[步长更新] 步长设为 10mm / 5°")
            elif cmd == "3":
                self.jog_step_mm = 50.0
                self.jog_step_deg = 15.0
                print(f"[步长更新] 步长设为 50mm / 15°")
            elif cmd == "w":
                self.loader.move_to(x=p["X"] + self.jog_step_mm)
            elif cmd == "s":
                self.loader.move_to(x=p["X"] - self.jog_step_mm)
            elif cmd == "a":
                self.loader.move_to(y=p["Y"] + self.jog_step_mm)
            elif cmd == "d":
                self.loader.move_to(y=p["Y"] - self.jog_step_mm)
            elif cmd == "u":
                self.loader.set_z_height(p["Z"] + self.jog_step_mm)
            elif cmd == "j":
                self.loader.set_z_height(p["Z"] - self.jog_step_mm)
            elif cmd == "q":
                self.loader.move_to(r=p["R"] + self.jog_step_deg)
            elif cmd == "e":
                self.loader.move_to(r=p["R"] - self.jog_step_deg)

    # --------------------------------------------------------------------------
    # 子菜单：直达目标坐标
    # --------------------------------------------------------------------------
    def _goto_menu(self):
        print("\n--- 直达目标绝对坐标 (G1 X Y Z R F) ---")
        p = self.client.current_pos
        print(f"当前参考坐标: X:{p['X']} Y:{p['Y']} Z:{p['Z']} R:{p['R']}")
        raw = input("请输入目标坐标 (格式如: X100 Y300 Z50 R0 或按回车取消): ").strip()
        if not raw:
            return

        x, y, z, r = None, None, None, None
        f = DEFAULT_FEEDRATE

        m_x = re.search(r"[Xx]([-+]?\d*\.?\d+)", raw)
        m_y = re.search(r"[Yy]([-+]?\d*\.?\d+)", raw)
        m_z = re.search(r"[Zz]([-+]?\d*\.?\d+)", raw)
        m_r = re.search(r"[Rr]([-+]?\d*\.?\d+)", raw)
        m_f = re.search(r"[Ff](\d+)", raw)

        if m_x: x = float(m_x.group(1))
        if m_y: y = float(m_y.group(1))
        if m_z: z = float(m_z.group(1))
        if m_r: r = float(m_r.group(1))
        if m_f: f = float(m_f.group(1))

        self.loader.move_to(x=x, y=y, z=z, r=r, feedrate=f)

    # --------------------------------------------------------------------------
    # 子菜单：预设特征点位快速跳转
    # --------------------------------------------------------------------------
    def _preset_positions_menu(self):
        presets = {
            "1": ("零点位置 (Origin)", 0.0, 0.0, 80.0, 0.0),
            "2": ("安全待机位 (Standby)", 0.0, 300.0, 80.0, 0.0),
            "3": ("标准抓取位 (Pick Pose)", 150.0, 350.0, 20.0, 0.0),
            "4": ("落料入料口 (Dealer Drop Pose)", -150.0, 350.0, 80.0, 30.0),
        }
        while True:
            p = self.client.current_pos
            print("\n--- 预设特征点位快速跳转 ---")
            print(f"当前坐标: X:{p['X']:.1f}, Y:{p['Y']:.1f}, Z:{p['Z']:.1f}, R:{p['R']:.1f}")
            for k, (name, x, y, z, r) in presets.items():
                print(f"  [{k}] {name}: X={x}, Y={y}, Z={z}, R={r}")
            print("  [s] 将当前坐标添加/更新为自定义点位")
            print("  [b] 返回主菜单")

            choice = input("请选择预设点位 [1-4/s/b]: ").strip().lower()
            if choice == "b":
                break
            elif choice in presets:
                name, x, y, z, r = presets[choice]
                print(f"\n[跳转点位] 目标: {name} (X={x}, Y={y}, Z={z}, R={r})")
                # 先抬起 Z 轴至安全高度 80mm 再平移，保护末端机构与夹爪
                self.loader.set_z_height(80.0)
                self.loader.move_to(x=x, y=y, r=r)
                self.loader.set_z_height(z)
                print(f"[完成] 已到达 {name}！\n")
            elif choice == "s":
                new_key = str(len(presets) + 1)
                new_name = input(f"请输入点位名称 (例如: 工位{new_key}): ").strip() or f"自定义点位{new_key}"
                presets[new_key] = (new_name, p["X"], p["Y"], p["Z"], p["R"])
                print(f"[保存成功] 已将当前位置保存为 [{new_key}] {new_name}！")
            else:
                print("[提示] 无效选项，请重新输入。")

    # --------------------------------------------------------------------------
    # 子菜单：末端工具与夹爪测试
    # --------------------------------------------------------------------------
    def _end_effector_menu(self):
        while True:
            print("\n--- 末端执行器与舵机控制 ---")
            print("  [1] 设置 Z 轴高度 (0 ~ 100 mm)")
            print("  [2] Z 轴一键最高安全位 (100 mm)")
            print("  [3] Z 轴一键下探工作位 (15 mm)")
            print("  [4] 夹爪 1 (头端 - D11) 打开 (0°)")
            print("  [5] 夹爪 1 (头端 - D11) 闭合 (90°)")
            print("  [6] 夹爪 2 (尾端 - D12) 打开 (0°)")
            print("  [7] 夹爪 2 (尾端 - D12) 闭合 (90°)")
            print("  [8] 双夹爪同时打开 (0°)")
            print("  [9] 双夹爪同时闭合 (90°)")
            print("  [b] 返回上一级")

            c = input("请选择: ").strip().lower()
            if c == "b":
                break
            elif c == "1":
                val = input("请输入 Z 轴高度 (0~100 mm): ").strip()
                if val:
                    self.loader.set_z_height(float(val))
            elif c == "2":
                self.loader.set_z_height(100.0)
            elif c == "3":
                self.loader.set_z_height(15.0)
            elif c == "4":
                self.loader.control_gripper(1, open_state=True)
            elif c == "5":
                self.loader.control_gripper(1, open_state=False)
            elif c == "6":
                self.loader.control_gripper(2, open_state=True)
            elif c == "7":
                self.loader.control_gripper(2, open_state=False)
            elif c == "8":
                self.loader.control_gripper(3, open_state=True)
            elif c == "9":
                self.loader.control_gripper(3, open_state=False)

    # --------------------------------------------------------------------------
    # 子菜单：原始 G-code 透传终端
    # --------------------------------------------------------------------------
    def _raw_gcode_terminal(self):
        print("\n--- 进入原生 G-code 透传模式 ---")
        print("直接输入 G-code 发送给 Marlin (输入 'exit' 或 'b' 返回菜单)")
        while True:
            cmd = input("Marlin G-code > ").strip()
            if not cmd:
                continue
            if cmd.lower() in ("exit", "b", "quit"):
                break
            lines = self.client.send_command(cmd)
            for l in lines:
                print(f"  < {l}")


# ==============================================================================
# 入口函数
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Flux Loader SCARA 机械臂调试工具")
    parser.add_argument("--port", "-p", type=str, default=None, help="串口号 (如 COM3, /dev/ttyUSB0)")
    args = parser.parse_args()

    cli = LoaderCLI(port=args.port)
    cli.start()


if __name__ == "__main__":
    main()
