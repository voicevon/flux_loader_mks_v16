# flux_loader_mks_v16

基于 MakerBase MKS Base V1.6 控制板的单臂 SCARA 机械臂（用于芦笋抓取与搬运）完整工程，包含**上位机 Python 控制软件**、**系统设计文档**与**Marlin 固件定制方案**。

---

## 1. 工程结构

```text
c:\my_source\
├── flux_loader_mks_v16\          # 本项目
│   ├── doc\                      # 系统设计文档
│   │   ├── requirements.md                  # 控制系统需求与设计规格书
│   │   ├── mechanical_structure.md          # 机械结构/减速比/引脚/参数登记表
│   │   ├── firmware_r_axis_decoupling_spec.md   # R 轴世界姿态固件解耦实施规范
│   │   └── firmware_segment_decoupling_plan.md  # 固件 Segment 级解耦规划案
│   ├── loader_core\              # 上位机核心控制库（OOP 架构，可独立导入）
│   │   ├── models.py             # Pose / JointAngles / LimitSwitchStatus 领域模型
│   │   ├── config.py             # LoaderConfig 集中式硬件配置（唯一配置真相源）
│   │   ├── kinematics.py         # ScaraKinematics 纯数学正逆运动学解算
│   │   ├── comm.py               # ITransceiver / SerialTransceiver / MockTransceiver / Marlin 协议解析
│   │   ├── subsystems.py         # GripperSubsystem（Z 轴舵机 + 双夹爪）
│   │   ├── robot.py              # ScaraRobot 聚合根（home/move/jog/G92/M114 状态同步）
│   │   ├── workflows.py          # PickAndPlaceWorkflow 抓取-搬运-释放节拍宏
│   │   └── jog.py                # JogController 点动步长与键位调度
│   ├── tools\
│   │   └── loader_cli.py         # 交互式调试终端（瘦入口，组装 loader_core）
│   ├── tests\                    # 单元测试（无需真实串口）
│   │   ├── test_kinematics.py    # 正逆运动学闭环/奇异点/可达域
│   │   ├── test_protocol.py      # M114/M119 解析与 ok 握手
│   │   └── test_robot.py         # R 轴 G6 直驱与点动逻辑
│   ├── implementation_plan.md    # loader_core OOP 重构设计方案（已实施）
│   ├── requirements.txt          # Python 依赖（pyserial）
│   └── README.md
└── Marlin\                       # 固件工程：Marlin 2.0+ 完整源码 (PlatformIO)
    ├── Marlin\                   # 固件 C/C++ 核心源码 (Configuration.h / src)
    └── platformio.ini            # 编译构建配置 (环境 mega2560)
```

* **固件工程绝对路径**：`c:\my_source\Marlin`（相对路径 `../Marlin`）

---

## 2. 快速开始

```powershell
# 安装依赖
pip install -r requirements.txt

# 连接 MKS 控制板（默认 COM11，可省略 -p 进入串口选择菜单）
python tools/loader_cli.py -p COM11

# 运行单元测试（纯软件，无需硬件）
python -m unittest discover -s tests -v
```

外部自动化程序（如视觉调度）可直接以 SDK 方式导入核心库：

```python
from loader_core import ScaraRobot, LoaderConfig, Pose
from loader_core.comm import SerialTransceiver, MarlinProtocolHandler

robot = ScaraRobot(MarlinProtocolHandler(SerialTransceiver()), LoaderConfig())
robot.connect("COM11")
robot.home()
robot.move_to_pose(Pose(x=150, y=350, z=80, r=90))
```

---

## 3. 核心文档导航

* [requirements.md](doc/requirements.md)：系统架构、SCARA 逆运动学、Z 轴/夹爪舵机控制逻辑、运动速度/加速度及全局宏配置清单。
* [mechanical_structure.md](doc/mechanical_structure.md)：大臂/小臂/R 轴齿数减速比（6.4 / 4.2 / 2.8）、舵机引脚（A11/D65, D11, D12）与开闭角度（30°/0°）、脉冲当量换算表及更新日志。
* [firmware_r_axis_decoupling_spec.md](doc/firmware_r_axis_decoupling_spec.md)：R 轴世界坐标绝对姿态的固件级高频解耦实施规范（含代码 Diff 与烧录验证清单）。
* [firmware_segment_decoupling_plan.md](doc/firmware_segment_decoupling_plan.md)：固件 Segment 级姿态解耦的架构规划与上位机协同设计。

---

## 4. 关键设计约束

* **Z 轴为舵机反向映射**：Z=0mm ↔ 舵机 270°，Z=100mm ↔ 舵机 0°（见 requirements.md §3.3）。
* **夹爪角度**：打开 30° / 闭合 0°（实测整定，全工程统一）。
* **G-code 指令格式**：`G1 X.. Y.. Z.. E.. F..`，其中 `E` 复用为末端旋转 R 轴的世界绝对朝向角。
* **Z 轴升降双重指令**：`M280 P0`（舵机 PWM 直驱）+ `G92 Z`（坐标同步），缺一不可。
* 所有可调参数集中于 `loader_core/config.py` 的 `LoaderConfig`，与固件 `Configuration.h` 宏一一对应。
