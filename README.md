# flux_loader_mks_v16

基于 MakerBase MKS Base V1.6 控制板的单臂 SCARA 机械臂（用于芦笋抓取与搬运）固件与系统设计工程。

---

## 1. 源码与工程结构关联

本项目为固件配置与系统设计文档项目，与实际的 **Marlin 固件源码工程** 位于同一工作区父目录下，二者呈**平行同级目录**结构：

```text
d:\Software\antigravity\
├── flux_loader_mks_v16\          # 本项目：机械臂系统需求、机械参数与固件设计文档
│   ├── doc\
│   │   ├── requirements.md       # 控制系统需求与设计规格书（控制/算法/通信/参数汇总）
│   │   └── mechanical_structure.md # 物理结构、减速比、引脚映射与参数登记表
│   └── README.md
└── Marlin\                       # 实际固件工程：Marlin 2.0+ 完整源码工程 (PlatformIO)
    ├── Marlin\                   # 固件 C/C++ 核心源码 (Configuration.h / Configuration_adv.h / src)
    ├── platformio.ini            # PlatformIO 编译构建配置 (默认环境 mega2560)
    └── ...
```

* **固件工程绝对路径**：`d:\Software\antigravity\Marlin`
* **相对路径**：`../Marlin`

---

## 2. 核心文档导航

* [requirements.md](doc/requirements.md)：系统架构、SCARA 逆运动学设计、Z 轴/夹爪舵机控制逻辑、运动速度/加速度及全局宏配置清单。
* [mechanical_structure.md](doc/mechanical_structure.md)：大臂/小臂/R轴齿数减速比、舵机引脚（A11/D65, A12/D66）与开闭角度（0°/90°）、脉冲当量换算表及更新日志。
