# SCARA 固件 Segment 级姿态解耦与三轴高频同步规划案 (Roadmap & Technical Spec)

**文档状态**：待下一阶段固件重构实施 (Planned for Next Release)  
**作者**：架构开发组  
**关联代码**：`Marlin/Marlin/src/module/planner.cpp`, `Marlin/Marlin/src/module/motion.cpp`, `Marlin/Marlin/src/module/scara.cpp`

---

## 1. 核心问题陈述 (Problem Statement)

在基于 Marlin 的 SCARA 机械臂控制系统中，上位机与固件之间采用标准笛卡尔坐标 G-code 协议通信：
```gcode
G1 X[x_pos] Y[y_pos] Z[z_pos] E[r_angle] F[feedrate]
```
其中 `E` 轴被复用为末端旋转轴（Joint 3 / R 轴），定义为**末端执行器（夹爪）在世界基座坐标系中的绝对朝向角度 ($R_{\text{world}}$)**。

### 1.1 纯上位机补偿方案的致命缺陷
若仅在上位机端根据终点目标点计算电机物理转角：
$$E_{\text{motor\_end}} = R_{\text{world}} - (\theta_{\text{end}} + \psi_{\text{end}})$$
并将该单条指令下发给固件，在机械臂移动过程中将发生严重的**动态姿态失真（Dynamic Attitude Distortion）**：
1. **SCARA 机构的强非线性**：在笛卡尔直线移动中，大臂角 $\theta(t)$ 与小臂角 $\psi(t)$ 随时间是高度非线性的三角函数曲线。
2. **固件原生 E 轴的纯线性切片**：Marlin 的 `Motion::goto_destination_kinematic()` 以 `DEFAULT_SEGMENTS_PER_SECOND = 200`（200Hz）将直线轨迹切分成微小线段。对于每个线段点，固件仅对 $X, Y$ 重新执行逆运动学（IK），而对 $E$ 轴进行匀速线性插补：
   $$E_{\text{segment}}(t) = E_{\text{start}} + \frac{t}{T}(E_{\text{end}} - E_{\text{start}})$$
3. **后果**：小臂姿态与电机转动产生相位差，夹爪在运动全行程中会剧烈摆动、晃动，动态朝向误差可能达到十几度，只有运动停止在终点的一瞬间角度才是正确的。这在精密抓取芦笋、狭窄空间避障场景下是不可接受的。

---

## 2. 固件级高频解耦架构方案 (Firmware Decoupling Design)

### 2.1 核心数学模型
* **小臂连杆在世界坐标系中的方位角**：
  $$\phi_{\text{forearm}}(t) = \theta(t) + \psi(t)$$
* **末端绝对世界朝向**：
  $$R_{\text{world}}(t) = \phi_{\text{forearm}}(t) + \alpha_{\text{motor}}(t) = (\theta(t) + \psi(t)) + \alpha_{\text{motor}}(t)$$
* **电机物理步进角解耦方程**：
  $$\alpha_{\text{motor}}(t) = R_{\text{world}}(t) - (\theta(t) + \psi(t))$$

### 2.2 固件内部执行链条（200Hz 闭环）
在 Marlin 的步进规划器核心函数 `Planner::buffer_line()` 中实现硬实时姿态解耦：

```
[上位机下发] G1 X... Y... E{R_world}
     ↓
[Motion::goto_destination_kinematic()] 每秒高频切分 200 个微线段 (Segment)
     ↓
循环对每个线段点调用: Planner::buffer_line(raw, ...)
     ↓
[inverse_kinematics(machine)]
  → 求解当前微线段的 SCARA 瞬时关节角:
    motion.delta.a = Theta_i (大臂)
    motion.delta.b = Psi_i   (小臂)
     ↓
[【新增解耦补丁】]
  #if ENABLED(SCARA) && ENABLED(SCARA_R_WORLD_DECOUPLING)
    // machine.e 即为 G1 传入的世界朝向角 R_world
    motion.delta.e = machine.e - (motion.delta.a + motion.delta.b);
  #else
    TERN_(HAS_EXTRUDERS, motion.delta.e = machine.e);
  #endif
     ↓
[buffer_segment(motion.delta)]
  → 规划器将 (delta.a, delta.b, delta.e) 压入步进环形队列，
    定时器中断以微秒级精度同步驱动 X、Y、E0 三台步进电机！
```

---

## 3. 固件实施修改清单 (Firmware Modification Checklist)

### 3.1 配置文件宏增加 (`Configuration.h`)
```cpp
// 启用 SCARA 末端 R 轴世界坐标系自动解耦
#define SCARA_R_WORLD_DECOUPLING

// 如果机械零位安装存在固定偏置角，可在此定义
#define SCARA_R_WORLD_OFFSET 0.0
```

### 3.2 规划器核心解耦逻辑 (`Marlin/src/module/planner.cpp`)
* **定位文件与行号**：`planner.cpp:3084` (`Planner::buffer_line`)
* **原代码**：
  ```cpp
  TERN_(HAS_EXTRUDERS, motion.delta.e = machine.e);
  ```
* **改写为**：
  ```cpp
  #if ENABLED(SCARA_R_WORLD_DECOUPLING)
    // machine.e 保存的是世界坐标系目标角 R_world
    // motion.delta.a 为当前微线段瞬时大臂角 Theta (deg)
    // motion.delta.b 为当前微线段瞬时小臂角 Psi (deg)
    motion.delta.e = machine.e - (motion.delta.a + motion.delta.b) + (SCARA_R_WORLD_OFFSET);
  #else
    TERN_(HAS_EXTRUDERS, motion.delta.e = machine.e);
  #endif
  ```

### 3.3 零位与初始化同步 (`Planner::set_position_mm` & `M114`)
* 在 `set_position_mm` 中同步适配世界坐标与电机位置的映射。
* 在 `M114` 查询中，保持 `E` 汇报世界坐标 $R_{\text{world}}$，使上位机读回的值始终代表世界绝对姿态。

---

## 4. 上位机（Python Core）协同设计

当固件升级支持 Segment 级姿态解耦后，上位机软件将迎来巨大简化：
1. **指令语义归一**：上位机发送的 `G1 ... E<r>` 中的 `E` 纯粹表示世界坐标 $R_{\text{world}}$。
2. **免除上位机插值**：上位机无需在高层切碎点位去拟合姿态，可直接发送长距离笛卡尔直线移动。
3. **点动与示教无缝衔接**：笛卡尔点动 $X/Y$ 时，保持 $E$ 不变下发，机械臂在平移全过程中夹爪空间方向完全被固件锁定。

---

## 5. 迭代排期与验证流程 (Milestone & Acceptance)

1. **第一阶段（当前）**：
   - 上位机支持世界坐标定义及 G6 关节点动姿态补偿。
   - 保持固件原状稳定运行，优先打通整机搬运业务逻辑。
2. **第二阶段（固件升级迭代）**：
   - 在固件源码分支中应用上述 patch。
   - 使用 PlatformIO / Arduino IDE 编译新固件并烧录至 MKS Base V1.6。
   - 示教轨迹验证：机械臂走大范围斜向对角线移动，观察夹爪物理朝向是否在全行程中纹丝不动。
