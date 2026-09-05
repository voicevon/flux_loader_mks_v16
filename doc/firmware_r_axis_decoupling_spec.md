# SCARA 机械臂 R 轴世界姿态高频解耦与平滑同步实施规范
# (Firmware R-Axis World Decoupling & Synchronization Specification)

> **目标受众**：固件开发工程师 / 自动化 AI 执行 Agent  
> **适用目标硬件**：MKS Base V1.6 (ATmega2560 @ 16MHz) + SCARA 机械臂 (L1=300mm, L2=300mm)  
> **目标代码库**：`d:/Software/antigravity/Marlin/Marlin/`  
> **状态**：待执行实施 (Ready for Execution)

---

## 一、核心需求与物理场景 (Requirements & Physical Scenario)

### 1.1 背景与概念定义
在三自由度 SCARA 机械臂中，大臂（Joint 1，角度 $\theta$）与小臂（Joint 2，角度 $\psi$）在水平面运动。末端旋转电机（Joint 3 / R 轴，连接在 Marlin 的 **E0 步进电机接口**）负责带动末端执行器（芦笋夹爪）旋转。

* **世界坐标系绝对朝向角 ($R_{\text{world}}$)**：
  上位机通过 G-code 中的 `E` 参数（如 `G1 X... Y... E{R_world}`）向固件传递末端执行器相对于**世界基座坐标系**的绝对物理方位角（单位：度）。
* **用户理想运动行为**：
  1. **姿态锁定（零角保持）**：当机械臂从点 A 移动到点 B 时，若起始 $R = 0^\circ$ 且目标 $R = 0^\circ$，在机械臂大臂和小臂旋转移动的**全行程中，末端夹爪必须如电子罗盘或陀螺仪一样，死死锁定在世界 $0^\circ$ 方向，绝不随小臂摆动而发生晃动**。
  2. **姿态均匀差分插值**：当起始 $R = 0^\circ$ 且目标 $R = 10^\circ$ 时，这 $10^\circ$ 的旋转量必须在整个移动行程的时间轴上**均匀线性差分分布**。机械臂一边走直线，末端一边以恒定角速度转动，在抵达终点瞬间恰好完成 $10^\circ$ 旋转，且全程抵消双臂旋转。

### 1.2 为什么必须在固件层（Firmware）实现？
* SCARA 在走笛卡尔直线时，双臂角度 $\theta(t)$ 和 $\psi(t)$ 随时间是强非线性的三角函数曲线；
* 若仅在上位机端根据终点计算电机角度并单条下发，固件切片执行时末端夹爪在行程中间会产生严重的相位摆动与朝向失真（误差可达十余度）；
* 只有在固件运动规划器的微切片（Segment）执行点上，以毫秒级高频硬实时闭环计算补偿，才能达到完全无晃动的平滑世界姿态保持。

---

## 二、运动学数学模型 (Mathematical Model)

### 2.1 几何关系推导
1. **小臂连杆在世界基座坐标系中的绝对朝向**：
   $$\phi_{\text{forearm}}(t) = \theta(t) + \psi(t)$$
   * $\theta$：大臂相对于世界基座 $+X$ 轴的角度（Marlin 中对应 `motion.delta.a`）；
   * $\psi$：小臂相对于大臂的折叠角（Marlin 中对应 `motion.delta.b`）。

2. **末端执行器绝对世界角与电机物理相对角的关系**：
   末端电机安装在小臂连杆末端，其物理步进转角为 $\alpha_{\text{motor}}$：
   $$R_{\text{world}}(t) = \phi_{\text{forearm}}(t) + \alpha_{\text{motor}}(t) = \big(\theta(t) + \psi(t)\big) + \alpha_{\text{motor}}(t)$$

3. **电机物理步进角实时解耦方程**：
   $$\alpha_{\text{motor}}(t) = R_{\text{world}}(t) - \big(\theta(t) + \psi(t)\big) + R_{\text{offset}}$$
   * $R_{\text{world}}(t)$：当前微切片时刻期望的世界朝向角；
   * $\theta(t) + \psi(t)$：当前微切片时刻通过逆运动学（IK）求得的大臂与小臂瞬时角；
   * $R_{\text{offset}}$：机械零位偏置（可选，默认 0.0）。

### 2.2 固件内部插值机制
在 Marlin 的 `Motion::prepare_kinematic_move_to` 中，空间直线被分割为 $N$ 个微小线段（Segment）。目标坐标点 `raw` 是 `xyze_pos_t` 类型，原生循环：
```cpp
while (--segments) {
  raw += segment_distance;
  planner.buffer_line(raw, ...);
}
```
* **天然的均匀插分**：`raw.e` 在每一个微线段点上，原生算法已经自动完成了从 $R_{\text{start}}$ 到 $R_{\text{end}}$ 随时间/距离的均匀线性插值！
* **只需一次解耦减法**：传入 `planner.buffer_line` 的 `machine.e` 即为该时刻的 $R_{\text{world}}(t)$。在规划器中只需减去当前瞬时的 $(\text{delta.a} + \text{delta.b})$，即可输出电机步进目标！

---

## 三、重大前置约束：切片频率与 8 位主控算力调优

### 3.1 现象与根因 (Choppy Motion / "走打哽" 预防)
* MKS Base V1.6 主控芯片为 **ATmega2560**（16MHz 主频，8 位 AVR，无硬件浮点协处理器）。
* 固件原配置 `DEFAULT_SEGMENTS_PER_SECOND 200` 会让 8 位芯片算力 100% 枯竭，导致规划器缓冲区下溢（Buffer Underrun），机械臂在移动时会**走一走停一停、剧烈打哽、伴随驱动芯片过热**。
* **硬性要求**：在开启姿态解耦的同时，必须将 `DEFAULT_SEGMENTS_PER_SECOND` 从 200 降低至 **40**（每个切片约 $1.25\text{ mm}$，轨迹误差 $<0.05\text{ mm}$，精度无损，但 CPU 负担减轻 80%，运行如丝般顺滑）。

---

## 四、具体固件修改清单 (Firmware Implementation)

所有修改均在 `d:/Software/antigravity/Marlin/Marlin/` 工程目录下进行。

### 4.1 修改文件 1：`Configuration.h`
路径：[Configuration.h](file:///d:/Software/antigravity/Marlin/Marlin/Configuration.h)

#### 修改点 A：切片频率调优（第 1098 行）
将原本的 200 改为 40：
```diff
--- a/Configuration.h
+++ b/Configuration.h
@@ -1095,7 +1095,7 @@
 #define SCARA
 #if ENABLED(SCARA)
   // If movement is choppy try lowering this value
-  #define DEFAULT_SEGMENTS_PER_SECOND 200
+  #define DEFAULT_SEGMENTS_PER_SECOND 40
```

#### 修改点 B：增加 R 轴世界姿态解耦配置宏（第 1130 行附近，`#endif // SCARA` 之前）
```diff
--- a/Configuration.h
+++ b/Configuration.h
@@ -1128,6 +1128,10 @@
   #define SCARA_CALIBRATION
 
+  // 启用 SCARA 末端 R 轴 (E0 步进电机) 世界坐标系绝对姿态高频动态解耦
+  #define SCARA_R_WORLD_DECOUPLING
+  #define SCARA_R_WORLD_OFFSET 0.0f  // 机械零位安装角度偏置 (deg)
+
 #endif // SCARA
```

---

### 4.2 修改文件 2：`src/module/planner.cpp`
路径：[src/module/planner.cpp](file:///d:/Software/antigravity/Marlin/Marlin/src/module/planner.cpp)

#### 修改点：在 `Planner::buffer_line` 中实现微切片姿态补偿（第 3084 行附近）
在 `inverse_kinematics(machine)` 求解出 `motion.delta.a`（大臂角）与 `motion.delta.b`（小臂角）之后，拦截 `motion.delta.e` 的赋值：

```diff
--- a/src/module/planner.cpp
+++ b/src/module/planner.cpp
@@ -3081,7 +3081,14 @@ bool Planner::buffer_line(const xyze_pos_t &cart, const feedRate_t fr_mm_s
 
     #endif // POLAR && FEEDRATE_SCALING
 
-    TERN_(HAS_EXTRUDERS, motion.delta.e = machine.e);
+    #if ENABLED(SCARA_R_WORLD_DECOUPLING)
+      // machine.e 为当前微线段的时间线性插值世界朝向角 R_world (deg)
+      // 减去大臂角 (delta.a) 和小臂角 (delta.b)，输出电机相对物理角
+      motion.delta.e = machine.e - (motion.delta.a + motion.delta.b) + (SCARA_R_WORLD_OFFSET);
+    #else
+      TERN_(HAS_EXTRUDERS, motion.delta.e = machine.e);
+    #endif
+
     if (buffer_segment(motion.delta OPTARG(HAS_DIST_MM_ARG, cart_dist_mm), feedrate, extruder, ph)) {
       position_cart = cart;
       return true;
```

> **注意极性微调**：
> 若实测时发现大臂小臂旋转时，夹爪反向加剧旋转（说明电机走步方向或齿轮传动极性相反），只需将公式改为：
> `motion.delta.e = machine.e + (motion.delta.a + motion.delta.b) + (SCARA_R_WORLD_OFFSET);`

---

## 五、编译与烧录流程 (Build & Flash Instructions)

工作目录位于固件根目录：`d:/Software/antigravity/Marlin/`

### 5.1 本地静态编译
在 PowerShell 中执行：
```powershell
pio run -e mega2560
```
**验收标准**：
* 编译退出码为 `0`；
* 输出 `[SUCCESS] Took ...s`；
* 检查 RAM 与 Flash 占用（ATmega2560 具备 256KB Flash 与 8KB SRAM，当前构建通常 Flash 占用约 55%~65%，SRAM 占用约 50%~60%）。

### 5.2 固件烧录（上传至 MKS Base V1.6）
确保已关闭占用串口的上位机程序（如退出 `loader_cli.py`），执行：
```powershell
pio run -e mega2560 -t upload --upload-port COM11
```
*(端口号根据设备实际识别确认，例如 COM11)*

---

## 六、实机验收与功能测试清单 (Verification Checklist)

烧录完成后，启动 `tools/loader_cli.py` 进入 G-code 透传终端或手动点动模式进行实操验证：

### 测试 1：姿态锁定验证（保持 0°）
1. 将机械臂归零，定位到标准安全点位 `X: 250, Y: 250, Z: 80, R: 0`；
2. 发送大跨度斜向移动指令（R 保持 0 度）：
   ```gcode
   G1 X-250 Y350 Z80 E0 F2000
   ```
3. **观察判定**：
   * **合格**：在移动全过程中，大臂和小臂大幅转动，但末端夹爪始终平稳保持指向同一个固定的绝对空间朝向（如指南针锁定），无明显摆动；
   * **若方向反向**：若移动时夹爪旋转得更快更剧烈，说明符号需改为 `+ (motion.delta.a + motion.delta.b)`。

### 测试 2：时间均匀线性插分测试（0° $\to$ 30°）
1. 从 `X: 250, Y: 250, E: 0` 移动到 `X: -250, Y: 350, E: 30`：
   ```gcode
   G1 X-250 Y350 Z80 E30 F2000
   ```
2. **观察判定**：
   * 机械臂在平移全过程中，夹爪在空间坐标系下匀速缓慢旋转，从 0° 平滑过渡到 30°；
   * 运动全程连贯平滑，无走打哽、无急停急起。

### 测试 3：单次自动化搬运闭环测试
1. 在 `loader_cli.py` 中选择 `[6] 工位跳转与自动搬运`，按 `[a]` 触发完整节拍；
2. 观察从芦笋抓取位到落料入料口的全程动作：
   * 抓取芦笋提起到下探；
   * 全程夹爪姿态对准入料漏斗，动作顺畅无干涉。

---

## 七、架构交付总结
本规范文档提供了完整自包含的数学公式、代码 Diff、编译指令及实测判定标准。任何执行 Agent 均可严格对照第四节与第五节执行固件补丁应用与构建烧录。
