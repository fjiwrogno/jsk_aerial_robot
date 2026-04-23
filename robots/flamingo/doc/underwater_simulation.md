# Flamingo 水下动力学仿真

本文档说明如何在 Gazebo 中为 Flamingo 机器人添加水下动力学仿真，使用 `uuv_simulator` 提供的流体动力学插件。

## 背景

Flamingo 是一种空水两用机器人，搭载：
- 2 个空中旋翼（`thrust1`, `thrust2`，提供向上推力）
- 2 个水下推进器（`thrust3`, `thrust4`，提供水平推力）
- 2 个云台舵机（`gimbal1`, `gimbal2`，控制推力方向）

原有仿真框架已实现空中飞行动力学仿真。本改动在其基础上叠加水下流体动力学，实现水中运动仿真。

---

## 实现思路

### 关键设计原则：插件并行，互不干扰

uuv_simulator 的水动力插件在 Gazebo 物理引擎层面独立工作，**不替换**现有的 `AerialRobotHWSim`，而是与之并行运行：

```
每个 Gazebo 物理步（2ms）
├── AerialRobotHWSim              ← 原有：读取 IMU/磁力计，向旋翼施加推力/扭矩
└── UnderwaterObjectPlugin        ← 新增：向机体施加浮力 + 流体阻力 + 水流力
```

两个插件都通过 Gazebo 的 `AddRelativeForce()` / `AddWorldForce()` API 向刚体施加力，物理引擎在每步合并所有外力后积分运动方程，因此二者天然兼容。

### 水动力学模型（Fossen 模型）

uuv_simulator 使用 Thor I. Fossen 在 *Guidance and Control of Ocean Vehicles* (1994) 中提出的流体动力学模型：

$$M_{RB}\dot{\nu} + C_{RB}(\nu)\nu + M_A\dot{\nu} + C_A(\nu)\nu + D(\nu)\nu = \tau_{thrust} + g(\eta) + f_{ext}$$

各项含义：

| 项 | 符号 | 说明 |
|---|---|---|
| 刚体惯性力 | $M_{RB}\dot{\nu}$ | 由 URDF 中的 inertial 提供 |
| 附加质量力 | $M_A\dot{\nu}$ | 加速时排开水体产生的反作用力 |
| 流体阻尼 | $D(\nu)\nu$ | 线性项（低速主导）+ 二次项（高速主导） |
| 回复力 | $g(\eta)$ | 浮力 - 重力，取决于 volume 和 CoB 位置 |
| 推力 | $\tau_{thrust}$ | 由 AerialRobotHWSim 施加 |
| 外部水流力 | $f_{ext}$ | 由 underwater_current_plugin 提供 |

---

## 文件改动说明

### 1. 新建：`aerial_robot_simulation/gazebo_model/world/underwater.world`

**作用**：定义水下仿真环境。

与 `empty.world` 的关键区别：

| 配置项 | empty.world | underwater.world |
|---|---|---|
| 物理步长 | 1ms (1000Hz) | 2ms (500Hz) |
| 水流插件 | 无 | `libuuv_underwater_current_ros_plugin.so` |
| 地面 | 有 (ground_plane) | 无 |
| 环境光 | 默认（白天） | 深蓝色调（水下氛围） |

物理步长从 1ms 改为 2ms 的原因：流体动力学积分对步长更敏感，较小的更新频率（500Hz）在保证精度的同时降低了仿真不稳定的风险。

水流插件发布话题 `/hydrodynamics/current_velocity`（`geometry_msgs/Vector3`），robot URDF 中的 UnderwaterObjectPlugin 订阅此话题来计算水流产生的阻力。

### 2. 新建：`robots/flamingo/robots/flamingo.gazebo.underwater.xacro`

**作用**：在原有 Gazebo URDF 基础上，向 Flamingo 的主刚体追加 UUV 水动力插件。

文件结构：
```
flamingo.gazebo.underwater.xacro
├── include: flamingo.urdf.xacro          （运动学/动力学描述，复用原有）
├── include: spinal.gazebo.xacro          （AerialRobotHWSim + IMU + 磁力计，复用原有）
└── <gazebo> uuv_plugin                   （新增：浮力 + Fossen 水动力学）
    └── link: root
        ├── volume                         → 浮力大小
        ├── center_of_buoyancy             → 重浮心偏置（影响静稳定性）
        └── hydrodynamic_model (fossen)
            ├── added_mass                 → 6×6 附加质量矩阵
            ├── linear_damping             → 线性阻尼系数
            └── quadratic_damping          → 二次阻尼系数
```

**只对主刚体添加水动力**：由于 `gimbal_link` 和各推力连杆的尺寸很小，其流体阻力相对机体可以忽略。如需更高精度，可为这些 link 也添加 `<link>` 条目。

注意：Flamingo 的 URDF 使用 dummy `root` 作为 KDL 根节点，Gazebo 会把 `root` 与若干固定关节子链折叠成同一个物理刚体。因此水动力插件配置挂在 `root` 上，并使用 `libuuv_underwater_object_plugin.so`，避免 ROS 包装插件内部对 `base_link` 名称的硬编码假设导致空指针崩溃。

### 3. 修改：`robots/flamingo/launch/bringup.launch`

新增 `underwater` 参数，使原有启动流程以零侵入方式支持水下模式：

```
underwater=False（默认）            underwater=True
       ↓                                   ↓
description_mode = "gazebo"        description_mode = "gazebo.underwater"
       ↓                                   ↓
flamingo.gazebo.xacro              flamingo.gazebo.underwater.xacro
       ↓                                   ↓
worldtype = empty.world            worldtype = underwater.world
```

`bringup.launch` 的具体改动（2处）：

```xml
<!-- 改动1：worldtype 根据 underwater 标志自动切换默认值 -->
<arg name="underwater" default="False" />
<arg name="worldtype" default=".../empty.world"     unless="$(arg underwater)" />
<arg name="worldtype" default=".../underwater.world" if="$(arg underwater)" />

<!-- 改动2：description_mode 增加第三个分支 -->
<!-- 原：if simulation → gazebo -->
<!-- 改：if simulation and not underwater → gazebo -->
<!--     if simulation and underwater     → gazebo.underwater  -->
<arg name="description_mode" value="gazebo"            if="$(eval arg('simulation') and not arg('underwater'))" />
<arg name="description_mode" value="gazebo.underwater" if="$(eval arg('simulation') and arg('underwater'))" />
```

用户已有的启动命令完全不受影响（`underwater` 默认为 False）。

---

## 使用方法

### 启动命令

```bash
# 原有空中仿真（不受影响）
roslaunch flamingo bringup.launch sim:=true

# 水下仿真
roslaunch flamingo bringup.launch sim:=true underwater:=true spawn_z:=0.5

# 水下仿真 + 可视化窗口
roslaunch flamingo bringup.launch sim:=true underwater:=true spawn_z:=0.5 headless:=false

# 设置初始水流速度（在 ROS 运行时动态调整）
rostopic pub /hydrodynamics/current_velocity_model \
  uuv_world_ros_plugins_msgs/SetCurrentVelocity \
  '{velocity: 0.5, horizontal_angle: 0.0, vertical_angle: 0.0}'
```

### 前提条件

确保 `uuv_simulator` 已 build 并 source：

```bash
# 确认插件可被 Gazebo 找到
find /home/chen_chen/Project/uuv_ws -name "libuuv_underwater_object_plugin.so"
find /home/chen_chen/Project/uuv_ws -name "libuuv_underwater_current_ros_plugin.so"

# 启动前 source
source /home/chen_chen/Project/uuv_ws/devel/setup.bash
```

---

## 参数调整指南

`flamingo.gazebo.underwater.xacro` 中的水动力参数均为初始估算值，需要根据实际情况调整。

### 浮力（volume）

```xml
<volume>0.0035</volume>  <!-- 单位：m³ -->
```

Flamingo 机体质量约 3.18 kg，在淡水（1000 kg/m³）中中性浮力所需体积为：

$$V_{neutral} = \frac{m}{\rho} = \frac{3.18}{1000} = 0.00318 \text{ m}^3$$

当前值 0.0035 m³ 约高出中性浮力 10%，产生约 0.32 N 的净上浮力。
调整目标：使水下推进器在合理功率范围内能舒适地控制深度。

### 重浮心偏置（center_of_buoyancy）

```xml
<center_of_buoyancy>-0.1326 0.002 -0.06</center_of_buoyancy>  <!-- 单位：m，base_link 坐标系 -->
```

CoB 位于 CoG 上方约 5 cm（URDF 中 CoG 在 root 坐标系下 z = -0.111 m）。
CoB 高于 CoG 时，机体产生恢复力矩，倾斜时自动回正（静稳定）。
偏置越大，恢复力矩越强，但也会增加控制器抵抗恢复力矩的负担。

### 附加质量（added_mass）

```xml
<added_mass>
   1.0  0    0    0    0    0
   0    1.0  0    0    0    0
   0    0    1.5  0    0    0
   0    0    0    0.01 0    0
   0    0    0    0    0.05 0
   0    0    0    0    0    0.05
</added_mass>
```

6×6 对称矩阵，行列顺序为 `[surge(x), sway(y), heave(z), roll, pitch, yaw]`。
当前值为对角线估算（平移方向约为排水质量的 30%），非对角项因机体不对称可能非零。
精确值需通过 CFD 分析或系统辨识获取。

### 阻尼（linear_damping / quadratic_damping）

```xml
<linear_damping>-5 -5 -10 -0.5 -0.5 -0.5</linear_damping>
<quadratic_damping>-30 -80 -80 -3 -3 -3</quadratic_damping>
```

- **线性阻尼**：低速时主导，对应 Stokes 流动区域
- **二次阻尼**：高速时主导，对应 $F_d = \frac{1}{2} C_d \rho A v^2$

当前二次阻尼的估算依据（以 heave/sway 为例）：
- 迎流面积：$A_{lateral} \approx 0.47 \times 0.33 = 0.155 \text{ m}^2$
- 拖曳系数：$C_d \approx 1.0$（矩形体）
- 阻尼系数：$0.5 \times 1.0 \times 1000 \times 0.155 \approx 77.5$（取约 80）

实际调整方法：在仿真中给定初速度后释放机体，观察速度衰减曲线，与预期拖曳特性对比迭代。

---

## 已知局限性

1. **单 link 水动力**：当前仅对 `base_link` 添加了水动力模型，云台连杆和推力连杆的阻力被忽略。

2. **参数为估算值**：附加质量和阻尼均为基于几何形状的粗略估算，实际值需通过实验或 CFD 辨识。

3. **无自由液面效应**：uuv_simulator 不模拟自由液面（水-空气界面），因此在水面附近的仿真不准确。Flamingo 的空水转换过渡段动力学需通过其他方式验证。

4. **控制器增益**：水下模式的控制参数已单独存放在 `FlamingoControl_water.yaml`（通过 `underwater` 命名空间加载），但这些增益针对的是空中过渡到水中的情况，纯水下悬停可能需要进一步调整。
