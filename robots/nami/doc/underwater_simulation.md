# Nami 水下动力学仿真

本文档说明如何在 Gazebo 中为 Nami 机器人添加水下动力学仿真，使用 `uuv_simulator` 提供的流体动力学插件。实现模式沿用 Flamingo 的水下仿真先例（提交 `0c4562684`），并针对 Nami 的双刚体结构做了适配。

## 背景

Nami 是一个空水两用四旋翼（gimbalrotor 构型），机体由 `yaw_joint`（z 轴 revolute，唯一主动关节）连接的头尾两段组成，每个推力单元同时带空中旋翼（`AerialRotor`）和水下旋翼（`AquaticRotor`，当前为固定几何）。

原有仿真框架已实现空中飞行动力学。本改动在其基础上叠加水下流体动力学（浮力、附加质量、阻尼），供后续水下控制算法测试。

## 实现思路

### 插件并行，互不干扰

uuv_simulator 的水动力插件在 Gazebo 物理引擎层面独立工作，**不替换**现有的 `AerialRobotHWSim`，而是与之并行：

```text
每个 Gazebo 物理步（2ms）
├── AerialRobotHWSim              ← 原有：读取 IMU/磁力计，向旋翼施加推力/扭矩
└── UnderwaterObjectPlugin        ← 新增：向机体施加浮力 + 流体阻力 + 水流力
```

两者都通过 Gazebo 刚体施力 API 叠加外力，物理引擎每步合并后积分，天然兼容。现有控制器的推力链路（spinal → rotor）在水下完全不变。

### 关键差异：Nami 是双刚体（相对 Flamingo 的单刚体）

Gazebo 的 fixed-joint reduction 会把固定关节相连的 link 合并为一个物理刚体。Nami 合并后为**两个主刚体** + 4 个推力虚 link：

| 物理刚体名 | 组成 | 质量 |
|---|---|---|
| `root` | root + base_link_tail + fc + virtual_base_link_tail + rotor_arm3/4 + gimbal_link3/4 + aquatic_rotor3/4 | 2.355 kg |
| `base_link_head` | base_link_head + rotor_arm1/2 + gimbal_link1/2 + aquatic_rotor1/2 | 1.600 kg |
| `thrust1..4` | 推力虚 link（continuous rotor joint 保留） | ~0 |

因此 `nami.gazebo.underwater.xacro` 中的 UUV 插件配置了**两个 `<link>` 块**（`root` 与 `base_link_head`），各自配平自身质量的浮力。若只给 `root` 配浮力，head 侧 1.6 kg 的重力无浮力抵消，会绕 `yaw_joint` 所在机体产生持续俯仰力矩。

另外沿用 Flamingo 的经验：使用**非 ROS 版** `libuuv_underwater_object_plugin.so`——ROS 包装版内部假设某个配置 link 名包含 `base_link` 字样，否则会空指针崩溃。

## 文件改动

| 文件 | 类型 | 说明 |
|---|---|---|
| `robots/nami/robots/quad/nami.gazebo.underwater.xacro` | 新建 | 原 gazebo xacro + UUV 水动力插件（双刚体 Fossen 模型） |
| `robots/nami/launch/bringup.launch` | 修改 | 新增 `underwater` 参数：切换 world / description_mode / spawn_z / GAZEBO 资源路径 |

水下 world 直接复用 `uuv_gazebo_worlds/worlds/ocean_waves.world`（即 `roslaunch uuv_gazebo_worlds ocean_waves.launch` 所用场景）：波浪海面(z=0)、沙地海床、2ms 步长、洋流插件（默认流速 0，发布 `/hydrodynamics/current_velocity`）。该 world 依赖 `model://ocean` 等资源，launch 已在 underwater 模式下自动设置 `GAZEBO_MODEL_PATH` / `GAZEBO_RESOURCE_PATH`，无需手动导出。

`underwater` 默认 `False`，原有空中仿真命令完全不受影响。

## 使用方法

### 前提条件

uuv_simulator 已编译于 `/home/cc/Research/simulator/uuv_sim`。启动前需叠加 source，使 `$(find uuv_gazebo_worlds)` 可解析、Gazebo 能找到 `libuuv_*` 插件：

```bash
source /home/cc/Research/simulator/uuv_sim/devel/setup.bash
source ~/Research/jsk_aerial_robot_ws/devel/setup.bash --extend
```

**注意**：第二个 source 必须带 `--extend`，否则会重置环境链，uuv 的包路径和插件库路径会丢失（已实测）。另外若 conda 处于激活状态需先 `conda deactivate`（conda 的 python3 没有 rospkg，xacro/roslaunch 会失败）。

### 启动命令

```bash
# 原有空中仿真（不受影响）
roslaunch nami bringup.launch sim:=true

# 水下仿真（默认 spawn 在 z=-1.0）
roslaunch nami bringup.launch sim:=true underwater:=true

# 水下仿真 + 可视化窗口
roslaunch nami bringup.launch sim:=true underwater:=true headless:=false

# 运行中设置水流（默认流速 0）
rostopic pub /hydrodynamics/current_velocity_model \
  uuv_world_ros_plugins_msgs/SetCurrentVelocity \
  '{velocity: 0.5, horizontal_angle: 0.0, vertical_angle: 0.0}'
```

注意：UUV 浮力仅在刚体 z<0 时生效，水下模式必须以负的 `spawn_z` 入水（launch 已默认 -1.0）。

## 参数调整指南

`nami.gazebo.underwater.xacro` 中所有水动力参数均为几何估算初值，需在仿真中迭代调参。

### 浮力（volume）

各刚体独立配平：

$$V_{neutral} = m / \rho$$

| 刚体 | 质量 | 中性体积 | 当前值（约 +1%） |
|---|---|---|---|
| `root` | 2.355 kg | 0.002355 m³ | 0.00238 m³ |
| `base_link_head` | 1.600 kg | 0.001600 m³ | 0.00162 m³ |

当前总净上浮力约 0.4 N（微正浮力，断电可安全上浮）。**警告**：体积远超中性值会把机器人弹出水面并翻覆，每次只做小幅调整。

### 重浮心偏置（center_of_buoyancy）

CoB 在各刚体坐标系中定义，置于该刚体 CoG 正上方约 0.08 m：

| 刚体 | 刚体 CoG（本体系） | 当前 CoB |
|---|---|---|
| `root` | (-0.105, 0.020, 0.023) | (-0.105, 0.020, 0.10) |
| `base_link_head` | (0.194, 0.000, -0.022) | (0.194, 0.000, 0.06) |

CoB 高于 CoG 产生横滚/俯仰被动回复力矩（静稳定）。偏置越大回正越强，但控制器主动改变姿态的负担也越大。

### 附加质量（added_mass）

对角近似：平移方向取排水质量的 ~30%（heave 因扁平机体取大），转动取小值。精确值需 CFD 或系统辨识。

### 阻尼（linear_damping / quadratic_damping）

- 线性阻尼：低速 Stokes 区主导，当前取小值抑制低速漂移
- 二次阻尼：高速主导，按 $0.5\,C_d\,\rho\,A$ 估算（$C_d \approx 1.0$，A 取 collision box 迎流面积，z 向额外加 4 个旋翼盘面 ≈ 0.05 m²/个）

调参方法：用 `rosservice call /gazebo/apply_body_wrench` 给刚体一个脉冲力，观察速度衰减曲线与预期拖曳特性对比迭代。

## 验证清单

1. **合并刚体名**：`rostopic echo -n1 /gazebo/link_states | grep nami`，应只见 `nami::root`、`nami::base_link_head`、`nami::thrust1..4`。若名字不符，修改 xacro 中插件的 `<link name>`。
2. **插件加载**：`rostopic list | grep hydrodynamics` 应有 `/hydrodynamics/current_velocity`；Gazebo 终端无 dlopen 报错。
3. **浮力配平**：spawn 后机器人从 z=-1 缓慢上浮趋近水面平衡，不弹飞、不下沉（`rostopic echo /nami/ground_truth/pose`）。
4. **阻尼生效**：施加脉冲力后速度指数衰减，而非匀速漂移。
5. **推力接口**：`rostopic pub /nami/teleop_command/start std_msgs/Empty` → `.../takeoff`，旋翼推力仍有响应；空中仿真（不带 `underwater`）无回归。

## 已知局限性

1. **参数为估算值**：附加质量与阻尼基于几何粗估，实际值需实验/CFD 辨识。
2. **水中推力仍用空中旋翼模型**：推力曲线来自 MotorInfo（空气介质），真实水下推进特性不同，属后续阶段（水下推进建模 + 控制）。
3. **无自由液面效应**：uuv_simulator 不模拟水-空气界面，水面附近（跨介质过渡段）动力学不准。
4. **yaw_joint 水下负载**：两刚体各自受浮力/阻尼，舵机 effort 限制（6.6 Nm）能否持位需仿真观察。
