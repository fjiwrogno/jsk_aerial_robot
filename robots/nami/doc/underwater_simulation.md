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
| --- | --- | --- |
| `root` | root + base_link_tail + fc + virtual_base_link_tail + rotor_arm3/4 + gimbal_link3/4 + aquatic_rotor3/4 + added_mass_ballast_tail | 2.355 + 1.0 = 3.355 kg |
| `base_link_head` | base_link_head + rotor_arm1/2 + gimbal_link1/2 + aquatic_rotor1/2 + added_mass_ballast_head | 1.600 + 0.7 = 2.300 kg |
| `thrust1..8` | 推力虚 link（continuous rotor joint 保留） | 0.01 kg/个 |

推力虚 link 使用 0.01 kg 而非惯例的 1e-5 kg：水下两主刚体带 30N 级外力，1:335000 的质量比会恶化 ODE 约束求解条件数（详见 xacro 内注释）。

因此 `nami.gazebo.underwater.xacro` 中的 UUV 插件配置了**两个 `<link>` 块**（`root` 与 `base_link_head`），各自配平自身质量的浮力。若只给 `root` 配浮力，head 侧 1.6 kg 的重力无浮力抵消，会绕 `yaw_joint` 所在机体产生持续俯仰力矩。

另外沿用 Flamingo 的经验：使用**非 ROS 版** `libuuv_underwater_object_plugin.so`——ROS 包装版内部假设某个配置 link 名包含 `base_link` 字样，否则会空指针崩溃。

## 文件改动

| 文件 | 类型 | 说明 |
| --- | --- | --- |
| `robots/nami/robots/quad/nami.gazebo.underwater.xacro` | 新建 | 原 gazebo xacro + UUV 水动力插件（双刚体 Fossen 模型） |
| `robots/nami/launch/bringup.launch` | 修改 | 新增 `underwater` 参数：切换 world / description_mode / spawn_z / GAZEBO 资源路径 |

水下 world 直接复用 `uuv_gazebo_worlds/worlds/ocean_waves.world`（即 `roslaunch uuv_gazebo_worlds ocean_waves.launch` 所用场景）：波浪海面(z=0)、沙地海床、2ms 步长、洋流插件（默认流速 0，发布 `/hydrodynamics/current_velocity`）。该 world 依赖 `model://ocean` 等资源，launch 已在 underwater 模式下自动设置 `GAZEBO_MODEL_PATH` / `GAZEBO_RESOURCE_PATH`，无需手动导出。

`underwater` 默认 `False`，原有空中仿真命令完全不受影响。

相关文档：[underwater_allocation_analysis.md](underwater_allocation_analysis.md)（分配自由度数学）、
[underwater_real_machine_test.md](underwater_real_machine_test.md)（实机测试准备与 checklist）。

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

| 刚体 | 质量（含压载） | 中性体积 | 当前值（约 +1%） |
| --- | --- | --- | --- |
| `root` | 3.355 kg | 0.003355 m³ | 0.00339 m³ |
| `base_link_head` | 2.300 kg | 0.002300 m³ | 0.002329 m³ |

当前总净上浮力约 0.6 N（微正浮力，断电可安全上浮）。**警告**：体积远超中性值会把机器人弹出水面并翻覆，每次只做小幅调整。

### 重浮心偏置（center_of_buoyancy）

CoB 在各刚体坐标系中定义，置于该刚体 CoG 正上方约 0.08 m：

| 刚体 | 刚体 CoG（本体系） | 当前 CoB |
| --- | --- | --- |
| `root` | (-0.105, 0.020, 0.023) | (-0.105, 0.020, 0.10) |
| `base_link_head` | (0.194, 0.000, -0.022) | (0.194, 0.000, 0.06) |

CoB 高于 CoG 产生横滚/俯仰被动回复力矩（静稳定）。偏置越大回正越强，但控制器主动改变姿态的负担也越大。

### 附加质量（added mass）——烘焙为真实惯量

**不要使用 uuv 插件的 `<added_mass>`**：该插件用数值微分加速度做显式反馈（其源码 TODO 自述可能振荡），配合 nami 的双刚体 + 波浪场，即使 scaling 0.1 也会间歇触发 `Linear or angular accelerations are invalid` 断言崩溃。

替代方案：`nami.gazebo.underwater.xacro` 顶部的 `added_mass_ballast_tail/head` 虚 link（fixed joint，被 gazebo 合并进两主刚体）把附加质量做成**真实惯量**——ODE 隐式求解，无条件稳定：

- 平移项取估算对角阵的均值（tail: mean(0.7, 0.9, 1.5) ≈ 1.0 kg；head: mean(0.5, 0.6, 1.0) ≈ 0.7 kg），各向异性无法表达（真实质量各向同性）
- 转动项由压载 link 的惯量张量**精确**表达
- 浮力体积同步放大 Δm/ρ，静配平不变（物理附加质量不带重量，这里增重与增浮相消）
- 附带收益：robot_description 质量包含附加质量，控制器的力分配自动补偿

调整时同步改三处：压载 link 质量/惯量、对应刚体的插件 `<volume>`、本文档表格。插件内 `<added_mass>` 保持全零。

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

## 水下模式（dev_goal2）

在上述仿真环境上实现了与空中飞行相区分的**水下模式**：水下推力仅来自 4 个双向 aquatic rotor（逻辑电机 4~7，推力沿各自 z 轴，**绕机体 y 轴 ±45° 倾斜安装**——前对推力轴 (−0.707, 0, +0.707)、后对 (+0.707, 0, +0.707)，全部位于机体 x-z 平面内，因此有前后（surge）推力权威、无侧向（sway）权威；由 Gazebo link 姿态实测确认），反馈仅用 IMU 姿态 + 深度计（无 xyz 位置反馈）。

### 架构

- **8 逻辑电机并存**：URDF 定义 thrust1..8 / rotor1..8；电机 0~3 = 空中旋翼、4~7 = aquatic rotor（推力范围 [−14.7, +25.48] N，取自 ApiSqueen X3 表端点）。各模式只分配自己的 4 个，另一组以 1e-6 N 掩码（过 spinal 门控、被 pwm 判禁用）。
- **NamiController**：`MODE{AERIAL, UNDERWATER}` 由 `underwater_mode` 参数决定；水下增益从 `underwater/controller/*` 加载；深度反馈直接进控制器（`pressure` [FluidPressure, 仿真] / `depth_sensor_node/depth` [PointStamped, 实机] 双源），手写深度 PID 替换系统 Z PID 输出（双向对称钳 ±`depth_thrust_limit`，`depth_hover_thrust` 配平微正浮力）。
- **NamiNavigator**：水下强制 `ACC_CONTROL_MODE`（xy 为开环加速度前馈）；订阅 `underwater/cmd_vel`（Twist）：`linear.x/y` → 机体系→世界系加速度前馈、`linear.z` → 目标深度增量（钳 [−max_depth, 0]）、`angular.z` → 目标 yaw 增量。指令流超过 `underwater/cmd_timeout`（默认 0.5 s）未更新时，watchdog 自动清零 xy 加速度，同时保留目标深度和 yaw。
- **状态机语义**：保留原状态机，但水下 **HOVER = 「深度保持已接通」**（无位置悬停概念）；TAKEOFF 保证 1.0s 最小窗口以确保控制器激活，随后自动进入 HOVER。
- **姿态增益上限**：spinal 增益经 rosserial int16 传输（×1000），**必须 < 32.7**，超出会溢出为负增益（正反馈发散）。当前水下 roll/pitch p=25/i=12/d=6，浮心恢复力矩（≈4.5 N·m/rad）的稳态差距由积分项补偿。
- **spinal 饱和守卫**：双向电机的负推力不适用单向电机的 min_thrust 饱和模型；`NamiControl_sim.yaml` 的 `bidirectional_motor_start/end: 4/8` 使仿真版 spinal 在 level-1/level-2 饱和检查中跳过双向电机与掩码电机（否则 yaw 执行会被清零）。实机由 Phase B 的 `BIDIRECTIONAL_PWM2` 宏承担同一角色。
- **surge 分配模式（可选，`underwater/surge_allocation`）**：把分配行从传统的 ($F_z$, $\tau_x$, $\tau_y$, $\tau_z$) 换成 ($F_x$, $F_z$, $\tau_x$, $\tau_z$)——机体 x 力被主动调节到指令值（0 = 消除前漂），前进/后退变为直接推力控制；$\tau_y$ 成为从动量，pitch 交给浮心回复力矩被动稳定（spinal pitch 增益同步置零防积分空转），侧向摇杆被忽略（本就无 sway 权威）。`surge_acc_limit`（默认 1.0 m/s²）钳制 surge 加速度前馈。数学依据见 [underwater_allocation_analysis.md](underwater_allocation_analysis.md)。默认关闭（legacy 行为不变）；仿真 yaml 已开启、实机 yaml 保持关闭待水试。注意：开启后 `underwater_flight_test.py` 的 pitch/roll 响应两项会按设计 FAIL（姿态不再随平移指令倾斜），验证请改用 `underwater_demo.py`。

### 使用

```bash
# 水下模式启动（自动加载 ocean_waves 场景、gazebo.underwater 描述、underwater_mode 参数）
roslaunch nami bringup.launch sim:=true underwater:=true

# 键盘遥操作（w/s 前后, a/d 左右, r/f 上浮/下潜, q/e 偏航; 1=start 2=进入深度保持 l=land 0=halt）
rosrun nami underwater_teleop.py

# 自动化四轴测试（arm→深度保持→下潜→pitch→roll→yaw，逐项 PASS/FAIL，非零退出码=失败）
rosrun nami underwater_flight_test.py
```

### 结果展示（demo 录制与绘图）

`scripts/` 下提供一套自动化展示流水线（输出 rosbag、每段 mp4 视频、跟踪曲线图与数值摘要）：

```bash
# 1. 启动水下仿真（headless 即可，相机渲染在 gzserver 内完成）
roslaunch nami bringup.launch sim:=true rm:=false underwater:=true headless:=true

# 2. 生成观察相机（跟随机器人）与视觉参照杆阵列（可选，纯 visual 无碰撞）
rosrun gazebo_ros spawn_model -sdf -file $(rospack find nami)/scripts/demo_cam.sdf \
  -model demo_cam -x 4.2 -y -4.2 -z -0.5 -Y 2.356
rosrun gazebo_ros spawn_model -sdf -file $(rospack find nami)/scripts/demo_grid.sdf -model demo_grid

# 3. 运行标注好的指令序列（下潜→深度保持→前/后/左/右/偏航），自动录 bag 和分段视频
rosrun nami underwater_demo.py _out_dir:=/path/to/out

# 4. 从 bag 生成图（attitude_tracking / depth_tracking / xy_motion .png + summary.json）
python3 $(rospack find nami)/scripts/plot_underwater_demo.py /path/to/out
```

深度保持目标值通过 debug 话题 `underwater/target_depth`（std_msgs/Float64）发布，供绘图对比。
视频为 mp4v 编码（本机 OpenCV 无 H.264），VLC/mpv 可直接播放，浏览器不行。

## 已知局限性

1. **参数为估算值**：附加质量与阻尼基于几何粗估，实际值需实验/CFD 辨识。
2. **仿真不经过 PWM 路径**：仿真施力走 `getForce()=target_thrust_`，双向推力靠 URDF rotor joint limit 钳位；实机的推力→PWM 转换（ApiSqueen X3 表、1500μs 中点、200Hz）已由 `dev/bi_directional_pwm` 合并进本分支（2026-07-19，台架实机验证通过）。注意：双向转换关系**固定**（24V 表、无 `v_factor_` 电压自适应，与空中电机不同，按设计）；混合构型下被掩码的空中电机在实机被强制 armed-stop（1ms）而非 min_duty 怠速（此差异仿真不可见）。实机准备详见 [underwater_real_machine_test.md](underwater_real_machine_test.md)。
3. **无自由液面效应**：uuv_simulator 不模拟水-空气界面，水面附近（跨介质过渡段）动力学不准。
4. **yaw_joint 水下负载**：两刚体各自受浮力/阻尼，舵机 effort 限制（6.6 Nm）能否持位需仿真观察。
5. **恒定前向漂移（surge 力泄漏，已定量确认）**：aquatic rotor 推力轴全部位于机体 x-z 平面（绕 y ±45°），任何前后推力不平衡都直接产生机体 +x/−x 净力；而 underactuate 分配只提取 (z, roll, pitch, yaw) 四行，x 力不在方程中。深度保持稳态下，控制器 base thrust + spinal roll/pitch 积分配平项合成的机体系净力实测为 (+2.62, 0.00, −1.24) N，与 0.16 m/s 漂速下的水阻 2.61 N 精确平衡（误差 0.2%）。电机关闭后漂速立即归零，排除外部流场因素。**这不是 pitch 姿态控制问题**（保持段 pitch RMSE 0.76°）。改进方向：把 x 力行纳入分配（前后差动直接控 surge），或对漂移做速度闭环。分配自由度的完整数学分析（为什么 5 个量只能选 4 个、该选哪 4 个）见 [underwater_allocation_analysis.md](underwater_allocation_analysis.md)。
6. **无侧向（sway）推力权威**：推力轴无任何 y 分量，左右平移只能靠横滚折转净垂直推力。但水下配平时净垂直推力仅 ~1 N 且**方向向下**（抵消正浮力），横滚 24° 产生的侧向力 <0.5 N 且与期望方向**相反**（空中"倾斜即平移"的直觉在净推力向下时失效）。实测 left/right 指令 10 s 位移只有 ∓0.2 m 且反向。侧向机动建议用 yaw+前进组合实现。
7. **aquatic rotor 反扭矩未标定**：`m_f_rate` 沿用空中值、方向交替近似，yaw 权威已实测足够（实测 ~0.35 rad/s 稳态角速度），精确值待实机标定。目前 teleop 的 yaw 目标增速（0.5 rad/s/单位摇杆）快于执行能力，目标会跑在实际前面，建议按实际权威限速。
