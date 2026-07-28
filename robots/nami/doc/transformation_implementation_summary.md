# Nami PID/LQI 空中变形实现总结

## 1. 当前实现状态

当前实现支持 Nami 在悬停状态下驱动内部 `yaw_joint`，使前后机体之间
产生构型变化。默认 demo 使用一条 0°→30° 的慢速轨迹，在 30° 保持后
返回 0° 并降落。

截至 2026-07-28，已经完成：

- 0°～30° 全路径的离线构型稳定性扫描；
- PID 和 LQI 两套仿真控制配置；
- PID/LQI 共用的自动变形脚本、数据记录和中止处理；
- PID/LQI 仿真飞行回归；
- 一次成功的实机空中变形验证。

最后一项来自当前实机测试结论，但尚未记录测试时使用的是 PID 还是
LQI、具体增益、bag 和飞行指标。因此本文只记录“实机变形成功”，不据此
推断 PID 和 LQI 均已分别完成实机验收。

控制理论和论文对应关系见
[`aerial_transformation_control.md`](aerial_transformation_control.md)。
本文重点说明当前代码实际如何工作。

## 2. 总体数据流

当前实现没有修改通用控制器基类，而是在 `robots/nami` 的机器人模型、
派生控制器、配置和脚本中完成 Nami 特化：

```text
transformation_demo.py
        │  /nami/joints_ctrl (yaw_joint target)
        ▼
ServoBridge ───────────────► 仿真关节控制器或实机舵机
        ▲                              │
        └──── /nami/joint_states ◄────┘
                         │
                         ▼
                   NamiRobotModel
        更新 CoG、惯量、旋翼位置/方向和分配矩阵
                         │
             ┌───────────┴───────────┐
             ▼                       ▼
      NamiController          NamiLQIController
          PID                 构型相关 LQI 增益
             └───────────┬───────────┘
                         ▼
              /nami/four_axes/command
                         │
                         ▼
                 spinal / 仿真飞控
```

变形的关键不是只把舵机转到 30°，而是让实际关节反馈持续更新机器人
模型，使控制分配或 LQI 增益与当前构型一致。

## 3. 共同的构型更新和稳定性检查

### 3.1 关节命令与反馈

脚本向 `/nami/joints_ctrl` 发布 `sensor_msgs/JointState`：

```text
name:     [yaw_joint]
position: [目标角度，单位 rad]
```

`ServoBridge` 根据 `config/quad/Servo.yaml` 中的舵机编号、方向、零点和
角度比例处理命令：

- 仿真中，将角度发送给
  `effort_controllers/JointPositionController`；
- 实机中，将弧度转换为舵机 bit 值，并发布到底层舵机接口；
- 实机舵机状态返回后，再转换为弧度并发布 `/nami/joint_states`。

机器人模型订阅 `/nami/joint_states`。每次收到新反馈，
`NamiRobotModel::updateRobotModelImpl()` 都重新计算当前构型下的：

- 质心位置和期望 CoG 坐标系；
- 整机惯量；
- 各旋翼相对质心的位置；
- 各旋翼推力坐标系相对 CoG 的旋转；
- 完整 wrench 矩阵和静态悬停推力；
- 构型稳定性指标。

因此，控制器使用的是实际关节角对应的模型，而不是只根据脚本目标角
推测构型。

### 3.2 四轴分配矩阵

Nami 的四个固定旋翼直接控制

\[
y=\begin{bmatrix}z&\phi&\theta&\psi\end{bmatrix}^{T}.
\]

从完整 wrench 矩阵中构造

\[
P_4(q)=
\begin{bmatrix}
P_{F_z}(q)/m\\
I(q)^{-1}P_{\tau_x}(q)\\
I(q)^{-1}P_{\tau_y}(q)\\
I(q)^{-1}P_{\tau_z}(q)
\end{bmatrix}.
\]

`NamiRobotModel::stabilityCheck()` 检查：

1. 模型和矩阵是否已经有效初始化；
2. \(P_4(q)\) 的数值秩是否为 4；
3. 行归一化后的最小奇异值是否不低于阈值；
4. 静态悬停推力是否存在；
5. 静态推力到电机上下限的最小储备是否不低于阈值。

当前参数位于 `config/RobotModel.yaml`：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `allocation_rank_relative_threshold` | `1.0e-6` | SVD 数值秩阈值 |
| `allocation_min_scaled_singular_value` | `0.05` | 近奇异保护阈值 |
| `minimum_static_thrust_reserve` | `1.0 N` | 最小静态推力储备 |

这里的 stability check 不只是在矩阵中寻找“奇异项”或零元素，而是检查
整个四轴输入映射是否满秩、是否接近奇异，以及当前构型是否仍有悬停
推力余量。

### 3.3 检查失败时的实际行为

三个层次的处理并不完全相同：

| 层次 | 当前行为 |
|---|---|
| PID 控制器 | 发布 `model/stability=false` 并限频报错，但飞行控制循环本身继续执行 |
| LQI 控制器 | 拒绝为无效构型生成新增益；已有有效增益时保留最后一组有效增益 |
| demo 脚本 | 立即停止关节轨迹，恢复最后安全关节角，然后请求降落 |

因此，运行 demo 时真正阻止关节继续进入无效构型的是脚本；PID 控制器
中的检查目前是在线监控，不是电机输出硬切断。这种设计避免突然清零
飞行控制输出，但也意味着脱离 demo 手动发送关节命令时，操作者必须
自己监控 `/nami/model/stability`。

## 4. PID 如何在变形中控制

### 4.1 控制器选择

PID 配置中的插件为：

```yaml
aerial_robot_control_name: aerial_robot_control/nami_controller
```

对应类为 `NamiController`，它继承现有 `PoseLinearController`。

`bringup.launch` 在 `use_lqi:=false` 时加载：

- 实机：`config/quad/NamiControl.yaml`；
- 仿真：`config/quad/NamiControl_sim.yaml`。

### 4.2 控制环结构

PID 模式使用：

- x/y：位置或速度 PID 外环，输出水平期望加速度；
- z：高度 PID；
- roll/pitch：姿态 PID；
- yaw：航向 PID。

x/y 外环输出先转换为目标姿态。非小角度近似下：

\[
\phi_{\mathrm{des}}=
\operatorname{atan2}
\left(-a_y,\sqrt{a_x^2+a_z^2}\right),
\qquad
\theta_{\mathrm{des}}=\operatorname{atan2}(a_x,a_z).
\]

这里的整体飞行器 yaw 目标保持不变。内部关节名虽然叫 `yaw_joint`，
但它表示前后机体之间的内部旋转，不是让整机改变航向。

### 4.3 构型相关控制分配

`NamiController::controlCore()` 每个控制周期从 `NamiRobotModel` 读取
当前惯量、旋翼位置和推力坐标系，然后：

1. 计算每个旋翼推力对 CoG 产生的力和力矩；
2. 对 `gimbal_dof: 0` 的固定旋翼，只保留各旋翼自身 z 推力方向；
3. 对 `underactuate: true` 的 Nami 提取
   `z/roll/pitch/yaw` 四个受控轴；
4. 对当前四轴矩阵计算伪逆；
5. 将期望 z 加速度和姿态角加速度映射为各电机推力/力矩贡献；
6. 发布 `FourAxisCommand`，并按配置把姿态分配矩阵发送给 spinal。

所以 PID 变形的核心是：

> PID 增益保持固定，但机器人模型和控制分配矩阵随实际关节角持续更新。

它并不是使用 0° 构型的固定 mixer 硬撑到 30°。

## 5. LQI 如何在变形中控制

### 5.1 控制器选择和派生类职责

LQI 配置中的插件为：

```yaml
aerial_robot_control_name: aerial_robot_control/nami_lqi_controller
```

`NamiLQIController` 继承通用
`UnderActuatedLQIController`，只实现 Nami 所需的：

- 强制使用四轴 `z/roll/pitch/yaw` 模式；
- 调用 `NamiRobotModel::stabilityCheck()`；
- 发布 `/nami/model/stability`；
- 为增益生成、发布和控制循环增加同一把递归互斥锁；
- 在构型暂时无效或求解失败时保留最后一组有效增益；
- 安全结束通用 LQI 增益生成线程。

通用 LQI 基类没有为 Nami 而修改。

### 5.2 x/y 外环仍然是 PID

LQI 模式仍继承 `PoseLinearController`，因此 x/y 位置控制继续使用 PID。
水平 PID 输出的加速度按小角度关系转换为：

\[
\theta_{\mathrm{des}}=a_x/g,\qquad
\phi_{\mathrm{des}}=-a_y/g.
\]

LQI 替换的是当前构型下的四轴反馈分配，即
`z/roll/pitch/yaw`，不是 x/y 位置外环。

### 5.3 构型相关增益生成

LQI 对当前 \(P_4(q)\) 建立四组双积分器，并增加四个误差积分状态。
状态维数为 12：

```text
z, z_dot,
roll, roll_dot,
pitch, pitch_dot,
yaw, yaw_dot,
integral_z, integral_roll, integral_pitch, integral_yaw
```

代码根据 YAML 中的状态权重构造 \(Q\)，根据 `r1`～`r4` 构造输入权重
\(R\)，求解连续时间代数 Riccati 方程：

\[
A^TS+SA-SBR^{-1}B^TS+Q=0,
\qquad
K=R^{-1}B^TS.
\]

求得的每个电机增益被拆分为 z、roll、pitch、yaw 的 P/I/D 形式。这里
的“P/I/D”是 LQI 最优反馈矩阵对应到误差、误差积分和角速度/速度状态
后的表示，不是重新运行四个独立的手调 PID。

由于 Nami 是可变构型模型，增益生成线程默认以 15 Hz 重复执行：

```text
读取当前构型
  → stability check
  → 构造 P4(q)、A、B、Q、R
  → CARE 求解 K(q)
  → int16 传输限幅
  → 发布新增益
```

增益通过 `spinal::RollPitchYawTerms` 发送到底层飞控，同时通过
`/nami/debug/four_axes/gain` 发布调试值。由于 rosserial 字段按 1000
缩放后使用 `int16`，roll/pitch P、D 和 yaw D 增益会限制在
`32.767` 以内。

### 5.4 LQI 控制输出

运行控制循环时：

- x/y PID 生成目标 roll/pitch；
- z 误差、积分和速度误差乘以每个电机的 LQI z 增益，形成基础推力；
- yaw 误差、积分和角速度误差乘以每个电机的 LQI yaw 增益；
- roll/pitch 的构型相关增益发送到底层姿态闭环；
- z/yaw 前馈分别来自目标 z 加速度和目标 yaw 角加速度；
- 推力超过限制时冻结对应积分并整体缩放该轴输出。

所以 LQI 变形的核心是：

> x/y 位置外环继续使用 PID，而 z/roll/pitch/yaw 的反馈增益根据当前
> 构型在线重新生成。

## 6. PID 与 LQI 的实现对比

| 项目 | PID | LQI |
|---|---|---|
| 控制插件 | `nami_controller` | `nami_lqi_controller` |
| x/y 位置外环 | PID | PID |
| z | 固定 PID 增益 | 构型相关 LQI 增益 |
| roll/pitch | 固定 PID 增益 | 构型相关 LQI 增益 |
| yaw | 固定 PID 增益 | 构型相关 LQI 增益 |
| 构型适应方式 | 每周期更新控制分配矩阵 | 更新模型并以 15 Hz 重新求解增益 |
| stability 失败 | 报错但控制循环继续 | 保留最后有效增益，不接受新构型增益 |
| 在线计算量 | 主要是矩阵构造和伪逆 | 额外包含 CARE 求解 |
| 控制器热切换 | 不支持 | 不支持 |

两种控制器只能在进程启动时选择。飞行中不能通过修改参数从 PID 热切换
到 LQI。

## 7. 变形脚本如何实现

脚本为 `scripts/transformation_demo.py`。它不实现新的飞行控制器，只是
一个带安全检查的任务状态机：

- 通过飞行导航接口完成解锁、起飞和降落；
- 通过 `/nami/joints_ctrl` 产生平滑关节目标；
- 变形期间持续检查飞行状态和模型稳定性；
- 记录关节、位置和姿态；
- 出错时恢复最后安全构型并降落。

### 7.1 主要参数

| 私有参数 | 默认值 | 含义 |
|---|---:|---|
| `robot_ns` | `/nami` | 机器人命名空间 |
| `joint_name` | `yaw_joint` | 被驱动的内部关节 |
| `target_angle_deg` | `30.0` | 目标关节角 |
| `certified_min_angle_deg` | `0.0` | 已离线检查的下界 |
| `certified_max_angle_deg` | `30.0` | 已离线检查的上界 |
| `transform_duration` | `8.0 s` | 单程变形时间 |
| `preflight_duration` | `2.0 s` | 解锁前保持时间 |
| `settle_duration` | `5.0 s` | 悬停后等待时间 |
| `hold_duration` | `10.0 s` | 30° 保持时间 |
| `return_to_zero` | `true` | 是否返回 0° |
| `command_rate` | `40 Hz` | 关节命令频率 |
| `joint_tolerance` | `0.05 rad` | 终点跟踪误差阈值 |
| `controller_label` | `unknown` | CSV 中记录的控制器名称 |
| `csv_path` | `/tmp/nami_transformation_metrics.csv` | CSV 输出路径 |

脚本在启动阶段拒绝：

- 非正的变形时间或命令频率；
- 负的预飞保持时间；
- 上下界颠倒的认证范围；
- 超出 0°～30° 认证范围的目标。

### 7.2 发布和订阅

发布：

| Topic | 用途 |
|---|---|
| `/nami/joints_ctrl` | 发布 `yaw_joint` 目标 |
| `/nami/teleop_command/start` | 请求解锁 |
| `/nami/teleop_command/takeoff` | 请求起飞 |
| `/nami/teleop_command/land` | 请求降落 |

订阅：

| Topic | 用途 |
|---|---|
| `/nami/flight_state` | 判断 disarmed、armed、hover |
| `/nami/joint_states` | 获取实际 `yaw_joint` 角度 |
| `/nami/uav/cog/odom` | 记录位置和姿态 |
| `/nami/model/stability` | 在线构型检查 |
| `/nami/imu` | 确认 IMU 数据链已经工作 |

### 7.3 五次平滑轨迹

设归一化进度为

\[
s=\operatorname{clip}(t/T,0,1),
\]

脚本使用：

\[
h(s)=10s^3-15s^4+6s^5,
\]

\[
q_{\mathrm{cmd}}(t)=q_0+(q_f-q_0)h(s).
\]

该函数在两端的速度和加速度均为零，比阶跃命令更不容易激励机体。
脚本按 40 Hz 发布命令，每次发布后同时写入一行 CSV。

到达终点后，脚本额外等待实际关节误差满足：

\[
|q_{\mathrm{actual}}-q_f|\leq0.05\ \mathrm{rad},
\]

超时时间为 3 s。

### 7.4 状态机

完整流程为：

1. 最多等待 30 s，直到飞行状态、关节状态、IMU 和
   `model/stability=true` 全部就绪；
2. 检查实际初始关节角位于认证范围；
3. 若当前未解锁，在当前关节角保持 2 s；
4. 以 2 Hz 重复发布 start，直到进入 armed；
5. 以 2 Hz 重复发布 takeoff，最多等待 45 s，直到进入 hover；
6. 在 hover 中保持当前关节角 5 s；
7. 以五次轨迹在 8 s 内移动到 30°；
8. 检查终点关节误差；
9. 在 30° 保持 10 s；
10. 默认以相同轨迹返回 0°；
11. 在 0° 再保持 2 s；
12. 请求降落，最多等待 30 s，直到回到 disarmed。

在变形、保持和返回的每个循环中，脚本都要求：

```text
flight_state == HOVER_STATE
model/stability == true
```

任何一个条件失败都会抛出异常并进入中止流程。

### 7.5 中止和恢复

脚本将最近一次收到 `model/stability=true` 时的实际关节角保存为
`last_safe_position`。

如果运行期间发生参数错误、超时、离开 hover 或 stability 失败：

1. 停止原轨迹；
2. 若仍处于 hover，以 20 Hz 持续 2 s 发布最后安全关节角；
3. 请求降落；
4. 返回非零退出码；
5. 在 `finally` 中刷新并关闭 CSV。

该恢复只保证重新发布最后安全关节目标，不等价于硬件级紧急停机，也
不替代遥控器 kill/disarm 通道。

### 7.6 数据记录

CSV 列为：

```text
ros_time, phase, controller,
joint_target, joint_actual,
x, y, z, roll, pitch, yaw
```

其中 `phase` 可取：

```text
preflight, arming, takeoff, settle, transform, hold,
return, return_settle, landing, complete,
abort_restore, abort_landing
```

CSV 每次运行会覆盖固定路径，并约每秒刷新一次。

launch 还可以使用 `rosbag record` 保存：

- `/nami/uav/cog/odom`
- `/nami/joint_states`
- `/nami/joints_ctrl`
- `/nami/model/stability`
- `/nami/four_axes/command`
- `/nami/debug/pose/pid`
- `/nami/debug/four_axes/gain`
- `/nami/motor_info`

默认文件为：

```text
/tmp/nami_pid_transformation_<timestamp>.bag
/tmp/nami_pid_transformation.csv
/tmp/nami_lqi_transformation_<timestamp>.bag
/tmp/nami_lqi_transformation.csv
```

## 8. Launch 组织和启动选择

`bringup.launch` 通过 `use_lqi` 选择控制器和对应 YAML：

```bash
# PID
roslaunch nami bringup.launch simulation:=true real_machine:=false use_lqi:=false

# LQI
roslaunch nami bringup.launch simulation:=true real_machine:=false use_lqi:=true
```

两个仿真 demo 入口为：

```bash
roslaunch nami transformation_pid.launch
roslaunch nami transformation_lqi.launch
```

它们都包含同一个 `transformation_demo.launch`，区别只有：

| Launch | `use_lqi` | `controller_label` |
|---|---:|---|
| `transformation_pid.launch` | `false` | `pid` |
| `transformation_lqi.launch` | `true` | `lqi` |

因此 PID/LQI 使用完全相同的起飞、关节轨迹、保持、返回、降落和记录
流程，便于公平比较。

## 9. 仿真入口与实机验证的边界

当前 `transformation_demo.launch` 明确设置：

```xml
<arg name="real_machine" value="False"/>
<arg name="simulation" value="True"/>
```

所以 `transformation_pid.launch` 和 `transformation_lqi.launch` 是仿真
入口，不能直接作为规范化实机 launch。

Python 脚本本身只依赖 ROS topic，可以连接到另行启动的实机系统；
但是当前实机 `bringup.launch` 还会启动 `yaw_joint_zero_lock`，以 0.2 Hz
向同一个 `/nami/joints_ctrl` 发布 0°。如果同时运行变形脚本，会形成
两个发布者竞争。

因此，虽然当前已经完成一次成功的实机变形验证，后续要形成可重复的
实机 demo，还应增加 Nami 专用的实机安全入口，至少做到：

- 显式关闭 `yaw_joint_zero_lock`，避免命令竞争；
- 明确选择 PID 或 LQI，禁止飞行中热切换；
- 保留遥控器 kill/disarm；
- 记录使用的控制器、配置版本、bag 和关节反馈；
- 在自动解锁前加入操作者确认或独立使能条件；
- 不复用硬编码为 simulation 的 demo launch。

在完成这一入口前，不应把仿真 demo 的启动命令直接复制到实机。

## 10. 离线测试与当前限制

`test/transformation_stability.cpp` 以 1° 为间隔扫描 0°～30°，每个构型
检查：

- metrics 有效；
- 四轴分配矩阵 rank 为 4；
- `NamiRobotModel::stabilityCheck()` 通过；
- 记录整条路径的最小奇异值和最小静态推力储备。

当前实现仍有以下边界：

- 采用冻结构型/准静态假设，不补偿快速关节运动的完整耦合动力学；
- 只认证了 `yaw_joint` 的 0°～30° 一维路径；
- PID stability 失败不会在控制器内部冻结关节或中断电机输出；
- 脚本只在终点检查关节跟踪容差，没有在轨迹中设置连续跟踪误差阈值；
- 静态推力储备不等同于飞行中实时电机饱和检测；
- CSV 使用固定文件名，重复运行会覆盖；
- 两套仿真 wrapper 没有暴露全部轨迹参数；
- 实机成功测试尚未固化为独立 launch 和可复现实验记录。

## 11. 代码位置索引

| 功能 | 文件 |
|---|---|
| PID 控制器 | `src/control/nami_controller.cpp` |
| LQI 派生控制器 | `src/control/nami_lqi_controller.cpp` |
| Nami 构型模型与 stability | `src/model/nami_robot_model.cpp` |
| PID 实机/仿真配置 | `config/quad/NamiControl.yaml`、`NamiControl_sim.yaml` |
| LQI 实机/仿真配置 | `config/quad/NamiLQIControl.yaml`、`NamiLQIControl_sim.yaml` |
| 稳定性阈值 | `config/RobotModel.yaml` |
| 控制器选择 | `launch/bringup.launch` |
| 共用仿真 demo | `launch/transformation_demo.launch` |
| PID/LQI 仿真入口 | `launch/transformation_pid.launch`、`transformation_lqi.launch` |
| 变形状态机 | `scripts/transformation_demo.py` |
| 0°～30° 离线扫描 | `test/transformation_stability.cpp` |
