# Nami 水下控制：算法流程总结与测试指南

本文汇总 Nami 空水两用四旋翼**水下模式**的整体算法流程，以及仿真/实机的测试方法与命令。
细分文档见文末索引。

---

## 1. 系统总览

Nami 有 8 个逻辑电机：**0–3 为空中旋翼**（单向，`pwm_htim1`），**4–7 为水下双向推进器**
（`pwm_htim2`，3D ESC，绕机体 y 轴 ±45° 安装，推力轴全在机体 x-z 平面内）。水下模式只用
4–7，空中电机 0–3 被掩码并强制停转。反馈只有 **IMU 姿态 + 深度计**，**没有 xyz 位置估计**
（无 mocap/VO），所以水平方向是开环。

```mermaid
flowchart LR
  subgraph Sense[传感/估计]
    IMU[IMU@spinal] --> ATT[姿态 roll/pitch/yaw]
    DEP[MS5837 / 仿真压力] --> DZ[深度 z 带符号]
  end
  subgraph Nav[NamiNavigator]
    TELE[键盘/手柄 cmd_vel] --> TGT[目标: 深度/yaw/xy加速度前馈]
    SM[状态机 START→ARM→TAKEOFF→HOVER]
    MODE[模式: ATTITUDE+DEPTH / FORWARD]
  end
  subgraph Ctrl[NamiController]
    DPID[深度手写PID] --> ALLOC[分配矩阵]
    ATTPID[roll/yaw 姿态PID] --> ALLOC
    ALLOC --> MASK[掩码空中电机 1e-6]
  end
  subgraph Spinal[spinal attitude_control]
    SAT[饱和守卫: 跳过双向+掩码] --> PWM[PWM转换]
    PWM --> BI[双向: X3表 1500us中点]
    PWM --> STOP[掩码空中: armed-stop 1ms]
  end
  Sense --> Nav --> Ctrl --> Spinal --> Motors[电机 0-3 停 / 4-7 推进]
```

---

## 2. 算法流程（分级）

### 2.1 传感与估计
- **姿态**：IMU 在 spinal 上做姿态估计，PC 侧控制器直接用 roll/pitch/yaw。
- **深度**：双源二选一，都换算成**带符号 z（向上为正，水下为负）**
  - 实机：`depth_sensor_node/depth`（`geometry_msgs/PointStamped`，MS5837 经 I2C）
  - 仿真：`pressure`（`sensor_msgs/FluidPressure`，uuv 插件，kPa）
  - 导航器对深度做 **staleness 检查**（`depth_timeout` 默认 0.25 s）+ 非有限值拒绝；
    过期即视为无深度反馈 → 深度推力归中性。
- **水平位置/速度**：无估计。xy 被钉在 `ACC_CONTROL_MODE`（纯加速度前馈）。

### 2.2 导航器 NamiNavigator
- **状态机**：`START → ARM_ON → TAKEOFF（保证 ≥1 s 窗口以激活控制器）→ HOVER`。
  水下 **HOVER = 深度保持已接通**（无位置悬停概念）。
- **遥操作**：统一走 `underwater/cmd_vel`（`Twist`），键盘和手柄共用一条链路：
  - `linear.x/y` → 机体系→世界系**加速度前馈**（开环）
  - `linear.z` → **目标深度增量**（钳 `[-max_depth, 0]`）
  - `angular.z` → **目标 yaw 增量**
- **模式开关** `surge_allocation_`（bool）：手柄十字键左 / 键盘 `m` / 话题
  `underwater/surge_allocation`（`Bool`）三种方式切换，切换时清零 xy。
- **看门狗**：`cmd_timeout`（默认 0.5 s）无指令则清零 xy 加速度（深度/yaw 目标保持）。

### 2.3 控制器 NamiController
- **电机掩码**：水下 active = 4–7；空中 0–3 以 `MASKED_MOTOR_THRUST=1e-6 N` 掩码
  （过 spinal yaw 饱和门、被 PWM 判禁用）。
- **深度环**：`depthControlLoop()` 手写 PID 替换系统 Z PID，输出世界系垂直力 [N]，
  双向对称钳（`depth_thrust_limit`）、倾斜补偿、`depth_hover_thrust` 配平微正浮力。
- **两种分配模式**（可切换，原代码都保留）：

| 模式 | 分配行 | 闭环轴 | 前进方式 | pitch |
|---|---|---|---|---|
| **ATTITUDE+DEPTH**（legacy） | (F_z, τ_x, τ_y, τ_z) | 深度/roll/pitch/yaw（4 轴） | 俯仰倾斜借力（开环平移穿过 pitch 闭环） | 主动控制 |
| **FORWARD**（surge） | (F_x, F_z, τ_x, τ_z) | 深度/roll/yaw（3 轴） | 直接 F_x 前馈（开环，无 forward PID） | 被动（spinal pitch 增益置 0，浮心自稳） |

- **模式切换处理**：`update()` 检测到切换 → reset ROLL/PITCH PID + 刷新 spinal 增益。
- **姿态增益**：经 rosserial int16 传输（×1000），**必须 < 32.7**（否则溢出成负增益发散，
  已有钳位保护）；FORWARD 模式 spinal pitch 增益发 0。

### 2.4 spinal（attitude_control，实机固件）
- 收 `four_axes/command`（每电机 base_thrust）+ `rpy/gain` + `torque_allocation_matrix_inv`。
- **双向 PWM**（`BIDIRECTIONAL_PWM2`，电机 4–7）：ApiSqueen X3 **固定** 24V 推力表分段线性，
  1000–2000 μs、1500 μs 中点（零推力）、200 Hz。**不做电压自适应**（与空中电机不同，按设计）。
- **饱和守卫**：level-1/level-2 检查**跳过双向电机与掩码电机**（否则负推力会把 yaw 执行清零）。
- **掩码空中电机** → 强制 `IDLE_DUTY`（armed-stop 1 ms 零油门），而非 min_duty，避免水下怠速旋转。
- **力控降落**：双向电机对称衰减到中点，单向电机按均值衰减。

### 2.5 执行器
- 电机 0–3（空中旋翼，`pwm_htim1`）：水下模式**恒停**（armed-stop）。
- 电机 4–7（水下双向，`pwm_htim2`）：承担全部水下推力。

---

## 3. 关键话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `underwater/cmd_vel` | Twist | 输入 | 键盘/手柄遥操作 |
| `joy` | Joy | 输入 | 手柄原始输入（joyParse 解析）|
| `underwater/surge_allocation` | Bool | 输入 | 模式切换（也可键盘 m / 手柄十字左）|
| `pressure` / `depth_sensor_node/depth` | FluidPressure / PointStamped | 输入 | 深度反馈（仿真/实机）|
| `underwater/target_depth` | Float64 | 输出(debug) | 当前目标深度，供绘图 |
| `four_axes/command` | spinal/FourAxisCommand | 输出 | 每电机推力指令 |
| `flight_state` | UInt8 | 输出 | 状态机 |

---

## 4. 测试

### 4.1 前提（每次开终端）

```bash
# 去掉 conda 的 python（否则 xacro/roslaunch 缺 rospkg 会失败）
conda deactivate 2>/dev/null
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
# source 顺序：uuv 先，jsk ws 必须带 --extend（否则丢 uuv 包/插件路径）
source /home/cc/Research/simulator/uuv_sim/devel/setup.bash
source ~/Research/jsk_aerial_robot_ws/devel/setup.bash --extend
```

### 4.2 启动仿真

```bash
# 水下仿真（headless；自动加载 ocean_waves 场景 + gazebo.underwater 描述 + underwater_mode）
roslaunch nami bringup.launch sim:=true rm:=false underwater:=true headless:=true

# 带可视化窗口
roslaunch nami bringup.launch sim:=true rm:=false underwater:=true headless:=false

# 手柄输入（仿真或实机都可加）
roslaunch nami bringup.launch sim:=true rm:=false underwater:=true joystick:=true
```

模式默认由 `NamiControl_sim.yaml` 的 `underwater/surge_allocation` 决定（仿真默认 `true` = FORWARD，
实机 `NamiControl.yaml` 默认 `false` = ATTITUDE+DEPTH 待水试）。

### 4.3 自动化测试

```bash
# A) 四轴快速测试：arm→深度保持→下潜→pitch→roll→yaw 逐项 PASS/FAIL（非零退出=失败）
#    注意：FORWARD 模式下 pitch/roll 响应两项会按设计 FAIL（姿态不随平移倾斜）；
#    要跑这个测试请先切到 ATTITUDE+DEPTH 模式。
rosrun nami underwater_flight_test.py

# B) 带标注的 demo 序列（下潜→深度保持→前/后/左/右/yaw），录 rosbag + 分段视频 + segments.json
#    先起观察相机与参照杆（可选，纯 visual 无碰撞）
rosrun gazebo_ros spawn_model -sdf -file $(rospack find nami)/scripts/demo_cam.sdf \
  -model demo_cam -x 4.2 -y -4.2 -z -0.5 -Y 2.356
rosrun gazebo_ros spawn_model -sdf -file $(rospack find nami)/scripts/demo_grid.sdf -model demo_grid
rosrun nami underwater_demo.py _out_dir:=/tmp/nami_demo

# C) 从 bag 出图（姿态/深度/xy 跟踪曲线 + summary.json）
python3 $(rospack find nami)/scripts/plot_underwater_demo.py /tmp/nami_demo
```

### 4.4 手动遥操作

```bash
# 键盘（w/s 前后, a/d 左右, r/f 上浮/下潜, q/e 偏航;
#        1=arm, 2=takeoff/深度保持, l=land, 0=halt, x=归零, m=切换模式）
rosrun nami underwater_teleop.py

# 手柄（Flamingo 同款映射，需 joystick:=true 启动）：
#   左摇杆上下=前后, 左摇杆左右=左右, 右摇杆上下=上浮/下潜, 右摇杆左右=yaw,
#   十字键左=切换模式, STOP=急停
```

### 4.5 三种方式切换控制模式

```bash
# 方式1：话题（脚本/命令行）
rostopic pub -1 /nami/underwater/surge_allocation std_msgs/Bool "data: true"    # FORWARD
rostopic pub -1 /nami/underwater/surge_allocation std_msgs/Bool "data: false"   # ATTITUDE+DEPTH
# 方式2：键盘 teleop 按 m
# 方式3：手柄十字键左
# 切换成功日志：[nami] applied FORWARD/ATTITUDE+DEPTH allocation and refreshed spinal attitude gains
```

### 4.6 快速健康检查命令

```bash
# 深度反馈是否新鲜（实机务必看均值/最小频率与时延）
rostopic hz /nami/depth_sensor_node/depth      # 实机
rostopic hz /nami/pressure                     # 仿真
# 空中电机是否停转（水下模式：0-3 应≈1e-6，4-7 承担推力）
rostopic echo -n1 /nami/four_axes/command/base_thrust
# 目标 vs 实测深度
rostopic echo /nami/underwater/target_depth
# 合并刚体名（仿真）
rostopic echo -n1 /gazebo/link_states/name
```

### 4.7 实机台架/系留/自由游动

分级 checklist 见 [underwater_real_machine_test.md](underwater_real_machine_test.md)。要点：

```bash
# 实机水下（自动：ms5837 深度计节点、禁用 mocap）
roslaunch nami bringup.launch rm:=true underwater:=true
# 深度计核对（VIM4 实测在 /dev/i2c-0）
i2cdetect -l ; i2cdetect -y 0        # 0x76 应可见 MS5837
rosservice call /nami/depth_sensor_node/calibrate   # 水面标零
```

**烧录前必做**：nami 板 `config.h` 的 `BIDIRECTIONAL_PWM2` 置 1（编译期开关，PC 无法动态改）。
台架用示波器/ESC 遥测确认电机 0–3 停脉宽、4–7 中点 1500 μs、正反向端点方向正确。

---

## 5. 已验证结果（仿真，2026-07-21）

| 验证项 | 结果 |
|---|---|
| catkin build（spinal / sim / nami）| 18 包全过 |
| 深度保持（legacy）| 4 s 漂移 0.002 m，下潜响应 −0.463 m |
| FORWARD vs legacy A/B | hold 前漂 0.15→0.01 m/s（−93%），前后对称 ±1 m |
| 深度 staleness 无回归 | pressure age 0.000 s，深度漂移 −0.001 m/8 s |
| 空中电机纯水下停转 | 0–3 精确 1.0e-6 N，4–7 活跃 −0.15 N |
| 运行时 FORWARD↔ATTITUDE 切换 | 两侧应用、空中保持掩码、深度稳、存活 |
| 空中 rostest | 2 tests，0 failures |

> 已知次要项（不阻塞上机）：深度 staleness 的 `now < source_stamp` 子条件对时间戳略超前的
> 消息（sim-time 抖动，实机不触发）偶发拒绝一帧，深度控制不受影响。

---

## 6. 相关文档索引

| 文档 | 内容 |
|---|---|
| [underwater_simulation.md](underwater_simulation.md) | 仿真环境搭建、水动力参数、模式总览 |
| [underwater_allocation_analysis.md](underwater_allocation_analysis.md) | 分配自由度数学（为什么 5 量只能选 4）|
| [underwater_forward_mode.md](underwater_forward_mode.md) | FORWARD 模式闭环结构与调参对象（只调 3 轴）|
| [depth_sensor_review.md](depth_sensor_review.md) | MS5837 driver 实时性/采样率 + 深度控制环 review |
| [underwater_real_machine_test.md](underwater_real_machine_test.md) | 实机分级测试 checklist |
| [underwater_control_review_2026-07-21.md](underwater_control_review_2026-07-21.md) | 两模式/推进器/深度链路审查 |
