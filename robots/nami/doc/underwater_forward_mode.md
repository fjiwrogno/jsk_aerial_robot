# Nami 水下两种控制模式：闭环结构与调参对象

回答一个具体问题：**FORWARD（surge）模式下是不是只需要调 3 个轴的参数？因为 forward 方向不是闭环。**

**结论：是的。** FORWARD 模式下真正的闭环控制轴只有 3 个——**roll、yaw、depth**，
需要调 PID 增益。forward（前进/后退）是**开环加速度前馈**，没有 PID 可调，只有一个前馈
标定系数；pitch 被动稳定（spinal 增益已置零）；sway（左右）没有推力权威。下面给出代码依据。

## 两种模式的命名与切换

`NamiController` 里两种水下分配模式（由 `underwater/surge_allocation` 参数 / joystick / 键盘切换）：

| 模式 | 分配行 | 别名（日志） |
| --- | --- | --- |
| 传统姿态控制 | (F_z, τ_x, τ_y, τ_z) = 深度力 + roll/pitch/yaw 力矩 | `ATTITUDE+DEPTH` |
| surge 前进控制 | (F_x, F_z, τ_x, τ_z) = 前向力 + 深度力 + roll/yaw 力矩 | `FORWARD` |

## 为什么 forward 不是闭环

水下**没有 xyz 位置/速度估计**（无 mocap、无 VO），所以导航器把 xy 钉死在
`ACC_CONTROL_MODE`。在这个模式下，位置环退化为纯前馈——看 `PoseLinearController` 的 PID 调用：

```cpp
// ACC_CONTROL_MODE
pid_controllers_.at(X).update(0, du, 0, target_acc_.x());  // err_p=0, err_d=0, 只有前馈
```

PID 的 `result = p_term + i_term + d_term + feedforward`。err_p=0 → p_term=0；
err_i 累加的是 err_p·du=0 → i_term=0；err_d=0 → d_term=0。所以

```
X_PID.result() == target_acc_.x()   （X 的 P/I/D 增益乘的全是 0，完全不起作用）
```

也就是说：**前进方向的输出 = 遥操作摇杆给的加速度前馈，直接映射成 F_x 推力，没有任何
反馈修正。** 你摇杆推多大、机器人实际走多快，取决于推力与水阻的平衡，系统不去闭环纠正它
（这正是"不闭环"的含义）。

### forward 方向唯一能调的量（不是 PID）

- `underwater/teleop_acc_gain`：摇杆 [-1,1] → 机体加速度前馈 [m/s²] 的换算
- `underwater/surge_acc_limit`：F_x 前馈的钳位上限 [m/s²]

这两个是**前馈标定/限幅**，不是闭环增益。调它们改变"同样的摇杆量给多大推力"，但不改变
稳定性——因为没有反馈回路。

## FORWARD 模式各轴到底谁在闭环

`controlCore()` 里 surge 分配路径：

| 轴 | 来源 | 闭环？ | 要调的参数 |
| --- | --- | --- | --- |
| **F_x（前进）** | X PID 的纯前馈（ACC 模式） | ❌ 开环前馈 | teleop_acc_gain / surge_acc_limit（前馈，非 PID） |
| **F_z（深度）** | `depthControlLoop()` 手写深度 PID | ✅ 闭环 | `underwater/depth_p/i/d_gain` 等 |
| **τ_x（roll）** | ROLL PID（IMU 姿态反馈） | ✅ 闭环 | `underwater/controller/roll_pitch` |
| **τ_z（yaw）** | YAW PID（IMU 姿态反馈） | ✅ 闭环 | `underwater/controller/yaw` |
| **pitch** | 目标恒 0 + spinal pitch 增益置零 | ⏸ 被动 | 不用调（浮心回复力矩自稳） |
| **sway（左右）** | 无 F_y 权威，摇杆被忽略 | — | 无 |

所以 **3 个闭环轴 = roll + yaw + depth**，这就是你说的"3 个轴"。pitch 在 FORWARD 模式
被动（代码里 `target_pitch_=0` 且 `setAttitudeGains()` 把 spinal 的 pitch p/i/d 发 0，
避免积分空转），不需要调参。

## 对照：ATTITUDE+DEPTH 模式要调 4 个闭环轴

传统模式下前进/后退是靠**俯仰倾斜借力**——加速度前馈经 hovering 近似写成
`target_pitch_`，再经 pitch 闭环产生倾角、由倾斜的推力分量推着走。所以那边前进仍然是开环
平移（无位置反馈），但它**穿过 pitch 闭环**，于是需要调的闭环轴是：

- roll、pitch、yaw（三个姿态轴）+ depth = **4 个闭环轴**

两模式的差别可以一句话概括：

- **ATTITUDE+DEPTH**：前进 = 调好的 pitch 闭环去倾斜借力（4 闭环轴，pitch 要调）。
- **FORWARD**：前进 = 直接给 F_x 前馈推力，pitch 交给浮力自稳（3 闭环轴，pitch 不用调）。

## 调参落点速查

FORWARD 模式实机调参时只碰这三处（都在 `NamiControl.yaml` 的 `underwater:` 段）：

```yaml
underwater:
  # 1. 深度闭环（手写 PID）
  depth_p_gain / depth_i_gain / depth_d_gain / depth_hover_thrust / depth_thrust_limit
  controller:
    # 2. roll 闭环
    roll_pitch: { p_gain, i_gain, d_gain }   # pitch 分量在 FORWARD 下被 spinal 忽略
    # 3. yaw 闭环
    yaw: { p_gain, i_gain, d_gain }
  # forward 前馈（非闭环，改手感不改稳定性）
  teleop_acc_gain / surge_acc_limit
```

> 注意 spinal 姿态增益经 rosserial int16 传输（×1000），**必须 < 32.7**，否则溢出成负增益
> 正反馈发散（已有钳位保护，但调参时别越界）。

相关：[underwater_allocation_analysis.md](underwater_allocation_analysis.md)（为什么 5 个量只能选 4 个、
FORWARD 舍掉 τ_y 的数学）、[underwater_simulation.md](underwater_simulation.md)（总览）。
