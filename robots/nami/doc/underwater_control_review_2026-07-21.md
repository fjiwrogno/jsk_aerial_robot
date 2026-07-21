# Nami 水下两种控制模式、推进器与深度链路审查

日期：2026-07-21

## 结论

- 空中模式仍只分配电机 0～3，水下模式只分配电机 4～7；空中 PWM 的
  `pwm_htim1` 初始化与转换关系没有因双向推进器改变。
- 水下实机固件启用 `BIDIRECTIONAL_PWM2` 后，电机 4～7 使用 1000～2000 us、
  1500 us 中点的双向转换。水下模式中电机 0～3 的显式掩码会被转换为
  `IDLE_DUTY`，即 armed-stop，不参与姿态或深度控制。
- 修复了一个空中模式边界回归：旧判定把任意“小于 1e-5 N”的空中电机指令
  都当作水下掩码。现在只识别 PC 明确发送的 1e-6 N 掩码哨兵，因此空中模式
  的低油门/零推力行为保持原逻辑。
- legacy（姿态+深度）仿真七项全部通过；现有空中 hovering rostest 也通过。
- forward/surge 模式可在运行时用键盘 `m` 或 joystick 十字键左切换；切换时
  清零 x/y 指令、重置 roll/pitch PID，并立即刷新 spinal pitch 增益。
- MS5837 ROS 节点原来存在启动即失败的 `rospy.advertise` API 错误，以及深度
  正负号与 Nami 约定相反的问题，均已修复。控制侧新增 0.25 s 数据超时与
  header 延迟检查；失去新鲜深度反馈时输出中性深度推力。

## 两种控制模式

| 模式 | 分配量 | 闭环 | 平移语义 | 限制 |
| --- | --- | --- | --- | --- |
| 姿态+深度（legacy） | Fz、roll、pitch、yaw | 深度、roll、pitch、yaw | forward/lateral 指令先变成目标倾角，水平位置和速度不开环纠偏 | Fx 没有进入分配约束，会有固有前向力泄漏；侧移权威很弱 |
| forward/surge | Fx、Fz、roll、yaw | 深度、roll、yaw | forward 是直接 Fx 前馈 | pitch 为被动浮心回复，无 pitch 执行通道；没有 sway 权威，lateral 被忽略 |

### forward 模式是不是只调三个轴参数？

如果“参数”指 PID，答案是是：需要调深度、roll、yaw 三个反馈环；pitch PID 在
forward 模式中被置零，因为四推进器的可用四自由度已经分配给 Fx、Fz、roll、yaw。
forward/surge 本身不是速度或位置闭环，所以没有 forward P/I/D 可调。

但 forward 仍有非 PID 参数需要标定：

- `teleop_acc_gain`：摇杆到期望前向加速度的比例；
- `surge_acc_limit`：前向指令限幅；
- 双向推进器推力-PWM 表、安装角度和 `rotor_devider`：决定期望 Fx 与真实 Fx
  是否一致。

因此当前 forward 能“按指令分配 Fx”，但不能消除水流、模型误差或载荷变化造成
的速度误差。若以后要求定速或定点，必须增加 DVL/视觉/声学等速度或位置反馈，
再增加 surge 闭环；仅继续增大 `teleop_acc_gain` 不能解决这类误差。

## Joystick 映射

沿用 aquatic Flamingo 的轴定义（经过公共 `joyParse()`，兼容已有 PS3、PS4、
蓝牙手柄与 ROG1 映射）：

| 输入 | 水下指令 |
| --- | --- |
| 左摇杆上下 | forward/backward |
| 左摇杆左右 | lateral（forward 模式忽略） |
| 右摇杆上下 | 上浮/下潜目标深度速率 |
| 右摇杆左右 | yaw 目标速率 |
| 十字键左 | 姿态+深度 / forward 模式切换 |
| STOP/Share | STOP 状态 |

键盘原映射保留，新增 `m` 切换模式。`underwater/cmd_timeout=0.5 s` 同时保护
键盘和 joystick；手柄断连后 x/y 开环指令自动归零，深度与 yaw 目标保持。

## 深度计实时性与采样设置

Nami 主循环为 40 Hz。MS5837 一次 `read()` 包含压力 D1 与温度 D2 两次 ADC
转换；OSR 越高，两次阻塞等待越长。使用 Python、Linux 调度和 VIM4 I2C 时，
它不是硬实时链路，因此不能只看配置的 `rate`，还必须看消息实际频率和时延。

当前默认选择：

- 50 Hz；保证每个 40 Hz 控制周期通常能获得一份新样本，同时保留调度余量；
- OSR1024；精度足够用于池内深度控制，延迟显著小于 OSR8192；
- queue size 1；不累计旧深度；
- 转换完成后打时间戳；控制侧拒绝超过 0.25 s 的旧消息；
- 水面调用 `rosservice call /nami/depth_sensor_node/calibrate`，用当时绝对压力
  作为零深度，避免天气和海拔变化直接形成深度偏置；
- 淡水默认 997 kg/m3；盐水应改为约 1029 kg/m3 或现场测量值。

实机首次测试必须记录：`rostopic hz` 的均值/最小频率、header 到接收时间差、
连续 I2C 错误、深度阶跃噪声和闭环超调。若 50 Hz 仍有明显长尾延迟，优先降低
系统负载、提高进程调度优先级或把采集迁到 MCU，而不是盲目提高 ROS `rate`。

## 验证结果

- `catkin build nami --no-status`：18 个相关包成功。
- `catkin build ms5837_ros --no-status`：成功。
- 水下 legacy Gazebo：arm、深度保持、下潜、pitch、roll、yaw、末端存活全部 PASS；
  深度保持 4 s 漂移 0.002 m，下潜响应 -0.463 m。
- 运行时切换 forward：日志确认分配与 spinal pitch 增益同步切换；
  `four_axes/command.base_thrust` 的电机 0～3 均为 1e-6 N 掩码，电机 4～7
  承担水下推力。
- 空中 `nami` rostest：2 tests、0 errors、0 failures；起飞、hover、轨迹与降落通过。
- Python 节点语法检查、launch XML 检查、`joystick:=true` 节点展开检查通过。

## 实机仍需确认

- `BIDIRECTIONAL_PWM2` 是板级编译开关，默认保持 0 以免影响共享固件的其他机器人；
  烧录 Nami 水下固件时必须对实际板型设为 1。软件无法从 PC 的
  `underwater_mode` 动态改变 MCU 定时器编译配置。
- 必须用示波器或 ESC 遥测确认电机 0～3 在水下 arm/takeoff 时为停止脉宽，
  电机 4～7 零推力为 1500 us，正反向端点和物理方向正确。
- 当前 forward 是开环 Fx，真实速度会随电池、流场和推进器个体差异变化；首次
  自由游动应从小 `surge_acc_limit` 开始。
