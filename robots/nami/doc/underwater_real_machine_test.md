# Nami 水下实机测试准备与 Checklist

目标：把水下模式（dev_goal2）与 surge 分配模式带上实机。本文档列出软件准备状态、
烧录/启动步骤、以及台架 → 系留 → 自由游动的分级测试清单。仿真基线数据在
`demo_output/run3`（legacy 分配）与 `demo_output/run4_surge`（surge 分配），
分配原理见 [underwater_allocation_analysis.md](underwater_allocation_analysis.md)。

## 1. 软件准备状态

| 项 | 状态 | 说明 |
| --- | --- | --- |
| PC 侧水下模式（深度环/掩码分配/遥操作） | ✅ 就绪 | 仿真七项测试全绿（2026-07-11） |
| PC 侧 surge 分配模式 | ✅ 就绪 | 仿真 A/B 验证：漂移 0.15→0.01 m/s（2026-07-19）；实机 yaml 默认 legacy |
| spinal 双向 PWM 转换（X3 表 / 1500μs 中点 / 200Hz) | ✅ **已实机验证** | `dev/bi_directional_pwm` 台架验证通过并已 merge 进本分支（2026-07-19） |
| 双向转换关系固定化 | ✅ 就绪 | `convertBiDirectional` 不做电压自适应（去掉 `v_factor_`）：X3 表 24V 数据即最终关系，与空中电机的电压补偿转换不同（按设计） |
| 掩码电机水下停转修复 | ✅ 代码就绪 | 混合 8 电机构型下被掩码的空中电机原会被钳到 `min_duty_` **在水里怠速旋转**（仿真不过 PWM 路径故未暴露），已改为 armed-stop（1ms 零油门）。**台架需复验此项** |
| spinal 饱和守卫（双向电机跳过 min_thrust 模型） | ✅ 就绪 | 宏门控，仿真同逻辑已验证 |
| motor_pwms 遥测 | ✅ 就绪 | 双向电机显示真实脉宽 μs（0.5→1500），便于台架比对 |
| ms5837 深度计节点 | ✅ 代码修复 | `rospy.advertise` → `Publisher`/`Service` API bug 已修，**未经硬件实测** |
| launch 接线（underwater 时启动深度计、禁用 mocap） | ✅ 就绪 | `sensors.launch.xml` |
| 姿态增益 int16 钳位（<32.7） | ✅ 就绪 | 溢出保护 + ROS_ERROR |
| 力控降落（force landing）双向电机归零衰减 | ✅ 代码就绪 | 宏门控，随固件台架复验 |

安全链路审计结论（2026-07-19，仿真+实机双路径核查）：所有"电机关闭"路径
（上电、disarm、halt、pwm test 回退、命令超时→force landing 衰减完成）对双向电机
输出的都是 `IDLE_DUTY=0.5` → 1500 μs 中点停转——0.5 恰好同时是单向电机的
armed-stop 占空比和双向电机的中点，惯例自洽；定时器初始化从第一个脉冲起即为中点。
命令超时（500 ms）走 force landing 而非直接切零，双向推力对称衰减到中点。

## 2. 固件烧录

1. nami 使用的 spinal 板对应 `mcu_project/lib/Jsk_Lib/configs/<板型>/config.h`，把
   `#define BIDIRECTIONAL_PWM2 0` 改为 `1`（**只改 nami 烧录的这一份**，共享库其他机型不受影响）。
2. 编译烧录后，rosserial 连接确认启动日志无异常，`rostopic echo /nami/motor_pwms` 有输出。
3. 确认逻辑电机布线：aquatic rotor 1..4 必须接 pwm_htim2 的 CH1..4（= 电机索引 4~7）；
   空中旋翼在索引 0~3。

## 3. 台架（干测，机器人固定、旋翼可安全转动）

启动：`roslaunch nami bringup.launch rm:=true underwater:=true`
（会自动：加载 `NamiControl.yaml` 的 underwater 段、启动 ms5837 节点、不启动 mocap）

- [ ] **深度计**：`rostopic echo /nami/depth_sensor_node/depth` 有 100Hz 数据；
      手持传感器下压水杯验证 z 变负；`rosservice call /nami/depth_sensor_node/calibrate` 归零。
- [ ] **PWM 中点**：上电未 arm 时 4 个水下电调收到 1500μs（示波器或电调不响铃提示）。
- [ ] **pwm test 单电机**：`rostopic pub /nami/pwm_test spinal/PwmTest ...` 逐个电机给
      0.55 / 0.45，确认正/反转方向与 URDF 推力轴一致（0.5=停转；双向电机允许 [0,1] 全域，
      空中电机仍限 [0.5,1]——固件守卫会拒绝越界值）。
- [ ] **掩码验证**：arm + takeoff（水下模式）后空中电机 0~3 完全不转（1e-6 N 掩码被
      pwm 判定为禁用）；`land` / `halt` 后水下电机回中点。
- [ ] **force landing**：飞行中发 `/nami/teleop_command/force_landing`，观察双向电机
      推力平滑衰减到中点（不是跳变）。
- [ ] **rosserial 带宽**：40Hz `four_axes/command` + `torque_allocation_matrix_inv`
      下无 "wireless connection" 断链告警。

## 4. 系留水测（水池，安全绳）

- [ ] **入水配平**：断电入水应缓慢上浮（微正浮力）。过快上浮/下沉 → 调压载。
- [ ] **深度计标定**：水面静置时 `calibrate` 置零。
- [ ] **深度保持**：arm → takeoff（= 深度保持接通）→ `r/f` 键下潜 0.5 m。
      观察：稳态误差（若 >5 cm 调 `depth_hover_thrust`，实机浮力与仿真 −0.4 N 不同）、
      无振荡（有则先降 `depth_p_gain`）。参考仿真基线：阶跃 4 s 收敛、稳态偏 4 cm。
- [ ] **姿态静稳**：深度保持时 roll/pitch 静止（浮心回复 + 姿态环）。目视无持续倾斜
      → 若有恒定倾角，记录数值（CoB/CoG 与仿真模型的差异，后续修正 xacro）。
- [ ] **yaw**：`q/e` 键左右转向响应正确。

## 5. 自由游动（对照仿真基线）

先 legacy 分配（`NamiControl.yaml` 默认 `surge_allocation: false`）：

- [ ] 深度保持时观察前漂：仿真为 0.15 m/s 机体前向漂移（分配几何泄漏），实机大概率同现象、
      量值随浮力配平变化。**这是预期行为，不是姿态失控**。
- [ ] 前后左右指令响应与仿真对照（left/right 位移≈0 且可能反向，同为预期）。

然后切 surge 分配（改 yaml `surge_allocation: true` 重启，或先在池边
`rosparam set /nami/underwater/surge_allocation true` 后重启 aerial_robot_base_node）：

- [ ] 深度保持前漂应大幅减小（仿真：0.15 → 0.01 m/s）。
- [ ] 前进/后退为直接推力、机体基本不俯仰；`surge_acc_limit`（默认 1.0 m/s²）可渐进放大。
- [ ] **预期现象**：机体带小幅恒定 pitch（仿真 ≈8°，实机取决于真实静态力矩）；
      backward 时 pitch 增大（约束 τy≈0.27Fx 的从动效应）。若实机 pitch 过大，
      优先用压载微调 CoB/CoG 对齐，而不是改控制器。
- [ ] a/d 键在 surge 模式下无动作（按设计忽略，无侧向权威）。

## 6. 安全须知与已知风险

1. **急停**：`0` 键 = halt，电机立即停转 → 微正浮力自动上浮（仿真验证 2 s 内漂速归零）。
2. **指令断流**：遥操作停止 0.5 s 后 xy 加速度自动清零（watchdog），深度与 yaw 目标保持。
3. **spinal 命令超时**：500 ms 无 four_axes/command 固件自动停控。
4. **mocap 必须保持关闭**（launch 已处理）：短暂捕获后断流会触发 unhealth level 3
   → 自动 force landing。
5. **未经实测的环节**（首次下水重点盯）：X3 推力表实机偏差（电压相关，`v_factor` 补偿）、
   ESC 死区（1.4~1.6 ms）实际宽度、`m_f_rate` 反扭矩、yaw_joint 舵机水下持位。
6. 姿态增益调参时牢记 **<32.7 上限**（int16 溢出 = 正反馈发散，已有钳位保护但别依赖）。

## 7. 启动命令备忘

```bash
# 实机水下模式（自动：underwater 参数、ms5837 深度计、无 mocap）
roslaunch nami bringup.launch rm:=true underwater:=true

# 键盘遥操作
rosrun nami underwater_teleop.py

# 数据记录（与仿真同一套绘图工具链）
rosrun nami underwater_demo.py _out_dir:=/tmp/pool_test   # 或手动 rosbag record
python3 $(rospack find nami)/scripts/plot_underwater_demo.py /tmp/pool_test
```

注意：`underwater_flight_test.py` 的 pitch/roll 响应两项在 surge 模式下按设计 FAIL
（姿态不再随平移指令倾斜），surge 模式验证请用 demo 脚本或手动对照第 5 节。
