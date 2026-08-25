# Nami 飞行、质量分布与 Aerial ZMP 研究笔记

---

- 文档状态：工作笔记，不是最终论文结论
- 整理日期：2026-08-16（Asia/Tokyo）
- 语言：中文；保留英文术语和数学符号，便于 Codex 检索与后续论文写作
- 主要数据：
  - /home/chen_chen/rosbag/20260813_nami_takeOff/2026-08-13-21-30-50_HalfHover.bag
  - /home/chen_chen/rosbag/20260813_nami_takeOff/ 中其余 rosbag
- 主要论文：
  - Design, Control, and Motion-Planning for a Root-Perching Rotor-Distributed Manipulator.pdf
  - Takuzumi Nishio, Moju Zhao, Kei Okada, Masayuki Inaba
  - 本地 PDF 含 arXiv:2405.12125v1；正式投稿时须重新核对最终卷期、页码和 DOI
- 原始 bag/机械数值分析快照：dev/aquatic_nami，673962ddd，[nami] half hover para
- 整理本文时仓库 HEAD：b72208f37，[nami][urdf] update the latest urdf model of nami

> **版本警告：** 2026-08-16 的 b72208f37 大幅更新了 head/tail 质量、惯量、旋翼位置和 mesh。本文第 6、7、10、12 节中的具体 CoG、惯量、ASP margin 和 head-only 优化数值，是对 2026-08-13 bag 对应的 673962ddd 模型快照所做的历史分析，不应直接作为 b72208f37 的制造参数。理论、bag 事实和控制链结论仍可保留；使用新 URDF 写论文或改机械前必须重算并重新称重。

---

## 0. 给 Codex 和论文作者的使用说明

本文把对话中的实验分析、代码审查、机械设计讨论和论文理论放在同一处。引用本文时必须保留以下证据等级：

- **[PAPER]**：给定论文直接给出的定义、假设、公式或实验结果。
- **[BAG]**：rosbag 直接记录的目标、状态、PWM、RPM、电压或时序。
- **[CODE]**：分析时工作树或指定提交中的软件行为。
- **[URDF]**：当前 xacro/URDF 的质量、惯量、坐标或推力几何。
- **[CALC]**：由 PAPER、BAG、CODE 或 URDF 离线计算得到。
- **[INFERENCE]**：由多项证据支持、但尚未被独立台架实验唯一确认的解释。
- **[RECOMMENDATION]**：设计目标、控制方案或验收门槛，不能写成已经实现的事实。

除非段落明确写“当前 b72208f37”，本文中的 **[URDF]** 数值默认指原始分析快照 673962ddd。

写论文时建议把“测得”“模型给出”“由模型计算”“推断”明确分开。例如：

> [BAG] M1 的 PWM 长时间达到 1800；[CALC] 在相应电压和 duty 上限下，标定曲线给出的推力上限约为 12.6 N；[INFERENCE] 单边执行器饱和破坏了期望 wrench 分配并放大了水平振荡。

不要把以下内容直接当成实测事实：

- URDF 的 3.413 kg 不等于实机称重；
- bag 推断的 4.0–4.3 kg “有效质量”不一定是真实增重，也可能包含推力曲线误差、桨气动干扰或硬件状态；
- 由 PWM 多项式反解的推力不是同步 load-cell 测力；
- Aerial support polygon 内部不等于存在足够的执行器动态余量。

## 1. 研究问题与对话决策记录

### 1.1 初始问题

典型飞行数据在起飞后出现大范围水平位置振荡。用户随后尝试过大量参数，但效果不稳定，同时后期可能出现机械损坏。

### 1.2 对话中形成的主要判断

| 讨论主题 | 当前判断 |
|---|---|
| 位置振荡是否主要由 mocap/估计跳变造成 | 否。原始 mocap 与估计位置同频、同相，去偏差残差很小。 |
| 是否只需继续降低或提高位置 PID | 否。主要约束包括姿态链相位滞后、单电机饱和、机械 CoG 偏置和起飞切换瞬态。 |
| 质量应优先“平衡”还是降低 pitch 惯量 | 先让 CoG 对准有效推力中心，消除稳态配平力矩；随后在保持 CoG 的条件下集中重件、降低 Iyy。 |
| “前方”坐标含义 | URDF root/base_link_tail 的 +X 指向 head、M1/M2；fc_joint 有 yaw=π，因此 FC 坐标 +X 与 URDF +X 相反。论文和实验中应优先写“head side / M1–M2 side”。 |
| 水下配平后更新 URDF 是否自动有利于飞行 | 不一定。水下水平配平是 CoG–CoB 对齐，空中是 CoG–推力中心对齐。两者可以协同设计，但不能把水中 added mass 直接写入空中干态惯量。 |
| tail 难改、只能调整 head 时怎么办 | 把 head 总成 aggregate CoG 向 yaw joint 移动，但不能无限移到关节中心；推荐 head 局部 x 约 0.170±0.005 m。 |
| transformation 范围 | 第一阶段起降固定 q=0，空中只做 0→+15°→0 或 ±15°慢速变形；±30°留给推力余量改善后的阶段。 |
| 论文最值得迁移的内容 | 构型相关 Q(q)、I(q)，以及在规划中显式检查推力、slew、构型和碰撞约束。 |
| 论文不能直接解决的问题 | 论文 flight LQI 没有最终电机硬上界、PWM/电压模型和 achieved-wrench anti-windup，不能直接修复本次 M1 饱和。 |

## 2. 数据与代码版本边界

### 2.1 典型 bag

主 bag：

    /home/chen_chen/rosbag/20260813_nami_takeOff/2026-08-13-21-30-50_HalfHover.bag

关键飞行状态：

| 事件 | bag 相对时间 |
|---|---:|
| TAKEOFF / 闭环激活 | 11.010 s |
| 实际离地和姿态积分开始附近 | 约 14.184 s |
| M1 首次达到 PWM 1800 | 15.359 s |
| z 首次达到 0.75 m | 19.493 s |
| 控制停止 | 21.814 s |

### 2.2 工作树和 bag 配置不同

**[BAG]** 目标 bag 中 max_pwm=0.90，对普通电机 motor_pwms=1800 表示 duty=0.90。

**[CODE]** 分析时工作树中 robots/nami/config/MotorInfoDShot.yaml 已被用户从 0.90 改为 0.95。这一修改发生在目标 bag 之后，不能用 0.95 的上限解释 0.90 bag。

### 2.3 代码变更时间

提交 673962ddd 同时包含：

- aerial thrust frame 从 rotor_armN 改为实际 thrustN；
- 加入 ±10° 倾斜产生的水平力和更强 yaw moment；
- 加入 rp_offset 补偿；
- 修改多组位置和姿态增益。

bag 中 allocation matrix 指纹表明，20:55:38 之后已经使用与该提交相符的新倾斜模型；提交时间晚于目标 bag，说明当时很可能使用了尚未提交的同内容工作树。

旧矩阵和新矩阵不能用飞行结果直接做严格 A/B：旧矩阵组的 8 次 TAKEOFF 均没有实质离地超过 0.1 m。

## 3. 典型 bag 的实验事实

### 3.1 位置目标与振荡

**[BAG]** 目标在飞行段保持常数：

    target = (x,y,z) = (0.371361, 0.869872, 0.8) m

主要结果：

- 水平误差在 14.635 s 首次超过 0.1 m；
- 15.114 s 超过 0.5 m；
- 19.135 s 达到 2.1166 m；
- 控制结束时仍约 2.0856 m；
- x/y 峰峰值约 1.58/2.09 m；
- 主振荡约 0.305 Hz，周期约 3.28 s；
- XY PID 输出后来才触及 ±4：x 约 9.9%，y 约 14.1%，首次约 18.1–18.3 s。

**[INFERENCE]** 位置 PID 限幅是振荡后期的结果和放大项，不是最早触发原因。

### 3.2 估计链不是主要原因

**[BAG/CALC]**

- 原始 mocap 与估计位置在主振荡频率处相干接近 1；
- 去掉固定偏差后，估计与 mocap 的残差 RMS 约为 x/y/z = 11/22/3 mm；
- 活跃飞行段控制、mocap、odom、IMU 序列无明显丢序或非单调。

因此，本次大范围运动是真实机体运动，而不是单纯的估计跳变。

### 3.3 姿态链相位滞后

在 0.305 Hz 主振荡处：

| 通道 | 实际/目标幅值比 | 相位滞后 | 等效时间 |
|---|---:|---:|---:|
| roll | 约 1.02 | 约 71.3° | 约 0.649 s |
| pitch | 约 0.744 | 约 36.9° | 约 0.336 s |

**[INFERENCE]** 位置环通过目标倾角控制水平加速度；上述滞后显著消耗外环相位裕量。直接提高 XY P/D 可能让外环更早进入低阻尼或饱和。

### 3.4 M1 的硬饱和

**[BAG]**

- M1 首次达到 1800：15.359 s；
- 整个闭环飞行段 M1 约 52.3% 样本达到 1800；
- 从实际离地/积分开启之后计算，约 74.7% 样本达到 1800；
- M2 只有约 1%，M3/M4 没有持续达到 1800；
- 首次饱和时电池约 23.551 V；
- 当时 base thrust 约为 [11.876, 11.890, 8.992, 8.981] N；
- 当时 roll command 约 −18.96°。

RPM–推力标定进一步支持 M1 是真实执行器剪切，而不只是诊断显示异常：

- M1 重建目标约 9.51–15.00 N；
- RPM 反算实际推力最高约 12.20 N；
- M1 首次饱和后，目标与 RPM 推力相关显著下降；
- M2–M4 的目标/RPM 推力仍高度闭合。

### 3.5 饱和不是唯一初因

水平误差在 M1 首次硬饱和之前已经增长：

- 0.1 m：14.635 s；
- 0.5 m：15.114 s；
- M1 首次硬饱和：15.359 s。

因此推荐的因果表述是：

1. 地面到离地的姿态/推力切换、rp_offset kick、模型偏差和内环相位滞后触发第一拍运动；
2. M1 随后进入长期硬饱和，破坏总 wrench 分配并放大振荡；
3. 外层位置 PID 最后撞限，进一步维持大振幅运动。

## 4. 推力上限实现中的关键缺口

### 4.1 当前逻辑

spinal 中的多项式换算大致为：

$$
d_i = \frac{P_V(0.1\,v_{\mathrm{factor}}F_i/r_{\mathrm{div}})}{100}.
$$

但 saturation avoidance 使用：

$$
F_{\mathrm{limit,logic}}
=\frac{F_{\mathrm{max,calibration}}}{v_{\mathrm{factor}}},
$$

没有使用 max_duty。最终 convert 后才执行：

$$
d_i\leftarrow\operatorname{clamp}(d_i,d_{\min},d_{\max}).
$$

相关代码：

- aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/flight_control/attitude/attitude_control.cpp:1135-1257
- 同文件最终 PWM clamp：1337-1348

### 4.2 数值后果

首次 M1 饱和附近：

| 项目 | 数值 |
|---|---:|
| 电池 | 23.551 V |
| 选择的参考曲线 | 23.85 V |
| 逻辑认为的力上限 | 约 16.519 N |
| duty=0.90 对应真实力上限 | 约 12.588 N |
| 保护盲区 | 约 3.932 N |

飞行末端约 23.075 V：

| 项目 | 数值 |
|---|---:|
| 逻辑认为的力上限 | 约 16.230 N |
| duty=0.90 对应真实力上限 | 约 12.018 N |
| 保护盲区 | 约 4.212 N |

**[CONCLUSION]** saturation avoidance 可能认为仍有约 4 N 余量，而最终 PWM 已经硬钳位。这解释了为什么 M1 可以长期停在 1800，而上游控制器仍继续增加目标。

### 4.3 推荐修正

定义唯一的 duty-aware 力上限函数：

$$
F_i^{\max}(V,d_{\max})
=\operatorname{convert}^{-1}(d_{\max};V),
$$

并让以下模块共享同一上限：

- hover trim；
- control allocation；
- yaw/Z desaturation；
- PID anti-windup；
- 最终 PWM 安全 clamp；
- 日志和规划器。

对于多项式模式，可在标定有效区间内对 convert(F)=max_duty 做有界二分。最终 PWM clamp 仍保留作为安全兜底。

## 5. 其他代码级限制

### 5.1 配置中的 tilt limit 未生效

robots/nami/config/NavigationConfig.yaml 中存在：

    max_target_tilt_angle: 0.2

但仓库没有读取该参数的代码。Nami 直接把 XY PID 的结果转换成 target roll/pitch，目标 bag 中 roll command 达到约 −0.423 rad（−24.3°）。

spinal 的 MAX_TILT_ANGLE=1.0 rad 是 force-landing 保护，不是 0.2 rad 的命令限幅。

### 5.2 外层 PID 没有 actuator-aware anti-windup

通用 PID 只分别 clamp P/I/D 和最终 sum：

- aerial_robot_control/include/aerial_robot_control/control/utils/pid.h:63-74

它不知道最终每电机是否被 PWM clamp，也没有用 achieved wrench 回算积分器。

### 5.3 FC 姿态积分器未实际 clamp

attitude_control.h 声明 error_angle_i_limit_，但 update 中只累加 error_angle_i_，没有应用该 limit：

- attitude_control.cpp:388-423
- attitude_control.h:198-207

### 5.4 Nami 缺少倾斜时的总推力模长补偿

通用 under-actuated controller 在 exact 模式中使用 target_acc_w.length() 分配总推力，并在 z_limit 饱和时回滚 Z 积分：

- aerial_robot_control/src/control/under_actuated_controller.cpp:108-123

Nami 自定义控制链只用 z 分量做 translation allocation，却按 exact atan2 产生较大倾角。单 roll=0.423 rad 已使世界 z 分量损失约 9%，Z 积分会继续提高基准推力并挤压姿态余量。

### 5.5 起飞目标是位置阶跃而非轨迹

motorArming() 在起飞前直接设置：

- target z = takeoff_height = 0.8 m；
- target velocity z = 0；
- target acceleration z = 0。

相关代码：

- aerial_robot_control/include/aerial_robot_control/flight_navigation.h:436-445

现有 min-jerk trajectory 能输出 position、velocity、acceleration，但 updatePoseFromTrajectory() 只在 HOVER_STATE 使用：

- aerial_robot_control/src/flight_navigation.cpp:1014-1079

因此起飞阶段没有使用已有的速度/加速度前馈能力。

## 6. 质量、CoG、惯量与推力几何

### 6.1 Bag 分析快照的 URDF（673962ddd）

**[URDF]** 控制器所见：

| 项目 | 数值 |
|---|---:|
| 总质量 | 3.41347 kg |
| CoG(root,q=0) | (0.06403, 0.051884, 0.031324) m |
| 惯量对角 | (0.05443, 0.18572, 0.21510) kg·m² |
| pitch 回转半径 | 约 0.233 m |

四个空中推力点的几何中心：

$$
p_T=(0.03147,\ 0.05200,\ 0.03359)\ \mathrm{m}.
$$

因此 CoG 相对推力中心向 head/+X 偏约 32.6 mm，y 基本对齐。

### 6.2 “前方”定义

- root/base_link_tail 的 +X：朝 head、M1/M2；
- root −X：朝 tail、M3/M4；
- fc_joint 的 yaw=π，因此不要把 FC +X 直接称为机械前方。

相关几何：

- robots/nami/urdf/quad/nami.urdf.xacro:19-62
- yaw_joint：153-160
- aerial rotor positions：165-189
- ±10° thrust axes：193-223

### 6.3 静态偏载

该历史快照在 q=0 的前/后总推力比约为 1.322。按约 44 N 总标量推力：

- 前对约 12.5 N/台；
- 后对约 9.47 N/台。

这与 bag 中约 12.4/9.4 N 的基准分配吻合。静态偏载持续占用 pitch 和单电机余量；降低 Iyy 只能减少动态转动所需力矩，不能消除这一稳态负担。

设计优先级：

1. 总质量与总推力余量；
2. CoG 与有效推力中心对齐；
3. 保持上述对齐时集中重件、降低 Iyy；
4. 保留轻质电机的前后力臂。

性能指标应优先使用：

$$
\alpha_{y,\mathrm{reserve}}
=\frac{\tau_{y,\mathrm{reserve}}}{I_{yy}},
$$

而不是单独最小化 Iyy。

### 6.4 当前 HEAD 的重新计算结果（b72208f37）

2026-08-16 的 URDF 更新后，不能继续把上一小节的数值称为“当前机械模型”。对 b72208f37 展开后的控制器可见 URDF，在 `q=0` 下重新计算得到：

| 项目 | b72208f37 数值 | 相对 673962ddd |
|---|---:|---:|
| 总质量 | 3.52151 kg | +3.2% |
| CoG(root) | (0.05493, 0.05187, 0.02742) m | x 向 tail 侧改善约 9.1 mm |
| Iyy about total CoG | 0.15613 kg·m² | 约降低 15.9% |
| pitch 回转半径 | 0.2106 m | 约降低 9.6% |
| aerial thrust-point center | (0.02995, 0.05200, 0.03393) m | x 改变约 −1.5 mm |
| CoG−thrust-center | (+24.98, −0.13, −6.51) mm | head 侧偏置仍存在 |

使用当前实际 `thrust1...4` 的 ±10°轴、反扭矩项及零 roll/pitch/yaw wrench 条件，q=0 的标量静推力 share 约为：

$$
s\approx(0.27679,\ 0.27715,\ 0.22318,\ 0.22288).
$$

因此前/后总推力比约为 1.242，固定倾斜轴产生的等效前向合力倾角约为 1.09°。与旧模型相比，Iyy 和前后偏载均有改善，但前对仍承担约 55.4% 的悬停标量推力，尚不能称为四电机均载。

质量口径还需注意：上述控制模型包含 `thrust1...8` 各 0.01 kg 的虚拟 link，共约 80 g。若只累计主要 CAD 实体，质量约为 3.44132 kg、CoG 约为 (0.05551, 0.05186, 0.02754) m、Iyy 约为 0.15091 kg·m²。实机控制器若加载同一 URDF，这 80 g 仍会进入 KDL 的质量、CoG 和惯量计算；论文应分别报告“控制模型口径”和“实体/实测口径”。

这些数值是 **[URDF/CALC]**，不是 b72208f37 的新飞行试验结果，也尚未替代实机称重、CoG 测量和推力台标定。第 7、10 和 12 节保留的是与原始 bag 对应的历史设计计算；若要制造或修改当前 head，必须以 b72208f37 和实测质量重新进行 q∈[−15°,15°] minimax 优化。

## 7. Tail 固定、Head-only、±15° 的机械设计

> **历史计算，需重算：** 本节全部精确目标均基于 673962ddd。当前 b72208f37 的 head/tail 质量和 head aggregate CoG 已变化；本节只保留优化方法与当时的决策轨迹，不能直接用作当前加工尺寸。

### 7.1 设计变量

原始分析模型（673962ddd）：

| 项目 | 数值 |
|---|---:|
| yaw joint root x | 0.03797 m |
| head 总成质量 | 1.640263 kg |
| tail 总成质量 | 1.773204 kg |
| 当前 head aggregate CoG，head local x | 0.236741 m |

在 tail 完全不动、只调整 head aggregate CoG x 的约束下，对 q∈[−15°,15°] 最小化最重电机静态 share：

$$
x_H^*
=\arg\min_{x_H}
\max_{q\in[-15^\circ,15^\circ],\,i}s_i(q,x_H).
$$

结果：

$$
x_H^*=0.170487\ \mathrm{m}.
$$

工程目标：

    head aggregate CoG local x = 0.170 ± 0.005 m

即向 yaw joint/tail 方向移动约 66 mm。

注意：这是整个 head 子树的 aggregate CoG，不是直接写入 base_link_head inertial origin 的数值。若只改变当前 0.940089 kg 的 base_link_head 本体、所有 rotor units 不动，则该本体自身 CoG x 约需从 0.221545 m 调到 0.10595 m；实际值必须根据实物质量重算。

### 7.2 一阶矩要求

$$
\Delta(M_Hx_H)\approx-0.1087\ \mathrm{kg\,m}.
$$

| 可移动质量 | 向 joint 移动距离 |
|---:|---:|
| 0.5 kg | 约 217 mm |
| 0.8 kg | 约 136 mm |
| 1.0 kg | 约 109 mm |

不应把 head CoG 一直移到 joint 的 x=0；这会过度修正并使后电机成为新的饱和端。

### 7.3 预测 share

| q | M1 | M2 | M3 | M4 | max |
|---:|---:|---:|---:|---:|---:|
| −15° | 0.260508 | 0.241263 | 0.237784 | 0.260446 | 0.260508 |
| 0° | 0.250629 | 0.250917 | 0.249370 | 0.249083 | 0.250917 |
| +15° | 0.240944 | 0.260764 | 0.260764 | 0.237527 | 0.260764 |

以 44 N 总标量推力：

| 范围 | 最重电机 | 23.55 V, duty=.9 余量 | 低电压约 23.08 V 余量 |
|---|---:|---:|---:|
| q=0 | 约 11.04 N | 约 1.55 N | 约 0.98 N |
| ±15° | 约 11.47 N | 约 1.11 N | 约 0.54 N |
| ±30° | 约 11.95 N | 约 0.64 N | 约 0.07 N |

因此 ±15°适合作为第一阶段，±30°在当前总推力需求下几乎没有低电压动态余量。

按当前模型符号，+15°主要增加 M2/M3 并减轻 M1；−15°增加 M1/M4。考虑目标 bag 中 M1 最早饱和，第一轮演示可优先做 0→+15°，但必须先在台架确认实际关节正方向和电机编号。

### 7.4 模块化设计的限制

当前 head 子树 CoG 与前桨局部中心本来相差仅约 8.2 mm；tail 子树 CoG 与后桨中心相差约 72.7 mm。按 rotor-distributed 设计思想，真正更优的是移动 tail/root 固定系重件，使各模块就地承重。

Head-only 方案是 tail 不可调整条件下的折中。它在 q=0 附近显著改善均载，但随 head 转动的 CoG 修正不能像 tail 固定系调整那样保持构型不变。这也是建议把第一阶段限制到 ±15°的原因之一。

## 8. 给定论文的模型与控制方法

### 8.1 Aerial ZMP 与 ASP

论文 Sec. II-A.1、PDF p.3–4、Eq.1–2 定义 aerial support polygon；Appendix A、PDF p.16、Eq.71–76 给出推导。详见第 9 节。

### 8.2 构型相关分配

论文 PDF p.6、Eq.7–9：

$$
W_{\mathrm{rotor}}(q)=Q(q)\lambda,
$$

其中第 i 列包含推力轴和力矩：

$$
Q_i(q)=
\begin{bmatrix}
u_i(q)\\
([p_i(q)\times]-c\sigma_i)u_i(q)
\end{bmatrix}.
$$

Nami 当前从真实 thrustN frame 读取推力方向，与这一思想一致：

- robots/nami/src/model/nami_robot_model.cpp:24-42

旧版本从 rotor_armN 读取，遗漏 ±10° 倾斜，导致 yaw effectiveness 和水平力建模错误。

### 8.3 Flight LQI

论文 PDF p.7–8、Eq.21–29：

- 选择 z/roll/pitch/yaw 四个受控量；
- 使用 Q(q) 和 I(q) 构造构型相关线性模型；
- 加入误差积分并用 LQI 反馈；
- Eq.29 加入 gyro moment 补偿；
- 文中明确说重力可以由积分器补偿。

对 Nami 不建议复制“依靠积分补重力”：

- 起飞前会积累很大 Z 积分；
- 质量或推力模型不准时积分承担长期 trim；
- 电机饱和时积分继续增长。

应显式计算受约束 hover trim f0(q,V)，积分只补偿小的模型残差。

### 8.4 水平控制与 vectoring

论文 PDF p.8、Eq.30–36：

- 世界系 XY PID 产生水平加速度；
- 小角度映射到 roll/pitch；
- 因长构型 pitch 惯量大、pitch latency 高，主动 vectoring apparatus 分担水平加速度。

这项思想与 Nami 很相关：它改变了水平运动从“位置→姿态→水平力”的慢通道。Nami 的 aerial rotors 只有固定 ±10°，gimbal_dof=0，rp_offset 只是静态补偿，并不是独立的主动水平推力通道。

### 8.5 约束 QP 的实际位置

论文 flight LQI 本身没有逐电机硬约束。推力上下界和每步变化约束出现在 perching QP（PDF p.8–9、Eq.37–49）及 motion planning（PDF p.9–11、Eq.50–70）。

论文 Eq.45–46 对附加 perching thrust 使用非负下界，不能直接用于 Nami 姿态差动。Nami 应优化最终电机力 f，或允许相对 hover trim 的正负增量：

$$
f_i^{\min}-f_{0,i}
\le\Delta f_i
\le f_i^{\max}-f_{0,i}.
$$

## 9. Aerial ZMP 与 Aerial Support Polygon 理论

### 9.1 定义和符号

- N：有效推力点数；
- p_i：第 i 个推力作用点；
- F_i=[0,0,F_{iz}]^T：论文假设下的竖直推力；
- F_{iz}≥0；
- p_A：aerial zero moment point；
- p_{i,xy}：推力点在水平面的投影。

### 9.2 合力与合力矩

**[PAPER, Appendix A, Eq.71–72]**

$$
F_{\mathrm{rotor},A}
=\sum_{i=1}^N F_i,
$$

$$
M_{\mathrm{rotor},A}
=\sum_{i=1}^N(p_i-p_A)\times F_i.
$$

准静态 roll/pitch 平衡要求：

$$
M_{A,x}=M_{A,y}=0.
$$

在纯竖直推力假设下：

$$
\sum_i(p_{iy}-p_{Ay})F_{iz}=0,
\qquad
\sum_i(p_{ix}-p_{Ax})F_{iz}=0.
$$

因此：

$$
p_{\mathrm{AZMP},xy}
=\frac{\sum_i p_{i,xy}F_{iz}}
       {\sum_iF_{iz}}
=\sum_i\gamma_i p_{i,xy},
$$

其中：

$$
\gamma_i
=\frac{F_{iz}}{\sum_jF_{jz}},
\qquad
\gamma_i\ge0,
\qquad
\sum_i\gamma_i=1.
$$

### 9.3 ASP 是凸包

**[PAPER, Eq.1]**

$$
\mathcal S_{\mathrm{ASP}}
=\left\{
\sum_i\gamma_i p_{i,xy}
\mid
\gamma_i\ge0,\ \sum_i\gamma_i=1
\right\}
=\operatorname{conv}\{p_{1,xy},\ldots,p_{N,xy}\}.
$$

**[PAPER, Eq.2]**

$$
p_{\mathrm{AZMP},xy}\in\mathcal S_{\mathrm{ASP}}.
$$

在无外部 roll/pitch 力矩的准静态悬停中，AZMP 与 CoG 水平投影重合，于是得到：

$$
p_{\mathrm{CoG},xy}\in\mathcal S_{\mathrm{ASP}}.
$$

### 9.4 必须保留的假设

论文原始 ASP 判据假设：

1. 所有推力单元垂直于地面；
2. 所有推力均为非负；
3. 单个推力单元有“足够”的推力能力；
4. z 和 yaw 已经可稳定；
5. 研究 steady state，忽略显著惯性项；
6. ASP 只检查 roll/pitch 静态力矩平衡。

因此，CoG 在 ASP 内并不自动意味着：

- 所有电机都未饱和；
- 有足够 yaw torque；
- 有足够动态姿态余量；
- 快速构型变化稳定；
- 单电机失效可容错。

### 9.5 四点最小构型的正确理解

论文 Fig.2 的结论：

- 两点只能形成线段，没有二维内部；
- 三点在某些直线构型会退化；
- 四个有效支撑点可在论文研究的直线构型保持非零 ASP 面积。

这只是几何 roll/pitch 条件。论文的 λ_i 对应 dual-rotor vectoring units，不能直接解释为“四个固定单桨具有失效冗余”。

## 10. Capacity-bounded ASP

### 10.1 推力上限

真实系统满足：

$$
0\le F_{iz}\le\bar F_{iz}(V,d_{\max}).
$$

给定目标总竖直力 F_z^d：

$$
\mathcal S_{\mathrm{ASP}}^{\mathrm{cap}}(F_z^d,V)
=
\left\{
\sum_i\gamma_i p_{i,xy}
\middle|
\sum_i\gamma_i=1,\quad
0\le\gamma_i\le
\frac{\bar F_{iz}(V)}{F_z^d}
\right\}.
$$

它仍是凸集，但通常严格小于几何 ASP。当：

$$
\sum_i\bar F_{iz}<F_z^d
$$

时，该集合为空。

若要保留比例为 r 的动态余量，可以使用：

$$
F_{iz}\le(1-r)\bar F_{iz}.
$$

### 10.2 Nami 的直观数值

以下数值对应原始 bag/URDF 快照 673962ddd；b72208f37 需重新求 bounded margin。

q=0 时四旋翼投影约为 0.470×0.405 m 的矩形：

- 几何 ASP 面积约 0.19035 m²；
- 该快照 CoG 到最近几何边界约 202 mm。

所以论文原始判据会认为 Nami 有很大的几何 margin。

但在 44 N 总推力和每台约 12.5875 N 上限下，该快照 CoG 朝 head 方向距离 capacity-bounded 边界只有约 1.3 mm。

这一对比是本研究最重要的理论观察之一：

> 几何 ASP 评价“是否存在非负分配”；capacity-bounded ASP 评价“在给定总推力和实际单机上限下是否仍存在分配”。前者可以有 202 mm margin，而后者只剩约 1.3 mm。

## 11. 倾斜推力下的完整 Wrench Polytope

Nami 的 aerial rotors 固定倾斜 ±10°，论文的竖直推力假设不成立。定义：

- u_i(q)：单位推力轴；
- r_i(q)=p_i(q)−p_CoG(q)；
- σ_i：旋向；
- k_m：反扭矩/推力比；
- f_i：标量推力。

单电机 wrench：

$$
w_i(q)=
\begin{bmatrix}
u_i(q)\\
r_i(q)\times u_i(q)+\sigma_i k_m u_i(q)
\end{bmatrix}f_i,
$$

其中反扭矩符号需与实际旋向约定一致。

完整映射：

$$
w=G(q)f.
$$

带边界的可行 wrench 集：

$$
\mathcal W(q,V)
=
\left\{
G(q)f+w_{\mathrm{env}}
\mid
\underline f_i(V)\le f_i\le\bar f_i(V)
\right\}.
$$

如再考虑变化率：

$$
-\dot f_i^-\Delta t
\le f_i-f_{i,\mathrm{prev}}
\le\dot f_i^+\Delta t.
$$

静态 hover 的严格条件应是目标 wrench 位于该有界集合中，而不只是 CoG 位于投影凸包中。

推荐的最差电机利用率指标：

$$
\rho(q,w_d)
=
\min_{G(q)f=w_d}
\max_i\frac{f_i}{\bar f_i(V)}.
$$

设计目标可写为：

$$
\min_{\theta_{\mathrm{mech}}}
\max_{q\in\mathcal Q}
\rho(q,w_{\mathrm{hover}};\theta_{\mathrm{mech}}),
$$

其中机械变量可包括质量位置、旋翼位置和推力轴。

## 12. ±10°、CoG 偏置和 rp_offset

本节的 1.40° 数值解释对应 673962ddd 和目标 bag；当前 b72208f37 的纯 URDF 静态预测约为 1.09°，尚无新飞行 bag 验证。

q=0 时前桨轴朝 +X，后桨轴朝 −X。当前前后推力不等产生：

$$
F_x
=\sin10^\circ
(\lambda_{\mathrm{front}}-\lambda_{\mathrm{rear}}).
$$

44 N 下原始快照静态分配约给出：

$$
F_x\approx1.06\ \mathrm N,
\qquad
\arctan(F_x/F_z)\approx1.40^\circ.
$$

这与 bag 和代码中的 rp_offset_pitch≈0.024 rad（1.4°）一致。

**[INFERENCE]** 当前 rp_offset 是“CoG 前偏、前对高推力、倾斜轴水平分量不再抵消”的机械–分配耦合表现。

但当前代码在地面就持续施加约 0.024 rad pitch target，实际 pitch 被接触约束保持在 0；离地后姿态快速冲到约 0.119 rad，并触发第一拍水平速度。推荐：

- ground/armed 阶段冻结或保持 level；
- 检测 airborne 后再平滑 ramp trim；
- 对 q 变化引起的 rp_offset 使用 slew limit；
- trim 应基于受约束 f0 和实际推力轴，而不是未实现的理想分配。

即使 q=0 机械均载，q≠0 时前半推力轴随 head 转、后半不转，仍会产生构型相关水平力。head-only ±15°端点的等效倾角约 1.319°；±30°约 2.614°。

## 13. 如何真正提高起飞位置环带宽

### 13.1 先区分两个目标

“减少离地时的位置偏移”和“提高闭环位置抗扰带宽”不是同一件事：

- smooth takeoff reference、hover feedforward、rp_offset ramp 主要改善离地瞬态；
- 更快的姿态/角速度内环、更低延迟和更大的 actuator margin 才能提高真正的反馈带宽；
- trajectory feedforward 对轨迹跟踪很有效，但对未知阵风或固定目标抗扰不能替代反馈。

### 13.2 当前结构的带宽瓶颈

水平级联链为：

    position error
      → desired horizontal acceleration
      → desired roll/pitch
      → attitude dynamics
      → horizontal acceleration
      → position

当前位置控制以 40 Hz 运行，更新率本身不是 0.305 Hz 振荡的主要限制。更关键的是：

- roll/pitch 在 0.305 Hz 已有约 71°/37°滞后；
- M1 长时间饱和；
- tilt limit 未生效；
- 总推力没有按倾角补偿；
- 没有 actuator-aware anti-windup；
- 起飞使用位置阶跃且缺少 hover/gravity feedforward。

当前 XY 参数约为 Kp=2、Kd=2、Ki=0.05。若暂时忽略姿态内环，把水平动力学近似为双积分器，则：

$$
\omega_n=\sqrt{K_p}
=1.414\ \mathrm{rad/s}
\approx0.225\ \mathrm{Hz},
$$

$$
\zeta
=\frac{K_d}{2\sqrt{K_p}}
\approx0.707.
$$

因此 bag 中的 0.305 Hz 不是“已经获得的稳定位置带宽”，而是高于理想外环自然频率、又叠加姿态延迟和饱和后形成的非线性振荡。

**[BAG]** 起飞命令后约 3.19 s 才首次比地面高 1 cm。在此期间 z target 已经是 0.8 m，而 Z 输出主要靠积分从约 3.97 m/s²爬到约 10.46 m/s²后才释放。离地后约 0.6 s，实际 pitch 达到约 +0.119 rad，而目标已经反向；这说明第一拍水平运动来自非无扰切换和姿态链滞后，早于 M1 的长期硬饱和。

级联设计建议满足：

$$
f_{\mathrm{attitude}}
\gtrsim 3\text{–}5\,f_{\mathrm{position}}.
$$

在希望的位置交越频率处，姿态跟踪幅值应接近 1，相位滞后建议低于约 20–30°。在达到这一条件之前，提高 XY P 不能视为提高了有效带宽。

### 13.3 第一优先级：显式 hover 和起飞前馈

每个构型和电压下先求受约束 hover trim：

$$
f_0(q,V)
=\arg\min_f
\|D^{-1}(f-f_{\mathrm{center}})\|^2
$$

subject to：

$$
G(q)f=w_{\mathrm{hover}},
\qquad
f_i^{\min}\le f_i\le f_i^{\max}(V,d_{\max}).
$$

起飞时把 f0 以平滑 ramp 加入，而不是让 Z 积分在地面慢慢积到重力。

Nami 当前的 integrated_map 是 acceleration-level map；target_acc_w 和 PID limit 的单位是 m/s²，而不是 N。因此最小实现应使用命名明确的：

$$
a_{\mathrm{hover,ff}}
=F_{\mathrm{hover,measured}}/m_{\mathrm{model}}.
$$

当质量和推力模型准确时它接近 g；若直接使用 bag 中经验悬停力，则必须避免再次乘质量。当前配置中 Z limit_sum 注释为 N，但在这条链路中实际是 acceleration-domain limit，论文和实现都应统一单位。

期望总力：

$$
F_d
=m\left(
g e_3
+a_{\mathrm{ff}}
+K_p e_p
+K_v e_v
+K_i e_i
\right).
$$

目标姿态由完整 F_d 方向得到，总推力使用其模长：

$$
T_d=\|F_d\|.
$$

这同时提供：

- 显式重力补偿；
- 轨迹加速度前馈；
- 倾斜时的 vertical-thrust compensation；
- 比当前只使用 z 分量更一致的 force-vector mapping。

若轨迹生成器还能给出 jerk j_d，可进一步生成期望 body-rate 前馈。小角度近似下：

$$
\dot\theta_d\approx j_{d,x}/g,
\qquad
\dot\phi_d\approx-j_{d,y}/g.
$$

当前 PC 已计算 target angular velocity/acceleration，但 FourAxisCommand 没有把 roll/pitch 的这些前馈送到 FC；若要真正降低姿态相位滞后，需要扩展消息和低层控制，而不仅是在 PC 内计算。

MotorInfoDShot.yaml 中的经验推力时间常数约为 0.094 s。模型验证后可对平滑指令尝试：

$$
f_{\mathrm{cmd}}
=f_d+\tau_m\dot f_d,
$$

但它必须位于 bounded allocator 和 slew limit 内，不能用来突破真实 PWM 上限。

### 13.4 起飞使用 jerk-limited trajectory

起飞不应直接把 z target 从地面设为 0.8 m、同时令 v_d=a_d=0。推荐状态机：

1. ARM：积分冻结，姿态目标 level，记录当前 XY；
2. THRUST_RAMP：平滑增加到受约束 f0 附近；
3. AIRBORNE_DETECT：使用高度变化、垂向速度、加速度或接触释放组合检测；
4. CLIMB：生成 min-jerk/S-curve z(t)，同时输出 z_d、v_d、a_d；
5. BLEND_XY：在 0.3–1 s 内平滑启用 XY feedback 和构型 trim；
6. HOVER：完成 bumpless transfer。

现有 trajectory 接口已经能把 target velocity 和 target acceleration 送入 PID，但 TAKEOFF_STATE 没有使用它。

### 13.5 第二优先级：真正的角速度内环

当前 FC 基本是 angle P/I 加 gyro D damping。建议改为清晰的两级姿态控制：

1. quaternion/SO(3) attitude outer loop 产生期望 body rate；
2. 高频 body-rate PI/PID 或 angular-acceleration loop 产生目标 torque；
3. 用惯量和 gyro term 做 torque feedforward：

$$
\tau_d
=I(q)\dot\omega_d
+\omega\times I(q)\omega
+\tau_{\mathrm{feedback}}.
$$

这通常比单层 angle PD 更容易识别、调参和保证内外带宽分离。

### 13.6 INDI / disturbance-observer 方案

如果推力曲线、惯量、ground effect 和结构弹性难以精确建模，可考虑 incremental nonlinear dynamic inversion（INDI）：

- 使用 IMU 角加速度或滤波后的加速度增量；
- 根据上一步实际控制量增量修正 torque/force；
- 主要依赖局部 control effectiveness，而不是绝对模型；
- 对质量、惯量和推力模型误差通常比纯 model inversion 更稳健。

对 Nami 的推荐顺序：

1. 先实现 attitude/rate INDI；
2. 再考虑 world-frame horizontal acceleration loop；
3. 必须保留 duty-aware bounds、actuator dynamics 和 anti-windup；
4. accelerometer feedback 要处理重力投影、振动滤波和传感延迟。

低带宽 disturbance observer 也可估计固定水平扰动力或 trim bias。它应在 airborne 后启用，在硬饱和或接触阶段冻结，不能把地面约束学习成空中扰动。

### 13.7 Geometric control、LQI 与 MPC 的定位

| 方法 | 对 Nami 的主要价值 | 不能自动解决的问题 |
|---|---|---|
| SO(3)/SE(3) geometric control | 避免 Euler 小角度近似和大倾角耦合；统一 force-vector 到姿态映射 | 电机上限、模型误差、anti-windup |
| configuration-scheduled LQI | 用 Q(q)、I(q) 处理小范围变形和多变量耦合 | 论文版没有最终电机硬约束 |
| INDI | 提高对 control effectiveness 和惯量误差的鲁棒性，适合提高内环带宽 | 仍受传感噪声、时延和饱和限制 |
| linear/NMPC | 显式处理 tilt、推力、slew、构型和轨迹约束 | 计算和建模复杂；未必提高最快内环带宽 |
| reference governor | 在现有控制器前投影到可达 wrench/tilt 集，低风险防饱和 | 不改变底层姿态动力学 |
| active vectoring / independent horizontal actuator | 直接绕开“水平位置必须先倾斜机体”的慢通道，物理上最能提高水平带宽 | 新增质量、舵机柔性、故障模式和机械复杂度 |

论文中“用 vectoring 绕开高 pitch inertia”的思想，比单纯换 LQI 更直接地提高水平响应。不过论文也报告 vectoring belt elasticity 会造成角度误差和失稳；若 Nami 未来增加主动矢量机构，必须使用实际角反馈和 rate/torque bounds。

### 13.8 推荐的最终控制架构

低速 PC 层（约 40–100 Hz）：

- takeoff trajectory / reference governor；
- desired force and attitude generation；
- tilt、acceleration、jerk 和 configuration bounds；
- 位置、速度、慢扰动反馈；
- 对可行 wrench 集预留姿态余量。

高速 FC 层：

- body-rate 或 INDI attitude control；
- 计算最终 desired wrench；
- 对 base+roll+pitch+yaw 的总命令做 bounded allocation；
- 输出 achieved wrench 和 active bounds；
- 用 residual 做 back-calculation anti-windup。

一个可用的最终分配问题：

$$
\min_{f,s}
\|W_s s\|^2
+\rho_\Delta\|f-f_{\mathrm{prev}}\|^2
+\rho_0\|D^{-1}(f-f_0)\|^2
$$

subject to：

$$
G_h(q)f+s=w_{h,d},
$$

$$
f_i^{\min}(V)+r_i^-
\le f_i\le
f_i^{\max}(V)-r_i^+,
$$

$$
-\dot f_i^-\Delta t
\le f_i-f_{i,\mathrm{prev}}
\le\dot f_i^+\Delta t.
$$

空气中 infeasible 时的建议优先级：

1. roll/pitch 安全；
2. vertical force；
3. yaw；
4. XY tracking。

不要让一个 40 Hz QP 取代高速姿态内环；它更适合做 reference governor 或低频规划，高速四电机 box-constrained allocation 可用固定维度 active-set/BVLS。

## 14. 空气–水下跨域说明

水下水平配平要求：

$$
(x_B,y_B)\approx(x_G,y_G),
$$

空中静态均载要求 CoG 接近有效推力中心。两者不是同一个条件，但可通过以下方式协同：

- 先固定干态重件，使空中 CoG 合理；
- 水下用轻质、可滑动、随 head 转动的闭孔浮材把 CoB_xy 对准 wet CoG_xy；
- 不用远端铅块破坏空中 CoG 和 Iyy；
- head/tail 分别近中性、分别对齐各自 CoB/CoG。

CoB 高于 CoG 的恢复刚度近似：

$$
K_{\mathrm{roll/pitch}}\approx B h_{BG}.
$$

h_BG 不是越大越好：FORWARD/surge 模式中较大恢复距有利于被动 pitch；主动倾斜前进时过大的恢复距会增加推进器负担。

水下四推进器为 signed、asymmetric thrust，近似范围 −14.7…+25.48 N。此时 γ_i 可为负，且近中性时 ΣF_z 可能接近零，所以 aerial ZMP/ASP 不适用；必须使用有符号 box-constrained wrench polytope。

四个水下推进器最多提供四个独立输入，而候选量 (F_x,F_z,τ_x,τ_y,τ_z) 有五个。当前几何约有：

$$
\tau_y\approx0.370F_x+0.022F_z.
$$

质量调整可减小 0.022F_z 项，但不能消除由推进器拓扑产生的 0.370F_x 耦合。

## 15. 推荐实验路线

### 15.1 台架与机械

1. 完整飞行配置称重；
2. 分别测 head/tail 质量和 CoG；
3. q=0、±15°测整机 CoG；
4. 台架标定每台电机在最低工作电压下的 duty–RPM–thrust；
5. 检查电机编号、旋向、推力轴、roll/pitch/yaw 符号；
6. 用摆法或 CAD+实测修正 Iyy。

### 15.2 控制调试

1. 修复 max_duty 对应力上限；
2. 发布 unclamped force/duty、active bound、w_cmd、w_ach 和 residual；
3. q=0、系留、I=0 调 body-rate/attitude step；
4. 加 hover trim 和 bumpless thrust ramp；
5. 使用 min-jerk takeoff；
6. 开启 Z，之后开启小 tilt 的 XY；
7. 最后加入 anti-windup integral；
8. 再做 0→5→10→15°慢速 transformation。

### 15.3 带宽验收

在不触发饱和的系留实验中，对 attitude command 做 chirp/multisine，得到幅相曲线。选择位置交越频率时至少满足：

- attitude/position bandwidth ratio ≥3–5；
- 在位置交越频率处 attitude gain 接近 1；
- attitude phase lag <20–30°；
- 最差电压下稳态电机利用率最好 ≤75–80%，至少保留明确的双向 torque margin；
- 无持续 PWM clamp；
- achieved-wrench residual 可接受且积分器不继续累积。

若希望位置交越约 0.5 Hz，则姿态闭环应至少达到约 2 Hz 量级，并把相关等效延迟降到约 0.1 s 或更低；这只是设计起点，最终应由实测频响决定。

### 15.4 ±15° transformation

推荐顺序：

1. q=0 起飞和降落；
2. 地面固定 5°、10°、15°核对分配；
3. 系留固定角悬停；
4. 以约 1–2°/s 做 0→+5→0；
5. 逐级到 +10°、+15°；
6. 再验证负角。

当前 robots/nami/launch/bringup.launch:98-100 会以 0.2 Hz 持续发布 q=0 锁定命令。进行 transformation 时必须禁用或替换该节点，并确保 Q(q)、I(q) 使用高频、真实关节反馈，而不是只使用命令角。

## 16. 可用于论文的核心论点

以下论点已有较强证据，但正式写作仍应引用原始数据、图和代码版本。

### 16.1 几何稳定不等于容量稳定

> Although the projected center of gravity lies well inside the conventional aerial support polygon, finite voltage- and duty-dependent rotor limits shrink the feasible support region substantially. For the tested configuration, the geometric margin is approximately 202 mm, whereas an explanatory capacity-bounded calculation at the measured collective demand yields only about 1.3 mm in the head direction.

### 16.2 机械偏载和固定倾斜轴耦合

> The forward CoG offset biases the front rotor pair, and the fixed ±10° rotor axes convert this thrust imbalance into a nonzero horizontal force. The resulting equivalent trim angle is approximately 1.4°, consistent with the configuration-dependent pitch offset observed in the controller and rosbag.

### 16.3 饱和保护必须使用最终 actuator domain

> Saturation handling based on a calibration-table maximum thrust is insufficient when a lower duty limit is imposed downstream. Allocation, desaturation, anti-windup, and diagnostics must share a voltage- and duty-consistent force bound.

### 16.4 提高位置带宽需要联合设计

> Increasing position-loop gains alone cannot improve the usable bandwidth when the attitude loop already contributes substantial phase lag and the actuator set is near its boundary. A practical bandwidth increase requires actuator headroom, a faster rate/attitude loop, explicit trim and trajectory feedforward, and constraint-aware allocation.

### 16.5 构型规划指标

> For transformable aerial robots, configuration feasibility should be evaluated by the worst normalized actuator utilization or the distance to a bounded wrench polytope, rather than only by the area of the unconstrained aerial support polygon or det(QQᵀ).

## 17. 尚未解决或需要补实验的问题

1. 实机准确质量、head/tail 独立 CoG 和 Iyy；
2. 最低工作电压下逐电机推力曲线和热限制；
3. M1 较早饱和中机械差异、电机差异和 yaw/roll 命令的相对贡献；
4. 起飞前 ground effect、接触释放和 rp_offset kick 的可重复性；
5. 低层 attitude loop 的完整频率响应，而不只是 0.305 Hz 单频拟合；
6. 机械调整后 q=0/±15° 的实测 thrust share；
7. achieved-wrench anti-windup 对位置振荡的改善；
8. INDI、body-rate loop、geometric controller 和 constrained allocator 的受控 A/B；
9. 水下真实 CoB、added mass、damping 和正反推力曲线；
10. 后期约 39.6 mm 地面几何高度阶跃的机械原因。

## 18. 关键文件索引

- 论文 PDF：
  - Design, Control, and Motion-Planning for a Root-Perching Rotor-Distributed Manipulator.pdf
- Nami 质量与旋翼几何：
  - robots/nami/urdf/quad/nami.urdf.xacro
- Nami 构型相关分配与 rp_offset：
  - robots/nami/src/control/nami_controller.cpp
  - robots/nami/src/model/nami_robot_model.cpp
- Nami 导航与参数：
  - robots/nami/config/NavigationConfig.yaml
  - robots/nami/config/quad/NamiControl.yaml
  - robots/nami/launch/bringup.launch
- 电机标定：
  - robots/nami/config/MotorInfoDShot.yaml
- 通用位置 PID：
  - aerial_robot_control/src/control/base/pose_linear_controller.cpp
  - aerial_robot_control/include/aerial_robot_control/control/utils/pid.h
- 通用 under-actuated compensation：
  - aerial_robot_control/src/control/under_actuated_controller.cpp
- LQI 参考：
  - aerial_robot_control/src/control/under_actuated_lqi_controller.cpp
  - aerial_robot_control/src/control/under_actuated_tilted_lqi_controller.cpp
- FC 姿态、推力换算与 PWM clamp：
  - aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/flight_control/attitude/attitude_control.cpp
  - aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/flight_control/attitude/attitude_control.h

## 19. 最短结论

Nami 的问题不是“CoG 不在旋翼凸包内”，而是“在真实电压和 duty 上限下，当前 CoG、倾斜推力轴、姿态修正和总推力需求共同把单个电机推到了可行 wrench 集边界”。Aerial ZMP/ASP 很适合解释几何条件，但必须扩展为 capacity-bounded ASP 或完整 wrench polytope。机械上先均载、再降惯量；控制上先修复真实 actuator bounds、hover feedforward、起飞切换和内环带宽，再提高位置环增益。
