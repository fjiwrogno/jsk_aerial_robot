# Nami 空中变形的控制理论与实现

## 1. 目标与范围

本文结合以下论文与当前 Nami 代码，说明如何在悬停中控制内部
`yaw_joint` 从 0° 缓慢转到 30°：

- M. Zhao et al., *Transformable multirotor with two-dimensional multilinks:
  modeling, control, and motion planning for aerial transformation*。
- M. Zhao et al., *Whole-body aerial manipulation by transformable multirotor
  with two-dimensional multilinks*。

主要依据第一篇论文的 §2.1–2.3、§4.1–4.3、§5 和 §6.3，以及第二篇
论文的 §III。

当前 PID/LQI 代码路径、ROS topic、状态机和实机入口边界的逐项说明见
[`transformation_implementation_summary.md`](transformation_implementation_summary.md)。

这里的“变形”是机体内部关节角变化，不是飞行器整体航向 yaw
变化。变形期间飞行器的位置和整体 yaw 目标保持不变。

第一阶段只处理一个关节和一条已验证的一维路径，不实现任意构型的
在线路径规划，也不实现飞行中的 PID/LQI 热切换。

### 1.1 实现边界

所有机器人特化代码都放在 `robots/nami` 中。通用的 `RobotModel`、
`PoseLinearController`、`UnderActuatedLQIController` 和状态估计器不作
修改：

- `NamiRobotModel` 派生类计算 Nami 专用稳定性指标，并在本类中清理
  Nami 关节数组；
- 现有 `NamiController` 继续作为 PID 控制器，只增加稳定性状态发布；
- 新增 `NamiLQIController` 派生类，负责 Nami 的四轴模式、增益线程
  同步和最后有效增益保留；
- 控制器选择、轨迹执行和故障中止分别由 Nami 的配置、launch 和 demo
  脚本完成。

这样不会把单一机器人的构型假设传播到基础类，也不改变其他机器人的
行为。

## 2. 论文中的准静态模型

设构型关节角为 \(q\)，第 \(i\) 个旋翼相对质心的位置、推力方向和
旋转方向分别为

\[
r_i(q),\qquad u_i(q),\qquad \sigma_i .
\]

单旋翼推力 \(f_i\) 对质心产生的 wrench 为

\[
\begin{bmatrix}
u_i(q)\\
r_i(q)\times u_i(q)+\sigma_i k_m u_i(q)
\end{bmatrix}f_i ,
\]

其中 \(k_m\) 是反扭矩与推力的比例。把所有旋翼排列起来得到随构型
变化的 wrench 分配矩阵 \(P(q)\)。

论文没有直接建立包含快速关节运动的完整时变控制器，而是把每个时刻
视为一个“冻结构型”：在该构型下，质量、惯量和旋翼排列是常数，然后
根据当前构型更新控制模型。这一近似要求：

- 关节轨迹相对姿态闭环足够慢；
- 每个中间构型都可控且有足够执行器裕度；
- 模型和控制增益的更新速度明显快于构型变化；
- 关节运动造成的未建模扰动能被反馈控制压制。

因此，本项目的第一版使用 8 s 的五次多项式完成 30° 变形，而不从
快速变形开始。

## 3. Annotation 1：满秩条件与稳定性的边界

Nami 使用四个固定竖直旋翼，直接控制的四个量为

\[
y =
\begin{bmatrix}
z & \phi & \theta & \psi
\end{bmatrix}^{T}.
\]

从完整 wrench 矩阵提取并进行质量、惯量归一化后，得到

\[
P_4(q)=
\begin{bmatrix}
P_{F_z}(q)/m\\
I(q)^{-1}P_{\tau_x}(q)\\
I(q)^{-1}P_{\tau_y}(q)\\
I(q)^{-1}P_{\tau_z}(q)
\end{bmatrix}.
\]

这里用 \(P_4\) 强调它是完整六维 wrench 矩阵的四个受控输出版本；
它对应论文式 (25)–(26) 中记作 \(P\) 的矩阵。

对四旋翼，\(P_4(q)\in\mathbb{R}^{4\times4}\)。论文中的奇异构型是

\[
\operatorname{rank}P_4(q)<4.
\]

这表示 `z/roll/pitch/yaw` 中至少存在一个线性组合不能由四个电机独立
产生。奇异不是“矩阵中出现零元素”：一个旋翼对某个轴的贡献为零可以
是正常的；问题在于矩阵的行或列发生线性相关。

论文对冻结构型建立 LQI 增广系统。对其双积分器结构，可稳定化、
可检测和增广矩阵满秩条件最终都可由 \(P_4(q)\) 满行秩满足。因此，
在论文模型和假设范围内，避开奇异构型是 LQI 渐近稳定的核心条件。

但是，以下推论不成立：

> \(P_4(q)\) 满秩，所以实际变形飞行必然稳定。

满秩只说明局部线性模型没有失去控制方向。实际系统还可能因为近奇异、
电机饱和、惯量估计误差、增益更新延迟、关节运动过快或外部扰动而
不稳定。

### 3.1 工程化构型检查

本实现使用四层判据：

1. **精确失秩检查**：SVD 计算的数值秩必须为 4。
2. **近奇异检查**：行归一化后的最小奇异值必须大于阈值。
3. **静态可悬停检查**：重力平衡推力必须在每个电机的上下限内。
4. **推力储备检查**：静态推力到上下限的最小距离必须大于安全阈值。

行归一化用于消除平动加速度与角加速度量纲不同对奇异值阈值的影响。
它只用于判断矩阵相关性；实际控制能力由推力和力矩裕度另行判断。

当前基础 `RobotModel::stabilityCheck()` 检查可行控制力/力矩凸包和静态
推力，但不直接检查 \(P_4\) 的秩。Nami 因此实现自己的
`NamiRobotModel::stabilityCheck()`。PID 控制循环和 LQI 增益生成器都
调用该检查，并在 `/nami/model/stability` 发布结果；demo 只有在收到
有效状态时才继续关节轨迹。

当检查失败时，正确的运行时行为是：

- 停止关节轨迹并保持最后安全构型；
- 保留最后一组有效 LQI 增益；
- 保持飞行控制，不清零增益或电机命令；
- 记录错误并中止当前 demo。

## 4. PID 与 LQI 的控制结构

### 4.1 共同的水平位置外环

论文与当前代码都使用水平位置 PID：

\[
\ddot{x}_{des} =
k_{px}e_x+k_{ix}\int e_xdt+k_{dx}\dot e_x ,
\]

\[
\ddot{y}_{des} =
k_{py}e_y+k_{iy}\int e_ydt+k_{dy}\dot e_y .
\]

在小角度附近，水平加速度转换为目标姿态：

\[
\theta_{des}\simeq\ddot{x}_{des}/g,\qquad
\phi_{des}\simeq-\ddot{y}_{des}/g.
\]

当前代码在 `PoseLinearController` 中计算 x/y PID；PID 和 LQI 控制器都
复用这一外环。

### 4.2 PID 模式

现有 `NamiController` 对 z、roll、pitch、yaw 分别计算 PID 目标加速度，
再使用当前构型的 \(P_4(q)^\dagger\) 分配到电机。控制循环会读取当前
质心、惯量和旋翼位置，因此分配矩阵随 `yaw_joint` 更新。

其特点是结构直接、现有行为已通过零构型悬停测试，但各轴 PID 没有
显式处理构型变化产生的多变量耦合。

### 4.3 LQI 模式

LQI 状态取为

\[
x =
\begin{bmatrix}
z&\dot z&\phi&\dot\phi&\theta&\dot\theta&
\psi&\dot\psi
\end{bmatrix}^{T},
\]

再增加四个输出误差积分状态。性能指标为

\[
J=\int_0^\infty
\left(x_a^TQx_a+u^TRu\right)dt .
\]

对每个有效构型求解连续代数 Riccati 方程，得到随构型变化的
\(K(q)\)。LQI 同时控制 z、roll、pitch、yaw，而不是只替换姿态控制器。
只有水平 x/y 继续使用 PID。

代码中的 `pid_controllers_` 仍保存 z 和姿态误差、积分量及限幅状态，
但实际反馈增益来自 LQI；这只是复用误差状态容器，不表示 z 仍由独立
PID 闭环控制。

## 5. 与普通固定构型飞控的区别

固定四旋翼通常把以下量当作常数：

\[
m,\quad I,\quad r_i,\quad u_i,\quad P_4 .
\]

Nami 变形时则有：

\[
I=I(q),\qquad r_i=r_i(q),\qquad P_4=P_4(q).
\]

所以变形飞行不仅要控制机体状态，还必须管理“当前控制器是否适用于
当前机械构型”。具体多出以下工作：

- 根据关节状态持续更新质心、惯量和分配矩阵；
- 验证整个关节路径，而不是只验证起点和终点；
- LQI 模式下重新生成构型相关增益；
- 防止增益生成线程与控制线程同时修改数据；
- 对近奇异、推力储备不足和增益求解失败定义安全退化行为；
- 把关节速度限制在准静态假设可接受的范围。

LQI 能处理冻结构型下的多轴耦合，但它不是完整的快速时变控制器。

## 6. 30° 变形轨迹

令起点、终点分别为 \(q_0\) 和 \(q_f=30^\circ\)，归一化时间
\(s=t/T\)。使用五次平滑函数

\[
h(s)=10s^3-15s^4+6s^5
\]

生成

\[
q(t)=q_0+(q_f-q_0)h(t/T).
\]

该轨迹在起点和终点满足速度、加速度为零，避免阶跃关节指令直接激励
机体。默认 \(T=8\,s\)，控制命令频率为 40 Hz。

Demo 状态机为：

1. 等待飞行状态、`yaw_joint`、IMU 和稳定性反馈；
2. 在未解锁状态保持当前构型 2 s，避免估计器尚未就绪；
3. 解锁并起飞；
4. 悬停稳定 5 s；
5. 8 s 内从当前角度变形到 30°；
6. 在 30° 保持 10 s；
7. 8 s 内返回 0°；
8. 稳定后降落。

30° 是首个离线认证范围。脚本拒绝超出该范围的目标，扩展范围前必须
重新运行构型扫描测试。

## 7. Annotation 2：全轨迹验证与量化验收

只观察 Gazebo 画面不能证明控制器通过。测试应覆盖：

- 零构型悬停回归；
- 固定 30° 悬停；
- 0°→30° 的全部中间构型；
- 30° 保持；
- 30°→0° 返回过程。

自动构型测试以 1° 间隔扫描 0° 到 30°，每个样本都检查 rank、
最小奇异值和静态推力储备。运行时控制器再次检查实际构型，形成
“离线认证 + 在线监控”的两层保护。

当前模型的自动扫描结果为：

- 所有 31 个构型均为 rank 4；
- 全路径最小行归一化奇异值为 0.0772593，阈值为 0.05；
- 全路径最小静态推力储备为 7.0306 N，阈值为 1.0 N。

PID 和 LQI demo 使用完全相同的关节轨迹，并记录：

- `/nami/uav/cog/odom`
- `/nami/joint_states`
- `/nami/joints_ctrl`
- `/nami/four_axes/command`
- `/nami/debug/pose/pid`
- `/nami/debug/four_axes/gain`
- `/nami/motor_info`

每次 demo 同时生成 rosbag 和 CSV。CSV 包含时间、阶段、控制器类型、
关节目标/实际值、位置和姿态，便于计算：

- 变形过程的 roll/pitch/yaw 峰值；
- 30° 保持阶段的位置和姿态 RMS；
- 变形完成后的恢复时间；
- 关节最大跟踪误差；
- 电机推力峰值和最小剩余裕度。

建议首轮验收阈值：

| 指标 | 阈值 |
|---|---:|
| `yaw_joint` 跟踪误差 | < 0.05 rad |
| roll/pitch 峰值 | < 0.10 rad |
| 整机 yaw 偏差 | < 0.10 rad |
| z 偏差 | < 0.15 m |
| x/y 偏差 | < 0.20 m |
| CARE 求解失败 | 0 次 |
| 构型稳定性检查失败 | 0 次 |
| 电机饱和 | 0 次 |

### 7.1 当前仿真基线

为缩短回归时间，当前自动执行结果使用 4 s 变形、3 s 保持，并完成
0°→30°→0°、降落和解锁关闭。偏差相对变形前最后 1 s 悬停均值计算：

| 指标 | PID | LQI |
|---|---:|---:|
| 最大关节跟踪误差 | 0.0165 rad | 0.0156 rad |
| 最大水平位置偏差 | 0.0588 m | 0.0104 m |
| 最大高度偏差 | 0.0160 m | 0.0112 m |
| 最大 roll/pitch 偏差 | 0.0127 rad | 0.0042 rad |
| 最大整机 yaw 偏差 | 0.0296 rad | 0.0727 rad |
| 30° 保持阶段 yaw RMS | 0.0070 rad | 0.0555 rad |

两者均通过首轮安全阈值，且没有构型检查或 CARE 求解失败。这个结果不
表示当前 LQI 已经优于 PID：LQI 的位置和 roll/pitch 偏差较小，但 yaw
明显较差。因此现有 LQI 权重应视为可运行的仿真初值；真机前仍需使用
默认 8 s/10 s 轨迹重复测试，并针对 yaw 权重与带宽进行辨识和调参。

## 8. 控制器选择与运行

`bringup.launch` 使用启动参数选择控制器：

```bash
roslaunch nami bringup.launch simulation:=true real_machine:=false use_lqi:=false
roslaunch nami bringup.launch simulation:=true real_machine:=false use_lqi:=true
```

完整 PID/LQI 变形 demo：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda deactivate
source ~/Research/jsk_aerial_robot_ws3/devel/setup.bash

roslaunch nami transformation_pid.launch
roslaunch nami transformation_lqi.launch
```

默认只启动 Gazebo GUI，不启动 RViz。两个可视化进程已在 Nami 的派生
launch 中解耦，避免 RViz 图形初始化失败时因自动重启阻塞整个仿真：

```bash
# 同时启动 Gazebo GUI 和 RViz
roslaunch nami transformation_pid.launch gazebo_gui:=true rviz:=true

# 无图形回归
unset DISPLAY
roslaunch nami transformation_pid.launch headless:=true record:=false
```

如果日志停在 `Waiting for /clock`，先检查 `gzserver` 的图形驱动，而不是
调整 controller spawner。当前 NVIDIA 内核模块版本可用
`cat /proc/driver/nvidia/version` 检查，磁盘上的待加载模块版本可用
`modinfo -F version nvidia` 检查；两者不一致时应先重启系统，使新内核
模块生效。`controller_spawner` 在这种情况下等待或在关闭时被 SIGKILL，
是 Gazebo 未发布 `/clock` 的后果。

默认输出：

```text
/tmp/nami_pid_transformation_<timestamp>.bag
/tmp/nami_pid_transformation.csv
/tmp/nami_lqi_transformation_<timestamp>.bag
/tmp/nami_lqi_transformation.csv
```

双控制器在进程启动时选择，不允许飞行中热切换。硬件 LQI 配置与仿真
配置分离，在 Gazebo 完成固定构型和动态变形验收之前，不应进行真机
变形。
