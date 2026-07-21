# 深度控制链路 review：MS5837 driver（I2C/实时性/采样率）+ 深度控制环

针对"深度计直连 VIM4 的 I2C、实时性没保证、driver 采样率是随手设的"这几点做的检查。
分两部分：**driver 侧**（`src/ms5837-python`，已改）和**控制环侧**（`nami_controller` / `nami_navigation`，只审不改）。

## 一、核心实时性问题（已定位）

### 1. 「100Hz」是达不到的——阻塞式转换等待

MS5837 每次读取要**两次 ADC 转换**（D1 压力 + D2 温度），每次转换后驱动库用
`time.sleep()` 阻塞等待固定时长（`ms5837.py:131,140`）：

```
单次 read() 阻塞时间 ≈ 2 × (2.5e-6 × 2^(8+osr))
```

| OSR | 每次转换 | 单次 read 阻塞（含 I2C 事务）| 可持续速率 |
| --- | --- | --- | --- |
| 256 | 0.64 ms | ~3 ms | ≤ 200 Hz |
| 512 | 1.28 ms | ~4 ms | ≤ 150 Hz |
| **1024** | 2.56 ms | ~7 ms | ≤ 100 Hz（新默认）|
| 2048（旧默认）| 5.12 ms | ~12 ms | ≤ 60 Hz |
| 8192 | 20.5 ms | ~45 ms | ≤ 18 Hz |

**原代码 `rate_hz=100` + `OSR_2048`**：单次 read 就要 ~12ms > 10ms 周期，
`rate.sleep()` 变成空操作，实际速率被卡在 ~70-80Hz 且抖动大。注释里写的 "fitting well
within 100Hz loop" 是错的。

### 2. VIM4 直连 I2C 的实时性本质

- VIM4 跑的是**非实时 Linux**，`smbus` 的 I2C 读是用户态阻塞 syscall，`time.sleep()`
  在负载下会过冲几 ms——所以这条链路**不可能是硬实时源**。
- 但这不致命：**深度动力学是亚 Hz 的**（附加质量 + 阻尼 → 秒级时间常数），深度控制根本
  不需要高带宽采样。关键是别过采样、别把抖动耦合进控制微分项、并让降级可观测/安全。

### 3. 采样率 vs 控制环的隐藏耦合（重要）

控制环 `main_rate = 40Hz`，深度 PID 的微分项 `err_d = (err - prev_err)/dt` 用的是
**控制环时钟**（`depthControlLoop` 每个控制周期调一次，`nami_controller.cpp:126-137`）。
如果**传感器速率 < 控制速率**，连续几个控制周期读到同一个深度值 → 那几拍 err_d=0，
新样本到来时 err_d 突跳 → **阶梯状微分**，被 `depth_d_lpf_rate` 平滑但仍不干净。

→ 结论：**传感器速率应 ≥ 控制环速率**。新默认 50Hz ≥ 40Hz，保证每个控制周期都有新样本。

## 二、driver 已做的修改（`ms5837_node.py`）

| 修改 | 原来 | 现在 | 理由 |
| --- | --- | --- | --- |
| 采样率默认 | 100 Hz（达不到）| **50 Hz** | ≥ 控制环 40Hz，且 OSR_1024 下 ~7ms read 留 ~13ms 余量 |
| OSR 默认 | 2048（~12ms 阻塞）| **1024（~7ms）** | 噪声/延迟平衡；50Hz 下有充足余量 |
| 全部魔数 | 硬编码 | **rosparam** | `i2c_bus / fluid_density / osr / rate / filter_factor / max_depth / min_depth / max_velocity / max_consecutive_rejects / frame_id` |
| I2C bus | 硬编码 0 | **param（默认 0）** | VIM4 深度计实测在 /dev/i2c-0，默认保持 0；做成 param 便于换机核对 |
| 尖峰门限 | `max_diff=0.1m/样本`（速率相关）| **速度门限 `max_velocity` [m/s]** | 与采样率解耦；机体深度速度远 <1 m/s，1.5 m/s 门限抓毛刺不误伤真实下潜 |
| 尖峰持续拒绝 | 永久 `continue` → 反馈饿死 | **连续拒 N 次后重播种** | 合法大阶跃/传感器恢复不会永久堵死深度反馈 |
| 速率健康 | 无 | **read+I2C 超 1.5× 周期告警** | 提示 OSR 过高/总线太慢，台架可诊断 |
| 时间戳 | read 后取 now | 保留但注释说明 | 落后压力采样约一个转换时长，对亚 Hz 控制可忽略 |
| shebang | `python`（py2）| `python3` | Noetic/ROS-O 需 python3 |
| API bug | `rospy.advertise`（不存在）| `Publisher`/`Service` | 上一轮已修，否则节点根本起不来 |

### 台架调参建议

```bash
# 默认即可用；若台架发现深度噪声偏大，抬 OSR（同时降 rate 保余量）：
rosrun ms5837_ros ms5837_node.py _osr:=3 _rate:=40    # OSR_2048, 40Hz
# 若是海水而非泳池：
rosrun ms5837_ros ms5837_node.py _fluid_density:=1029
# 核对 I2C 总线号（VIM4）：
i2cdetect -l ; i2cdetect -y <bus>   # 应在 0x76 看到 MS5837
```

## 三、控制环侧审计（`nami_controller` / `nami_navigation`，只审不改）

这两个文件 Codex 正在改（joystick / 模式切换 / staleness），下面是审计结论，供 review 时核对：

### 已经做对的
- **深度反馈 staleness 检查**（Codex 新加）：`getDepthSensorReady()` 校验
  `depth_sensor_stamp_` 新鲜度（`depth_timeout_`），过期 → `depthControlLoop` 返回 0
  中性推力。这正好兜住 I2C 卡死/掉线的实时性风险——传感器停了，深度力归中性、微正浮力
  缓慢上浮（安全失效方向）。**这是最关键的一道防线，务必保留。**
- **微分项 dt 用控制环时钟**（非传感器时钟）+ `dt<0.2` 守卫 + LPF：对传感器抖动稳健。
- **深度力对称钳位**（`depth_thrust_limit`）：双向推力上下都能压。
- **倾斜补偿**：`tilt_factor` 在机体倾斜时维持垂直力分量。

### 建议核对/改进（留给 Codex 或后续）
1. **`depth_timeout_` 取值**：应 ≥ 2~3 个传感器周期。50Hz → 单周期 20ms，取
   `depth_timeout ≈ 0.1~0.15s`（容忍几帧丢失，又能快速发现真正掉线）。别设太小（正常
   抖动误判掉线），也别太大（掉线后还长时间用陈旧值）。
2. **深度微分的阶梯问题**：既然已把传感器提到 50Hz ≥ 控制 40Hz，基本缓解；若仍嫌 D 项
   毛糙，可考虑"仅在深度值真正更新时才更新 err_d"（用消息 stamp 判断新样本），而不是每个
   控制周期都算。当前 LPF 已能接受，属可选优化。
3. **深度环积分**：`depth_i_gain=0` 时有 ~4cm 稳态差（浮力配平残差），加一点积分（如 0.5，
   `depth_i_limit` 已有钳位）即可消除。实机浮力与仿真不同，台架标 `depth_hover_thrust`。
4. **时间戳来源**：`depthControlLoop` 用 `ros::Time::now()` 算 dt，而不是深度消息的
   stamp。控制环规律调用时这没问题；但如果哪天深度反馈改成事件驱动，要改用消息 stamp。

## 四、实时性总评

这条链路**不是也不需要是硬实时**：深度是慢过程，40Hz 控制 + 50Hz 传感足够。真正的风险是
"I2C 卡死/掉线时控制器还在用陈旧深度值"——这一点已被控制侧的 staleness 检查兜住（过期→
中性推力→安全上浮）。driver 侧的改动把"名义 100Hz 实则卡顿"改成"稳稳的 50Hz、有余量、
可诊断"，并让尖峰/掉线降级安全可观测。剩下的是台架上把 OSR/rate/`depth_timeout`/浮力
配平这几个数按实测标定（原来是随手设的）。
