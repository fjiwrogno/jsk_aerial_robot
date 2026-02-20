# Gimbalrotor 控制算法深度解析

## 1. 核心思想：内外环分离的控制架构

`gimbalrotor` 的控制系统采用了经典的分层控制思想，将复杂的六自由度（6-DOF）控制任务分解为两个协同工作的循环：

* **外环 (Outer Loop) - 任务与轨迹控制**:
  * **执行者**: `GimbalrotorController` (运行于上位机，如PC/NUC)。
  * **核心任务**: 根据导航系统给出的目标位置/速度/姿态，与飞行器当前的状态进行比较，通过PID控制器计算出期望的**总推力（Force）**和**总力矩（Torque）**。这个组合（三维力和三维力矩）在控制理论中被称为**Wrench**。
  * **关键步骤**: **控制分配 (Control Allocation)**。将计算出的总Wrench，根据飞行器的动力学模型，分配给每一个独立的电机和舵机，计算出每个电机需要产生的推力大小和方向。

* **内环 (Inner Loop) - 姿态稳定与执行**:
  * **执行者**: `AttitudeController` (运行于下位机，即飞控MCU，如STM32)。
  * **核心任务**: 接收外环分配好的指令，执行高频率的姿态稳定控制，并将最终的控制量转化为PWM信号驱动电机和舵机。
  * **关键步骤**: 将外环发来的高级指令（如各电机推力分量、目标姿态角等）转化为底层的电机PWM占空比。

这种架构的优势在于：

1. **解耦**: 将复杂的、计算量大的非线性动力学解算和控制分配放在算力强的上位机。
2. **实时性**: 将需要高速响应的姿态稳定（对扰动的快速反应）放在专用的、实时性高的MCU上。

## 2. 算法详解：从目标到PWM的完整链路

### 第 I 步: 外环 - 期望Wrench的计算 (在 `PoseLinearController` 中)

这是控制的起点。`PoseLinearController` 作为基类，负责计算6个自由度的误差，并通过PID控制器生成一个期望的加速度向量。

1. **计算误差**:
    * 位置误差: $e_p = p_{target} - p_{current}$
    * 速度误差: $e_v = v_{target} - v_{current}$
    * 姿态误差: $e_R = \frac{1}{2}(R_{target}^T R_{current} - R_{current}^T R_{target})^\vee$ (旋转矩阵误差的一种表达)
    * 角速度误差: $e_\omega = \omega_{target} - \omega_{current}$

2. **PID计算**: 对每个自由度（X, Y, Z, Roll, Pitch, Yaw）应用PID控制。以X轴为例：
    $a_{x,des} = K_{p,x} e_{p,x} + K_{i,x} \int e_{p,x} dt + K_{d,x} e_{v,x} + a_{x,ff}$
    其中 $a_{x,ff}$ 是前馈加速度。

3. **输出**: 这一步的最终输出是一个期望的**加速度向量** $a_{des}$ 和**角加速度向量** $\alpha_{des}$。

### 第 II 步: 外环 - 控制分配 (在 `GimbalrotorController` 中)

这是 `gimbalrotor` 算法的核心，它需要回答一个问题：**为了实现期望的 $a_{des}$ 和 $\alpha_{des}$，每个带云台的电机应该如何动作？**

1. **构建期望Wrench向量**:
    首先，将期望加速度转化为期望Wrench（力与力矩）。
    * 期望力: $F_{des} = m \cdot a_{des}$
    * 期望力矩: $\tau_{des} = I \cdot \alpha_{des} + \omega \times (I \cdot \omega)$
    其中，$m$ 是质量, $I$ 是转动惯量, $\omega \times (I \cdot \omega)$ 是陀螺力矩补偿。

2. **构建控制分配矩阵 (Control Allocation Matrix)**:
    这是最关键的一步。我们需要建立一个矩阵 $B$，它描述了每个执行器（电机推力、云台角度）如何对机体产生力和力矩。这个关系可以表示为：
    $W_{real} = B \cdot u$
    其中，$W_{real}$ 是实际产生的Wrench，$u$ 是控制输入向量（每个电机的推力分量）。

3. **求解控制输入**:
    我们的目标是让实际产生的Wrench等于期望的Wrench，即 $W_{des} = B \cdot u$。为了求解控制输入 $u$，我们需要计算 $B$ 的伪逆 $B^{\dagger}$。
    $u = B^{\dagger} \cdot W_{des}$

4. **分解指令**:
    求解出的向量 $u$ 包含了每个电机需要产生的推力分量。现在需要将其分解为**上位机（PC）**发送给**下位机（MCU）**的指令。这些指令被打包在 `spinal::FourAxisCommand` 消息中。
    * **`base_thrust`**: 这是每个电机需要产生的推力分量。对于2自由度云台，每个电机有3个分量（x, y, z）。
    * **`angles`**: 包含了补偿性的姿态角和用于前馈的偏航项。

### 第 III 步: 内环 - 姿态控制与执行 (在 `AttitudeController` 中)

下位机MCU接收到 `spinal::FourAxisCommand` 消息后，`AttitudeController` 开始工作。

1. **接收指令**: 在 `fourAxisCommandCallback` 中，MCU接收 `base_thrust` 和 `angles`。

2. **PWM转换 (`pwmConversion`)**:
    这是MCU中的核心计算。它根据云台的自由度（`gimbal_dof_`），将 `base_thrust` 中的推力分量重新合成为每个电机**总的推力大小**和**云台的目标角度**。
    * **计算总推力**: 例如，对于2自由度云台，总推力 $f_{total} = \sqrt{f_x^2 + f_y^2 + f_z^2}$。
    * **计算云台角度**: 例如，`gimbal_roll = atan2(-fy, fz)`。
    * **推力到PWM**: 将计算出的总推力 $f_{total}$，通过查表或多项式拟合（`convert`函数），转换为一个 `0.0` 到 `1.0` 之间的逻辑PWM值 `target_pwm_`。

3. **最终PWM生成 (`pwmsControl`)**:
    这一步是最终的执行环节。它将 `target_pwm_` 这个逻辑值，根据我们之前讨论过的逻辑，翻译成最终写入定时器捕获/比较寄存器（CCR）的物理值。

    ```cpp
    // 示例: 双向电机
    float pulse_bi = 1000.0f + target_pwm_[i] * 1000.0f;
    pwm_htim1_->Instance->CCR1 = (uint32_t)pulse_bi;
    ```

    这里的 `pulse_bi` 就是最终的脉冲宽度（单位µs），它被写入 `CCR` 寄存器，硬件定时器据此生成精确的PWM波形，驱动电调和电机。

## 4. 总结与流程图

```mermaid
graph TD
    A[导航目标\n(Target Pose/Vel)] --> B{PoseLinearController (Outer Loop PID)};
    B --> C[期望Wrench\n(Force & Torque)];
    C --> D{GimbalrotorController\n<b>Control Allocation</b>\n$u = B^{\dagger} \cdot W_{des}$};
    D --> E[分解为电机推力分量\n(target_base_thrust_)];
    E --> F[打包为FourAxisCommand消息];
    F -- ROS Serial --> G{AttitudeController (Inner Loop)};
    G --> H{PWM Conversion\n推力分量 -> 总推力 -> target_pwm_};
    H --> I{pwmsControl\ntarget_pwm_ -> Pulse Width (µs)};
    I --> J[STM32定时器硬件\n(TIM1->CCR1/2...)];
    J --> K[电机 & 电调];

    subgraph 上位机 (PC/NUC)
        A; B; C; D; E; F;
    end

    subgraph 下位_机 (STM32 MCU)
        G; H; I; J;
    end

    subgraph 物理世界
        K
    end
```
