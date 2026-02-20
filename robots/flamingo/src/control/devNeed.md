## Develop goal

兼容具有两种推力pwm转换逻辑的两套电机系统在一个机器人上，并在不同状态时使用不同的电机系统。

- 实现使用joystick切换状态，一种状态为cross domain状态，一种状态为underwater状态。cross domain状态就是只是使用1和2号电机使用directJoystickControl来起飞。一种状态为underwater状态，只是按照原来的水下控制器的逻辑去控制机器人，只使用3号和4号电机。电机的切换在flaming_controller.cpp中controlCore()函数中的allocationMatrix来实现。只使用3号和4号电机的时候直接将对应的1和2电机对应allocation matrix的对应元素设置为0。
- 我的allocation matrix是从urdf中读取计算出来的，我的motoInfo也是从motorInfo.yaml中读取出来的，但是由于目前的系统无法支持不同的电机系统，只能读取一套参数配置。所以我预期的方案是直接在attitude_controll中hardcoding,直接将3号和4号电机的配置信息显式地code在里面，这个环节应该主要发生在pwm_conversion()函数中，也就是将target_thrust期望推力转换为target_pwm的过程中。
- 由于我的目标场景是在水面上机器人受控的前提下起飞，所以当切换为cross domain状态时，才将3和4号电机的allocation matrix设置为0并将1和2号电机对应的元素恢复为原本的数。也就是这个时候才关掉使用水下电机的控制。这点考虑主要是如果这个时候起飞到了空中，那么由于水下电机的motorInfo只是在水下测量得到的，所以在空中时水下电机其实没有什么推力所以无法对控制起到什么贡献。而空中电机无法在水下模式使用，因为其高速旋转时碰到水会有危险，所以在水下运动时严格禁止使用空中电机也就是需要将对应的allocation matrix设置为0
- attitude_control.cpp为运行在stm32上的姿态控制程序，flamingo_controller.cpp为运行在上层的机载电脑上的程序，两者之间通过ros serial来进行通讯。但是尽可能不要在attitude_control.cpp中来通过if else来切换不同的状态，比如cross domain或者underwater,因为我对这个地方的传输的代码还不是特别清楚。所以我想到的目前的方案是通过allocation matrix也就是控制下面的函数中对应电机的元素的值来切换正在运行的电机编号。如果对应的电机的allocation matrix为0,直接将对应的target_pwm设置为该种的min_duty
void FlamingoController::sendTorqueAllocationMatrixInv()
{
  spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
  torque_allocation_matrix_inv_msg.rows.resize(motor_num_*rotor_coef_);
  Eigen::MatrixXd torque_allocation_matrix_inv = integrated_map_inv_rot_;
  if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX* 0.001f)
    ROS_ERROR("Torque Allocation Matrix overflow");
  for (unsigned int i = 0; i < motor_num_*rotor_coef_; i++)
  {
    torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0)* 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) *1000;
    torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2)* 1000;
  }
  torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
}

### 具体细节

我的机器人有四个电机，1号和2号电机为空中电机，3号和4号为水下电机。
空中电机的target_pwm初始值应该0.5，水下电机的初始值应该为0.0.更加具体的motor_info如下面的信息：
空中电机的motor_info为：
motor_info:
        min_pwm: 0.58 # = 1800 [ms]
        max_pwm: 0.85 #0.92 # = 1840 [ms], 0.85 = 1700 will consume 60a current
        min_thrust: 0 #[N]
        force_landing_thrust: 7 #[N]
        m_f_rate: 0.0188 # 24.2: 0.0186, 23.2: 0.0185, 22.2: 0.0191
        pwm_conversion_mode: 1 # Polynominal Mode
        vel_ref_num: 4

        ref1:
                voltage: 25.2
                max_thrust: 31.7275
                polynominal4: 0.68654801 # x 10^4
                polynominal3: -3.83126721 # x10^3
                polynominal2: 6.91763696 # x10^2
                polynominal1: 2.30138744 # x10
                polynominal0: 60.03497876 # x1
        ref2:
                voltage: 24.2
                max_thrust: 31.4298 #18.07
                polynominal4: 0.63905147 # x 10^4
                polynominal3: -3.64968905 # x10^3
                polynominal2: 6.80317161 # x10^2
                polynominal1: 2.60947221 # x10
                polynominal0: 60.09208539  # x1
        ref3:
                voltage: 23.2
                max_thrust: 29.5775 #17.35
                polynominal4: 0.75756372 # x 10^4
                polynominal3: -4.07004437  # x10^3
                polynominal2: 7.13100100 # x10^2
                polynominal1: 2.98655147 # x10
                polynominal0: 60.14898719 # x1
        ref4:
                voltage: 22.2
                max_thrust: 26.5011 #15.91
                polynominal4: 0.95134666 # x 10^4
                polynominal3: -3.89975450 # x10^3
                polynominal2: 4.40142443 # x10^2
                polynominal1: 7.16376149 # x10
                polynominal0: 59.00332169 # x1

        # for simualtion
        rotor_damping_rate: 0.1 # = 0.001 / (0.01 + 0.001); 0.01: time constant; 0.001: dt
        rotor_force_noise: 0.01 # [N] but good performance in gazebo

水下电机的motor_info为：
motor_info:
        min_pwm: 0.0 # aquatic motor
        max_pwm: 0.92 #0.92 # = 1920ms
        min_thrust: 0 #[N]
        force_landing_thrust: 7 #[N]
        m_f_rate: 0.0136
        pwm_conversion_mode: 1 # Polynominal Mode
        vel_ref_num: 4

        ref1:
                voltage: 25.2
                max_thrust: 21.1743 #20.26
                polynominal4: 0.27220104 # x10^4
                polynominal3: 7.84268887 # x10^3
                polynominal2: -27.35529774 # x10^2
                polynominal1: 59.62689433 # x10^1
                polynominal0: 5.31928587 # x10^0
        ref2:
                voltage: 24.2
                max_thrust: 21.3149 # N
                polynominal4: -4.49312257 # x10^4
                polynominal3: 25.67117640 # x10^3
                polynominal2: -47.75818788 # x10^2
                polynominal1: 67.62445163 # x10^1
                polynominal0: 5.48946853 # x10^0
        ref3:
                voltage: 23.2
                max_thrust: 20.4890 # N
                polynominal4: -2.95746127 # x10^4
                polynominal3: 20.77747488 # x10^3
                polynominal2: -44.15328209 # x10^2
                polynominal1: 69.42786237 # x10^1
                polynominal0: 5.37462297 # x10^0
        ref4:
                voltage: 22.2
                max_thrust: 19.6376 # N
                polynominal4: -6.74821754 # x10^4
                polynominal3: 35.12434932 # x10^3
                polynominal2: -61.02618663 # x10^2
                polynominal1: 77.45026705 # x10^1
                polynominal0: 5.30078903 # x10^0

        # for simualtion
        rotor_damping_rate: 0.1 # = 0.001 / (0.01 + 0.001); 0.01: time constant; 0.001: dt
        rotor_force_noise: 0.01 # [N] but good performance in gazebo
