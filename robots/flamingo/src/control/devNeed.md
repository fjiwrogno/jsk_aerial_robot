## Background

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
- 目前来说，这两种状态之间的切换可能也会出现一些问题，比如从水下切换为空中模式时，这个时候应该需要一些安全措施？
- 以及从空中切换为水下的时候，应该也要添加一些rules比如set current yaw为target yaw, set target_pitch&& target_roll = 0.0, target_depth = current_depth_
  - 总的来说就是类似于添加一些reset函数这样？
todo:
- attitude_control中hard coding 为3号和4号电机
  - for rotor 3&& rotor4(index is 2 and 3)pwm conversion, directly write the conversion param according to MotorInfo_water.yaml 
-     - 直接把水下controller的参数放在另外一个yaml file里面，同时读进来！
-     when swithcing to different mode, 控制器的参数也需要重新发布一次针对不同mode's gain. for underwater, publish gain directly      according to the value from flamingoControl.yaml. for cross domain mode, directly use the param read from flamingoControl.yaml file
  - meanwhile, use pid_controller->setGain to change the gain like p_gain, i_gain of pid_controller(yaw) since this part of calculation is done not in attitude_controller.cpp
  - 或者应该也可以直接去修改pid_controller(yaw).gain去set p_gain？
- 重新校对urdf
- 测试


## PLAN:
for attitude_control.cpp
- attitude_control中hard coding 为3号和4号电机 conversion and safety protection like min_duty and max_duty beacause value is read for rotor 1&2, not for rotor 3&4. so we have to manually add the value according to the value from MotorInfo_water.yaml.
- for rotor 3&& rotor4(index is 2 and 3)pwm conversion, directly write the conversion param according to MotorInfo_water.yaml. 
- 直接把水下controller的参数放在另外一个yaml file里面，同时读进来！
- when swithcing to different mode, 控制器的参数也需要重新发布一次针对不同mode's gain. for underwater, publish gain directly according to the value from flamingoControl.yaml. for cross domain mode, directly use the param read from flamingoControl.yaml file
- meanwhile, use pid_controller->setGain to change the gain like p_gain, i_gain of pid_controller(yaw) since this part of calculation is done not in attitude_controller.cpp
- 或者应该也可以直接去修改pid_controller(yaw).gain去set p_gain？
for flamingo_controll.cpp
- when in underwater mode, the item in allocation for rotor 1&rotor2 should be zero. and in attitude_controller.cpp, we need to set target_thrust[0] target_thrust[1] to 0.5(the min_duty of aerial motor)
- when in aerial mode, the item in allocation for rotor 3&rotor4 should be zero, and in attitude_controll.cpp we need to set target_thrust[2] target_thrust[3] to 0.0((the min_duty of aquatic motor))


-  
### Cross Domain Strategyfwe
WuKong uses two independent sets of thrusters, which simplifies the air-to-water transition strategy. Assisted by water pressure sensor, the transition can be completed smoothly. During
submergence, if pressure exceeds a threshold, the aerial thruster
shut down and the aquatic thrusters activate. Conversely, when
the pressure drops below the threshold during emergence, the
aquatic thrusters shut down and the aerial propellers activate.
For safety reasons, the operator needs to engage the transition
detector switch before transition to prevent repeated detection
from triggering the transition. Additionally, the throttle should
be set to around the hover level. By using the dual thruster
system and the layout of the thrusters distributed, the time of
the aerial propellers hitting the water surface is reduced during
the transition, which also improves the stability and efficiency
of the water-to-air transition.

- 感觉确实可以用depth sensor的pressure来作为一个分界点，比分界点小认为cross domain mode,比分界点大：underwater mode
- 在试验过程中直接拿手来完成这个过程
  - 在空气中calibrate,开启系统的时候，一块电池直接去measure pressure,然后直接看什么时候水桨刚刚出水面？这个时候设置为cross domain？
    - 水压大于一个阈值的时候，我们应该将空桨强制关闭！！强制转换为水下模式！
    - switch到cross domain的时候需要将初始油门设置为不是的值！
    - 在实际cross domain的时候应该还是油门初始值给大一点！
- 所以看来其实整体上完全没有出现两种电机同时控制的情况？反而是直接独立的两种模式？那么的话貌似还是可以直接通过发布的方式来切换motor info?
- 水下切换为cross domain的时候，能不能直接arming + takeoff呢？
- cross domain切换为underwater的话，那么就得立即停桨！那就直接halt?然后呢？再直接将发pwm值，arming和takeoff!
  - 把这三个步骤写成一个函数！直接通过一个按键实现了！\
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
