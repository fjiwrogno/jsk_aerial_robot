/*
******************************************************************************
* File Name          : attitude_control.h
* Description        : attitude control interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "flight_control/attitude/attitude_control.h"

#ifdef SIMULATION
#include <sensor_msgs/JointState.h>
AttitudeController::AttitudeController(): DELTA_T(0), prev_time_(-1), sim_voltage_(0), gimbal_dof_(0), rotor_coef_(1)
{
}

void AttitudeController::init(ros::NodeHandle* nh, StateEstimate* estimator)
{
  nh_ = nh;
  estimator_ = estimator;

  pwms_pub_ = nh_->advertise<spinal::Pwms>("motor_pwms", 1);
  control_term_pub_ = nh_->advertise<spinal::RollPitchYawTerms>("rpy/pid", 1);
  control_feedback_state_pub_ = nh_->advertise<spinal::RollPitchYawTerm>("rpy/feedback_state", 1);
  anti_gyro_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("gyro_moment_compensation", 1);
  four_axis_cmd_sub_ = nh_->subscribe("four_axes/command", 1, &AttitudeController::fourAxisCommandCallback, this);
  pwm_info_sub_ = nh_->subscribe("motor_info", 1, &AttitudeController::pwmInfoCallback, this);
  rpy_gain_sub_ = nh_->subscribe("rpy/gain", 1, &AttitudeController::rpyGainCallback, this);
  p_matrix_pseudo_inverse_inertia_sub_ = nh_->subscribe("p_matrix_pseudo_inverse_inertia", 1, &AttitudeController::pMatrixInertiaCallback, this);
  pwm_test_sub_ = nh_->subscribe("pwm_test", 1, &AttitudeController::pwmTestCallback, this);
  att_control_srv_ = nh_->advertiseService("set_attitude_control", &AttitudeController::setAttitudeControlCallback, this);
  torque_allocation_matrix_inv_sub_ = nh_->subscribe("torque_allocation_matrix_inv", 1, &AttitudeController::torqueAllocationMatrixInvCallback, this);
  sim_vol_sub_ = nh_->subscribe("set_sim_voltage", 1, &AttitudeController::setSimVolCallback, this);
  offset_rot_sub_ = nh_->subscribe("desire_coordinate", 1, &AttitudeController::offsetRotCallback, this);
  baseInit();
  gimbal_control_pub_ = nh_->advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
}

#else

AttitudeController::AttitudeController():
  pwms_pub_("motor_pwms", &pwms_msg_),
  control_term_pub_("rpy/pid", &control_term_msg_),
  control_feedback_state_pub_("rpy/feedback_state", &control_feedback_state_msg_),
  four_axis_cmd_sub_("four_axes/command", &AttitudeController::fourAxisCommandCallback, this ),
  pwm_info_sub_("motor_info", &AttitudeController::pwmInfoCallback, this),
  rpy_gain_sub_("rpy/gain", &AttitudeController::rpyGainCallback, this),
  pwm_test_sub_("pwm_test", &AttitudeController::pwmTestCallback, this ),
  p_matrix_pseudo_inverse_inertia_sub_("p_matrix_pseudo_inverse_inertia", &AttitudeController::pMatrixInertiaCallback, this),
  torque_allocation_matrix_inv_sub_("torque_allocation_matrix_inv", &AttitudeController::torqueAllocationMatrixInvCallback, this),
  offset_rot_sub_("desire_coordinate", &AttitudeController::offsetRotCallback, this ),
  att_control_srv_("set_attitude_control", &AttitudeController::setAttitudeControlCallback, this),
  esc_telem_pub_("esc_telem", &esc_telem_msg_)
{
}

void AttitudeController::init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2, StateEstimate* estimator,
                              DShot* dshot, DirectServo* servo, BatteryStatus* bat, ros::NodeHandle* nh, osMutexId* mutex)
{

  pwm_htim1_ = htim1;
  pwm_htim2_ = htim2;
  nh_ = nh;
  estimator_ = estimator;
  dshot_ = dshot;
  servo_ = servo;
  bat_ = bat;
  mutex_ = mutex;

  if(!dshot_)
    {
      HAL_TIM_PWM_Stop(pwm_htim1_, TIM_CHANNEL_1);
      HAL_TIM_Base_Stop(pwm_htim1_);
      HAL_TIM_Base_DeInit(pwm_htim1_);

      pwm_htim1_->Init.Prescaler = 3;
      pwm_htim1_->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
      pwm_htim1_->Init.Period = 50000;

      TIM_OC_InitTypeDef sConfigOC = {0};
      sConfigOC.OCMode = TIM_OCMODE_PWM1;
      sConfigOC.Pulse = 1000;
      sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

      while(HAL_TIM_Base_Init(pwm_htim1_) != HAL_OK);
      while(HAL_TIM_PWM_Init(pwm_htim1_) != HAL_OK);
      while(HAL_TIM_PWM_ConfigChannel(pwm_htim1_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK);

      if (pwm_htim1_->hdma[TIM_DMA_ID_UPDATE] != NULL) {
        HAL_DMA_DeInit(pwm_htim1_->hdma[TIM_DMA_ID_UPDATE]);
        pwm_htim1_->hdma[TIM_DMA_ID_UPDATE] = NULL;
      }

      HAL_TIM_Base_Start(pwm_htim1_);

      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_4);
    }
#if !GX_SERVO_PWM
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_4);
#endif
  nh_->advertise(pwms_pub_);
  nh_->advertise(control_term_pub_);
  nh_->advertise(control_feedback_state_pub_);
  nh_->advertise(esc_telem_pub_);

  nh_->subscribe(four_axis_cmd_sub_);
  nh_->subscribe(pwm_info_sub_);
  nh_->subscribe(rpy_gain_sub_);
  nh_->subscribe(pwm_test_sub_);
  nh_->subscribe(p_matrix_pseudo_inverse_inertia_sub_);
  nh_->subscribe(torque_allocation_matrix_inv_sub_);
  nh_->subscribe(offset_rot_sub_);

  nh_->advertiseService(att_control_srv_);

  baseInit();
}
#endif

void AttitudeController::baseInit()
{
 // base param for uav model
  motor_number_ = 0;
  uav_model_ = -1;
  rotor_devider_ = 1;

  start_control_flag_ = false;
  force_landing_flag_ = false;
  att_control_flag_ = true;

  // pwm
  pwm_conversion_mode_ = -1;
  min_duty_ = IDLE_DUTY;
  max_duty_ = IDLE_DUTY; //should assign right value from PC(ros)
  min_thrust_ = 0;
  force_landing_thrust_ = 0;
  pwm_pub_last_time_ = 0;
  pwm_test_flag_ = false;

  // voltage
  motor_info_.resize(0);
  v_factor_ = 1;
  motor_ref_index_ = 0;
  water_v_factor_ = 1;
  water_motor_ref_index_ = 0;
  voltage_update_last_time_ = 0;

  control_term_pub_last_time_ = 0;
  control_feedback_state_pub_last_time_ = 0;

  // frame
  offset_rot_.identity();

  reset();
}

void AttitudeController::pwmsControl(void)
{
  /* target thrust -> target pwm */
  pwmConversion();

#ifdef SIMULATION
  /* control result publish */
  if(HAL_GetTick() - control_term_pub_last_time_ > CONTROL_TERM_PUB_INTERVAL)
    {
      control_term_pub_last_time_ = HAL_GetTick();
      control_term_pub_.publish(control_term_msg_);
    }

  if(HAL_GetTick() - control_feedback_state_pub_last_time_ > CONTROL_FEEDBACK_STATE_PUB_INTERVAL)
    {
      control_feedback_state_pub_last_time_ = HAL_GetTick();
      control_feedback_state_pub_.publish(control_feedback_state_msg_);
    }

  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(pwms_msg_);
    }

#else
  /* control result publish */
  if(uav_model_ == spinal::UavInfo::DRAGON)
    {
      if(HAL_GetTick() - control_feedback_state_pub_last_time_ > CONTROL_FEEDBACK_STATE_PUB_INTERVAL)
        {
          control_feedback_state_pub_last_time_ = HAL_GetTick();
          control_feedback_state_pub_.publish(&control_feedback_state_msg_);
        }
    }
  else
    {
      if(HAL_GetTick() - control_term_pub_last_time_ > CONTROL_TERM_PUB_INTERVAL)
        {
          control_term_pub_last_time_ = HAL_GetTick();
          control_term_pub_.publish(&control_term_msg_);
        }
    }

  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(&pwms_msg_);
    }

  /* nerve comm type */
#if NERVE_COMM
  for(int i = 0; i < motor_number_; i++) {
#if MOTOR_TEST

    if (i == (HAL_GetTick() / 2000) % motor_number_)
      Spine::setMotorPwm(200, i);
    else
      Spine::setMotorPwm(0, i);
#else
    Spine::setMotorPwm(target_pwm_[i] * 2000 - 1000, i);
#endif
  }
#endif

  if(dshot_)
    {
      /* direct pwm type */
      uint16_t motor_value[4] = { 0, 0, 0, 0 };
      for (int i = 0; i < 4; i++)
        {
          // target_pwm_: 0.5 ~ 1.0
          uint16_t motor_v = (uint16_t)((target_pwm_[i] - 0.5) / 0.5 * DSHOT_RANGE + DSHOT_MIN_THROTTLE);

          if (motor_v > DSHOT_MAX_THROTTLE)
            motor_v = DSHOT_MAX_THROTTLE;
          else if (motor_v < DSHOT_MIN_THROTTLE)
            motor_v = DSHOT_MIN_THROTTLE;
    
          motor_value[i] = motor_v;
        }

      dshot_->write(motor_value, dshot_->is_telemetry_);

      if (dshot_->is_telemetry_)
        {
          if (dshot_->esc_reader_.is_update_all_msg_)
            {
              esc_telem_msg_.stamp = nh_->now();
              esc_telem_msg_.esc_telemetry_1 = dshot_->esc_reader_.esc_msg_1_;
              esc_telem_msg_.esc_telemetry_2 = dshot_->esc_reader_.esc_msg_2_;
              esc_telem_msg_.esc_telemetry_3 = dshot_->esc_reader_.esc_msg_3_;
              esc_telem_msg_.esc_telemetry_4 = dshot_->esc_reader_.esc_msg_4_;
              esc_telem_pub_.publish(&esc_telem_msg_);

              float voltage_ave = (float)(dshot_->esc_reader_.esc_msg_1_.voltage + dshot_->esc_reader_.esc_msg_2_.voltage +
                                          dshot_->esc_reader_.esc_msg_3_.voltage + dshot_->esc_reader_.esc_msg_4_.voltage) / 400.0;
              bat_->update(voltage_ave);

              dshot_->esc_reader_.is_update_all_msg_ = false;
            }
        }
    }
  else
    {
      // --- Channels 1 & 2: aerial rotors, pwm range[0.5~1.0] ---
      // --- Channels 3 & 4: auqatic Thrusters ,pwm range[0.0~1.0]---

      // 0.0 ~ 0.5 / 0.5 ~ 1.0 -> currently abnormal -> only half thrust, double direction
      // double direction driver is kepy but not used here.
        float pulse_bi = 1000.0f + target_pwm_[i] * 1000.0f;

        if (pulse_bi < 1000.0f)
        {
          pulse_bi = 1000.0f;
        }
        if (pulse_bi > 2000.0f)
        {
          pulse_bi = 2000.0f;
        }


          pwm_htim2_->Instance->CCR3 = (uint32_t)1000.0f + target_pwm_[2] * 1000.0f;
          pwm_htim2_->Instance->CCR4 = (uint32_t)1000.0f + target_pwm_[3] * 1000.0f;

      // pwm command for single-direction aerial motor
      //  hardcoding for enable target_pwm1&2 for aerial motor
      pwm_htim1_->Instance->CCR1 = (uint32_t)(target_pwm_[0] * pwm_htim1_->Init.Period);
      pwm_htim1_->Instance->CCR2 = (uint32_t)(target_pwm_[1] * pwm_htim1_->Init.Period);
      // pwm_htim1_->Instance->CCR3 = (uint32_t)(target_pwm_[2] * pwm_htim1_->Init.Period);
      // pwm_htim1_->Instance->CCR4 = (uint32_t)(target_pwm_[3] * pwm_htim1_->Init.Period);   
    }
#if !GX_PWM_SERVO
  pwm_htim2_->Instance->CCR1 = (uint32_t)(target_pwm_[4] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR2 = (uint32_t)(target_pwm_[5] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR3 = (uint32_t)(target_pwm_[6] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR4 = (uint32_t)(target_pwm_[7] * pwm_htim2_->Init.Period);
#endif

#endif
}

void AttitudeController::update(void)
{
#ifndef SIMULATION
  /* mutex to wait for the completion of update of ros callback function */
  if(mutex_ != NULL)
    {
      osMutexWait(*mutex_, osWaitForever);
      osMutexRelease(*mutex_);
    }
#endif

  if(start_control_flag_ && att_control_flag_)
    {
      /* failsafe 1: check the timeout of the flight command receive process */
      if(failsafe_ && !force_landing_flag_ &&
         (int32_t)(HAL_GetTick() - flight_command_last_stamp_) > FLIGHT_COMMAND_TIMEOUT)
        {
          /* timeout => start force landing */
#ifdef SIMULATION
          ROS_ERROR("failsafe: cannot connect with ROS. time now: %d, time last stamp: %d", HAL_GetTick(), flight_command_last_stamp_);
#else
          nh_->logerror("failsafe: cannot connect with ROS");
#endif
          setForceLandingFlag(true);
        }

#ifdef SIMULATION

      if(prev_time_ < 0) DELTA_T = 0;
      else DELTA_T = ros::Time::now().toSec() - prev_time_;
      prev_time_ = ros::Time::now().toSec();
#endif

      ap::Matrix3f base_rot = estimator_->getAttEstimator()->getRotation();
      ap::Vector3f base_vel = estimator_->getAttEstimator()->getAngular();
      ap::Matrix3f rot = base_rot * offset_rot_.transposed();
      ap::Vector3f vel = offset_rot_ * base_vel;
      ap::Vector3f angles; // euler angles
      rot.to_euler(&angles.x, &angles.y, &angles.z);

      /* failsafe 3: too large tile angle */
      /// TODO  here when switch to underwater mode, the max tilt angle limit should be wider
      if(!force_landing_flag_  && (fabs(angles[X]) > MAX_TILT_ANGLE || fabs(angles[Y]) > MAX_TILT_ANGLE))
        {
#ifdef SIMULATION
          ROS_ERROR("failsafe: the roll pitch angles are too large, roll: %f (%f), pitch: %f (%f)",
                    angles[X], MAX_TILT_ANGLE, angles[Y], MAX_TILT_ANGLE);
#else
          nh_->logerror("failsafe: the roll pitch angles are too large");
#endif
          setForceLandingFlag(true);
          error_angle_i_[X] = 0;
          error_angle_i_[Y] = 0;
        }

      /* Force Landing Flag */
      if(force_landing_flag_)
        {
          target_angle_[X] = 0;
          target_angle_[Y] = 0;
          target_angle_[Z] = 0;

          for(int i = 0; i < motor_number_; i++) extra_yaw_pi_term_[i] = 0;
        }

      // linear control method
      {
        /* gyro moment */
        ap::Vector3f gyro_moment = vel % (inertia_ * vel);
#ifdef SIMULATION
        std_msgs::Float32MultiArray anti_gyro_msg;
#endif

        float error_angle[3];
        for(int axis = 0; axis < 3; axis++)
          {
            error_angle[axis] = target_angle_[axis] - angles[axis];
            if(integrate_flag_) error_angle_i_[axis] += error_angle[axis] * DELTA_T;

            if(axis == X)
              {
                control_feedback_state_msg_.roll_p = error_angle[axis] * 1000;
                control_feedback_state_msg_.roll_i = error_angle_i_[axis] * 1000;
                control_feedback_state_msg_.roll_d = vel[axis]  * 1000;
              }
            if(axis == Y)
              {
                control_feedback_state_msg_.pitch_p = error_angle[axis] * 1000;
                control_feedback_state_msg_.pitch_i = error_angle_i_[axis] * 1000;
                control_feedback_state_msg_.pitch_d = vel[axis]  * 1000;

              }
            if(axis == Z)
              {
                control_feedback_state_msg_.yaw_d = vel[axis] * 1000;
              }
          }

        float p_term = 0;
        float i_term = 0;
        float d_term = 0;
        for(int i = 0; i < motor_number_; i++)
          {
            for(int axis = 0; axis < 3; axis++)
              {
                p_term = error_angle[axis] * thrust_p_gain_[i][axis];
                i_term = error_angle_i_[axis] * thrust_i_gain_[i][axis];
                d_term = -vel[axis] * thrust_d_gain_[i][axis];
                if(axis == X)
                  {
                    roll_pitch_term_[i] = p_term + i_term + d_term; // [N]
                    control_term_msg_.motors[i].roll_p = p_term * 1000;
                    control_term_msg_.motors[i].roll_i= i_term * 1000;
                    control_term_msg_.motors[i].roll_d = d_term * 1000;
                  }
                if(axis == Y)
                  {
                    roll_pitch_term_[i] += (p_term + i_term + d_term); // [N]
                    control_term_msg_.motors[i].pitch_p = p_term * 1000;
                    control_term_msg_.motors[i].pitch_i = i_term * 1000;
                    control_term_msg_.motors[i].pitch_d = d_term * 1000;
                  }
                if(axis == Z)
                  {
                    yaw_term_[i] = extra_yaw_pi_term_[i] + d_term;
                    control_term_msg_.motors[i].yaw_d = d_term * 1000; //d_term;
                  }
              }

            /* gyro moment compensation */
            float gyro_moment_compensate =
              p_matrix_pseudo_inverse_[i][0] * gyro_moment.x +
              p_matrix_pseudo_inverse_[i][1] * gyro_moment.y +
              p_matrix_pseudo_inverse_[i][2] * gyro_moment.z;
            roll_pitch_term_[i] += gyro_moment_compensate;

#ifdef SIMULATION
            anti_gyro_msg.data.push_back(gyro_moment_compensate);
#endif
          }

#ifdef SIMULATION
        anti_gyro_pub_.publish(anti_gyro_msg);
#endif
      }

      if(force_landing_flag_)
        {
          float total_thrust = 0;
          /* sum */
          for(int i = 0; i < motor_number_; i++) total_thrust += base_thrust_term_[i];
          /* average */
          float average_thrust = total_thrust / motor_number_;

          if(average_thrust > force_landing_thrust_)
            {
              for(int i = 0; i < motor_number_; i++)
                base_thrust_term_[i] -= (base_thrust_term_[i] / average_thrust * FORCE_LANDING_INTEGRAL);
            }
        }
    }
  /* target thrust -> target pwm -> HAL */
  pwmsControl();
}


void AttitudeController::reset(void)
{
  for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
    {
      target_thrust_[i] = 0;
      target_pwm_[i] = IDLE_DUTY;
      pwm_test_value_[i] = IDLE_DUTY;

      base_thrust_term_[i] = 0;
      roll_pitch_term_[i] = 0;
      yaw_term_[i] = 0;
      extra_yaw_pi_term_[i] = 0;

      for (int j = 0; j < 3; j++)
        {
          thrust_p_gain_[i][j] = 0;
          thrust_i_gain_[i][j] = 0;
          thrust_d_gain_[i][j] = 0;
          torque_allocation_matrix_inv_[i][j] = 0.0;
        }
    }

  for(int i = 0; i < 3; i++)
    {
      target_angle_[i] = 0;
      error_angle_i_[i] = 0;

      torque_p_gain_[i] = 0;
      torque_i_gain_[i] = 0;
      torque_d_gain_[i] = 0;
    }

  max_yaw_term_index_ = -1;
  integrate_flag_ = false;

  /* failsafe */
  failsafe_ = false;
  flight_command_last_stamp_ = HAL_GetTick();

#ifdef SIMULATION
  prev_time_ = -1;
#endif
}

void AttitudeController::fourAxisCommandCallback( const spinal::FourAxisCommand &cmd_msg)
{
  if(!start_control_flag_) return; //do not receive command


  /* failsafe: if the pitch and roll angle is too big, start force landing */
  if(fabs(cmd_msg.angles[0]) > MAX_TILT_ANGLE || fabs(cmd_msg.angles[1]) > MAX_TILT_ANGLE )
    {
      setForceLandingFlag(true);
#ifdef SIMULATION
      ROS_ERROR("failsafe: target angles are too large, roll: %f (%f), pitch: %f ()%f",
                target_angle_[X], MAX_TILT_ANGLE, target_angle_[Y], MAX_TILT_ANGLE);
#else
      nh_->logerror("failsafe: target angles are too large");
#endif
    }

  /* check the number of motor which should be equal to the ros thrust */
#ifdef SIMULATION
  if(cmd_msg.base_thrust.size() != motor_number_)
    {
      ROS_ERROR("fource axis commnd: motor number is not identical between fc(%d) and pc(%ld)", motor_number_, cmd_msg.base_thrust.size());
      return;
    }
#else
  if(cmd_msg.base_thrust_length != motor_number_)
    {
      nh_->logerror("fource axis commnd: motor number is not identical between fc and pc");
      return;
    }
#endif

  if(force_landing_flag_)
    {
      float total_thrust = 0;
      for(int i = 0; i < motor_number_; i++) total_thrust += cmd_msg.base_thrust[i];
      float average_thrust = total_thrust / motor_number_;
      if(average_thrust < force_landing_thrust_) return;
    }


#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  /* start failsafe func if not activate */
  if(!failsafe_) failsafe_ = true;
  flight_command_last_stamp_ = HAL_GetTick();

  target_angle_[X] = cmd_msg.angles[0];
  target_angle_[Y] = cmd_msg.angles[1];

  int max_yaw_term_index = max_yaw_term_index_;
  float max_yaw_thrust_d_gain = thrust_d_gain_[max_yaw_term_index][Z];

  for (int i = 0; i < motor_number_; i++)
  {
    // base thrust is about the z control
    base_thrust_term_[i] = cmd_msg.base_thrust[i];

    // reconstruct the pi term for yaw (temporary measure for pwm saturation avoidance)
    if (max_yaw_term_index_ != -1)
    {
      extra_yaw_pi_term_[i] = cmd_msg.angles[Z] * thrust_d_gain_[i][Z] / max_yaw_thrust_d_gain;
    }
  }

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::pwmInfoCallback( const spinal::PwmInfo &info_msg)
{
#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  force_landing_thrust_ = info_msg.force_landing_thrust;

#if DEBUG_WATER_MODE
  min_duty_ = 0;
#else
  min_duty_ = info_msg.min_pwm;
#endif
  max_duty_ = info_msg.max_pwm;
  pwm_conversion_mode_ = info_msg.pwm_conversion_mode;

  min_thrust_ = info_msg.min_thrust; // make a variant min_duty_

  motor_info_.resize(0);

#ifdef SIMULATION
  for(int i = 0; i < info_msg.motor_info.size(); i++)
#else
    for(int i = 0; i < info_msg.motor_info_length; i++)
#endif
      {
        motor_info_.push_back(info_msg.motor_info[i]);
      }

#ifdef SIMULATION
  if(sim_voltage_== 0) sim_voltage_ = motor_info_[0].voltage;
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::rpyGainCallback( const spinal::RollPitchYawTerms &gain_msg)
{
  if(motor_number_ == 0) return; //not be activated

  /* check the number of motor which should be equal to the ros thrust */
#ifdef SIMULATION
  if(gain_msg.motors.size() != motor_number_ && gain_msg.motors.size() != 1)
    {
      ROS_ERROR("rpy gain: motor number is not identical between fc:%d and pc:%d", motor_number_, (int)gain_msg.motors.size());
      return;
    }
#else
  if(gain_msg.motors_length != motor_number_ && gain_msg.motors_length != 1)
    {
      nh_->logerror("rpy gain: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

#ifdef SIMULATION
  if(gain_msg.motors.size() == 1)
#else
  if(gain_msg.motors_length == 1)
#endif
    {
      torque_p_gain_[X] = gain_msg.motors[0].roll_p * 0.001f;
      torque_p_gain_[Y] = gain_msg.motors[0].pitch_p * 0.001f;
      torque_i_gain_[X] = gain_msg.motors[0].roll_i * 0.001f;
      torque_i_gain_[Y] = gain_msg.motors[0].pitch_i * 0.001f;
      torque_d_gain_[X] = gain_msg.motors[0].roll_d * 0.001f;
      torque_d_gain_[Y] = gain_msg.motors[0].pitch_d * 0.001f;
      torque_d_gain_[Z] = gain_msg.motors[0].yaw_d * 0.001f;

      thrustGainMapping(); // gain mapping
    }
  else
    {
      for(int i = 0; i < motor_number_; i++)
        {
          thrust_p_gain_[i][X] = gain_msg.motors[i].roll_p * 0.001f;
          thrust_i_gain_[i][X] = gain_msg.motors[i].roll_i * 0.001f;
          thrust_d_gain_[i][X] = gain_msg.motors[i].roll_d * 0.001f;
          thrust_p_gain_[i][Y] = gain_msg.motors[i].pitch_p * 0.001f;
          thrust_i_gain_[i][Y] = gain_msg.motors[i].pitch_i * 0.001f;
          thrust_d_gain_[i][Y] = gain_msg.motors[i].pitch_d * 0.001f;
          thrust_d_gain_[i][Z] = gain_msg.motors[i].yaw_d * 0.001f;
        }
    }
  maxYawGainIndex();

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::torqueAllocationMatrixInvCallback(const spinal::TorqueAllocationMatrixInv& msg)
{
  if(motor_number_ == 0 || !start_control_flag_) return;

#ifdef SIMULATION
  if(msg.rows.size() != motor_number_)
    {
      ROS_ERROR("torqueAllocationMatrixInvCallback: motor number is not identical between fc(%d) and pc(%ld)", motor_number_, msg.rows.size());
      return;
    }
#else
  if(msg.rows_length != motor_number_)
    {
      nh_->logerror("torqueAllocationMatrixInvCallback: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  for (int i = 0; i < motor_number_; i++)
    {
      torque_allocation_matrix_inv_[i][X] = msg.rows[i].x * 0.001f;
      torque_allocation_matrix_inv_[i][Y] = msg.rows[i].y * 0.001f;
      torque_allocation_matrix_inv_[i][Z] = msg.rows[i].z * 0.001f;
    }

  thrustGainMapping(); // gain mapping

  maxYawGainIndex();

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::thrustGainMapping()
{
  for(int i = 0; i < motor_number_; i++)
    {
      thrust_p_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_p_gain_[X];
      thrust_i_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_i_gain_[X];
      thrust_d_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_d_gain_[X];
      thrust_p_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_p_gain_[Y];
      thrust_i_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_i_gain_[Y];
      thrust_d_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_d_gain_[Y];
      thrust_d_gain_[i][Z] = torque_allocation_matrix_inv_[i][Z] * torque_d_gain_[Z];
    }
}

void AttitudeController::maxYawGainIndex()
{
  float max_yaw_gain = 0;
  max_yaw_term_index_ = -1;
  for(int i = 0; i < motor_number_; i++)
    {
      /* only find the maximum (positive) value */
      /* to avoid identical absolute value */
      if(fabs(thrust_d_gain_[i][Z]) > max_yaw_gain)
        {
          max_yaw_gain = fabs(thrust_d_gain_[i][Z]);
          max_yaw_term_index_ = i;
        }
    }
}

void AttitudeController::pwmTestCallback(const spinal::PwmTest& pwm_msg)
{
#ifndef SIMULATION  
  if(pwm_msg.pwms_length && !pwm_test_flag_)
    {
      pwm_test_flag_ = true;
      nh_->logwarn("Enter pwm test mode");
    }
  else if(!pwm_msg.pwms_length && pwm_test_flag_)
    {
      pwm_test_flag_ = false;
      nh_->logwarn("Escape from pwm test mode");
      return;
    }

  if(pwm_msg.motor_index_length)
    {
      /*Individual test mode*/
      if(pwm_msg.motor_index_length != pwm_msg.pwms_length)
        {
          nh_->logerror("The number of index does not match the number of pwms.");
          return;
        }
      for(int i = 0; i < pwm_msg.motor_index_length; i++){
        int motor_index = pwm_msg.motor_index[i];
                /*fail safe*/
        if (pwm_msg.pwms[i] >= IDLE_DUTY && pwm_msg.pwms[i] <= MAX_PWM)
          {
            pwm_test_value_[motor_index] = pwm_msg.pwms[i];
          }
        else if ((i > 1) && pwm_msg.pwms[i] >= 0 && pwm_msg.pwms[i] <= 0.92)
          {
        // hardcoding to enable pwm test for bi-directional rotor usage
        // right now only the 3th and 4th rotor(2,3) are enabled
        // throttle range for bi-directional rotor: 0.5-0:reverse 0.5-1.0:
            pwm_test_value_[motor_index] = pwm_msg.pwms[i];
          }
        else
          {
            nh_->logwarn("FAIL SAFE!  Invaild PWM value for motor");
            pwm_test_value_[motor_index] = IDLE_DUTY;
          }
      }
    }
  else
    {
      /*Simultaneous test mode*/
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++){
        /*fail safe*/
        if (pwm_msg.pwms[0] >= IDLE_DUTY && pwm_msg.pwms[0] <= MAX_PWM)
          {
            pwm_test_value_[i] = pwm_msg.pwms[0];
          }
        else if ((i > 1) && pwm_msg.pwms[0] >= 0 && pwm_msg.pwms[0] <= 0.92)
          {
        // hardcoding to enable pwm test for bi-directional rotor usage
        // right now only the 3th and 4th rotor(2,3) are enabled
        // throttle range for bi-directional rotor: 0.5-0:reverse 0.5-1.0:
          pwm_test_value_[i] = pwm_msg.pwms[0];
          }
        else
          {
            nh_->logwarn("FAIL SAFE!  Invaild PWM value for motors");
            pwm_test_value_[i] = IDLE_DUTY;
          }
      }
    }
#endif
}

void AttitudeController::setStartControlFlag(bool start_control_flag)
{
  start_control_flag_ = start_control_flag;

  if(!start_control_flag_) reset();
}

void AttitudeController::setMotorNumber(uint16_t motor_number)
{
  /* check the motor number which has spine system */
  if(motor_number_ > 0)
    {
      if(motor_number_ != motor_number)
        {
          motor_number_ = 0;
#ifdef SIMULATION
          ROS_ERROR("ATTENTION: motor number is 0");
#else
          nh_->logerror("ATTENTION: motor number is 0");
#endif
        }
    }
  else
    {
	  if(motor_number == 0) return;

      size_t control_term_msg_size  = motor_number;

#ifdef SIMULATION
      pwms_msg_.motor_value.resize(motor_number);
      control_term_msg_.motors.resize(control_term_msg_size);
#else
      pwms_msg_.motor_value_length = motor_number;
      control_term_msg_.motors_length = control_term_msg_size;
      pwms_msg_.motor_value = new uint16_t[motor_number];
      control_term_msg_.motors = new spinal::RollPitchYawTerm[control_term_msg_size];
#endif
      for(int i = 0; i < motor_number; i++) pwms_msg_.motor_value[i] = 0;

      /* the initialize order is important */
      motor_number_ = motor_number ;
    }
}

void  AttitudeController::setUavModel(int8_t uav_model)
{
  /* check the uav model which has spine system */
  uav_model_ = uav_model;

  if(uav_model_ == spinal::UavInfo::DRAGON) {
    rotor_devider_ = 2; // dual-rotor
  }
}

void AttitudeController::pMatrixInertiaCallback(const spinal::PMatrixPseudoInverseWithInertia& msg)
{
  if(motor_number_ == 0) return;

#ifdef SIMULATION
  if(msg.pseudo_inverse.size() != motor_number_)
    {
      if(motor_number_ > 0) ROS_ERROR("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
      return;
    }
#else
  if(msg.pseudo_inverse_length != motor_number_)
    {
      nh_->logerror("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  for(int i = 0; i < motor_number_; i ++)
    {
      p_matrix_pseudo_inverse_[i][0] = msg.pseudo_inverse[i].r * 0.001f;
      p_matrix_pseudo_inverse_[i][1] = msg.pseudo_inverse[i].p * 0.001f;
      p_matrix_pseudo_inverse_[i][2] = msg.pseudo_inverse[i].y * 0.001f;
    }

  /* inertia */
  inertia_ = ap::Matrix3f(msg.inertia[0] * 0.001f, msg.inertia[3] * 0.001f, msg.inertia[5] * 0.001f,
                          msg.inertia[3] * 0.001f, msg.inertia[1] * 0.001f, msg.inertia[4] * 0.001f,
                          msg.inertia[5] * 0.001f, msg.inertia[4] * 0.001f, msg.inertia[2] * 0.001f);

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::offsetRotCallback(const spinal::DesireCoord& msg)
{
  offset_rot_.from_euler(msg.roll, msg.pitch, msg.yaw);
}

bool AttitudeController::activated()
{
  /* uav model check and motor property */
  if(motor_number_ > 0 && uav_model_ >= spinal::UavInfo::DRONE && max_duty_ > min_duty_) return true;
  else return false;
}

void AttitudeController::pwmConversion()
{
  auto convert = [this](float target_thrust)
    {
      float scaled_thrust = v_factor_ * target_thrust / rotor_devider_;
      float target_pwm = 0;
      if (scaled_thrust < 0) scaled_thrust = 0;

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            float sqrt_tmp = motor_info_[motor_ref_index_].polynominal[1] * motor_info_[motor_ref_index_].polynominal[1] - 4 * 10 * motor_info_[motor_ref_index_].polynominal[2] * (motor_info_[motor_ref_index_].polynominal[0] - scaled_thrust); //special decimal order shift (x10)
            target_pwm = (-motor_info_[motor_ref_index_].polynominal[1] + sqrt_tmp * ap::inv_sqrt(sqrt_tmp)) / (2 * motor_info_[motor_ref_index_].polynominal[2]);
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            float tenth_scaled_thrust = scaled_thrust * 0.1f; //special decimal order shift (x0.1)
            /* 4 dimensional */
            int max_dimenstional = 4;
            target_pwm = motor_info_[motor_ref_index_].polynominal[max_dimenstional];
            for (int j = max_dimenstional - 1; j >= 0; j--)
              target_pwm = target_pwm * tenth_scaled_thrust + motor_info_[motor_ref_index_].polynominal[j];
            break;
          }
        default:
          {
            break;
          }
        }
      return target_pwm / 100; // target_pwm is [%]
    };

  /* -----------------------------------------------------------------------
   * Hardcoded aquatic motor (rotor index 2 & 3) parameters from MotorInfo_water.yaml.
   * pwm_conversion_mode = POLYNOMINAL_MODE (1), min_duty = 0.0, max_duty = 0.92
   * poly[j] order: [p0, p1, p2, p3, p4]
   * ----------------------------------------------------------------------- */
  static const float WATER_MIN_DUTY = 0.0f;
  static const float WATER_MAX_DUTY = 0.92f;
  struct WaterMotorInfo { float voltage; float max_thrust; float poly[5]; };
  static const WaterMotorInfo water_motor_table[] = {
    {25.2f, 21.1743f, {5.31928587f,  59.62689433f, -27.35529774f,   7.84268887f,  0.27220104f}},
    {24.2f, 21.3149f, {5.48946853f,  67.62445163f, -47.75818788f,  25.67117640f, -4.49312257f}},
    {23.2f, 20.4890f, {5.37462297f,  69.42786237f, -44.15328209f,  20.77747488f, -2.95746127f}},
    {22.2f, 19.6376f, {5.30078903f,  77.45026705f, -61.02618663f,  35.12434932f, -6.74821754f}},
  };
  static const int WATER_MOTOR_REF_NUM = 4;

  /* Convert aquatic thrust [N] to PWM duty using hardcoded water motor polynominals */
  auto convertWater = [this, &water_motor_table](float target_thrust)
    {
      /* pwm = F_inv[(V_ref / V)^1.5 f], POLYNOMINAL_MODE */
      float scaled_thrust = water_v_factor_ * target_thrust / rotor_devider_;
      if (scaled_thrust < 0) scaled_thrust = 0;
      float tenth_scaled_thrust = scaled_thrust * 0.1f;
      float target_pwm = water_motor_table[water_motor_ref_index_].poly[4];
      for (int j = 3; j >= 0; j--)
        target_pwm = target_pwm * tenth_scaled_thrust + water_motor_table[water_motor_ref_index_].poly[j];
      return target_pwm / 100.0f; // [%] -> [0..1]
    };

  if(pwm_test_flag_) /* motor pwm test */
    {
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
        {
          target_pwm_[i] = pwm_test_value_[i];
        }
      return;
    }

  if(motor_info_.size() == 0) return;

  /* update the factor regarding the robot voltage */
  if(HAL_GetTick() - voltage_update_last_time_ > 500) //[500ms = 0.5s]
    {
#ifdef SIMULATION
      float voltage = sim_voltage_;
#else
      float voltage = bat_->getVoltage();
#endif

      /* find the best reference voltage */
      float min_voltage_diff = 1e6;
      for(int i = 0; i < motor_info_.size(); i++)
        {
          float voltage_diff = fabs(voltage - motor_info_[i].voltage);
          if(min_voltage_diff > voltage_diff)
            {
              motor_ref_index_ = i;
              min_voltage_diff = voltage_diff;
            }
        }

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            v_factor_ = (motor_info_[motor_ref_index_].voltage / voltage) *  (motor_info_[motor_ref_index_].voltage / voltage) ;
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            v_factor_ = motor_info_[motor_ref_index_].voltage / voltage * ap::inv_sqrt(voltage / motor_info_[motor_ref_index_].voltage);
            break;
          }
        default:
          {
            break;
          }
        }

      if(min_thrust_> 0) min_duty_ = convert(min_thrust_);

      /* also update water motor voltage factor (POLYNOMINAL_MODE hardcoded) */
      {
        float min_vdiff = 1e6f;
        for (int wi = 0; wi < WATER_MOTOR_REF_NUM; wi++)
          {
            float vdiff = fabs(voltage - water_motor_table[wi].voltage);
            if (vdiff < min_vdiff) { min_vdiff = vdiff; water_motor_ref_index_ = wi; }
          }
        water_v_factor_ = water_motor_table[water_motor_ref_index_].voltage / voltage
                        * ap::inv_sqrt(voltage / water_motor_table[water_motor_ref_index_].voltage);
      }

      voltage_update_last_time_ = HAL_GetTick();
    }

  /* pwm saturation avoidance */
  /* get the decreasing rate for the thrust to avoid the divergence because of the pwm saturation */
  float base_thrust_decreasing_rate = 0;
  float yaw_decreasing_rate = 0;
  float thrust_limit = motor_info_[motor_ref_index_].max_thrust / v_factor_;

  /* check saturation level 2: z control saturation */
  float max_thrust = 0;
  int max_thrust_index = 0;
  for(int i = 0; i < motor_number_ / rotor_coef_; i++)
    {
      float thrust;
      switch(gimbal_dof_)
        {
        case 2:
          thrust = ap::pythagorous3(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1], base_thrust_term_[rotor_coef_ * i+2] + roll_pitch_term_[rotor_coef_ *i+2]);
          break;
        case 1:
          thrust = ap::pythagorous2(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1]);
          break;
        case 0:
          thrust = base_thrust_term_[i] + roll_pitch_term_[i];
          break;
        default:
          break;
        }
      if(max_thrust < thrust)
        {
          max_thrust = thrust;
          max_thrust_index = i;
        }
    }
  if(start_control_flag_)
    {
      float residual_term = thrust_limit - max_thrust / rotor_devider_;

      if(residual_term < 0 && base_thrust_term_[max_thrust_index] > 0)
        {
          base_thrust_decreasing_rate = residual_term / (base_thrust_term_[max_thrust_index] / rotor_devider_);
          yaw_decreasing_rate = -1; // also, we have to ignore the yaw control
        }
      else
        {
          if(max_yaw_term_index_ != -1 && fabs(base_thrust_term_[0]) > 0 )
            {
              /* check saturation level1: yaw control saturation */
              max_thrust = 0;
              float min_thrust = 10000;
              int min_thrust_index = 0;
              for(int i = 0; i < motor_number_ / (rotor_coef_); i++)
                {
                  float thrust;
                  switch(gimbal_dof_)
                    {
                    case 2:
                      thrust = ap::pythagorous3(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1], base_thrust_term_[rotor_coef_ * i+2] + roll_pitch_term_[rotor_coef_ *i+2]);
                      break;
                    case 1:
                      thrust = ap::pythagorous2(base_thrust_term_[rotor_coef_ * i] + roll_pitch_term_[rotor_coef_ *i],base_thrust_term_[rotor_coef_ * i+1] + roll_pitch_term_[rotor_coef_ * i+1]);
                      break;
                    case 0:
                      thrust = base_thrust_term_[i] + roll_pitch_term_[i];
                      break;
                    default:
                      break;
                    }
                  if(max_thrust < thrust)
                    {
                      max_thrust = thrust;
                      max_thrust_index = i;
                    }
                  if(min_thrust > thrust)
                    {
                      min_thrust = thrust;
                      min_thrust_index = i;
                    }
                }

              float residual_term_max =  thrust_limit - max_thrust / rotor_devider_;
              float residual_term_min =  min_thrust / rotor_devider_ - min_thrust_;
              int thrust_index = 0;
              if (residual_term_min < residual_term_max)
                {
                  residual_term = residual_term_min;
                  thrust_index = min_thrust_index;
                }
              else
                {
                  residual_term = residual_term_max;
                  thrust_index = max_thrust_index;
                }

              if(residual_term < 0)
                {
                  yaw_decreasing_rate = residual_term / (fabs(yaw_term_[thrust_index]) / rotor_devider_);
                }

              if(yaw_decreasing_rate < -1) yaw_decreasing_rate = -1;
              if(yaw_decreasing_rate > 0) yaw_decreasing_rate = 0;
            }
          else
            {
              yaw_decreasing_rate = -1;
            }
        }
    }
  
  for(int i = 0; i < motor_number_; i++)
    target_thrust_[i] = roll_pitch_term_[i] + (1 + base_thrust_decreasing_rate) * base_thrust_term_[i] + (1 + yaw_decreasing_rate) * yaw_term_[i];

  /* convert to target pwm and calculate target gimbal angles */
  /* TODO: adjust not only for gimbalrotor but also for fixed rotor */
  for(int i = 0; i < motor_number_ / (rotor_coef_); i++)
    {
      if(start_control_flag_)
        {
          switch(gimbal_dof_)
            {
            case 2:
              {
                ap::Vector3f f_i;
                f_i.x = target_thrust_[i*3];
                f_i.y = target_thrust_[i*3+1];
                f_i.z = target_thrust_[i*3+2];
            
                float gimbal_candidate_roll = atan2f(-f_i.y, f_i.z);
                float gimbal_candidate_pitch = atan2f(f_i.x, -f_i.y * sin(gimbal_candidate_roll) + f_i.z * cos(gimbal_candidate_roll));
                target_thrust_[i] = ap::pythagorous3(f_i.x,f_i.y,f_i.z);

                /* simple lpf */
                if(std::isfinite(gimbal_candidate_roll) && std::isfinite(gimbal_candidate_pitch)){
                  target_gimbal_angles_[2*i] =(target_gimbal_angles_[2*i]+ gimbal_candidate_roll)/2;
                  target_gimbal_angles_[2*i+1] =(target_gimbal_angles_[2*i+1]+ gimbal_candidate_pitch)/2;
            
                }
                break;
              }
            case 1:
              {
                ap::Vector3f f_i;
                f_i.x = target_thrust_[i*2];
                f_i.z = target_thrust_[i*2+1];
                float gimbal_candidate = atan2f(-f_i.x, f_i.z);
                target_thrust_[i] = ap::pythagorous2(f_i.x,f_i.z);

                /* simple lpf */
                if(std::isfinite(gimbal_candidate)) target_gimbal_angles_[i] =(target_gimbal_angles_[i]+ gimbal_candidate)/2;

                break;
              }
            default:
              break;
            }

          /* ---------------------------------------------------------------
           * Per-motor PWM conversion:
           *   i = 0, 1 → aerial rotors  (normal convert, clamp [min_duty_, max_duty_])
           *                if base_thrust == 0 (motor disabled) → set to IDLE_DUTY (0.5)
           *   i = 2, 3 → aquatic rotors (water convert, clamp [WATER_MIN_DUTY, WATER_MAX_DUTY])
           *                if base_thrust == 0 (motor disabled) → set to WATER_MIN_DUTY (0.0)
           * --------------------------------------------------------------- */
          bool aerial_motor_disabled =
            (i < 2) &&
            (fabs(base_thrust_term_[i * rotor_coef_]) < 1e-5f) &&
            (fabs(base_thrust_term_[i * rotor_coef_ + 1]) < 1e-5f);
          bool aquatic_motor_disabled =
            (i >= 2) &&
            (fabs(base_thrust_term_[i * rotor_coef_]) < 1e-5f) &&
            (fabs(base_thrust_term_[i * rotor_coef_ + 1]) < 1e-5f);

          if (aerial_motor_disabled)
            {
              target_pwm_[i] = IDLE_DUTY;  // keep aerial motor at idle
            }
          else if (aquatic_motor_disabled)
            {
              target_pwm_[i] = WATER_MIN_DUTY;  // aquatic motor off (0.0 duty)
            }
          else if (i >= 2)
            {
              /* aquatic motor active: use hardcoded water motor conversion */
              target_pwm_[i] = convertWater(target_thrust_[i]);
              if (target_pwm_[i] < WATER_MIN_DUTY) target_pwm_[i] = WATER_MIN_DUTY;
              else if (target_pwm_[i] > WATER_MAX_DUTY) target_pwm_[i] = WATER_MAX_DUTY;
            }
          else
            {
              /* aerial motor active: normal conversion */
              target_pwm_[i] = convert(target_thrust_[i]);
              if (target_pwm_[i] < min_duty_) target_pwm_[i] = min_duty_;
              else if (target_pwm_[i] > max_duty_) target_pwm_[i] = max_duty_;
            }
        }

      /* for ros */
      pwms_msg_.motor_value[i] = (target_pwm_[i] * 2000);
    }
  //TODO: send target gimbal angles in real machiene
#ifdef SIMULATION
  //TODO: directly send target gimbal angles to gazebo
  switch(gimbal_dof_)
    {
    case 2:
      {
        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          gimbal_control_msg.position.push_back(target_gimbal_angles_[2*i]);
          gimbal_control_msg.position.push_back(target_gimbal_angles_[2*i+1]);
        }
        gimbal_control_pub_.publish(gimbal_control_msg);    
        break;
      }
    case 1:
      {
        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          gimbal_control_msg.position.push_back(target_gimbal_angles_[i]);
        }
        gimbal_control_pub_.publish(gimbal_control_msg);
        break;
      }
    default:
      break;
    }

#else
  switch(gimbal_dof_)
    {
    case 2:
      {
        std::map<uint8_t, float> gimbal_map;
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          if(start_control_flag_)
            {
              gimbal_map[2*i] =  target_gimbal_angles_[2*i];
              gimbal_map[2*i+1] = target_gimbal_angles_[2*i+1];
            }
          else
            {
              gimbal_map[2*i] =  0;
              gimbal_map[2*i+1] = 0;
            }
        }
        if(start_control_flag_)
          servo_->setGoalAngle(gimbal_map,ValueType::RADIAN);
        else
          servo_->torqueEnable(gimbal_map);
        break;
      }
    case 1:
      {
        std::map<uint8_t, float> gimbal_map;
        for(int i = 0; i < motor_number_ / (rotor_coef_); i++){
          if(start_control_flag_)
          {
          #if GX_PWM_SERVO  
            if (target_gimbal_angles_[i] < -0.4 * M_PI)
            {
              target_gimbal_angles_[i] = -0.4 * M_PI;
            }

            if (target_gimbal_angles_[i] > 0.4 * M_PI)
            {
              target_gimbal_angles_[i] = 0.4 * M_PI;
            }
          #endif           
            gimbal_map[i] = target_gimbal_angles_[i];
          }
          else
            gimbal_map[i] = 0;
        }
        if(start_control_flag_)
          servo_->setGoalAngle(gimbal_map,ValueType::RADIAN);
        else
          servo_->torqueEnable(gimbal_map);
        break;
      }
    default:
      break;
    }
#endif
}
