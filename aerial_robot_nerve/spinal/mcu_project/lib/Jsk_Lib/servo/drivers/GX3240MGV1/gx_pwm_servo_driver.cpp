#include "gx_pwm_servo_driver.h"

const std::array<uint32_t, MAX_SERVO_NUM> channels = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };

bool GxPwmServo::init(TIM_HandleTypeDef* htim, uint8_t servo_num)
{
  if (servo_num > MAX_SERVO_NUM)
    return false;
  // pwm config
  htim_ = htim;
  htim_->Instance = TIM4;

  HAL_TIM_PWM_Stop(htim_, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop(htim_);
  HAL_TIM_Base_DeInit(htim_);

  htim_->Init.Prescaler = 199;
  htim_->Init.CounterMode = TIM_COUNTERMODE_UP;  // Use standard UP mode
  htim_->Init.Period = 3029;

  while (HAL_TIM_Base_Init(htim_) != HAL_OK);
  while (HAL_TIM_PWM_Init(htim_) != HAL_OK);

  TIM_OC_InitTypeDef sConfigOC = { 0 };
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // --- Configure Channels 3 & 4 for bidirectional neutral throttle
  // (1.5ms) ---
  sConfigOC.Pulse = 1500;
  while (HAL_TIM_PWM_ConfigChannel(htim_, &sConfigOC, TIM_CHANNEL_3) != HAL_OK);
  while (HAL_TIM_PWM_ConfigChannel(htim_, &sConfigOC, TIM_CHANNEL_4) != HAL_OK);

  HAL_TIM_Base_Start(htim_);

  for (uint8_t i = 0; i < servo_num; ++i)
  {
    init_positions_[i] = M_PI;
    FlashMemory::addValue(&init_positions_[i], sizeof(float));
  }
  FlashMemory::read();

  for (uint8_t i = 0; i < servo_num; ++i)
  {
    servo_[i] = ServoData(i + 1);
    servo_[i].setGoalPositionRad(init_positions_[i]);
    servo_[i].torque_enable_ = true;
  }

  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_4);

  uint32_t pclk1_freq = HAL_RCC_GetPCLK1Freq();

  if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) == 0)
  {
    timer_kernel_clock_ = pclk1_freq;
  }
  else
  {
    timer_kernel_clock_ = 2 * pclk1_freq;
  }

  tick_time_us_ = static_cast<float>(htim_->Init.Prescaler + 1) / (static_cast<float>(timer_kernel_clock_) / 1000000.0f);

  servo_num_ = servo_num;
  initialized_ = true;

  update();

  return true;
}

void GxPwmServo::update()
{
  if (!initialized_ || !htim_)
    return;
  for (uint8_t i = 0; i < servo_num_; ++i)
  {
    float pulse_us = servo_[i].torque_enable_ ? angleToPulseWidth(servo_[i].getGoalPositionRad()) : 0.0f;
    setCCR(channels[i], pulse_us);
  }
}

void GxPwmServo::setTorque(uint8_t servo_index)
{
  if (!initialized_ || servo_index >= servo_num_)
    return;

  if (servo_[servo_index].torque_enable_ != true)
  {
    servo_[servo_index].torque_enable_ = !servo_[servo_index].torque_enable_;
  }
}

void GxPwmServo::setTorqueFromPresetnPos(uint8_t servo_index)
{
  if (!initialized_ || servo_index >= servo_num_)
    return;
  if (servo_[servo_index].torque_enable_)
    servo_[servo_index].goal_position_ = servo_[servo_index].getPresentPosition();
  servo_[servo_index].torque_enable_ = true;
}

void GxPwmServo::setAndSaveInitPosition(uint8_t id, float angle_rad)
{
  if (!initialized_ || id < 1 || id > servo_num_)
    return;
  init_positions_[id - 1] = angle_rad;
  FlashMemory::erase();
  FlashMemory::write();
}

float GxPwmServo::angleToPulseWidth(float angle_rad) const
{
  float pulse_width_us = MIN_PULSE_WIDTH_US + (angle_rad / ANGLE_RANGE_RAD) * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);
///TODO restrict the servo angle to [93,(178),263]
  if (pulse_width_us < MIN_PULSE_WIDTH_US)
  {
    pulse_width_us = MIN_PULSE_WIDTH_US;
  }
  if (pulse_width_us > MAX_PULSE_WIDTH_US)
  {
    pulse_width_us = MAX_PULSE_WIDTH_US;
  }

  return pulse_width_us;
}

void GxPwmServo::setCCR(uint32_t channel, float pulse_us)
{
  if (htim_ == nullptr)
    return;
 
  uint32_t ccr_value = static_cast<uint32_t>(pulse_us / tick_time_us_);
  __HAL_TIM_SET_COMPARE(htim_, channel, ccr_value);
}
