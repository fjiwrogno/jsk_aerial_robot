#include "gx_pwm_servo_driver.h"

const std::array<uint32_t, MAX_SERVO_NUM> channels = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

bool GxPwmServo::init(TIM_HandleTypeDef* htim, uint8_t servo_num) {
    if (servo_num > MAX_SERVO_NUM) return false;
    // pwm config
    htim_ = htim;
    htim_->Instance = TIM4;
    // Configure for 300Hz PWM, based on a 200MHz timer kernel clock.
    // Timer clock = 200MHz. Prescaler = 19 => Tick Freq = 200M / 20 = 10MHz.
    // Tick time = 0.1us. Period = 33332 => PWM Period = (33332+1) * 0.1us =
    // 3333.3us => ~300Hz.
    htim_->Init.Prescaler = 19;
    htim_->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_->Init.Period = 33332;
    htim_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_DeInit(htim_);
    HAL_TIM_Base_Init(htim_);
    HAL_TIM_PWM_Init(htim_);

    for (uint8_t i = 0; i < servo_num; ++i) {
        FlashMemory::addValue(&init_positions_[i], sizeof(float));
    }
    FlashMemory::read();

    for (uint8_t i = 0; i < servo_num; ++i) {
        servo_[i] = ServoData(i + 1);
        servo_[i].setGoalPositionRad(init_positions_[i]);
        servo_[i].torque_enable_ = true;
        HAL_TIM_PWM_Start(htim_, channels[i]);
    }
    servo_num_ = servo_num;
    initialized_ = true;

    update();

    return true;
}

void GxPwmServo::update() {
    if (!initialized_ || !htim_) return;
    for (uint8_t i = 0; i < servo_num_; ++i) {
        float pulse_us = servo_[i].torque_enable_ ? angleToPulseWidth(servo_[i].getGoalPositionRad()) : 0.0f;
        setCCR(channels[i], pulse_us);
    }
}

void GxPwmServo::setTorque(uint8_t servo_index) {
    if (!initialized_ || servo_index >= servo_num_) return;

    if (servo_[servo_index].torque_enable_ != true)
    {
        servo_[servo_index].torque_enable_ = !servo_[servo_index].torque_enable_;
    }
}

void GxPwmServo::setTorqueFromPresetnPos(uint8_t servo_index) {
    if (!initialized_ || servo_index >= servo_num_) return;
    if(servo_[servo_index].torque_enable_) servo_[servo_index].goal_position_ = servo_[servo_index].getPresentPosition();
    servo_[servo_index].torque_enable_ = true;
}

void GxPwmServo::setAndSaveInitPosition(uint8_t id, float angle_rad) {
    if (!initialized_ || id < 1 || id > servo_num_) return;
    init_positions_[id - 1] = angle_rad;
    FlashMemory::erase();
    FlashMemory::write();
}

float GxPwmServo::angleToPulseWidth(float angle_rad) const {
    float pulse_width_us = MIN_PULSE_WIDTH_US + (angle_rad / ANGLE_RANGE_RAD) * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);
    
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

void GxPwmServo::setCCR(uint32_t channel, float pulse_us) {
    if (htim_ == nullptr) return;
    uint32_t ccr_value = static_cast<uint32_t>(pulse_us / 0.1f);
    __HAL_TIM_SET_COMPARE(htim_, channel, ccr_value);
}
