#include "drivers/GX3240MGV1/gx_pwm_servo_driver.h"

constexpr float MIN_PULSE_WIDTH_US = 500.0f;
constexpr float MAX_PULSE_WIDTH_US = 2500.0f;
constexpr float ANGLE_RANGE_RAD =
    357.0f * M_PI / 180.0f;                  // 357 degrees total range
constexpr float DEFAULT_PULSE_US = 1500.0f;  // Neutral position
constexpr int MAX_SERVO_NUM = 2;

GxPwmServo::GxPwmServo(uint8_t id, TIM_HandleTypeDef* htim,
                       uint32_t tim_channel)
    : id_(id),
      htim_(htim),
      tim_channel_(tim_channel),
      max_pwm_count_(0),
      torque_enabled_(false),
      last_pulse_us_(DEFAULT_PULSE_US),
      goal_angle_rad_(0.0f) {}

void GxPwmServo::init() {
    if (htim_ == nullptr) return;
    max_pwm_count_ = __HAL_TIM_GET_AUTORELOAD(htim_);
    HAL_TIM_PWM_Start(htim_, tim_channel_);
    torqueEnable(false);  // Start with torque off for safety
}

void GxPwmServo::setGoalAngle(float angle_rad) {
    if (!torque_enabled_ || isnan(angle_rad)) return;

    goal_angle_rad_ = angle_rad;
    float pulse_us = angleToPulseWidth(angle_rad);
    last_pulse_us_ = pulse_us;
    setCCR(pulse_us);
}

void GxPwmServo::torqueEnable(bool enable) {
    if (enable) {
        torque_enabled_ = true;
        setCCR(last_pulse_us_);
    } else {
        torque_enabled_ = false;
        setCCR(0);
    }
}

float GxPwmServo::angleToPulseWidth(float angle_rad) {
    float pulse_width_us =
        MIN_PULSE_WIDTH_US + (angle_rad / ANGLE_RANGE_RAD) *
                                 (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);

    if (pulse_width_us < MIN_PULSE_WIDTH_US) {
        pulse_width_us = MIN_PULSE_WIDTH_US;
    }

    if (pulse_width_us > MAX_PULSE_WIDTH_US) {
        pulse_width_us = MAX_PULSE_WIDTH_US;
    }

    return pulse_width_us;
}

void GxPwmServo::setCCR(float pulse_us) {
    if (htim_ == nullptr) return;
    float pwm_period_us = (float)(max_pwm_count_ + 1) *
                          (float)(htim_->Init.Prescaler + 1) /
                          (float)(HAL_RCC_GetPCLK1Freq() / 1000000.0f);
    if (pwm_period_us < 1.0f) {
        pwm_period_us = 3030.0f;  // Safety default for 330Hz
    }
    uint32_t ccr_value = static_cast<uint32_t>((pulse_us / pwm_period_us) * max_pwm_count_);
    __HAL_TIM_SET_COMPARE(htim_, tim_channel_, ccr_value);
}


bool GxPwmServoHandler::init(uint8_t servo_num) {
    if (servo_num > 4) return false;

    //Timer4 configuration 
    htim_.Instance = TIM4;
    htim_.Init.Prescaler = 36;
    htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_.Init.Period = 3075;
    htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_DeInit(&htim_);
    HAL_TIM_Base_Init(&htim_);
    HAL_TIM_PWM_Init(&htim_);

    const std::array<uint32_t, 4> channels = {TIM_CHANNEL_1, TIM_CHANNEL_2,
                                              TIM_CHANNEL_3, TIM_CHANNEL_4};
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    for (uint8_t i = 0; i < servo_num; ++i) {
        HAL_TIM_PWM_ConfigChannel(&htim_, &sConfigOC, channels[i]);
    }

    for (uint8_t i = 0; i < servo_num; ++i) {
        HAL_TIM_PWM_Start(&htim_, channels[i]);
    }

    for (uint8_t i = 0; i < servo_num; ++i) {
        servos_[i].emplace(i + 1, &htim_, channels[i]);
        servos_[i]->init();
    }
    servo_count_ = servo_num;
    initialized_ = true;
    return true;
}

void GxPwmServoHandler::setGoalAngle(uint8_t id, float angle_rad) {
    if (!initialized_ || id < 1 || id > servo_count_) return;
    (*servos_[id - 1]).setGoalAngle(angle_rad);
}

void GxPwmServoHandler::torqueEnable(uint8_t id, bool enable) {
    if (!initialized_ || id < 1 || id > servo_count_) return;
    (*servos_[id - 1]).torqueEnable(enable);
}

// Implementation updated for pointer and size
void GxPwmServoHandler::setSyncGoalAngle(const uint8_t* ids,
                                         const float* angles_rad, size_t size) {
    if (!initialized_ || ids == nullptr || angles_rad == nullptr) return;

    for (size_t i = 0; i < size; ++i) {
        setGoalAngle(ids[i], angles_rad[i]);
    }
}

