#ifndef __GX_PWM_SERVO_DRIVER_H
#define __GX_PWM_SERVO_DRIVER_H

#include <array>
#include <cmath>
#include <cstdint>

#include "config.h"
#include "flashmemory/flashmemory.h"
#include "servo/drivers/servo_base_driver.h"
#include "stm32h7xx_hal.h"  // Add this include for HAL types
#include "main.h"
constexpr float MIN_PULSE_WIDTH_US = 500.0f;
constexpr float MAX_PULSE_WIDTH_US = 2500.0f;
constexpr float ANGLE_RANGE_RAD = M_PI;

class GxPwmServo : public ServoBase {
   public:
    GxPwmServo() = default;
    ~GxPwmServo() = default;

    bool init(TIM_HandleTypeDef* htim, uint8_t servo_num);
    void init(UART_HandleTypeDef* huart, osMutexId* mutex = NULL) override {}
    void update() override;
    void setTorque(uint8_t servo_index) override;
    void setTorqueFromPresetnPos(uint8_t servo_index) override;
    void setAndSaveInitPosition(uint8_t id, float angle_rad);
    float angleToPulseWidth(float angle_rad) const;
    void setCCR(uint32_t channel, float pulse_us);

    // Implement all pure virtual functions from ServoBase
    void pinReconfig() override {}
    void ping() override {}
    HAL_StatusTypeDef read(uint8_t* data, uint32_t timeout) override {
        return HAL_OK;
    }
    void reboot(uint8_t servo_index) override {}
    void setHomingOffset(uint8_t servo_index) override {}
    void setRoundOffset(uint8_t servo_index, int32_t ref_value) override {}
    void setPositionGains(uint8_t servo_index) override {}
    void setProfileVelocity(uint8_t servo_index) override {}
    void setCurrentLimit(uint8_t servo_index) override {}

   private:
    // Implement protected pure virtual functions from ServoBase
    void setStatusReturnLevel() override {}
    void getHomingOffset() override {}
    void getCurrentLimit() override {}
    void getPositionGains() override {}
    void getProfileVelocity() override {}

    TIM_HandleTypeDef* htim_ = nullptr;
    std::array<float, MAX_SERVO_NUM> init_positions_;
    float tick_time_us_,timer_kernel_clock_;
    bool initialized_ = false;
};

#endif