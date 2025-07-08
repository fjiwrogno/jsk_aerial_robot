#ifndef __GX_PWM_SERVO_DRIVER_H
#define __GX_PWM_SERVO_DRIVER_H

#include <array>
#include <cmath>

#include "config.h"

class GxPwmServo {
   public:
    // Constructor now takes a reference, guaranteeing it's not null.
    GxPwmServo(uint8_t id, TIM_HandleTypeDef& htim, uint32_t tim_channel);
    ~GxPwmServo() = default;

    GxPwmServo(const GxPwmServo&) = delete;
    GxPwmServo& operator=(const GxPwmServo&) = delete;
    GxPwmServo(GxPwmServo&&) = delete;
    GxPwmServo& operator=(GxPwmServo&&) = delete;

    void init();
    void setGoalAngle(float angle_rad);
    void torqueEnable(bool enable);

    inline uint8_t getId() const { return id_; }
    inline bool isTorqueEnabled() const { return torque_enabled_; }
    inline float getGoalAngleRad() const { return goal_angle_rad_; }

   private:
    uint8_t id_;
    TIM_HandleTypeDef& htim_;
    uint32_t tim_channel_;
    uint32_t max_pwm_count_;
    bool torque_enabled_;
    float last_pulse_us_;
    float goal_angle_rad_;

    float angleToPulseWidth(float angle_rad);
    void setCCR(float pulse_us);
};

class GxPwmServoHandler {
   public:
    GxPwmServoHandler() = default;
    ~GxPwmServoHandler() = default;

    bool init(uint8_t servo_num);

    void setGoalAngle(uint8_t id, float angle_rad);
    void torqueEnable(uint8_t id, bool enable);

    void setSyncGoalAngle(const uint8_t* ids, const float* angles_rad, size_t size);

    inline void update() {}  // Does nothing for PWM servos without feedback
    inline size_t getServoNum() const { return servo_count_; }
    const std::array<GxPwmServo, 4>& getServos() const { return servos_; }
    inline void setGoalPosition(uint8_t id, float angle_rad) { setGoalAngle(id, angle_rad); }
    inline void setTorque(uint8_t id, bool enable) { torqueEnable(id, enable); }

   private:
    std::array<GxPwmServo, 4> servos_;
    uint8_t servo_count_ = 0;
    bool initialized_ = false;
};

#endif  // __GX_PWM_SERVO_DRIVER_H
