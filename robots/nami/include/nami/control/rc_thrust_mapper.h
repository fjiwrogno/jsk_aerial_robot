// -*- mode: c++ -*-

#pragma once

namespace aerial_robot_control
{
class RcThrustMapper
{
public:
  struct Config
  {
    double min_force = 0.0;
    double max_force = 0.0;
    double hover_throttle = 0.5;
    double hover_force = 0.0;  // <= 0 uses vehicle weight
    double hover_acceleration = 9.797;
    double down_authority = 0.0;
    double up_authority = 0.0;
    double expo = 0.5;
    double hover_deadband = 0.01;
    bool tilt_compensation = true;
    double max_tilt_compensation = 1.3;
  };

  Config& config()
  {
    return config_;
  }

  const Config& config() const
  {
    return config_;
  }

  void sanitize();
  double hoverForce(double mass) const;
  double throttleToForce(double throttle, double mass) const;
  double tiltCompensation(double roll, double pitch) const;
  double compensateTilt(double force, double roll, double pitch) const;

private:
  Config config_;
};
}  // namespace aerial_robot_control
