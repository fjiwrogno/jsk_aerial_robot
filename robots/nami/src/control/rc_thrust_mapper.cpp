#include <nami/control/rc_thrust_mapper.h>

#include <algorithm>
#include <cmath>

namespace aerial_robot_control
{
void RcThrustMapper::sanitize()
{
  config_.max_force = std::max(config_.max_force, config_.min_force + 1.0);
  config_.hover_throttle = std::clamp(config_.hover_throttle, 0.05, 0.95);
  config_.expo = std::clamp(config_.expo, 0.0, 1.0);
  config_.hover_deadband = std::clamp(config_.hover_deadband, 0.0, 0.5);
  if (config_.hover_acceleration <= 0.0)
    config_.hover_acceleration = 9.797;
  config_.down_authority = std::max(config_.down_authority, 0.0);
  config_.up_authority = std::max(config_.up_authority, 0.0);
  config_.max_tilt_compensation = std::max(config_.max_tilt_compensation, 1.0);
}

double RcThrustMapper::hoverForce(double mass) const
{
  const double nominal_hover_force =
      config_.hover_force > 0.0 ? config_.hover_force : std::max(mass, 0.0) * config_.hover_acceleration;
  return std::clamp(nominal_hover_force, config_.min_force, config_.max_force);
}

double RcThrustMapper::throttleToForce(double throttle, double mass) const
{
  const double hover_force = hoverForce(mass);
  const double lower_range = std::max(config_.hover_throttle, 1e-3);
  const double upper_range = std::max(1.0 - config_.hover_throttle, 1e-3);
  double centered = throttle >= config_.hover_throttle ?
                        (throttle - config_.hover_throttle) / upper_range :
                        (throttle - config_.hover_throttle) / lower_range;
  centered = std::clamp(centered, -1.0, 1.0);

  const double magnitude = std::fabs(centered);
  if (magnitude <= config_.hover_deadband)
  {
    centered = 0.0;
  }
  else
  {
    const double rescaled =
        (magnitude - config_.hover_deadband) / std::max(1.0 - config_.hover_deadband, 1e-6);
    centered = std::copysign(rescaled, centered);
  }

  // ArduPilot-style cubic expo: retain a finite slope at hover for small corrections.
  const double shaped = centered * ((1.0 - config_.expo) + config_.expo * centered * centered);
  const double down_authority =
      config_.down_authority > 0.0 ? config_.down_authority : std::max(hover_force - config_.min_force, 0.0);
  const double up_authority =
      config_.up_authority > 0.0 ? config_.up_authority : std::max(config_.max_force - hover_force, 0.0);
  const double force = hover_force + (shaped >= 0.0 ? shaped * up_authority : shaped * down_authority);

  return std::clamp(force, config_.min_force, config_.max_force);
}

double RcThrustMapper::tiltCompensation(double roll, double pitch) const
{
  if (!config_.tilt_compensation)
    return 1.0;

  const double cos_tilt = std::max(std::cos(roll) * std::cos(pitch), 1e-3);
  return std::clamp(1.0 / cos_tilt, 1.0, config_.max_tilt_compensation);
}

double RcThrustMapper::compensateTilt(double force, double roll, double pitch) const
{
  return std::clamp(force * tiltCompensation(roll, pitch), config_.min_force, config_.max_force);
}
}  // namespace aerial_robot_control
