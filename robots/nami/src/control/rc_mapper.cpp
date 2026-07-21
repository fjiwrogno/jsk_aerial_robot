#include <nami/control/rc_mapper.h>

namespace aerial_robot_control
{
bool RcMapper::initialize(ros::NodeHandle rc_nh)
{
  rc_nh.param("timeout", timeout_, 0.2);

  ros::NodeHandle channels_nh(rc_nh, "channels");
  bool valid = true;
  valid &= loadAxis(channels_nh, "yaw_rate", yaw_rate_axis_);
  valid &= loadAxis(channels_nh, "pitch", pitch_axis_);
  valid &= loadAxis(channels_nh, "throttle", throttle_axis_);
  valid &= loadAxis(channels_nh, "roll", roll_axis_);
  valid &= loadSwitch(channels_nh, "arm", arm_switch_config_);
  valid &= loadSwitch(channels_nh, "kill", kill_switch_config_);
  valid &= loadSwitch(channels_nh, "takeoff_enable", takeoff_enable_config_);

  ros::NodeHandle safety_nh(rc_nh, "safety");
  safety_nh.param("throttle_low_threshold", throttle_low_threshold_, 0.05);

  if (timeout_ <= 0.0 || throttle_low_threshold_ < 0.0 || throttle_low_threshold_ > 1.0)
  {
    ROS_ERROR("RcMapper: timeout must be positive and throttle_low_threshold must be in [0, 1]");
    valid = false;
  }

  return valid;
}

bool RcMapper::loadAxis(ros::NodeHandle& nh, const std::string& name, AxisConfig& config)
{
  ros::NodeHandle axis_nh(nh, name);
  axis_nh.param("channel", config.channel, config.channel);
  axis_nh.param("min", config.min, config.min);
  axis_nh.param("center", config.center, config.center);
  axis_nh.param("max", config.max, config.max);
  axis_nh.param("reverse", config.reverse, config.reverse);
  if (!axis_nh.getParam("deadband", config.deadband))
    axis_nh.param("low_deadband", config.deadband, config.deadband);
  axis_nh.param("max_value", config.max_value, config.max_value);

  const bool valid = config.channel >= 1 && config.channel <= 16 && config.min < config.center &&
                     config.center < config.max && config.deadband >= 0.0 && config.deadband < 1.0 &&
                     config.max_value > 0.0;
  if (!valid)
    ROS_ERROR_STREAM("RcMapper: invalid axis config for " << name << " (channel=" << config.channel
                                                           << ", min=" << config.min << ", center=" << config.center
                                                           << ", max=" << config.max << ", deadband="
                                                           << config.deadband << ")");
  return valid;
}

bool RcMapper::loadSwitch(ros::NodeHandle& nh, const std::string& name, SwitchConfig& config)
{
  ros::NodeHandle switch_nh(nh, name);
  switch_nh.param("channel", config.channel, config.channel);
  switch_nh.param("threshold", config.threshold, config.threshold);
  switch_nh.param("reverse", config.reverse, config.reverse);

  const bool valid = config.channel >= 1 && config.channel <= 16;
  if (!valid)
    ROS_ERROR_STREAM("RcMapper: invalid switch config for " << name << " (channel: " << config.channel << ")");
  return valid;
}

void RcMapper::feed(const aerial_robot_msgs::RcRaw& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_frame_ = msg;
  frame_received_ = true;
}

void RcMapper::update(const ros::Time& now)
{
  aerial_robot_msgs::RcRaw frame;
  bool received;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame = latest_frame_;
    received = frame_received_;
  }

  arm_rising_edge_ = false;

  link_valid_ =
      received && !frame.failsafe && !frame.frame_lost && (now - frame.header.stamp).toSec() <= timeout_;
  if (!link_valid_)
  {
    /* Section 12 of the mapping spec: link loss has the highest priority and
       must latch kill, disarm, and discard every pilot command.  Do not mark
       arm_seen_low_: only a valid low arm frame may enable a later arm edge. */
    kill_latched_ = true;
    arm_switch_ = false;
    takeoff_enable_ = false;
    sticks_ = Sticks();
    return;
  }

  sticks_.yaw_rate = normalizeCentered(yaw_rate_axis_, rawChannel(frame, yaw_rate_axis_.channel));
  sticks_.pitch = normalizeCentered(pitch_axis_, rawChannel(frame, pitch_axis_.channel));
  sticks_.roll = normalizeCentered(roll_axis_, rawChannel(frame, roll_axis_.channel));
  sticks_.throttle = normalizeUnipolar(throttle_axis_, rawChannel(frame, throttle_axis_.channel));

  bool arm_prev = arm_switch_;
  arm_switch_ = switchHigh(arm_switch_config_, rawChannel(frame, arm_switch_config_.channel));
  if (!arm_switch_)
    arm_seen_low_ = true;
  /* refuse the edge until the switch has been seen low once, so a transmitter
     left in the arm position cannot arm the robot right after startup */
  arm_rising_edge_ = arm_switch_ && !arm_prev && arm_seen_low_;

  takeoff_enable_ = switchHigh(takeoff_enable_config_, rawChannel(frame, takeoff_enable_config_.channel));

  bool kill_high = switchHigh(kill_switch_config_, rawChannel(frame, kill_switch_config_.channel));
  if (kill_high)
    kill_latched_ = true;
  else if (kill_latched_ && !arm_switch_ && throttleLow())
  {
    kill_latched_ = false;  // spec: clear only with kill released + disarm + throttle low
    ROS_WARN("RcMapper: kill latch cleared");
  }
}

uint16_t RcMapper::rawChannel(const aerial_robot_msgs::RcRaw& frame, int channel) const
{
  if (channel < 1 || channel > static_cast<int>(frame.channels.size()))
    return 0;
  return frame.channels.at(channel - 1);
}

double RcMapper::normalizeCentered(const AxisConfig& config, uint16_t raw) const
{
  double value;
  if (raw >= config.center)
    value = (raw - config.center) / std::max(config.max - config.center, 1.0);
  else
    value = (raw - config.center) / std::max(config.center - config.min, 1.0);
  value = std::max(-1.0, std::min(1.0, value));

  if (std::fabs(value) <= config.deadband)
    return 0;
  /* rescale so output is continuous at the deadband edge */
  value = (value - (value > 0 ? config.deadband : -config.deadband)) / (1.0 - config.deadband);

  if (config.reverse)
    value = -value;
  return value * config.max_value;
}

double RcMapper::normalizeUnipolar(const AxisConfig& config, uint16_t raw) const
{
  double value = (raw - config.min) / std::max(config.max - config.min, 1.0);
  value = std::max(0.0, std::min(1.0, value));
  if (config.reverse)
    value = 1.0 - value;

  if (value <= config.deadband)
    return 0;
  return (value - config.deadband) / (1.0 - config.deadband);
}

bool RcMapper::switchHigh(const SwitchConfig& config, uint16_t raw) const
{
  bool high = raw > config.threshold;
  return config.reverse ? !high : high;
}
}  // namespace aerial_robot_control
