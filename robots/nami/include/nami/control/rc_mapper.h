// -*- mode: c++ -*-

#pragma once

#include <mutex>
#include <string>
#include <ros/ros.h>
#include <aerial_robot_msgs/RcRaw.h>

namespace aerial_robot_control
{
/* Maps raw SBUS channels (aerial_robot_msgs/RcRaw) to semantic pilot commands.
   Pure input layer: no publishers/subscribers, thread-safe against the rc/raw
   callback via feed() + update() (call update() once per control loop). */
class RcMapper
{
public:
  struct Sticks
  {
    double roll = 0;      // [rad]
    double pitch = 0;     // [rad]
    double yaw_rate = 0;  // [rad/s]
    double throttle = 0;  // [0, 1]
  };

  bool initialize(ros::NodeHandle rc_nh);
  void feed(const aerial_robot_msgs::RcRaw& msg);
  void update(const ros::Time& now);

  bool linkValid() const
  {
    return link_valid_;
  }
  bool killLatched() const
  {
    return kill_latched_;
  }
  bool armSwitch() const
  {
    return arm_switch_;
  }
  bool armRisingEdge() const
  {
    return arm_rising_edge_;
  }
  bool takeoffEnable() const
  {
    return takeoff_enable_;
  }
  bool throttleLow() const
  {
    return sticks_.throttle <= throttle_low_threshold_;
  }
  const Sticks& sticks() const
  {
    return sticks_;
  }

private:
  struct AxisConfig
  {
    int channel = 0;  // 1-indexed
    double min = 352, center = 1024, max = 1696;
    bool reverse = false;
    double deadband = 0;   // normalized, applied around center (centered) or min (unipolar)
    double max_value = 1;  // physical scale for centered axes
  };
  struct SwitchConfig
  {
    int channel = 0;  // 1-indexed
    double threshold = 1024;
    bool reverse = false;
  };

  bool loadAxis(ros::NodeHandle& nh, const std::string& name, AxisConfig& config);
  bool loadSwitch(ros::NodeHandle& nh, const std::string& name, SwitchConfig& config);
  uint16_t rawChannel(const aerial_robot_msgs::RcRaw& frame, int channel) const;
  double normalizeCentered(const AxisConfig& config, uint16_t raw) const;  // -> [-1, 1] * max_value
  double normalizeUnipolar(const AxisConfig& config, uint16_t raw) const;  // -> [0, 1]
  bool switchHigh(const SwitchConfig& config, uint16_t raw) const;

  AxisConfig yaw_rate_axis_, pitch_axis_, throttle_axis_, roll_axis_;
  SwitchConfig arm_switch_config_, kill_switch_config_, takeoff_enable_config_;

  double timeout_ = 0.2;                 // [s], on rc/raw header stamp
  double throttle_low_threshold_ = 0.05;

  std::mutex mutex_;
  aerial_robot_msgs::RcRaw latest_frame_;
  bool frame_received_ = false;

  Sticks sticks_;
  bool link_valid_ = false;
  bool kill_latched_ = false;
  bool arm_switch_ = false;
  bool arm_rising_edge_ = false;
  bool arm_seen_low_ = false;  // arming needs a low->high toggle observed after startup
  bool takeoff_enable_ = false;
};
}  // namespace aerial_robot_control
