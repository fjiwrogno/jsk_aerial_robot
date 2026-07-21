// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/FluidPressure.h>
#include <spinal/DesireCoord.h>
#include <spinal/FlightConfigCmd.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace aerial_robot_navigation
{
class NamiNavigator : public BaseNavigator
{
public:
  NamiNavigator();
  ~NamiNavigator()
  {
  }

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du) override;

  void update() override;

  /* underwater mode interface (read by NamiController) */
  inline bool getUnderwaterMode() const { return underwater_mode_; }
  inline double getTargetDepth() const { return target_depth_; }
  inline double getCurrentDepth() const { return current_depth_; }
  bool getDepthSensorReady() const;
  inline bool getSurgeAllocation() const { return surge_allocation_; }

private:
  ros::Publisher target_baselink_rpy_pub_;
  ros::Subscriber final_target_baselink_rot_sub_, final_target_baselink_rpy_sub_;

  /* underwater mode */
  ros::Subscriber underwater_cmd_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber surge_allocation_sub_;
  ros::Subscriber pressure_sub_;  // sim: uuv SubseaPressure (sensor_msgs/FluidPressure)
  ros::Subscriber depth_sub_;     // real machine: ms5837 node (geometry_msgs/PointStamped)
  ros::Publisher target_depth_pub_;  // debug: current target depth [m, signed z]

  void baselinkRotationProcess();
  void rosParamInit() override;
  void targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg);
  void targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void reset() override;

  /* underwater mode */
  void underwaterStateProcess();
  void underwaterCmdCallback(const geometry_msgs::TwistConstPtr& msg);
  void joyCallback(const sensor_msgs::JoyConstPtr& msg);
  void surgeAllocationCallback(const std_msgs::BoolConstPtr& msg);
  void pressureCallback(const sensor_msgs::FluidPressureConstPtr& msg);
  void depthCallback(const geometry_msgs::PointStampedConstPtr& msg);

  /* target baselink rotation */
  double prev_rotation_stamp_;
  tf::Quaternion curr_target_baselink_rot_, final_target_baselink_rot_;
  bool eq_cog_world_;

  /* underwater mode states */
  bool underwater_mode_;
  double target_depth_;   // [m], signed z (<= 0 underwater)
  double current_depth_;  // [m], signed z
  bool depth_sensor_ready_;
  double depth_sensor_stamp_;
  bool underwater_flight_setup_done_;
  double underwater_cmd_stamp_;
  double takeoff_enter_stamp_ = -1;  // explicit TAKEOFF->HOVER window (see underwaterStateProcess)

  /* rosparam */
  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;

  /* rosparam: underwater */
  double teleop_acc_gain_;       // [m/s^2 per unit stick]
  double teleop_yaw_rate_gain_;  // [rad/s per unit stick]
  double underwater_cmd_timeout_;  // [s], clear open-loop xy command after input loss
  double depth_timeout_;           // [s], stop using stale I2C depth feedback
  double joy_deadzone_;
  bool surge_allocation_;
  bool surge_toggle_pressed_ = false;
  double max_dive_rate_;         // [m/s]
  double max_depth_;             // [m], positive value
  double standard_pressure_kpa_;
  double kpa_per_meter_;
};
};  // namespace aerial_robot_navigation
