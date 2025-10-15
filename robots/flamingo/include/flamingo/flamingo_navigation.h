// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <spinal/DesireCoord.h>
#include "aerial_robot_control/util/joy_parser.h"
#include <flamingo/model/flamingo_robot_model.h>

namespace aerial_robot_navigation
{
enum ROBOT_MODE
{
  AERIAL_MODE,
  UNDERWATER_MODE,
};
class FlamingoNavigator : public BaseNavigator
{
public:
  FlamingoNavigator();
  ~FlamingoNavigator()
  {
  }

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du) override;

  void update() override;
  void joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg);

private:
  ros::Publisher target_baselink_rpy_pub_;
  ros::Subscriber final_target_baselink_rot_sub_, final_target_baselink_rpy_sub_;
  ros::Subscriber robot_mode_sub_;
  ros::Publisher robot_mode_pub_;

  void switchRobotMode();
  void updateRobotMode();
  void baselinkRotationProcess();
  void rosParamInit() override;
  void targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg);
  void targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;
  void robotModeCallback(const std_msgs::UInt8ConstPtr& msg);

  void reset() override;

  boost::shared_ptr<FlamingoRobotModel> flamingo_robot_model_;

  /* target baselink rotation */
  double prev_rotation_stamp_;
  tf::Quaternion curr_target_baselink_rot_, final_target_baselink_rot_;
  bool eq_cog_world_;

  /* rosparam */
  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;
  /* robot mode */
  uint8_t robot_mode_;
};
};  // namespace aerial_robot_navigation
