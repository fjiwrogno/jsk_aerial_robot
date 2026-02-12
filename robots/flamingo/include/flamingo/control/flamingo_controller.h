// -*- mode: c++ -*-

#pragma once

#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Joy.h>
#include <aerial_robot_control/util/joy_parser.h>
#include <flamingo/model/flamingo_robot_model.h>

namespace aerial_robot_control
{
class FlamingoController : public PoseLinearController
{
public:
  FlamingoController();
  ~FlamingoController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

private:
  ros::Publisher flight_cmd_pub_;
  ros::Publisher gimbal_control_pub_;
  ros::Publisher gimbal_state_pub_;
  ros::Publisher target_vectoring_force_pub_;
  ros::Publisher rpy_gain_pub_;                      // for spinal
  ros::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros::Publisher gimbal_dof_pub_;                    // for spinal

  boost::shared_ptr<FlamingoRobotModel> flamingo_robot_model_;
  std::vector<float> target_base_thrust_;
  std::vector<float> target_full_thrust_;
  std::vector<double> target_gimbal_angles_;
  bool hovering_approximate_;
  Eigen::VectorXd target_vectoring_f_;
  Eigen::VectorXd target_vectoring_f_trans_;
  Eigen::VectorXd target_vectoring_f_rot_;
  Eigen::MatrixXd integrated_map_inv_trans_;
  Eigen::MatrixXd integrated_map_inv_rot_;
  double candidate_yaw_term_;
  int gimbal_dof_;
  int rotor_coef_;
  bool gimbal_calc_in_fc_;
  bool underactuate_;
  double target_roll_ = 0.0, target_pitch_ = 0.0;

  // Direct joystick control mode — Mode 2 (American hand), Betaflight Angle mode equivalent
  //   Left stick:  vertical = throttle, horizontal = yaw rate
  //   Right stick: vertical = pitch,    horizontal = roll
  bool direct_joystick_mode_;
  bool yaw_control_flag_ = false;
  double direct_thrust_bias_;       // hover thrust per rotor (N)
  double direct_thrust_scale_;      // throttle stick → thrust offset scale (N)
  double direct_pitch_max_;         // max pitch angle from joystick (rad)
  double direct_roll_max_;          // max roll angle from joystick (rad)
  double direct_yaw_rate_max_;      // max yaw rate from joystick (rad/s)
  ros::Subscriber joy_sub_;
  double joy_throttle_cmd_;         // normalized [-1, 1] from left stick vertical
  double joy_pitch_cmd_;            // normalized [-1, 1] from right stick vertical
  double joy_roll_cmd_;             // normalized [-1, 1] from right stick horizontal
  double joy_yaw_val_cmd_;              // normalized [-1, 1] from left stick horizontal
  double joy_yaw_rate_;
  double direct_yaw_target_;        // integrated yaw heading target (rad)
  void joyCallback(const sensor_msgs::JoyConstPtr& msg);
  void directJoystickControl();

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
};
};  // namespace aerial_robot_control
