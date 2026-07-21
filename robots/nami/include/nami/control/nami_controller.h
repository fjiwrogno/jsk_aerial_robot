// -*- mode: c++ -*-

#pragma once

#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <nami/model/nami_robot_model.h>
#include <nami/nami_navigation.h>

namespace aerial_robot_control
{
class NamiController : public PoseLinearController
{
public:
  NamiController();
  ~NamiController() = default;

  /* flight mode: motor 0~3 (aerial rotors) vs motor 4~7 (bi-directional aquatic rotors).
     only the active group joins the allocation; the other one is masked with 1e-6 N
     (large enough to pass the spinal yaw saturation gate, small enough to be judged
     as disabled by the pwm conversion; convention from flamingo test/cross_domain) */
  enum MODE
  {
    AERIAL = 0,
    UNDERWATER = 1,
  };

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

  boost::shared_ptr<NamiRobotModel> nami_robot_model_;
  boost::shared_ptr<aerial_robot_navigation::NamiNavigator> nami_navigator_;
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

  /* underwater mode */
  MODE current_mode_ = AERIAL;
  static constexpr float MASKED_MOTOR_THRUST = 1e-6f;  // [N]
  int aerial_motor_num_ = 4;  // motor [0, aerial_motor_num_) aerial, the rest aquatic

  /* underwater surge allocation mode (doc/underwater_allocation_analysis.md):
     allocate (Fx, Fz, tau_x, tau_z) instead of the legacy (Fz, tau_x, tau_y, tau_z).
     tau_y becomes the dependent quantity (pitch is left to the passive buoyancy
     righting moment) and the body-x force leak that caused the constant forward
     drift is actively regulated to the commanded surge acceleration (0 = hold). */
  bool last_surge_allocation_ = false;
  double surge_acc_limit_;  // [m/s^2] clamp on the surge acc feedforward
  bool surgeAllocation() const
  {
    return current_mode_ == UNDERWATER && nami_navigator_ && nami_navigator_->getSurgeAllocation() && underactuate_;
  }

  /* depth control (hand-written PID, direct feedback from the depth sensor via navigator;
     bypasses the state estimation, following the flamingo dev/aquatic precedent) */
  double depth_p_gain_, depth_i_gain_, depth_d_gain_;
  double depth_i_limit_;         // [Ns] integral clamp
  double depth_hover_thrust_;    // [N] buoyancy trim (negative = push down for positive buoyancy)
  double depth_thrust_limit_;    // [N] symmetric output clamp (bi-directional)
  double depth_d_lpf_rate_;      // low-pass rate for the D term
  double depth_err_i_ = 0, depth_prev_err_ = 0, depth_err_d_filtered_ = 0;
  double depth_prev_stamp_ = 0;

  bool isMotorActive(int i) const
  {
    return current_mode_ == UNDERWATER ? (i >= aerial_motor_num_) : (i < aerial_motor_num_);
  }
  double depthControlLoop();
  void loadUnderwaterGains();

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendUnderwaterIdleCommand();  // hold air rotors at armed-stop while armed-but-inactive
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
};
};  // namespace aerial_robot_control
