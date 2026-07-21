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
#include <nami/control/rc_mapper.h>
#include <nami/control/rc_thrust_mapper.h>

namespace aerial_robot_control
{
class NamiController : public PoseLinearController
{
public:
  NamiController();
  ~NamiController() = default;

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
  double max_tilt_angle_;
  double target_roll_ = 0.0, target_pitch_ = 0.0;

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();

  /* RC hand flight (manual mode from rc/raw, see config/RcMapping.yaml) */
  enum class RcFlightState
  {
    IDLE,
    ARMING,
    FLYING
  };

  ros::Subscriber rc_raw_sub_;
  ros::Publisher rc_debug_pub_;
  std::unique_ptr<RcMapper> rc_mapper_;
  boost::shared_ptr<aerial_robot_navigation::NamiNavigator> nami_navigator_;
  bool rc_enable_ = false;
  RcFlightState rc_flight_state_ = RcFlightState::IDLE;
  double rc_arming_start_time_ = 0;
  bool rc_takeoff_sent_ = false;
  double rc_target_yaw_ = 0;
  double rc_throttle_force_ = 0;   // slew-limited pilot command before angle boost [N]
  double rc_output_force_ = 0;     // force sent to allocation after angle boost [N]
  tf::Vector3 rc_manual_acc_dash_;  // yaw-free (dash) frame acceleration command
  bool rc_require_takeoff_enable_ = true;
  double rc_arming_timeout_ = 3.0;
  double rc_force_slew_rate_ = 40.0;
  double rc_min_acc_z_ = 1.0;
  RcThrustMapper rc_thrust_mapper_;

  void rcRawCallback(const aerial_robot_msgs::RcRawConstPtr& msg);
  void rcStateMachine();
  void rcManualSetpointUpdate();
  bool rcManualActive() const;
  bool rcOutputInhibited() const;
  double rcDesiredForce(const RcMapper::Sticks& sticks) const;
  void rcPublishDebug();
};
};  // namespace aerial_robot_control
