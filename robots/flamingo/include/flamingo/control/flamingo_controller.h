// -*- mode: c++ -*-

#pragma once

#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
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
  ros::Publisher debug_rpy_pub_;                    // for underwater debug
  ros::Publisher desire_coord_pub_;                    // for spinal when underwater motion
  ros::Publisher target_depth_pub_;                  // for depth control debug and analysis



  ros::Subscriber joy_sub_;                   
  ros::Subscriber depth_sub_;



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
  double joy_yaw_rate_;
  int gimbal_dof_;
  int rotor_coef_;
  bool gimbal_calc_in_fc_;
  bool underactuate_;
  double target_roll_ = 0.0, target_pitch_ = 0.0, target_yaw_ = 0.0;
  bool if_stablize_ = false, if_dive_ = false;

  // Depth control
  double current_depth_ = 0.0;
  double target_depth_ = 0.0;
  double depth_p_gain_ = 1.0, depth_i_gain_ = 0.0, depth_d_gain_ = 0.0;
  double depth_err_i_ = 0.0;
  double depth_prev_err_ = 0.0;
  double depth_hover_thrust_ = 4.0;  // Default hover thrust for depth control
  double filtered_depth_err_d_ = 0;
  double max_depth_ = 0.0; // maximum depth for depth control
  double max_dive_rate_ = 0.05; // maximum dive rate for depth control

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
  void setSafeAttitudeGains();

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void depthCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  double depthControlLoop();
};
};  // namespace aerial_robot_control
