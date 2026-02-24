// -*- mode: c++ -*-

#pragma once

#include <array>
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
  ros::Publisher debug_rpy_pub_;                    // for underwater debug
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
  double target_roll_ = 0.0, target_pitch_ = 0.0;

  // Direct joystick control mode — Mode 2 (American hand), Betaflight Angle mode equivalent
  //   Left stick:  vertical = throttle, horizontal = yaw rate
  //   Right stick: vertical = pitch,    horizontal = roll
  bool direct_joystick_mode_;
  bool yaw_control_flag_ = false;

  // Throttle curve parameters (Betaflight-style)
  double throttle_min_thrust_;      // thrust per rotor at stick bottom (N), default 0
  double throttle_max_thrust_;      // thrust per rotor at full stick (N)
  double throttle_mid_;             // normalized stick position (0-1) where hover sensitivity centers
  double throttle_expo_;            // exponential factor 0-1 (0 = linear, higher = flatter near mid)
  double throttle_deadzone_bottom_; // bottom deadzone for throttle (fraction 0-1 of stick range)
  double motor_max_thrust_;         // hard clamp per rotor (N), must be < motor physical max

  // Attitude stick parameters
  double direct_pitch_max_;         // max pitch angle from joystick (rad)
  double direct_roll_max_;          // max roll angle from joystick (rad)
  double stick_expo_;               // expo for roll/pitch sticks (0 = linear, higher = gentler near center)

  double joy_throttle_cmd_;         // normalized [0, 1] from left stick vertical (0=bottom, 1=top)
  double joy_pitch_cmd_;            // normalized [-1, 1] from right stick vertical
  double joy_roll_cmd_;             // normalized [-1, 1] from right stick horizontal
  double joy_yaw_val_cmd_;          // normalized [-1, 1] from left stick horizontal
  double direct_yaw_target_;        // integrated yaw heading target (rad)
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void directJoystickControl();

  // Betaflight-style curve helpers
  double applyThrottleCurve(double normalized_input);  // input [0,1] → output [0,1]
  double applyStickExpo(double input);                 // input [-1,1] → output [-1,1]
  double target_yaw_ = 0.0;
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

  // Speed Modes
  enum SpeedMode {
    SPEED_LOW = 0,      // Precsion mode
    SPEED_MEDIUM = 1,   // Cruise mode
    SPEED_HIGH = 2      // Sport mode
  };
  SpeedMode current_speed_mode_ = SPEED_LOW;
  bool speed_mode_button_pressed_ = false;
  bool stablize_button_pressed_ = false;
  bool dive_button_pressed_ = false;
  bool mode_switch_button_pressed_ = false;

  // Max pitch angles for each mode (in radians)
  const double PITCH_LIMIT_LOW = 0.15;    // ~8.6 deg
  const double PITCH_LIMIT_MED = 0.26;    // ~15 deg
  const double PITCH_LIMIT_HIGH = 0.45;   // ~25 deg 
  
  enum MODE {
    CROSS_DOMAIN = 0,      // cross domain mode
    UNDERWATER = 1,   // underwater mode   
  };
  MODE current_mode_ = UNDERWATER;

  /* Per-axis PID gain set for mode switching */
  struct GainSet { double p, i, d; };
  /* Indexed by [X, Y, Z, ROLL, PITCH, YAW] */
  std::array<GainSet, 6> aerial_gains_{};
  std::array<GainSet, 6> underwater_gains_{};

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  /* Switch between CROSS_DOMAIN and UNDERWATER modes, updating gains and safety state */
  void switchMode();
  void aerialControlCore();
  void underwaterControlCore();

  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();

  void aerialJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void underwaterJoyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void depthCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  double depthControlLoop();
};
};  // namespace aerial_robot_control
