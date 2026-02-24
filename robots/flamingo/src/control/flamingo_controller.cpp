#include <flamingo/control/flamingo_controller.h>

using namespace std;

namespace aerial_robot_control
{
FlamingoController::FlamingoController() : PoseLinearController()
{
}

void FlamingoController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                    double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  flamingo_robot_model_ = boost::dynamic_pointer_cast<FlamingoRobotModel>(robot_model);

  FlamingoController::rosParamInit();

  rotor_coef_ = gimbal_dof_ + 1;  // number of virtual rotors in each rotor arm

  target_base_thrust_.resize(motor_num_ * rotor_coef_);
  target_full_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * gimbal_dof_, 0);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ =
      nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);

  // Direct joystick control (Mode 2 American hand)
  joy_throttle_cmd_ = 0.0;
  joy_pitch_cmd_ = 0.0;
  joy_roll_cmd_ = 0.0;
  joy_yaw_val_cmd_ = 0.0;
  direct_yaw_target_ = 0.0;
  yaw_control_flag_ = false;
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &FlamingoController::joyCallback, this);
}

void FlamingoController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
}

void FlamingoController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
  getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "underactuate", underactuate_, false);

  // Direct joystick control mode params (Mode 2 American hand)
  getParam<double>(control_nh, "joy_yaw_rate", joy_yaw_rate_, 0.01);
  getParam<bool>(control_nh, "direct_joystick_mode", direct_joystick_mode_, false);

  // Betaflight-style throttle curve parameters
  getParam<double>(control_nh, "throttle_min_thrust", throttle_min_thrust_, 0.0);     // thrust per rotor at stick bottom (N)
  getParam<double>(control_nh, "throttle_max_thrust", throttle_max_thrust_, 28.0);    // thrust per rotor at full stick (N)
  getParam<double>(control_nh, "throttle_mid", throttle_mid_, 0.89);                  // hover point (25/28≈0.89)
  getParam<double>(control_nh, "throttle_expo", throttle_expo_, 0.5);                 // expo factor (0=linear, 1=max expo)
  getParam<double>(control_nh, "throttle_deadzone_bottom", throttle_deadzone_bottom_, 0.1); // bottom deadzone fraction
  getParam<double>(control_nh, "motor_max_thrust", motor_max_thrust_, 28.0);          // hard clamp per rotor (N)

  // Attitude stick parameters
  getParam<double>(control_nh, "direct_pitch_max", direct_pitch_max_, 0.20);          // max pitch ~11.5 deg
  getParam<double>(control_nh, "direct_roll_max", direct_roll_max_, 0.20);            // max roll ~11.5 deg
  getParam<double>(control_nh, "stick_expo", stick_expo_, 0.3);                       // expo for roll/pitch sticks
}

bool FlamingoController::update()
{
  sendGimbalCommand();
  if (gimbal_calc_in_fc_)
  {
    std_msgs::UInt8 msg;
    msg.data = gimbal_dof_;
    gimbal_dof_pub_.publish(msg);
  }

  return PoseLinearController::update();
}

void FlamingoController::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
  sensor_msgs::Joy joy_cmd = joyParse(*msg);
  if (joy_cmd.axes.size() == 0) return;

  const double deadzone = 0.05;

  // Mode 2 (American hand) mapping:
  //   Left stick vertical   → throttle  (remap to 0..1)
  //   Left stick horizontal → yaw rate
  //   Right stick vertical  → pitch
  //   Right stick horizontal→ roll

  // Throttle: use only upper half of spring-centered stick [0, 1].
  // Stick at rest (center) = 0, push up = 0→1. Below center is ignored (0 thrust).
  double raw_throttle = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS];
  joy_throttle_cmd_ = std::max(0.0, raw_throttle); 
  joy_throttle_cmd_ = std::min(1.0, joy_throttle_cmd_);

  // Apply small deadzone near center so stick rest noise doesn't produce thrust
  if (joy_throttle_cmd_ < throttle_deadzone_bottom_)
    joy_throttle_cmd_ = 0.0;
  else
    joy_throttle_cmd_ = (joy_throttle_cmd_ - throttle_deadzone_bottom_) / (1.0 - throttle_deadzone_bottom_);

  // double raw_yaw = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];
  // joy_yaw_val_cmd_ = (fabs(raw_yaw) > deadzone) ? raw_yaw : 0.0;

  double raw_pitch = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];
  joy_pitch_cmd_ = (fabs(raw_pitch) > deadzone) ? raw_pitch : 0.0;

  double raw_roll = -1.0 * joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];
  joy_roll_cmd_ = (fabs(raw_roll) > deadzone) ? raw_roll : 0.0;

  if(joy_cmd.buttons[JOY_BUTTON_STOP] == 1)
  {
    navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
    /* update the target pos(maybe not necessary) */
    // navigator_->setTargetXyFromCurrentState();
    double current_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
    navigator_->setTargetYaw(current_yaw);
  }

}

tf::Vector3 FlamingoController::directJoystickControl() 
{
  // Returns acceleration command in the dash frame (yaw-aligned body frame).
  // Pitch stick  → forward/backward (dash x),  range [-5, 5] m/s²
  // Roll stick   → left/right       (dash y),  range [-5, 5] m/s²
  // Throttle     → vertical          (dash z),  range [ 0, 25] m/s²
  // Values exceeding physical motor limits are clamped (not rescaled).
  constexpr double target_acc_z_max = 25.0;
  constexpr double target_acc_xy_max = 2.5;

  const double mass = flamingo_robot_model_->getMass();
  const double acc_z_physical_max = motor_max_thrust_ * motor_num_ / mass;

  // Apply Betaflight-style expo curve so sensitivity is reduced around hover point,
  // then scale to acceleration and clamp to physical limit.
  const double curved_throttle = applyThrottleCurve(joy_throttle_cmd_);

  const double target_acc_x = applyStickExpo(joy_pitch_cmd_) * target_acc_xy_max;
  const double target_acc_y = applyStickExpo(joy_roll_cmd_) * target_acc_xy_max;
  const double target_acc_z = std::min(curved_throttle * target_acc_z_max, acc_z_physical_max);

  return tf::Vector3(target_acc_x, target_acc_y, target_acc_z);
}

double FlamingoController::applyThrottleCurve(double input)
{
  // Betaflight-style throttle expo with mid point.
  // input: [0, 1], output: [0, 1]
  // thr_mid: stick position that maps to the curve center (sensitivity inflection)
  // thr_expo: 0 = linear, 1 = maximum expo (flat near mid)
  //
  // Formula (from Betaflight src/main/fc/rc_controls.c):
  //   t = input - thr_mid
  //   output = thr_mid + t * (1 + expo * (t*t / (thr_mid*thr_mid) - 1))
  //   (normalized so that the curve passes smoothly through the mid point)
  //
  // This gives reduced sensitivity around thr_mid (fine hover control)
  // and increased sensitivity at the extremes.

  input = std::max(0.0, std::min(1.0, input));
  double expo = std::max(0.0, std::min(1.0, throttle_expo_));
  double mid = std::max(0.01, std::min(0.99, throttle_mid_));

  if (expo < 0.001)
    return input;  // linear if no expo

  double t = input - mid;
  // Betaflight scales the quadratic term by max(mid, 1-mid)^2 for normalization
  double range = (t > 0) ? (1.0 - mid) : mid;
  double t_normalized = t / range;  // normalize to [-1, 1] within each half
  double output = mid + range * t_normalized * (1.0 + expo * (t_normalized * t_normalized - 1.0));

  return std::max(0.0, std::min(1.0, output));
}

double FlamingoController::applyStickExpo(double input)
{
  // Standard RC expo for centered sticks (roll/pitch/yaw).
  // input: [-1, 1], output: [-1, 1]
  // Formula (Betaflight): output = input * (1 - expo + expo * input^2)
  // This reduces sensitivity near center while preserving full deflection range.

  double expo = std::max(0.0, std::min(1.0, stick_expo_));
  if (expo < 0.001)
    return input;

  return input * (1.0 - expo + expo * input * input);
}

void FlamingoController::controlCore()
{
  // if (direct_joystick_mode_)
  // {
  //   const double current_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
  //   if (!yaw_control_flag_)
  //   {
  //     direct_yaw_target_ = current_yaw;
  //     yaw_control_flag_ = true;
  //   }
  //   direct_yaw_target_ += joy_yaw_val_cmd_ * joy_yaw_rate_;
  //   navigator_->setTargetYaw(direct_yaw_target_);
  // }
  // else
  // {
  //   yaw_control_flag_ = false;
  // }

  PoseLinearController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  if (direct_joystick_mode_)
  {
    // directJoystickControl() returns acceleration in dash frame (yaw-aligned),
    // rotate to world frame so downstream target_acc_dash computation is consistent.
    tf::Vector3 input_acc_dash = directJoystickControl();
    target_acc_w = tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z())) * input_acc_dash;
  }
  
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

  if (underactuate_)
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
  else
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  Eigen::Matrix3d inertia = flamingo_robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  if (gimbal_calc_in_fc_)
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  else
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + gyro;

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);

  double mass_inv = 1 / flamingo_robot_model_->getMass();

  Eigen::Matrix3d inertia_inv = inertia.inverse();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      flamingo_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = flamingo_robot_model_->getRotorDirection();
  const double m_f_rate = flamingo_robot_model_->getMFRate();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  int last_col = 0;

  /* calculate normal allocation */
  for (int i = 0; i < motor_num_; i++)
  {
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }

  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  /* calculate masked rotation matrix */
  // so from here, we get the important allocation matrices 
  // maybe we can direclt add the aquatic one here to enable coxial rotor configuration?
  // it's better to have a simulation first, but for my case maybe just try
  std::vector<KDL::Rotation> thrust_coords_rot = flamingo_robot_model_->getThrustCoordRot<KDL::Rotation>();
  std::vector<Eigen::MatrixXd> masked_rot;
  for (int i = 0; i < motor_num_; i++)
  {
    tf::Quaternion r;
    tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);
    Eigen::Matrix3d conv_cog_from_thrust;
    tf::matrixTFToEigen(tf::Matrix3x3(r), conv_cog_from_thrust);
    if (gimbal_dof_ == 1)
    {
      Eigen::MatrixXd mask(3, 2);
      mask << 0, 0, 1, 0, 0, 1;
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
    else if (gimbal_dof_ == 2)
    {
      Eigen::MatrixXd mask = Eigen::Matrix3d::Identity();
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
  }

  /* mask integrated allocation */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);
  Eigen::MatrixXd integrated_map = Eigen::MatrixXd::Zero(6, (gimbal_dof_ + 1) * motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    integrated_rot.block(3 * i, rotor_coef_ * i, 3, rotor_coef_) = masked_rot[i];
  }
  integrated_map = full_q_mat * integrated_rot;

  /* extract controlled axis  */
  if (underactuate_)
  {
    target_wrench_acc_cog = target_wrench_acc_cog.tail(4);  // z, roll, pitch, yaw
    integrated_map = integrated_map.bottomRows(4);          // z, roll, pitch, yaw
  }

  /* vectoring force mapping */
  Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
  integrated_map_inv_trans_ = integrated_map_inv.leftCols(underactuate_ ? 1 : 3);
  integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
  if (underactuate_)
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
  else
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
  target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3);  // debug
  last_col = 0;

  /* under actuated axis  */
  if (underactuate_)
  {
    if (hovering_approximate_)
    {
      target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
      target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
    else
    {
      target_roll_ = atan2(-target_acc_dash.y(),
                           sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
  }

  /*  calculate target base thrust (considering only translational components)*/
  double max_yaw_scale = 0;  // for reconstruct yaw control term in spinal
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    if (gimbal_dof_ == 1)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
    }
    else if (gimbal_dof_ == 2)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
      target_base_thrust_.at(rotor_coef_ * i + 2) = f_i[2];
    }
    if (integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW)) > max_yaw_scale)
      max_yaw_scale = integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW));  // underactuated: yaw col is shifted

    last_col += rotor_coef_;
  }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  /* calculate target full thrusts and gimbal angles (considering full components)*/
  last_col = 0;
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) +
                                     target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    target_full_thrust_.at(i) = f_i_integrated.norm();
    if (gimbal_dof_ == 1)
    {
      target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
    }
    else if (gimbal_dof_ == 2)
    {
      if (f_i_integrated[0] == 0 || f_i_integrated[2] == 0)
        continue;

      double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
      double gimbal_pitch =
          atan2(f_i_integrated[0], -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2] * cos(gimbal_roll));
      target_gimbal_angles_.at(2 * i) = gimbal_roll;
      target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
    }
    last_col += rotor_coef_;
  }
}

void FlamingoController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();

  if (gimbal_calc_in_fc_)
  {
    sendTorqueAllocationMatrixInv();
  }
  else
  {
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < motor_num_; i++)
    {
      if (gimbal_dof_ == 1)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
      }
      else if (gimbal_dof_ == 2)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i));
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      }
    }
    gimbal_control_pub_.publish(gimbal_control_msg);

    std_msgs::Float32MultiArray target_vectoring_force_msg;
    target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
    for (int i = 0; i < target_vectoring_f_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
    }
    target_vectoring_force_pub_.publish(target_vectoring_force_msg);
  }
}

void FlamingoController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;

  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;

  if (gimbal_calc_in_fc_)
  {
    flight_command_data.base_thrust = target_base_thrust_;
    flight_command_data.angles[2] = candidate_yaw_term_;
  }
  else
  {
    flight_command_data.base_thrust = target_full_thrust_;
  }

  flight_cmd_pub_.publish(flight_command_data);
}

void FlamingoController::sendGimbalCommand()
{
  sensor_msgs::JointState gimbal_state_msg;
  gimbal_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < motor_num_; i++)
  {
    if (gimbal_dof_ == 1)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(i));
      std::string gimbal_name = "gimbal" + std::to_string(i + 1);
      gimbal_state_msg.name.push_back(gimbal_name);
    }
    else if (gimbal_dof_ == 2)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i));
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
      std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";
      gimbal_state_msg.name.push_back(gimbal_roll_name);
      gimbal_state_msg.name.push_back(gimbal_pitch_name);
    }
  }
  // gimbal_state_pub_.publish(gimbal_state_msg);
}

void FlamingoController::sendTorqueAllocationMatrixInv()
{
  spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
  torque_allocation_matrix_inv_msg.rows.resize(motor_num_ * rotor_coef_);
  Eigen::MatrixXd torque_allocation_matrix_inv = integrated_map_inv_rot_;
  if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
    ROS_ERROR("Torque Allocation Matrix overflow");
  for (unsigned int i = 0; i < motor_num_ * rotor_coef_; i++)
  {
    torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
  }
  torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
}

void FlamingoController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::FlamingoController, aerial_robot_control::ControlBase);
