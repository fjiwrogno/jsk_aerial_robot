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
  debug_rpy_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("debug/current_rpy", 1);

  target_depth_pub_ = nh_.advertise<std_msgs::Float32>("debug/target_depth", 1);
  
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &FlamingoController::joyCallback, this, ros::TransportHints().tcpNoDelay());
  depth_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("depth_sensor_node/depth", 1, &FlamingoController::depthCallback, this, ros::TransportHints().tcpNoDelay());
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
  getParam<bool>(control_nh, "underactuate", underactuate_,true);

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
  getParam<double>(control_nh, "depth_p_gain", depth_p_gain_, 1.0);
  getParam<double>(control_nh, "depth_i_gain", depth_i_gain_, 0.0);
  getParam<double>(control_nh, "depth_d_gain", depth_d_gain_, 0.0);
  getParam<double>(control_nh, "depth_hover_thrust", depth_hover_thrust_, 4.0);
  getParam<double>(control_nh, "max_depth", max_depth_, 0.5);
  getParam<double>(control_nh, "max_dive_rate", max_dive_rate_, 0.05);
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

void FlamingoController::directJoystickControl()
{
  // === 1) Thrust from joystick with Betaflight-style expo curve ===
  // joy_throttle_cmd_ is [0, 1] (0 = stick center/rest, 1 = stick top)
  double curved_throttle = applyThrottleCurve(joy_throttle_cmd_);
  double total_z_thrust = throttle_min_thrust_ + curved_throttle * (throttle_max_thrust_ - throttle_min_thrust_);
  total_z_thrust = std::max(0.0, std::min(total_z_thrust, motor_max_thrust_ * motor_num_));

  // Convert total z-thrust to target z-acceleration (same unit as PID Z output),
  // then use the already-computed integrated_map_inv_trans_ to distribute to each rotor,
  // exactly as the mocap z-control does in controlCore().
  double mass = flamingo_robot_model_->getMass();
  double target_acc_z = total_z_thrust / mass;

  Eigen::VectorXd joy_wrench_z(underactuate_ ? 1 : 3);
  joy_wrench_z(0) = target_acc_z;
  target_vectoring_f_trans_ = integrated_map_inv_trans_ * joy_wrench_z;

  // Re-fill target_base_thrust_ from the allocation result, identical to the loop in controlCore().
  int last_col = 0;
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    target_base_thrust_.at(rotor_coef_ * i)     = f_i[0];
    target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
    
    last_col += rotor_coef_;
  }

  // === 2) Roll/Pitch with expo for finer control ===
  if (joy_throttle_cmd_ == 0.0)
  {
    // no attitude control before actively increasing throttle to takeoff
    target_roll_ = estimator_->getEuler(Frame::COG, estimate_mode_).x();
    target_pitch_ = estimator_->getEuler(Frame::COG, estimate_mode_).y();
  }
  else
  {
    target_roll_ = applyStickExpo(joy_roll_cmd_) * direct_roll_max_;
    target_pitch_ = applyStickExpo(joy_pitch_cmd_) * direct_pitch_max_;
  }
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
  PoseLinearController::controlCore();
  if (current_mode_ == UNDERWATER)
  {
    underwaterControlCore();
  }
  else
  {
    aerialControlCore();
  }
}

void FlamingoController::aerialControlCore()
{
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
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

  if (direct_joystick_mode_)
  {
    directJoystickControl();
  }
}

void FlamingoController::underwaterControlCore()
{
  double pid_depth_w = depthControlLoop() / flamingo_robot_model_->getMass();  
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(0, 0, pid_depth_w);
    tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;
    Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());

    // for underactuate flaingo, rpy control is done in fc
    double target_ang_acc_x = 0.0;
    double target_ang_acc_y = 0.0;
    double target_ang_acc_z = 0.0;

    if (gimbal_calc_in_fc_)
      target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
    // logic is not complete here

    pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();;
    pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
    pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
    pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
    pid_msg_.roll.target_p = target_roll_;
    pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
    pid_msg_.roll.target_d = target_omega_.x();
    pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
    pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();;
    pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
    pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
    pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
    pid_msg_.pitch.target_p = target_pitch_;
    pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
    pid_msg_.pitch.target_d = target_omega_.y();
    pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

    // allocation matrix caculation for attitude_controller
    Eigen::Matrix3d inertia = flamingo_robot_model_->getInertia<Eigen::Matrix3d>();

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
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);

    last_col = 0;
    // depth control mapping
    for (int i = 0; i < motor_num_; i++)
    {
      if(if_dive_)
      {
        Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
        target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
        target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
      }
      last_col += rotor_coef_;
    }

    // pick the largest yaw scale for spinal yaw term reconstruction
    // this operation is mainly due to the rate limit of rosserial
    // reference： attitude_control.cpp:line 644 to 647 
    double max_yaw_scale = 0;  // for reconstruct yaw control term in spinal
    for (int i = 0; i < motor_num_ * rotor_coef_; i++)
    {
      if (fabs(integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW))) > fabs(max_yaw_scale))
      {
        max_yaw_scale = integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW));  // underactuated: yaw col is shifted
      }
    }
    candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

double FlamingoController::depthControlLoop()
{
  double total_thrust = 0.0;
  if (if_dive_)
  {
    // clamp target to valid sensor range: depth <= 0, and not deeper than -max_depth_
    if (target_depth_ > 0.0) target_depth_ = 0.0;
    if (target_depth_ < -max_depth_) target_depth_ = -max_depth_;

    const double err_p = fabs(target_depth_) - fabs(current_depth_);

    const double ctrl_loop_rate_ = 1000.0;
    const double dt = 1.0 / ctrl_loop_rate_;

    depth_err_i_ += err_p * dt;
    depth_err_i_ = std::max(-2.0, std::min(2.0, depth_err_i_));

    const double err_d = (err_p - depth_prev_err_) / dt;
    depth_prev_err_ = err_p;

    const double lpf_factor = 0.6;
    filtered_depth_err_d_ = lpf_factor * err_d + (1.0 - lpf_factor) * filtered_depth_err_d_;

    const double pid_depth_item =
      depth_p_gain_ * err_p + depth_i_gain_ * depth_err_i_ + depth_d_gain_ * filtered_depth_err_d_;

    tf::Vector3 current_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
    double current_pitch = current_rpy.y();
    double current_roll = current_rpy.x();

    double base_hover_thrust = depth_hover_thrust_ + pid_depth_item;

    // when roll changes, the vertical thrust will be smaller at that point
    // when pitch changes, 
    double tilt_factor = 1.0 / std::max(0.7, cos(current_pitch) * cos(current_roll));

    total_thrust = base_hover_thrust * tilt_factor;

    total_thrust = std::max(0.0, std::min(10.0, total_thrust));
  }
  
  return total_thrust;
}

void FlamingoController::depthCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_depth_ = msg->point.z;
}

void FlamingoController::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{  
  sensor_msgs::Joy joy_cmd = joyParse(*msg);
  if (joy_cmd.axes.size() == 0 || joy_cmd.buttons.size() == 0)
  {
    ROS_WARN_ONCE("FlamingoController: Joy message is not parsed correctly.");
    return;
  }
  
  if(joy_cmd.buttons[JOY_BUTTON_STOP] == 1)
  {
    navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
    /* update the target pos(maybe not necessary) */
    // navigator_->setTargetXyFromCurrentState();
    double current_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
    navigator_->setTargetYaw(current_yaw);
  }

  if(joy_cmd.buttons[JOY_BUTTON_START] == 1)
  {
    current_mode_ != current_mode_;
  }

  if(current_mode_ == MODE::CROSS_DOMAIN)
  {
    aerialJoyCallback(msg);
    ROS_ERROR("CROSS DOMAIN MODE IS ON");
  }
  else 
  {
    underwaterJoyCallback(msg);
    ROS_ERROR("UNDERWATER MODE IS ON");
  }
}

void FlamingoController::aerialJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  sensor_msgs::Joy joy_cmd = joyParse(*msg);
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

  double raw_yaw = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];
  joy_yaw_val_cmd_ = (fabs(raw_yaw) > deadzone) ? raw_yaw : 0.0;

  double raw_pitch = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];
  joy_pitch_cmd_ = (fabs(raw_pitch) > deadzone) ? raw_pitch : 0.0;

  double raw_roll = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];
  joy_roll_cmd_ = (fabs(raw_roll) > deadzone) ? raw_roll : 0.0;
}

void FlamingoController::underwaterJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{  
  sensor_msgs::Joy joy_cmd = joyParse(*msg);
  float joy_stick_deadzone_ = 0.2;
  
  tf::Vector3 current_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);
  double current_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
  static bool yaw_control_flag = false;

  // publish current attitude angle
  geometry_msgs::Vector3Stamped rpy_msg;
  rpy_msg.header.stamp = ros::Time::now();
  rpy_msg.vector.x = target_depth_;
  rpy_msg.vector.y = target_yaw_;
  rpy_msg.vector.z = omega.z();
  debug_rpy_pub_.publish(rpy_msg);
  // Publish target depth for debugging and analysis
  std_msgs::Float32 target_depth_msg;
  target_depth_msg.data = target_depth_;
  target_depth_pub_.publish(target_depth_msg);
  double mapped_total_thrust = depth_hover_thrust_;


  if(joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] && joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1])
  {
    if_stablize_ = !if_stablize_;

    if(if_stablize_)
    {
      ROS_ERROR("STABLIZED MODE IS ON!");
    }
    else
    {
      ROS_ERROR("MANUAL MODE IS ON!");
    }

  }
  
  if(joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] && joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2])
  {
    if_dive_ = !if_dive_;

    if(if_dive_)
    {
      ROS_ERROR("DEPTH CONTROL IS ON!");
      target_depth_ = current_depth_;
      depth_err_i_ = 0;
      depth_prev_err_ = 0;
    }
    else
    {
      ROS_ERROR("DEPTH CONTROL IS OFF");
    }
  }
  // speed mode switch
  if (joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] && joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT])
  {
    if (!speed_mode_button_pressed_) 
    {
      current_speed_mode_ = static_cast<SpeedMode>((current_speed_mode_ + 1) % 3);
      speed_mode_button_pressed_ = true;
      
      const char* mode_str = "";
      switch(current_speed_mode_) {
        case SPEED_LOW:    mode_str = "LOW (Max 8 deg)"; break;
        case SPEED_MEDIUM: mode_str = "MEDIUM (Max 15 deg)"; break;
        case SPEED_HIGH:   mode_str = "HIGH (Max 25 deg)"; break;
      }
      ROS_WARN("[Flamingo] Speed Mode: %s", mode_str);
    }
  }
  else
  {
    speed_mode_button_pressed_ = false;
  }

  if (if_dive_)
  {
    if (fabs(joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS]) > joy_stick_deadzone_)
    {
      // stick up (+) -> shallower (toward 0), stick down (-) -> deeper (more negative)
      target_depth_ = current_depth_ + joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] * max_dive_rate_;

      // clamp to sensor-valid range
      if (target_depth_ > 0.0) target_depth_ = 0.0;
      if (target_depth_ < -max_depth_) target_depth_ = -max_depth_;
    }
    else
    {
      // hold current depth
      target_depth_ = current_depth_;
    }

  }
  else
  {
    // 1. Right Stick Vertical -> Thrust
    // JOY_AXIS_STICK_RIGHT_UPWARDS: Up is +1.0, Down is -1.0. Middle is 0.0.
    // could be set to different throttle levles later
    float max_thrust = 10.0;
    float thrust_input = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];

    // currently only allows lift force:single direction
    if (thrust_input > joy_stick_deadzone_)
    {
      // Here, we map the stick input directly to thrust. Up is positive thrust, Down is negative thrust.
      mapped_total_thrust = thrust_input * max_thrust;
    }
    else
    {
      // float on the surface
      // 6.0 will fully submerged
      mapped_total_thrust = 3.0;
    }

    // workaround: pad non-zero small value for base thrust (base_throttle) to avoid the
    // unexpected condition check `fabs(base_thrust_term_[0]) > 0` in spinal.
    // pelase refer to attitude_control.cpp, l.1175
    std::fill(target_base_thrust_.begin(), target_base_thrust_.end(), 1e-6);

    // Assign thrust to the Z-component of each motor's virtual thrust vector
    for (int i = 0; i < motor_num_; ++i)
    {
      // The Z-component is at index (i * rotor_coef_ + 1) for gimbal_dof_ = 1
      target_base_thrust_.at(i * rotor_coef_ + 1) = mapped_total_thrust / motor_num_;
    }

  }


  if(!if_stablize_)
  {
    double current_max_pitch = PITCH_LIMIT_LOW;
    switch(current_speed_mode_) {
      case SPEED_LOW:    current_max_pitch = PITCH_LIMIT_LOW; break;
      case SPEED_MEDIUM: current_max_pitch = PITCH_LIMIT_MED; break;
      case SPEED_HIGH:   current_max_pitch = PITCH_LIMIT_HIGH; break;
    }

    double raw_target_pitch = 0.0;
    if (joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] > joy_stick_deadzone_) 
    {
      raw_target_pitch = -1.0 * current_max_pitch;
    }
    else if (joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] < -joy_stick_deadzone_)
    {
      raw_target_pitch = current_max_pitch;
    }
    else
    {
      raw_target_pitch = 0.0;
    } 
  
    target_pitch_ = raw_target_pitch;
    
    double raw_target_roll = 0.0;
    float max_roll_angle = 0.31; // ~30 degrees
    if(fabs(joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS]) > joy_stick_deadzone_)
    {
      raw_target_roll = -1.0 * joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] * max_roll_angle;
    }

    target_roll_ = raw_target_roll;
    
  }
  else
  {

    float current_max_pitch = 0.26; // ~15 degrees
    if(fabs(joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS]) > joy_stick_deadzone_)
    {
      target_pitch_ = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] * current_max_pitch;
    } 
    else
    {
      target_pitch_ = 0;
    }

    // 2. Left Stick Horizontal -> Roll Ange
    // JOY_AXIS_STICK_LEFT_LEFTWARDS: Left is +1.0, Right is -1.0
    float max_roll_angle = 0.21; // ~15 degrees
    if(fabs(joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS]) > joy_stick_deadzone_)
    {
      target_roll_ = 3.14 - joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] * max_roll_angle; 
    }
    else
    {
      // becasue the initial rool angle is 3.14rad so currently set this value to 3.14
      target_roll_ = 3.14;
    }
  }

  // 3. Right Stick Horizontal -> Yaw
  double joy_val = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];
  if(fabs(joy_val) > joy_stick_deadzone_)
    {
      target_yaw_ = current_yaw + joy_val * joy_yaw_rate_;
      navigator_->setTargetYaw(angles::normalize_angle(target_yaw_));
      navigator_->setTargetOmegaZ(joy_val * joy_yaw_rate_);

      yaw_control_flag = true;
    }
  else
    {
      if(yaw_control_flag)
        {
          navigator_->setTargetYaw(current_yaw);
          navigator_->setTargetOmegaZ(0);
          yaw_control_flag = false;
        }
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
