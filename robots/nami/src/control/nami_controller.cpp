#include <nami/control/nami_controller.h>

#include <algorithm>
#include <angles/angles.h>
#include <cmath>

using namespace std;

namespace aerial_robot_control
{
NamiController::NamiController() : PoseLinearController()
{
}

void NamiController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  nami_robot_model_ = boost::dynamic_pointer_cast<NamiRobotModel>(robot_model);

  NamiController::rosParamInit();

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

  /* RC hand flight */
  ros::NodeHandle rc_nh(nh_, "controller/rc");
  getParam<bool>(rc_nh, "enable", rc_enable_, false);
  if (rc_enable_)
  {
    nami_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::NamiNavigator>(navigator);
    if (!nami_navigator_)
    {
      ROS_ERROR("RC hand flight requires NamiNavigator, disable RC mode");
      rc_enable_ = false;
    }
  }
  if (rc_enable_)
  {
    rc_mapper_ = std::make_unique<RcMapper>();
    if (!rc_mapper_->initialize(rc_nh))
    {
      ROS_ERROR("RC hand flight disabled because RcMapping.yaml is invalid");
      rc_mapper_.reset();
      rc_enable_ = false;
      return;
    }

    ros::NodeHandle safety_nh(rc_nh, "safety");
    getParam<bool>(safety_nh, "require_takeoff_enable_to_arm", rc_require_takeoff_enable_, true);
    getParam<double>(safety_nh, "arming_timeout", rc_arming_timeout_, 3.0);

    ros::NodeHandle thrust_nh(rc_nh, "thrust_mapping");
    RcThrustMapper::Config& thrust_config = rc_thrust_mapper_.config();
    getParam<double>(thrust_nh, "f_min", thrust_config.min_force, 0.0);
    getParam<double>(thrust_nh, "f_max", thrust_config.max_force, 0.0);
    getParam<double>(thrust_nh, "max_slew_rate", rc_force_slew_rate_, 40.0);
    getParam<double>(thrust_nh, "min_acc_z", rc_min_acc_z_, 1.0);
    getParam<double>(thrust_nh, "hover_throttle", thrust_config.hover_throttle, 0.5);
    getParam<double>(thrust_nh, "hover_force", thrust_config.hover_force, 0.0);
    getParam<double>(thrust_nh, "hover_acceleration", thrust_config.hover_acceleration,
                     aerial_robot_estimation::G);
    getParam<double>(thrust_nh, "f_down", thrust_config.down_authority, 0.0);
    getParam<double>(thrust_nh, "f_up", thrust_config.up_authority, 0.0);
    getParam<double>(thrust_nh, "throttle_expo", thrust_config.expo, 0.5);
    getParam<double>(thrust_nh, "hover_deadband", thrust_config.hover_deadband, 0.01);
    getParam<bool>(thrust_nh, "tilt_compensation", thrust_config.tilt_compensation, true);
    getParam<double>(thrust_nh, "max_tilt_compensation", thrust_config.max_tilt_compensation, 1.3);

    if (thrust_config.max_force <= thrust_config.min_force)
      ROS_WARN("RC thrust f_max %.1f N is not greater than f_min %.1f N; using a 1 N range",
               thrust_config.max_force, thrust_config.min_force);
    rc_thrust_mapper_.sanitize();
    if (rc_force_slew_rate_ <= 0.0)
    {
      ROS_WARN("RC thrust max_slew_rate must be positive; use 40 N/s");
      rc_force_slew_rate_ = 40.0;
    }
    rc_min_acc_z_ = std::max(rc_min_acc_z_, 0.0);

    rc_raw_sub_ = nh_.subscribe("rc/raw", 1, &NamiController::rcRawCallback, this);
    rc_debug_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("rc/manual_command", 1);
    ROS_INFO("RC hand flight enabled: f_max %.1f N, hover stick %.2f, expo %.2f", thrust_config.max_force,
             thrust_config.hover_throttle, thrust_config.expo);
  }
}

void NamiController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
}

void NamiController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
  getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "underactuate", underactuate_, false);
  // keep below the spinal failsafe threshold MAX_TILT_ANGLE (1.0 rad)
  getParam<double>(control_nh, "max_tilt_angle", max_tilt_angle_, 0.8);
  max_tilt_angle_ = std::clamp(max_tilt_angle_, 0.0, 0.99);
}

bool NamiController::update()
{
  if (rc_enable_)
    rcStateMachine();

  sendGimbalCommand();
  if (gimbal_calc_in_fc_)
  {
    std_msgs::UInt8 msg;
    msg.data = gimbal_dof_;
    gimbal_dof_pub_.publish(msg);
  }

  return PoseLinearController::update();
}

void NamiController::controlCore()
{
  const bool rc_manual_active = rcManualActive();
  if (rc_manual_active)
    rcManualSetpointUpdate();

  PoseLinearController::controlCore();
  if (rc_manual_active)
  {
    pid_controllers_.at(X).reset();
    pid_controllers_.at(Y).reset();
    pid_controllers_.at(Z).reset();
  }
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  if (rc_manual_active)
    target_acc_w = tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z())) * rc_manual_acc_dash_;
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
  Eigen::Matrix3d inertia = nami_robot_model_->getInertia<Eigen::Matrix3d>();
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

  double mass_inv = 1 / nami_robot_model_->getMass();

  Eigen::Matrix3d inertia_inv = inertia.inverse();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      nami_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = nami_robot_model_->getRotorDirection();
  const double m_f_rate = nami_robot_model_->getMFRate();

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
  std::vector<KDL::Rotation> thrust_coords_rot = nami_robot_model_->getThrustCoordRot<KDL::Rotation>();
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
    else if (gimbal_dof_ == 0)
    {
      // fixed rotor (no gimbal): keep only the thrust z-direction
      Eigen::MatrixXd mask(3, 1);
      mask << 0, 0, 1;
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
    }
    else
    {
      /* clamp the vertical component so the reconstruction stays well-defined even when
         the z acceleration command is not positive (e.g. force landing on the ground),
         which otherwise flips atan2 to +-pi and trips the spinal tilt failsafe */
      double acc_dash_z = std::max(target_acc_dash.z(), 1e-2);
      target_roll_ =
          atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + acc_dash_z * acc_dash_z));
      target_pitch_ = atan2(target_acc_dash.x(), acc_dash_z);
    }
    target_roll_ = std::clamp(target_roll_, -max_tilt_angle_, max_tilt_angle_);
    target_pitch_ = std::clamp(target_pitch_, -max_tilt_angle_, max_tilt_angle_);
    navigator_->setTargetRoll(target_roll_);
    navigator_->setTargetPitch(target_pitch_);
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
    else if (gimbal_dof_ == 0)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];  // rotor_coef_ == 1: scalar thrust per rotor
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

void NamiController::sendCmd()
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

void NamiController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;

  if (rcOutputInhibited())
  {
    flight_command_data.angles[0] = 0;
    flight_command_data.angles[1] = 0;
    flight_command_data.angles[2] = 0;
    const size_t thrust_size = gimbal_calc_in_fc_ ? target_base_thrust_.size() : target_full_thrust_.size();
    flight_command_data.base_thrust.assign(thrust_size, 0.0f);
    flight_cmd_pub_.publish(flight_command_data);
    return;
  }

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

void NamiController::sendGimbalCommand()
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

void NamiController::sendTorqueAllocationMatrixInv()
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

void NamiController::rcRawCallback(const aerial_robot_msgs::RcRawConstPtr& msg)
{
  rc_mapper_->feed(*msg);
}

bool NamiController::rcManualActive() const
{
  return rc_enable_ && rc_flight_state_ == RcFlightState::FLYING && rc_mapper_->linkValid() &&
         !rc_mapper_->killLatched() && !navigator_->getForceLandingFlag();
}

bool NamiController::rcOutputInhibited() const
{
  return rc_enable_ &&
         (!rc_mapper_ || !rc_mapper_->linkValid() || rc_mapper_->killLatched() || !rc_mapper_->armSwitch());
}

/* safety priority (mapping spec sec.12): link loss > kill latch > disarm > normal output */
void NamiController::rcStateMachine()
{
  ros::Time now = ros::Time::now();
  rc_mapper_->update(now);
  int navi_state = navigator_->getNaviState();

  if (!rc_mapper_->linkValid())
  {
    if (navi_state != aerial_robot_navigation::ARM_OFF_STATE && navi_state != aerial_robot_navigation::STOP_STATE)
    {
      ROS_ERROR("RC link lost: latch kill, disarm, and stop motor output");
      navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
    }
    rc_flight_state_ = RcFlightState::IDLE;
    rc_throttle_force_ = 0;
    rc_output_force_ = 0;

    rcPublishDebug();
    return;
  }

  if (rc_mapper_->killLatched())
  {
    if (navi_state != aerial_robot_navigation::ARM_OFF_STATE && navi_state != aerial_robot_navigation::STOP_STATE)
    {
      ROS_ERROR("RC kill switch: stop motors immediately");
      navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
    }
    rc_flight_state_ = RcFlightState::IDLE;
    rc_throttle_force_ = 0;
    rc_output_force_ = 0;

    rcPublishDebug();
    return;
  }

  switch (rc_flight_state_)
  {
    case RcFlightState::IDLE:
    {
      /* note: a stale force_landing_flag may survive a crash landing (sensor unhealth sets it
         even when disarmed) and is only cleared by the START_STATE branch on the next arming,
         so do not gate arming on it here; navi_state == ARM_OFF already blocks in-air rearm */
      if (rc_mapper_->armRisingEdge() && navi_state == aerial_robot_navigation::ARM_OFF_STATE)
      {
        if (!rc_mapper_->throttleLow())
        {
          ROS_WARN("RC arm refused: throttle is not low");
          break;
        }
        if (rc_require_takeoff_enable_ && !rc_mapper_->takeoffEnable())
        {
          ROS_WARN("RC arm refused: takeoff enable switch is off");
          break;
        }
        /* the robot model mass is only valid at runtime, so check the thrust range here */
        const double mass = nami_robot_model_->getMass();
        if (mass <= 0.0)
        {
          ROS_ERROR("RC arm refused: robot mass must be positive");
          break;
        }
        RcThrustMapper::Config& thrust_config = rc_thrust_mapper_.config();
        const double nominal_hover_force =
            thrust_config.hover_force > 0.0 ? thrust_config.hover_force : mass * thrust_config.hover_acceleration;
        if (thrust_config.max_force < nominal_hover_force * 1.2)
        {
          ROS_WARN("RC thrust f_max %.1f N gives less than 20%% margin over hover force %.1f N, expand to %.1f N",
                   thrust_config.max_force, nominal_hover_force, nominal_hover_force * 2);
          thrust_config.max_force = nominal_hover_force * 2;
        }
        const double hover_force = rc_thrust_mapper_.hoverForce(mass);

        rc_target_yaw_ = estimator_->getEuler(Frame::COG, estimate_mode_).z();
        rc_throttle_force_ = 0;
        rc_output_force_ = 0;
        rc_takeoff_sent_ = false;
        rc_arming_start_time_ = now.toSec();
        ROS_INFO("RC arm: start motor arming (hover force %.2f N at stick %.2f)", hover_force,
                 thrust_config.hover_throttle);
        nami_navigator_->rcArm();
        rc_flight_state_ = RcFlightState::ARMING;
      }
      break;
    }
    case RcFlightState::ARMING:
    {
      if (!rc_mapper_->armSwitch())
      {
        ROS_WARN("RC disarm during arming sequence");
        navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
        rc_flight_state_ = RcFlightState::IDLE;
        rc_throttle_force_ = 0;
        rc_output_force_ = 0;
        break;
      }
      if (navi_state == aerial_robot_navigation::ARM_ON_STATE && !rc_takeoff_sent_)
      {
        nami_navigator_->rcTakeoff();
        rc_takeoff_sent_ = true;
      }
      if (navi_state == aerial_robot_navigation::TAKEOFF_STATE || navi_state == aerial_robot_navigation::HOVER_STATE)
      {
        const double mass = nami_robot_model_->getMass();
        const RcMapper::Sticks& sticks = rc_mapper_->sticks();
        const double target_force = rc_thrust_mapper_.throttleToForce(sticks.throttle, mass);
        ROS_INFO("RC manual flight engaged (ramping collective toward %.1f N from throttle %.2f)", target_force,
                 sticks.throttle);
        rc_flight_state_ = RcFlightState::FLYING;
      }
      else if (!rc_takeoff_sent_ && now.toSec() - rc_arming_start_time_ > rc_arming_timeout_)
      {
        ROS_ERROR("RC arming timeout: back to idle");
        navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
        rc_flight_state_ = RcFlightState::IDLE;
        rc_throttle_force_ = 0;
        rc_output_force_ = 0;
      }
      break;
    }
    case RcFlightState::FLYING:
    {
      if (!rc_mapper_->armSwitch())
      {
        ROS_WARN("RC disarm: stop motors");
        navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
        rc_flight_state_ = RcFlightState::IDLE;
        rc_throttle_force_ = 0;
        rc_output_force_ = 0;
        break;
      }
      if (navi_state == aerial_robot_navigation::ARM_OFF_STATE || navi_state == aerial_robot_navigation::STOP_STATE)
        rc_flight_state_ = RcFlightState::IDLE;
      break;
    }
  }

  rcPublishDebug();
}

double NamiController::rcDesiredForce(const RcMapper::Sticks& sticks) const
{
  const double roll = std::clamp(sticks.roll, -max_tilt_angle_, max_tilt_angle_);
  const double pitch = std::clamp(sticks.pitch, -max_tilt_angle_, max_tilt_angle_);
  return rc_thrust_mapper_.compensateTilt(rc_throttle_force_, roll, pitch);
}

void NamiController::rcManualSetpointUpdate()
{
  const RcMapper::Sticks& sticks = rc_mapper_->sticks();

  /* yaw: integrate the rate command into the yaw target consumed by the yaw PID */
  rc_target_yaw_ = angles::normalize_angle(rc_target_yaw_ + sticks.yaw_rate * ctrl_loop_du_);
  navigator_->setTargetYaw(rc_target_yaw_);
  navigator_->setTargetOmegaZ(sticks.yaw_rate);

  /* throttle -> collective force (slew limited) -> z acceleration in the dash frame,
     distributed over the rotors by the allocation around the real CoG */
  const double mass = nami_robot_model_->getMass();
  const double roll = std::clamp(sticks.roll, -max_tilt_angle_, max_tilt_angle_);
  const double pitch = std::clamp(sticks.pitch, -max_tilt_angle_, max_tilt_angle_);
  const double target_throttle_force = rc_thrust_mapper_.throttleToForce(sticks.throttle, mass);
  const double max_force_step = rc_force_slew_rate_ * ctrl_loop_du_;
  rc_throttle_force_ +=
      std::clamp(target_throttle_force - rc_throttle_force_, -max_force_step, max_force_step);

  // Match ArduPilot's ordering: angle boost follows pilot-throttle filtering so it reacts immediately to tilt.
  rc_output_force_ = rcDesiredForce(sticks);
  const double acc_z = std::max(rc_min_acc_z_, rc_output_force_ / mass);

  /* stick angles -> lateral acceleration. Tilt compensation is feed-forward only:
     it preserves vertical thrust near hover without altitude or position feedback. */
  rc_manual_acc_dash_.setValue(std::tan(pitch) * acc_z, -std::tan(roll) * acc_z / std::max(std::cos(pitch), 1e-3),
                               acc_z);
}

void NamiController::rcPublishDebug()
{
  std_msgs::Float32MultiArray msg;
  const RcMapper::Sticks& sticks = rc_mapper_->sticks();
  msg.data = { static_cast<float>(sticks.roll),
               static_cast<float>(sticks.pitch),
               static_cast<float>(sticks.yaw_rate),
               static_cast<float>(sticks.throttle),
               static_cast<float>(rc_output_force_),
               static_cast<float>(rc_mapper_->armSwitch()),
               static_cast<float>(rc_mapper_->killLatched()),
               static_cast<float>(rc_mapper_->takeoffEnable()),
               static_cast<float>(rc_mapper_->linkValid()),
               static_cast<float>(rc_flight_state_),
               static_cast<float>(rc_throttle_force_) };
  rc_debug_pub_.publish(msg);
}

void NamiController::setAttitudeGains()
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
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::NamiController, aerial_robot_control::ControlBase);
