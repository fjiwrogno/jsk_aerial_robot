#include <nami/control/nami_controller.h>

#include <algorithm>
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
  nami_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::NamiNavigator>(navigator);

  NamiController::rosParamInit();

  if (current_mode_ == UNDERWATER)
  {
    if (!nami_navigator_)
      ROS_ERROR("[nami] underwater mode needs NamiNavigator (depth / target depth interface)");
    loadUnderwaterGains();
    last_surge_allocation_ = surgeAllocation();
    ROS_INFO("[nami] controller starts in UNDERWATER mode: motor 0~%d masked, motor %d~%d active",
             aerial_motor_num_ - 1, aerial_motor_num_, motor_num_ - 1);
    if (surgeAllocation())
      ROS_INFO("[nami] surge allocation mode: rows (Fx, Fz, roll, yaw); pitch passive, "
               "lateral stick ignored, surge acc clamped to +-%.2f m/s^2", surge_acc_limit_);
  }

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

  bool underwater_mode;
  getParam<bool>(nh_, "underwater_mode", underwater_mode, false);
  current_mode_ = underwater_mode ? UNDERWATER : AERIAL;
  getParam<int>(control_nh, "aerial_motor_num", aerial_motor_num_, 4);

  if (current_mode_ == UNDERWATER && !gimbal_calc_in_fc_)
  {
    /* the gimbal_calc_in_fc=false path sends target_full_thrust_ (a norm, always
       non-negative) and would drop the sign of the bi-directional aquatic thrust */
    ROS_WARN("[nami] underwater mode forces gimbal_calc_in_fc=true (signed base thrust needed)");
    gimbal_calc_in_fc_ = true;
  }

  ros::NodeHandle uw_nh(nh_, "underwater");
  getParam<double>(uw_nh, "surge_acc_limit", surge_acc_limit_, 1.0);
  getParam<double>(uw_nh, "depth_p_gain", depth_p_gain_, 20.0);
  getParam<double>(uw_nh, "depth_i_gain", depth_i_gain_, 0.0);
  getParam<double>(uw_nh, "depth_d_gain", depth_d_gain_, 0.0);
  getParam<double>(uw_nh, "depth_i_limit", depth_i_limit_, 2.0);
  getParam<double>(uw_nh, "depth_hover_thrust", depth_hover_thrust_, -0.4);
  getParam<double>(uw_nh, "depth_thrust_limit", depth_thrust_limit_, 10.0);
  getParam<double>(uw_nh, "depth_d_lpf_rate", depth_d_lpf_rate_, 0.6);
}

void NamiController::loadUnderwaterGains()
{
  /* underwater gain set from the underwater/controller/* namespace (flamingo
     test/cross_domain convention); each gain falls back to the aerial value if absent */
  ros::NodeHandle uw_control_nh(nh_, "underwater/controller");
  auto loadGains = [&](const std::string& ns, const std::vector<int>& axes) {
    ros::NodeHandle gain_nh(uw_control_nh, ns);
    for (int axis : axes)
    {
      double p, i, d;
      getParam<double>(gain_nh, "p_gain", p, pid_controllers_.at(axis).getPGain());
      getParam<double>(gain_nh, "i_gain", i, pid_controllers_.at(axis).getIGain());
      getParam<double>(gain_nh, "d_gain", d, pid_controllers_.at(axis).getDGain());
      pid_controllers_.at(axis).setGains(p, i, d);
      pid_controllers_.at(axis).reset();
    }
  };
  loadGains("xy", {X, Y});
  loadGains("z", {Z});
  /* Keep roll_pitch as the backward-compatible common default, then allow
     either axis to override it for asymmetric underwater dynamics/tuning. */
  loadGains("roll_pitch", {ROLL, PITCH});
  loadGains("roll", {ROLL});
  loadGains("pitch", {PITCH});
  loadGains("yaw", {YAW});
  /* the new attitude gains reach spinal via setAttitudeGains() inside reset() on activation */
}

double NamiController::depthControlLoop()
{
  /* hand-written depth PID (flamingo dev/aquatic_flamingo skeleton, signed z convention):
     returns the desired total vertical force [N] in the world frame (negative = downward).
     depth_hover_thrust_ trims the residual buoyancy (nami is slightly positive buoyant). */
  if (!nami_navigator_ || !nami_navigator_->getDepthSensorReady())
    return 0;  // no depth feedback yet: neutral output

  const double now = ros::Time::now().toSec();
  double dt = (depth_prev_stamp_ > 0) ? (now - depth_prev_stamp_) : 0;
  depth_prev_stamp_ = now;

  const double err = nami_navigator_->getTargetDepth() - nami_navigator_->getCurrentDepth();

  if (dt > 0 && dt < 0.2)
  {
    depth_err_i_ += err * dt;
    depth_err_i_ = std::max(-depth_i_limit_, std::min(depth_i_limit_, depth_err_i_));
    const double err_d = (err - depth_prev_err_) / dt;
    depth_err_d_filtered_ = depth_d_lpf_rate_ * depth_err_d_filtered_ + (1 - depth_d_lpf_rate_) * err_d;
  }
  depth_prev_err_ = err;

  double total_thrust = depth_hover_thrust_ + depth_p_gain_ * err + depth_i_gain_ * depth_err_i_ +
                        depth_d_gain_ * depth_err_d_filtered_;

  /* tilt compensation: keep the vertical force component while the body tilts */
  const double tilt_factor = 1.0 / std::max(0.7, cos(rpy_.x()) * cos(rpy_.y()));
  total_thrust *= tilt_factor;

  /* symmetric clamp: bi-directional aquatic rotors can push both up and down */
  total_thrust = std::max(-depth_thrust_limit_, std::min(depth_thrust_limit_, total_thrust));

  return total_thrust;
}

bool NamiController::update()
{
  if (current_mode_ == UNDERWATER && surgeAllocation() != last_surge_allocation_)
  {
    last_surge_allocation_ = surgeAllocation();
    pid_controllers_.at(ROLL).reset();
    pid_controllers_.at(PITCH).reset();
    setAttitudeGains();
    ROS_WARN("[nami] applied %s allocation and refreshed spinal attitude gains",
             last_surge_allocation_ ? "FORWARD" : "ATTITUDE+DEPTH");
  }
  sendGimbalCommand();
  if (gimbal_calc_in_fc_)
  {
    std_msgs::UInt8 msg;
    msg.data = gimbal_dof_;
    gimbal_dof_pub_.publish(msg);
  }

  bool active = PoseLinearController::update();

  /* underwater, the controller only becomes active at TAKEOFF; before that
     (ARM_ON, armed but not flying) no four_axes/command is published, so spinal
     drives every armed motor to its idle throttle -> the air rotors (0~3) would
     spin up in the water. Publish the masked idle command while armed-but-inactive
     so the air group is held at armed-stop (mask -> IDLE_DUTY) and the aquatic
     rotors stay at the 1.5 ms neutral pulse. */
  if (!active && current_mode_ == UNDERWATER &&
      navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE)
    sendUnderwaterIdleCommand();

  return active;
}

void NamiController::sendUnderwaterIdleCommand()
{
  for (int i = 0; i < motor_num_; i++)
    for (int j = 0; j < rotor_coef_; j++)
      target_base_thrust_.at(rotor_coef_ * i + j) = isMotorActive(i) ? 0.0f : MASKED_MOTOR_THRUST;
  target_roll_ = 0;
  target_pitch_ = 0;
  candidate_yaw_term_ = 0;
  sendFourAxisCommand();
}

void NamiController::controlCore()
{
  PoseLinearController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  if (current_mode_ == UNDERWATER)
  {
    /* z: replace the estimator-based Z PID with the depth loop (only IMU + depth
       sensor feedback underwater); x/y results are the pure teleop acc feedforward
       since the navigator pins ACC_CONTROL_MODE */
    target_acc_w.setZ(depthControlLoop() / nami_robot_model_->getMass());
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
    /* mode masking: the inactive motor group (aquatic in AERIAL, aerial in UNDERWATER)
       gets a zero column so the pseudo-inverse allocates it no force at all */
    if (!isMotorActive(i))
    {
      masked_rot.push_back(Eigen::MatrixXd::Zero(3, rotor_coef_));
      continue;
    }

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
    if (surgeAllocation())
    {
      /* surge mode: allocate (Fx, Fz, tau_x, tau_z). the aquatic thrust axes span
         the body x-z plane, so (Fx, Fz, tau_y) share only two DoFs (front/rear pair
         sums) and one of the three must be dependent -> drop tau_y (pitch rights
         itself on the CoB-above-CoG buoyancy moment) and regulate Fx instead of
         leaving it free-running (the cause of the constant forward drift) */
      Eigen::VectorXd wrench(4);
      wrench << std::max(-surge_acc_limit_, std::min(surge_acc_limit_, target_wrench_acc_cog(0))),
          target_wrench_acc_cog(2), target_wrench_acc_cog(3), target_wrench_acc_cog(5);
      Eigen::MatrixXd map(4, integrated_map.cols());
      map.row(0) = integrated_map.row(0);  // Fx
      map.row(1) = integrated_map.row(2);  // Fz
      map.row(2) = integrated_map.row(3);  // tau_x
      map.row(3) = integrated_map.row(5);  // tau_z
      target_wrench_acc_cog = wrench;
      integrated_map = map;
    }
    else
    {
      target_wrench_acc_cog = target_wrench_acc_cog.tail(4);  // z, roll, pitch, yaw
      integrated_map = integrated_map.bottomRows(4);          // z, roll, pitch, yaw
    }
  }

  /* vectoring force mapping */
  Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
  if (surgeAllocation())
  {
    integrated_map_inv_trans_ = integrated_map_inv.leftCols(2);  // Fx, Fz
    /* spinal expects the torque allocation columns in (roll, pitch, yaw) order;
       pitch has no allocation channel in this mode -> zero column (its spinal
       gains are also zeroed in setAttitudeGains) */
    Eigen::MatrixXd rot = Eigen::MatrixXd::Zero(integrated_map_inv.rows(), 3);
    rot.col(0) = integrated_map_inv.col(2);
    rot.col(2) = integrated_map_inv.col(3);
    integrated_map_inv_rot_ = rot;
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.head(2);
    target_vectoring_f_rot_ = integrated_map_inv.rightCols(2) * target_wrench_acc_cog.tail(2);
  }
  else
  {
    integrated_map_inv_trans_ = integrated_map_inv.leftCols(underactuate_ ? 1 : 3);
    integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
    if (underactuate_)
      target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
    else
      target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
    target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3);  // debug
  }
  last_col = 0;

  /* under actuated axis  */
  if (underactuate_)
  {
    if (surgeAllocation())
    {
      /* surge mode: forward motion comes from the Fx allocation row, not from a
         pitch tilt; sway has no thrust authority at all (axes span x-z only), so
         the lateral stick is ignored instead of commanding a useless (reversed-
         direction) roll. both attitude targets stay level. */
      target_roll_ = 0;
      target_pitch_ = 0;
      navigator_->setTargetRoll(0);
      navigator_->setTargetPitch(0);
    }
    /* underwater: always use the hovering approximation. the exact atan2 mapping
       diverges to +-90 deg when target_acc_dash.z ~ 0 (neutral buoyancy) */
    else if (hovering_approximate_ || current_mode_ == UNDERWATER)
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
    else if (gimbal_dof_ == 0)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];  // rotor_coef_ == 1: scalar thrust per rotor
    }
    /* yaw column: index 3 both in the legacy layout (z, roll, pitch, yaw) and in
       the surge layout (Fx, Fz, roll, yaw) */
    if (integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW)) > max_yaw_scale)
      max_yaw_scale = integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW));  // underactuated: yaw col is shifted

    last_col += rotor_coef_;
  }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  /* overwrite the inactive motor group with the mask thrust: passes the spinal yaw
     saturation gate (fabs > 0) but is judged as disabled by the pwm conversion */
  for (int i = 0; i < motor_num_; i++)
  {
    if (!isMotorActive(i))
    {
      for (int j = 0; j < rotor_coef_; j++)
        target_base_thrust_.at(rotor_coef_ * i + j) = MASKED_MOTOR_THRUST;
    }
  }

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

  /* never publish a non-finite command: a NaN/inf here propagates into every motor
     thrust on spinal (observed as physics blow-up in simulation). skip this cycle
     (spinal keeps the previous command; its 500ms timeout is the final fallback)
     and dump the yaw-term decomposition to locate the numerical source. */
  bool corrupted = !std::isfinite(flight_command_data.angles[0]) ||
                   !std::isfinite(flight_command_data.angles[1]) ||
                   !std::isfinite(flight_command_data.angles[2]);
  for (const auto& t : flight_command_data.base_thrust)
    corrupted = corrupted || !std::isfinite(t);
  if (corrupted)
  {
    ROS_ERROR_THROTTLE(1.0,
                       "[nami] non-finite four-axis command, skipped. angles=(%f %f %f) "
                       "yaw pid: p=%f i=%f d=%f result=%f target_yaw=%f yaw=%f",
                       flight_command_data.angles[0], flight_command_data.angles[1],
                       flight_command_data.angles[2], pid_controllers_.at(YAW).getPTerm(),
                       pid_controllers_.at(YAW).getITerm(), pid_controllers_.at(YAW).getDTerm(),
                       pid_controllers_.at(YAW).result(), target_rpy_.z(), rpy_.z());
    return;
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
  const double alloc_max = torque_allocation_matrix_inv.cwiseAbs().maxCoeff();
  if (!std::isfinite(alloc_max) || alloc_max > INT16_MAX * 0.001f)
  {
    /* a non-finite/overflowed matrix (e.g. near-singular allocation pseudo-inverse)
       must not reach the int16 rosserial fields; keep the previous matrix on spinal */
    ROS_ERROR_THROTTLE(1.0, "Torque Allocation Matrix overflow (max %f), not published", alloc_max);
    return;
  }
  for (unsigned int i = 0; i < motor_num_ * rotor_coef_; i++)
  {
    torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
  }
  torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
}

void NamiController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000; the msg fields are int16,
     so any gain above 32.7 would silently overflow into a NEGATIVE gain on spinal
     (positive feedback -> violent divergence). Clamp and complain instead. */
  auto scaled_gain = [](double gain) {
    double v = gain * 1000;
    if (std::abs(v) > INT16_MAX)
    {
      ROS_ERROR("attitude gain %.1f overflows the int16 rosserial field (max 32.7), clamped", gain);
      v = std::copysign(INT16_MAX, v);
    }
    return static_cast<int16_t>(v);
  };
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = scaled_gain(pid_controllers_.at(ROLL).getPGain());
  rpy_gain_msg.motors.at(0).roll_i = scaled_gain(pid_controllers_.at(ROLL).getIGain());
  rpy_gain_msg.motors.at(0).roll_d = scaled_gain(pid_controllers_.at(ROLL).getDGain());
  /* surge allocation mode has no tau_y channel: silence the spinal pitch PID so
     its integrator does not wind up against an actuator that no longer exists */
  const bool no_pitch_channel = surgeAllocation();
  rpy_gain_msg.motors.at(0).pitch_p = no_pitch_channel ? 0 : scaled_gain(pid_controllers_.at(PITCH).getPGain());
  rpy_gain_msg.motors.at(0).pitch_i = no_pitch_channel ? 0 : scaled_gain(pid_controllers_.at(PITCH).getIGain());
  rpy_gain_msg.motors.at(0).pitch_d = no_pitch_channel ? 0 : scaled_gain(pid_controllers_.at(PITCH).getDGain());
  rpy_gain_msg.motors.at(0).yaw_d = scaled_gain(pid_controllers_.at(YAW).getDGain());
  rpy_gain_pub_.publish(rpy_gain_msg);
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::NamiController, aerial_robot_control::ControlBase);
