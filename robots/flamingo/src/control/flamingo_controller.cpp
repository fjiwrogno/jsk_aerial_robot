#include <flamingo/control/flamingo_controller.h>
#include <sensor_msgs/Joy.h>
// ADDED: Include the joy_parser utility
#include <aerial_robot_control/util/joy_parser.h>
#include <deque>
#include <algorithm>
#include <spinal/DesireCoord.h> 

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
  debug_rpy_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("debug/current_rpy", 1);

  desire_coord_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
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
  getParam<bool>(control_nh, "underactuate", underactuate_, true);
  getParam<double>(control_nh, "joy_yaw_rate", joy_yaw_rate_, 0.01);
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

    //for underwater motion, the whole robot will be flipped back which measn the roll angle will be 180deg/-180deg
    // for faciliating target_roll control, here set desire coordinate to flip the robot back to 0deg roll angle 
    spinal::DesireCoord coord_msg;
    coord_msg.roll = 0; // PI
    coord_msg.pitch = 0;
    coord_msg.yaw = 0;
    desire_coord_pub_.publish(coord_msg);
  }

  return PoseLinearController::update();
}

void FlamingoController::controlCore()
{
    PoseLinearController::controlCore();
    double pid_depth_w = 1.0 * depthControlLoop() / flamingo_robot_model_->getMass();  
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
    
    // FIX: 因为旋翼产生向下的推力，力和力矩的映射都必须乘以 -1.0
    double thrust_dir = 1.0; 
    wrench_map.block(0, 0, 3, 3) = thrust_dir * Eigen::MatrixXd::Identity(3, 3);
    int last_col = 0;

    /* calculate normal allocation */
    for (int i = 0; i < motor_num_; i++)
    {
      // FIX: 力矩 = r x F = r x (thrust_dir * f) = thrust_dir * (r x f)
      wrench_map.block(3, 0, 3, 3) = thrust_dir * aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                    thrust_dir * rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
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
  float joy_stick_deadzone_ = 0.2;

  if(joy_cmd.buttons[JOY_BUTTON_STOP] == 1)
  {
    navigator_->setNaviState(aerial_robot_navigation::STOP_STATE);
    /* update the target pos(maybe not necessary) */
    // navigator_->setTargetXyFromCurrentState();
    double current_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
    navigator_->setTargetYaw(current_yaw);
  }
  
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


  // Stabilize toggle: D-pad Left
  if(joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] == 1)
  {
    if (!stablize_button_pressed_)
    {
      if_stablize_ = !if_stablize_;
      stablize_button_pressed_ = true;

      if(if_stablize_)
        ROS_ERROR("STABLIZED MODE IS ON!");
      else
        ROS_ERROR("MANUAL MODE IS ON!");
    }
  }
  else
  {
    stablize_button_pressed_ = false;
  }
  
  // Dive toggle: D-pad Right
  if(joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] == 1)
  {
    if (!dive_button_pressed_)
    {
      if_dive_ = !if_dive_;
      dive_button_pressed_ = true;

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
  }
  else
  {
    dive_button_pressed_ = false;
  }
  // speed mode switch
  if (joy_cmd.buttons[JOY_BUTTON_CROSS_UP] == 1)
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
      mapped_total_thrust = 5.0;
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
      raw_target_pitch = -current_max_pitch;
    }
    else if (joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] < -joy_stick_deadzone_)
    {
      raw_target_pitch = current_max_pitch;
    }
    else
    {
      raw_target_pitch = estimator_->getEuler(Frame::COG, estimate_mode_).y();
    } 
  
    target_pitch_ = raw_target_pitch;
    
    double raw_target_roll = 0.0;
    float max_roll_angle = 0.31; // ~30 degrees
    if(fabs(joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS]) > joy_stick_deadzone_)
    {
      raw_target_roll = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] * max_roll_angle;
    }
    else
    {
      raw_target_roll = estimator_->getEuler(Frame::COG, estimate_mode_).x();
    }

    target_roll_ = raw_target_roll;
    
  }
  else
  {

    double current_max_pitch = PITCH_LIMIT_LOW;
    switch(current_speed_mode_) {
      case SPEED_LOW:    current_max_pitch = PITCH_LIMIT_LOW; break;
      case SPEED_MEDIUM: current_max_pitch = PITCH_LIMIT_MED; break;
      case SPEED_HIGH:   current_max_pitch = PITCH_LIMIT_HIGH; break;
    }
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
      target_roll_ = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] * max_roll_angle; 
    }
    else
    {
      // becasue the initial rool angle is 3.14rad so currently set this value to 3.14
      target_roll_ = 0;
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


void FlamingoController::setSafeAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = 0;
  rpy_gain_msg.motors.at(0).roll_i = 0;
  rpy_gain_msg.motors.at(0).roll_d = 0;
  rpy_gain_msg.motors.at(0).pitch_p = 0;
  rpy_gain_msg.motors.at(0).pitch_i = 0;
  rpy_gain_msg.motors.at(0).pitch_d = 0;
  rpy_gain_msg.motors.at(0).yaw_d = 0;
  rpy_gain_pub_.publish(rpy_gain_msg);

}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::FlamingoController, aerial_robot_control::ControlBase);
