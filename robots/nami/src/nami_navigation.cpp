// -*- mode: c++ -*-

#include <nami/nami_navigation.h>

#include <algorithm>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

NamiNavigator::NamiNavigator()
  : BaseNavigator(),
    eq_cog_world_(false),
    underwater_mode_(false),
    target_depth_(0),
    current_depth_(0),
    depth_sensor_ready_(false),
    underwater_flight_setup_done_(false),
    underwater_cmd_stamp_(0)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void NamiNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);  // to spinal
  final_target_baselink_rot_sub_ =
      nh_.subscribe("final_target_baselink_rot", 1, &NamiNavigator::targetBaselinkRotCallback, this);
  final_target_baselink_rpy_sub_ =
      nh_.subscribe("final_target_baselink_rpy", 1, &NamiNavigator::targetBaselinkRPYCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();

  if (underwater_mode_)
  {
    underwater_cmd_sub_ = nh_.subscribe("underwater/cmd_vel", 1, &NamiNavigator::underwaterCmdCallback, this);
    pressure_sub_ = nh_.subscribe("pressure", 1, &NamiNavigator::pressureCallback, this);
    depth_sub_ = nh_.subscribe("depth_sensor_node/depth", 1, &NamiNavigator::depthCallback, this);
    target_depth_pub_ = nh_.advertise<std_msgs::Float64>("underwater/target_depth", 1);
    ROS_INFO("[nami] UNDERWATER mode: teleop on underwater/cmd_vel, depth from pressure / depth_sensor_node/depth");
  }
}

void NamiNavigator::update()
{
  if (underwater_mode_)
    underwaterStateProcess();

  BaseNavigator::update();
  baselinkRotationProcess();
}

void NamiNavigator::underwaterStateProcess()
{
  const uint8_t state = getNaviState();
  const double now = ros::Time::now().toSec();
  const bool underwater_flight = state == TAKEOFF_STATE || state == HOVER_STATE;

  /* x/y is open-loop underwater, so a lost teleop publisher must not leave its
     last non-zero acceleration latched. Outside flight, clear it immediately;
     during flight, clear it when the command stream times out. Keep depth and
     yaw targets unchanged so the vehicle continues holding them safely. */
  if (!underwater_flight)
  {
    setTargetAccX(0);
    setTargetAccY(0);
    underwater_cmd_stamp_ = 0;
  }
  else if (underwater_cmd_stamp_ > 0 &&
           (now < underwater_cmd_stamp_ || now - underwater_cmd_stamp_ > underwater_cmd_timeout_))
  {
    setTargetAccX(0);
    setTargetAccY(0);
    underwater_cmd_stamp_ = 0;  // warn once per interrupted command stream
    ROS_WARN("[nami] underwater command timed out; cleared open-loop xy acceleration");
  }

  /* no position-based takeoff underwater: anchor the takeoff target z to the current z
     (keeps the base-class convergence check quiet; the actual transition below is
     managed explicitly by time) */
  if (state == START_STATE || state == ARM_ON_STATE || state == TAKEOFF_STATE)
    setTargetPosZ(estimator_->getPos(Frame::COG, estimate_mode_).z());

  if (state == TAKEOFF_STATE || state == HOVER_STATE)
  {
    if (!underwater_flight_setup_done_)
    {
      /* enable roll/pitch integration on spinal right away: underwater there is
         no takeoff climb, so the height-based trigger in PoseLinearController never fires */
      spinal::FlightConfigCmd cmd;
      cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
      getFlightConfigPublisher().publish(cmd);

      /* hold the current depth as the initial target */
      if (depth_sensor_ready_)
        target_depth_ = current_depth_;

      underwater_flight_setup_done_ = true;
    }
  }
  else
  {
    underwater_flight_setup_done_ = false;
  }

  /* explicit TAKEOFF -> HOVER transition with a guaranteed minimum TAKEOFF window:
     the controller activates only while it observes TAKEOFF_STATE (ControlBase::update
     sets control_timestamp_ there), so an instantaneous transition can leave the
     controller inactive forever (no four_axes/command at all) */
  if (state == TAKEOFF_STATE)
  {
    /* keep the base-class convergence timer reset so BaseNavigator::update() never
       transitions on its own (possibly instantly, from a stale timer) */
    hover_convergent_start_time_ = now;

    if (takeoff_enter_stamp_ < 0)
      takeoff_enter_stamp_ = now;
    if (now - takeoff_enter_stamp_ > 1.0)
    {
      setNaviState(HOVER_STATE);
      ROS_INFO("[nami] underwater: depth hold engaged (HOVER state)");
    }
  }
  else
  {
    takeoff_enter_stamp_ = -1;
  }

  /* xy has no position feedback underwater: teleop acc feedforward only.
     also pin prev mode so BaseNavigator::update() does not switch back to POS
     when the estimator (e.g. ground truth in sim) provides x/y states */
  xy_control_mode_ = ACC_CONTROL_MODE;
  prev_xy_control_mode_ = ACC_CONTROL_MODE;

  /* debug: expose the depth-hold target for logging / plotting */
  std_msgs::Float64 target_depth_msg;
  target_depth_msg.data = target_depth_;
  target_depth_pub_.publish(target_depth_msg);
}

void NamiNavigator::underwaterCmdCallback(const geometry_msgs::TwistConstPtr& msg)
{
  if (!underwater_mode_)
    return;

  const double now = ros::Time::now().toSec();
  double dt = now - underwater_cmd_stamp_;
  underwater_cmd_stamp_ = now;
  if (dt < 0 || dt > 0.5)
    dt = 0;  // first message or a stale stream: no increment jump

  /* forward/lateral: body-frame stick -> world-frame acc feedforward (open loop, ArduSub style) */
  const double yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
  tf::Vector3 acc_w =
      frameConversion(tf::Vector3(msg->linear.x * teleop_acc_gain_, msg->linear.y * teleop_acc_gain_, 0), yaw);
  setTargetAccX(acc_w.x());
  setTargetAccY(acc_w.y());

  /* dive/surface: increment the target depth (clamped to [-max_depth, 0]) */
  target_depth_ += msg->linear.z * max_dive_rate_ * dt;
  target_depth_ = std::max(-max_depth_, std::min(0.0, target_depth_));

  /* yaw: stick rate -> target yaw increment */
  if (msg->angular.z != 0)
    addTargetYaw(msg->angular.z * teleop_yaw_rate_gain_ * dt);
}

void NamiNavigator::pressureCallback(const sensor_msgs::FluidPressureConstPtr& msg)
{
  /* uuv SubseaPressure publishes [kPa]; recover signed z (z<0 underwater) */
  current_depth_ = -(msg->fluid_pressure - standard_pressure_kpa_) / kpa_per_meter_;
  depth_sensor_ready_ = true;
}

void NamiNavigator::depthCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  /* ms5837 node publishes point.z as signed z (upward positive) */
  current_depth_ = msg->point.z;
  depth_sensor_ready_ = true;
}

void NamiNavigator::reset()
{
  BaseNavigator::reset();

  // reset underwater states
  underwater_flight_setup_done_ = false;
  underwater_cmd_stamp_ = 0;
  setTargetAccX(0);
  setTargetAccY(0);
  target_depth_ = depth_sensor_ready_ ? current_depth_ : 0;

  // reset SO3
  eq_cog_world_ = false;
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);
}

void NamiNavigator::targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity

  // special process
  if (getTargetRPY().z() != 0)
  {
    curr_target_baselink_rot_.setRPY(0, 0, getTargetRPY().z());
    eq_cog_world_ = true;
  }
}

void NamiNavigator::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity
}

void NamiNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  BaseNavigator::naviCallback(msg);
  if (msg->roll_nav_mode == 2)
    setTargetRoll(msg->target_roll);
  if (msg->pitch_nav_mode == 2)
    setTargetPitch(msg->target_pitch);
}

void NamiNavigator::baselinkRotationProcess()
{
  if (curr_target_baselink_rot_ == final_target_baselink_rot_)
    return;

  if (ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
  {
    tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
    double angle = delta_q.getAngle();
    if (angle > M_PI)
      angle -= 2 * M_PI;

    if (fabs(angle) > baselink_rot_change_thresh_)
    {
      curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
    }
    else
      curr_target_baselink_rot_ = final_target_baselink_rot_;

    KDL::Rotation rot;
    tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
    robot_model_->setCogDesireOrientation(rot);

    // send to spinal
    spinal::DesireCoord msg;
    double r, p, y;
    tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
    msg.roll = r;
    msg.pitch = p;
    msg.yaw = y;
    target_baselink_rpy_pub_.publish(msg);

    prev_rotation_stamp_ = ros::Time::now().toSec();
  }
}

void NamiNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_,
                   0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_,
                   0.1);  // the rate to pub baselink rotation command

  getParam<bool>(nh_, "underwater_mode", underwater_mode_, false);

  ros::NodeHandle uw_nh(nh_, "underwater");
  getParam<double>(uw_nh, "teleop_acc_gain", teleop_acc_gain_, 1.0);
  getParam<double>(uw_nh, "teleop_yaw_rate_gain", teleop_yaw_rate_gain_, 0.5);
  getParam<double>(uw_nh, "cmd_timeout", underwater_cmd_timeout_, 0.5);
  if (underwater_cmd_timeout_ <= 0)
  {
    ROS_WARN("[nami] underwater/cmd_timeout must be positive; using 0.5 s");
    underwater_cmd_timeout_ = 0.5;
  }
  getParam<double>(uw_nh, "max_dive_rate", max_dive_rate_, 0.3);
  getParam<double>(uw_nh, "max_depth", max_depth_, 5.0);
  getParam<double>(uw_nh, "standard_pressure_kpa", standard_pressure_kpa_, 101.325);
  getParam<double>(uw_nh, "kpa_per_meter", kpa_per_meter_, 9.80638);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::NamiNavigator, aerial_robot_navigation::BaseNavigator);
