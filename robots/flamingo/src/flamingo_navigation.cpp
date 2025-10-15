// -*- mode: c++ -*-

#include <flamingo/flamingo_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

FlamingoNavigator::FlamingoNavigator() : BaseNavigator(), eq_cog_world_(false)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void FlamingoNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  flamingo_robot_model_ = boost::dynamic_pointer_cast<FlamingoRobotModel>(robot_model);
  robot_mode_ = ROBOT_MODE::AERIAL_MODE;
  robot_mode_pub_ = nh_.advertise<std_msgs::UInt8>("robot_mode", 1);
  robot_mode_sub_ = nh_.subscribe("robot_mode", 1, &FlamingoNavigator::robotModeCallback, this);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);  // to spinal
  final_target_baselink_rot_sub_ =
      nh_.subscribe("final_target_baselink_rot", 1, &FlamingoNavigator::targetBaselinkRotCallback, this);
  final_target_baselink_rpy_sub_ =
      nh_.subscribe("final_target_baselink_rpy", 1, &FlamingoNavigator::targetBaselinkRPYCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();
  updateRobotMode();
}

void FlamingoNavigator::joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg)
{
  BaseNavigator::joyStickControl(joy_msg);

  sensor_msgs::Joy joy_cmd = joyParse(*joy_msg);
  if (joy_cmd.buttons.size() == 0)
    return;

  if (joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] == 1 && joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] == 1)
  {
    switchRobotMode();
  }
}

void FlamingoNavigator::switchRobotMode()
{
  switch (robot_mode_)
  {
    case ROBOT_MODE::AERIAL_MODE: {
      robot_mode_ = ROBOT_MODE::UNDERWATER_MODE;
      ROS_INFO("Switch to UNDERWATER Mode");
      break;
    }
    case ROBOT_MODE::UNDERWATER_MODE: {
      robot_mode_ = ROBOT_MODE::UNDERWATER_MODE;
      ROS_INFO("Switch to UNDERWATER Mode");
      break;
    }
    default: {
      robot_mode_ = ROBOT_MODE::UNDERWATER_MODE;
      ROS_INFO("Switch to UNDERWATER Mode");
      break;
    }
  }
  updateRobotMode();
}

void FlamingoNavigator::updateRobotMode()
{
  std_msgs::UInt8 msg;
  msg.data = robot_mode_;
  robot_mode_pub_.publish(msg);
}

void FlamingoNavigator::update()
{
  BaseNavigator::update();
  baselinkRotationProcess();
}

void FlamingoNavigator::robotModeCallback(const std_msgs::UInt8ConstPtr& msg)
{
  if (msg->data == ROBOT_MODE::AERIAL_MODE)
  {
    flamingo_robot_model_->setRobotMode("air");
  }
  else if (msg->data == ROBOT_MODE::UNDERWATER_MODE)
  {
    flamingo_robot_model_->setRobotMode("water");
  }
}

void FlamingoNavigator::reset()
{
  BaseNavigator::reset();

  // reset SO3
  eq_cog_world_ = false;
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);
}

void FlamingoNavigator::targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg)
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

void FlamingoNavigator::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity
}

void FlamingoNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  BaseNavigator::naviCallback(msg);
  if (msg->roll_nav_mode == 2)
    setTargetRoll(msg->target_roll);
  if (msg->pitch_nav_mode == 2)
    setTargetPitch(msg->target_pitch);
}

void FlamingoNavigator::baselinkRotationProcess()
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

void FlamingoNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_,
                   0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_,
                   0.1);  // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::FlamingoNavigator, aerial_robot_navigation::BaseNavigator);
