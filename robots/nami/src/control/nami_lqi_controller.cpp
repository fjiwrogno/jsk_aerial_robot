#include <nami/control/nami_lqi_controller.h>

namespace aerial_robot_control
{
NamiLQIController::NamiLQIController()
{
  realtime_update_ = false;
}

NamiLQIController::~NamiLQIController()
{
  // The base destructor joins this thread after derived members have already
  // been destroyed. Join it here while the Nami mutex and model pointer are
  // still alive, then prevent a second join in the base destructor.
  if (realtime_update_ && gain_generator_thread_.joinable())
  {
    gain_generator_thread_.join();
    realtime_update_ = false;
  }
}

void NamiLQIController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  nami_robot_model_ = boost::dynamic_pointer_cast<NamiRobotModel>(robot_model);
  UnderActuatedLQIController::initialize(
    nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  stability_pub_ = nh_.advertise<std_msgs::Bool>("model/stability", 1, true);

  // The generic LQI implementation has a gain-generation thread. Rebind the
  // dynamic-reconfigure callback so Nami's configuration and gain vectors are
  // protected by the same robot-specific lock.
  dynamic_reconf_func_lqi_ =
    boost::bind(&NamiLQIController::cfgNamiLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);
}

bool NamiLQIController::checkRobotModel()
{
  if (!nami_robot_model_)
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "LQI gain generator",
                             "Nami LQI controller requires NamiRobotModel");
    return false;
  }

  std_msgs::Bool stability_msg;
  stability_msg.data = nami_robot_model_->stabilityCheck(false);
  if (stability_pub_)
    stability_pub_.publish(stability_msg);

  lqi_mode_ = 4;
  return stability_msg.data || has_valid_gain_.load();
}

bool NamiLQIController::optimalGain()
{
  std::lock_guard<std::recursive_mutex> lock(lqi_mutex_);

  std_msgs::Bool stability_msg;
  stability_msg.data = nami_robot_model_ &&
                       nami_robot_model_->stabilityCheck(false);
  if (stability_pub_)
    stability_pub_.publish(stability_msg);

  // Once generated, keep publishing the last valid gains while an invalid
  // configuration is being rejected. Returning true avoids the generic gain
  // loop clearing them during flight.
  if (!stability_msg.data)
  {
    ROS_ERROR_THROTTLE_NAMED(1.0, "LQI gain generator",
                             "Nami LQI controller rejected the current configuration");
    return has_valid_gain_.load();
  }

  const bool solved = UnderActuatedLQIController::optimalGain();
  if (solved)
  {
    // Clamp in the same critical section as generation. The generic loop calls
    // clampGain() once more afterward, which is idempotent.
    UnderActuatedLQIController::clampGain();
    has_valid_gain_.store(true);
  }
  return solved;
}

void NamiLQIController::clampGain()
{
  std::lock_guard<std::recursive_mutex> lock(lqi_mutex_);
  UnderActuatedLQIController::clampGain();
}

void NamiLQIController::publishGain()
{
  std::lock_guard<std::recursive_mutex> lock(lqi_mutex_);
  UnderActuatedLQIController::publishGain();
}

void NamiLQIController::controlCore()
{
  std::lock_guard<std::recursive_mutex> lock(lqi_mutex_);
  UnderActuatedLQIController::controlCore();
}

void NamiLQIController::cfgNamiLQICallback(
    aerial_robot_control::LQIConfig& config, uint32_t level)
{
  std::lock_guard<std::recursive_mutex> lock(lqi_mutex_);
  UnderActuatedLQIController::cfgLQICallback(config, level);
}
}  // namespace aerial_robot_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::NamiLQIController,
                       aerial_robot_control::ControlBase);
