// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/under_actuated_lqi_controller.h>
#include <nami/model/nami_robot_model.h>
#include <std_msgs/Bool.h>

#include <atomic>
#include <mutex>

namespace aerial_robot_control
{
class NamiLQIController : public UnderActuatedLQIController
{
public:
  NamiLQIController();
  ~NamiLQIController() override;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;

protected:
  bool checkRobotModel() override;
  bool optimalGain() override;
  void clampGain() override;
  void publishGain() override;
  void controlCore() override;

private:
  void cfgNamiLQICallback(aerial_robot_control::LQIConfig& config, uint32_t level);

  boost::shared_ptr<NamiRobotModel> nami_robot_model_;
  ros::Publisher stability_pub_;
  std::recursive_mutex lqi_mutex_;
  std::atomic<bool> has_valid_gain_{false};
};
}  // namespace aerial_robot_control
