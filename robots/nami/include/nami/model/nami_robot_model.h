// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <Eigen/SVD>
#include <limits>
#include <mutex>

using namespace aerial_robot_model;

class NamiRobotModel : public transformable::RobotModel
{
public:
  struct StabilityMetrics
  {
    bool valid{false};
    int allocation_rank{0};
    double scaled_min_singular_value{0.0};
    double scaled_condition_number{std::numeric_limits<double>::infinity()};
    double minimum_static_thrust_reserve{-std::numeric_limits<double>::infinity()};
    Eigen::VectorXd static_thrust;
  };

  NamiRobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_t_min_thre = 0,
                        double epsilon = 10);
  virtual ~NamiRobotModel() = default;

  StabilityMetrics getStabilityMetrics() const;
  bool stabilityCheck(bool verbose = true) override;

  template <class T>
  std::vector<T> getLinksRotationFromCog();
  template <class T>
  std::vector<T> getThrustCoordRot();

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  void updateStabilityMetrics();

  KDL::JntArray gimbal_processed_joint_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Rotation> thrust_coords_rot_;
  std::mutex links_rotation_mutex_;
  std::mutex thrust_rotation_mutex_;
  mutable std::mutex stability_metrics_mutex_;
  StabilityMetrics stability_metrics_;
  double allocation_rank_relative_threshold_{1.0e-6};
  double allocation_min_scaled_singular_value_{0.05};
  double minimum_static_thrust_reserve_{0.0};
};

template <>
inline std::vector<KDL::Rotation> NamiRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template <>
inline std::vector<KDL::Rotation> NamiRobotModel::getThrustCoordRot()
{
  std::lock_guard<std::mutex> lock(thrust_rotation_mutex_);
  return thrust_coords_rot_;
}
