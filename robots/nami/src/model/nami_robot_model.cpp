#include <nami/model/nami_robot_model.h>

#include <algorithm>
#include <cmath>

NamiRobotModel::NamiRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre,
                                             double epsilon)
  : RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  ros::NodeHandle nh;
  nh.param("stability/allocation_rank_relative_threshold", allocation_rank_relative_threshold_, 1.0e-6);
  nh.param("stability/allocation_min_scaled_singular_value", allocation_min_scaled_singular_value_, 0.05);
  nh.param("stability/minimum_static_thrust_reserve", minimum_static_thrust_reserve_, 0.0);

  const int rotor_num = getRotorNum();

  links_rotation_from_cog_.resize(rotor_num);
  thrust_coords_rot_.resize(rotor_num);
  updateStabilityMetrics();
}

void NamiRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  // JointState messages contain the actuated transformation joints but not
  // passive rotor joints. Sanitize the KDL array here so the Nami plugin never
  // consumes unspecified entries from the generic conversion path.
  KDL::JntArray nami_joint_positions(getTree().getNrOfJoints());
  KDL::SetToZero(nami_joint_positions);
  for (const auto& joint : getJointIndexMap())
  {
    if (joint.second < joint_positions.rows())
      nami_joint_positions(joint.second) = joint_positions(joint.second);
  }

  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  // /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(nami_joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  transformable::RobotModel::updateRobotModelImpl(nami_joint_positions);
  const auto seg_tf_map = getSegmentsTf();

  /* get local coords of thrust links */
  for (int i = 0; i < getRotorNum(); ++i)
  {
    std::string thrust = "rotor_arm" + std::to_string(i + 1);
    KDL::Frame f;
    fk_solver.JntToCart(nami_joint_positions, f, thrust);
    thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;
  }

  updateStabilityMetrics();
}

void NamiRobotModel::updateStabilityMetrics()
{
  StabilityMetrics metrics;
  const int rotor_num = getRotorNum();
  if (rotor_num < 4 || getMass() <= 0.0)
  {
    std::lock_guard<std::mutex> lock(stability_metrics_mutex_);
    stability_metrics_ = metrics;
    return;
  }

  const Eigen::MatrixXd wrench_matrix = calcWrenchMatrixOnCoG();
  const Eigen::Matrix3d inertia = getInertia<Eigen::Matrix3d>();
  if (wrench_matrix.rows() != 6 || wrench_matrix.cols() != rotor_num ||
      !wrench_matrix.allFinite() || !inertia.allFinite() ||
      std::abs(inertia.determinant()) < 1.0e-12)
  {
    std::lock_guard<std::mutex> lock(stability_metrics_mutex_);
    stability_metrics_ = metrics;
    return;
  }

  Eigen::MatrixXd allocation = Eigen::MatrixXd::Zero(4, rotor_num);
  allocation.row(0) = wrench_matrix.row(2) / getMass();
  allocation.bottomRows(3) = inertia.inverse() * wrench_matrix.bottomRows(3);

  // Row normalization makes the singular-value threshold independent of the
  // mixed translational/angular units. Actuator authority is checked
  // separately using the static-thrust reserve.
  for (int row = 0; row < allocation.rows(); ++row)
  {
    const double row_norm = allocation.row(row).norm();
    if (row_norm <= std::numeric_limits<double>::epsilon())
    {
      std::lock_guard<std::mutex> lock(stability_metrics_mutex_);
      stability_metrics_ = metrics;
      return;
    }
    allocation.row(row) /= row_norm;
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(allocation);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() != 4 || !singular_values.allFinite())
  {
    std::lock_guard<std::mutex> lock(stability_metrics_mutex_);
    stability_metrics_ = metrics;
    return;
  }

  const double max_singular_value = singular_values.maxCoeff();
  const double rank_threshold = allocation_rank_relative_threshold_ * max_singular_value;
  metrics.allocation_rank = (singular_values.array() > rank_threshold).count();
  metrics.scaled_min_singular_value = singular_values.minCoeff();
  if (metrics.scaled_min_singular_value > 0.0)
    metrics.scaled_condition_number = max_singular_value / metrics.scaled_min_singular_value;

  metrics.static_thrust = getStaticThrust();
  if (metrics.static_thrust.size() == rotor_num && metrics.static_thrust.allFinite())
  {
    const double lower_reserve =
      (metrics.static_thrust.array() - getThrustLowerLimit()).minCoeff();
    const double upper_reserve =
      (getThrustUpperLimit() - metrics.static_thrust.array()).minCoeff();
    metrics.minimum_static_thrust_reserve = std::min(lower_reserve, upper_reserve);
    metrics.valid = true;
  }

  std::lock_guard<std::mutex> lock(stability_metrics_mutex_);
  stability_metrics_ = metrics;
}

NamiRobotModel::StabilityMetrics NamiRobotModel::getStabilityMetrics() const
{
  std::lock_guard<std::mutex> lock(stability_metrics_mutex_);
  return stability_metrics_;
}

bool NamiRobotModel::stabilityCheck(bool verbose)
{
  const StabilityMetrics metrics = getStabilityMetrics();
  if (!initialized() || !metrics.valid)
  {
    if (verbose) ROS_ERROR("Nami stability check: robot model metrics are not ready");
    return false;
  }

  if (metrics.allocation_rank < 4)
  {
    if (verbose)
      ROS_ERROR("Nami stability check: z/roll/pitch/yaw allocation rank is %d, expected 4",
                metrics.allocation_rank);
    return false;
  }

  if (metrics.scaled_min_singular_value < allocation_min_scaled_singular_value_)
  {
    if (verbose)
      ROS_ERROR("Nami stability check: scaled minimum singular value %.6f is below %.6f",
                metrics.scaled_min_singular_value, allocation_min_scaled_singular_value_);
    return false;
  }

  if (metrics.minimum_static_thrust_reserve < minimum_static_thrust_reserve_)
  {
    if (verbose)
      ROS_ERROR("Nami stability check: static thrust reserve %.3f N is below %.3f N",
                metrics.minimum_static_thrust_reserve, minimum_static_thrust_reserve_);
    return false;
  }

  return true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NamiRobotModel, aerial_robot_model::RobotModel);
