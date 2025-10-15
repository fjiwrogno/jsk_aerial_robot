#include <flamingo/model/flamingo_robot_model.h>

FlamingoRobotModel::FlamingoRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon)
  : RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  ros::NodeHandle nhp("~");  // Private node handle
  nhp.param("robot_mode", robot_mode_, std::string("air"));

  full_rotor_num_ = RobotModel::getRotorNum();
  active_rotor_num_ = 2;
  active_rotor_indices_ = { 1, 2 };
  active_rotor_direction_ = { { 1, 1 }, { 2, -1 } };

  if (full_rotor_num_ != 4)
  {
    ROS_WARN("[FlamingoRobotModel] Expected 4 rotors from URDF, but found %d.", full_rotor_num_);
  }

  links_rotation_from_cog_.resize(getRotorNum());
  thrust_coords_rot_.resize(getRotorNum());
}

void FlamingoRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();

  transformable::RobotModel::updateRobotModelImpl(joint_positions);

  if (full_rotors_origin_from_cog_.empty() || full_rotors_origin_from_cog_.size() != full_rotor_num_)
  {
    full_rotors_origin_from_cog_ = RobotModel::getRotorsOriginFromCog<KDL::Vector>();
    full_rotors_normal_from_cog_ = RobotModel::getRotorsNormalFromCog<KDL::Vector>();
    full_rotor_direction_ = RobotModel::getRotorDirection();
  }

  // 1. Define the mapping from active rotor index (0, 1) to physical rotor index (0-3)
  active_rotor_indices_.clear();
  active_rotor_direction_.clear();
  /// TODO automative read the index from the yaml file
  // manually set the value of active rotor index assigned with index from urdf
  if (robot_mode_ == "water")
  {
    active_rotor_indices_ = { 3, 4 };  // Use physical rotors 3 and 4
  }
  else  // Default to "air" mode
  {
    active_rotor_indices_ = { 1, 2 };  // Use physical rotors 1 and 2
  }
  active_rotor_num_ = active_rotor_indices_.size();

  std::vector<KDL::Vector> active_rotors_origin, active_rotors_normal;
  for (size_t i = 0; i < active_rotor_indices_.size(); ++i)
  {
    int physical_index = active_rotor_indices_[i];
    if (physical_index <= full_rotors_origin_from_cog_.size())
    {
      active_rotors_origin.push_back(full_rotors_origin_from_cog_.at(physical_index - 1));
      active_rotors_normal.push_back(full_rotors_normal_from_cog_.at(physical_index - 1));
      active_rotor_direction_[i + 1] = full_rotor_direction_.at(physical_index);
      ROS_WARN("active_rotor_direction[%d] is: %d and physical index is %d", i + 1, active_rotor_direction_[i + 1],
               physical_index);
    }
  }

  setRotorsOriginFromCog(active_rotors_origin);
  setRotorsNormalFromCog(active_rotors_normal);

  for (int i = 0; i < getRotorNum(); ++i)
  {
    std::string thrust = "rotor_arm" + std::to_string(active_rotor_indices_.at(i));

    KDL::Frame f;
    fk_solver.JntToCart(joint_positions, f, thrust);

    thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(FlamingoRobotModel, aerial_robot_model::RobotModel);
