#include <flamingo/model/flamingo_robot_model.h>

FlamingoRobotModel::FlamingoRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon)
  : RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  ros::NodeHandle nhp("~"); // Private node handle
  nhp.param("robot_mode", robot_mode_, std::string("air"));

  full_rotor_num_ = RobotModel::getRotorNum();
  active_rotor_num_ = 2;
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

  // 1. Define the mapping from active rotor index (0, 1) to physical rotor index (0-3)
  std::vector<int> active_rotor_indices;
  ///TODO automative read the index from the yaml file
  if (robot_mode_ == "air")
  {
    active_rotor_indices = {1, 2}; // Use physical rotors 1 and 2
  }
  else if (robot_mode_ == "water")
  {
    active_rotor_indices = {3, 4}; // Use physical rotors 3 and 4
  }
  else
  {
    for(int i = 0; i < full_rotor_num_; ++i) active_rotor_indices.push_back(i);
  }

  const std::vector<KDL::Vector> all_rotors_origin = RobotModel::getRotorsOriginFromCog<KDL::Vector>();
  const std::vector<KDL::Vector> all_rotors_normal = RobotModel::getRotorsNormalFromCog<KDL::Vector>();

  std::vector<KDL::Vector> active_rotors_origin, active_rotors_normal;
  for (int index : active_rotor_indices)
  {
    if (index < all_rotors_origin.size()) {
        active_rotors_origin.push_back(all_rotors_origin.at(index));
        active_rotors_normal.push_back(all_rotors_normal.at(index));
    }
  }    
  
  active_rotor_num_ = active_rotors_origin.size();
  setRotorsOriginFromCog(active_rotors_origin);
  setRotorsNormalFromCog(active_rotors_normal);


  for (int i = 0; i < getRotorNum(); ++i)
  {
    int physical_rotor_index = active_rotor_indices.at(i);
    std::string thrust = "rotor_arm" + std::to_string(physical_rotor_index + 1);

    KDL::Frame f;
    fk_solver.JntToCart(joint_positions, f, thrust);

    thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(FlamingoRobotModel, aerial_robot_model::RobotModel);
