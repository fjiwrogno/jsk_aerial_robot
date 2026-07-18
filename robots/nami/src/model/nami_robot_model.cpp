#include <nami/model/nami_robot_model.h>

NamiRobotModel::NamiRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre,
                                             double epsilon)
  : RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();

  links_rotation_from_cog_.resize(rotor_num);
  thrust_coords_rot_.resize(rotor_num);
}

void NamiRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  // /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  transformable::RobotModel::updateRobotModelImpl(joint_positions);
  const auto seg_tf_map = getSegmentsTf();

  /* get local coords of thrust links:
     motor 0~3 = aerial rotors (frame: rotor_arm1..4, thrust along +z),
     motor 4~7 = aquatic rotors (frame: aquatic_rotor1..4, thrust along their tilted z) */
  for (int i = 0; i < getRotorNum(); ++i)
  {
    std::string thrust_frame;
    if (i < 4)
      thrust_frame = "rotor_arm" + std::to_string(i + 1);
    else
      thrust_frame = "aquatic_rotor" + std::to_string(i - 3);
    KDL::Frame f;
    fk_solver.JntToCart(joint_positions, f, thrust_frame);
    thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NamiRobotModel, aerial_robot_model::RobotModel);
