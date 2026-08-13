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

  /* get local coords of thrust links.

     Read them from the "thrustN" links themselves: those are the very links the base
     RobotModel derives rotors_origin_from_cog_ from, so the position and the thrust
     direction of a rotor can never come from two different frames.

     The aerial rotors used to be read from "rotor_armN", which sits *above* the fixed
     gimbalN joint carrying their +-10 deg tilt, so the allocation modelled them as
     perfectly vertical.  That dropped two real effects: the tilt generates ~6x more
     yaw torque than the blade drag alone (yaw commands were executed 6x too strong),
     and it produces a body-x force the map reported as identically zero (~0.24 m/s^2
     of unmodelled forward acceleration at hover).
     aquatic_rotorN and thrust(N+4) are the same frame, so motors 4~7 are unchanged. */
  for (int i = 0; i < getRotorNum(); ++i)
  {
    KDL::Frame f;
    fk_solver.JntToCart(joint_positions, f, "thrust" + std::to_string(i + 1));
    thrust_coords_rot_[i] = cog_frame.Inverse() * f.M;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NamiRobotModel, aerial_robot_model::RobotModel);
