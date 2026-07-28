#include <gtest/gtest.h>
#include <ros/ros.h>

#include <algorithm>
#include <iostream>
#include <limits>

#include <nami/model/nami_robot_model.h>

namespace
{
constexpr double kDegreeToRadian = 0.017453292519943295;

TEST(NamiTransformationStability, ZeroToThirtyDegreesIsValid)
{
  NamiRobotModel model;
  KDL::JntArray joint_positions(model.getTree().getNrOfJoints());
  KDL::SetToZero(joint_positions);
  model.updateRobotModel(joint_positions);
  ASSERT_TRUE(model.initialized());

  const auto yaw_joint = model.getJointIndexMap().find("yaw_joint");
  ASSERT_NE(yaw_joint, model.getJointIndexMap().end());

  ASSERT_LT(yaw_joint->second, joint_positions.rows());

  double minimum_singular_value = std::numeric_limits<double>::infinity();
  double minimum_thrust_reserve = std::numeric_limits<double>::infinity();
  for (int angle_deg = 0; angle_deg <= 30; ++angle_deg)
  {
    joint_positions(yaw_joint->second) = angle_deg * kDegreeToRadian;
    model.updateRobotModel(joint_positions);

    const NamiRobotModel::StabilityMetrics metrics = model.getStabilityMetrics();
    EXPECT_TRUE(metrics.valid) << "angle: " << angle_deg;
    EXPECT_EQ(metrics.allocation_rank, 4) << "angle: " << angle_deg;
    EXPECT_TRUE(model.stabilityCheck(false)) << "angle: " << angle_deg;
    minimum_singular_value =
      std::min(minimum_singular_value, metrics.scaled_min_singular_value);
    minimum_thrust_reserve =
      std::min(minimum_thrust_reserve, metrics.minimum_static_thrust_reserve);
  }

  std::cout << "0-30 degree stability sweep: minimum scaled singular value "
            << minimum_singular_value << ", minimum static thrust reserve "
            << minimum_thrust_reserve << " N" << std::endl;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nami_transformation_stability_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
