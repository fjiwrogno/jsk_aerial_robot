#include <gtest/gtest.h>

#include <cmath>

#include <nami/control/rc_thrust_mapper.h>

namespace aerial_robot_control
{
namespace
{
RcThrustMapper makeMapper()
{
  RcThrustMapper mapper;
  RcThrustMapper::Config& config = mapper.config();
  config.min_force = 0.0;
  config.max_force = 80.0;
  config.hover_throttle = 0.5;
  config.hover_force = 40.0;
  config.down_authority = 18.0;
  config.up_authority = 22.0;
  config.expo = 0.5;
  config.hover_deadband = 0.01;
  config.max_tilt_compensation = 1.3;
  mapper.sanitize();
  return mapper;
}
}  // namespace

TEST(RcThrustMapper, MapsHoverAndEndpoints)
{
  RcThrustMapper mapper = makeMapper();

  EXPECT_DOUBLE_EQ(mapper.throttleToForce(0.5, 40.0), 40.0);
  EXPECT_DOUBLE_EQ(mapper.throttleToForce(0.0, 40.0), 22.0);
  EXPECT_DOUBLE_EQ(mapper.throttleToForce(1.0, 40.0), 62.0);
}

TEST(RcThrustMapper, ComputesAutomaticHoverForceFromMass)
{
  RcThrustMapper mapper = makeMapper();
  mapper.config().hover_force = 0.0;
  mapper.config().hover_acceleration = 9.8;

  EXPECT_DOUBLE_EQ(mapper.hoverForce(4.0), 39.2);
  EXPECT_DOUBLE_EQ(mapper.throttleToForce(0.5, 4.0), 39.2);
}

TEST(RcThrustMapper, RetainsSmallSignalAuthorityAroundHover)
{
  RcThrustMapper mapper = makeMapper();
  const double hover_force = mapper.throttleToForce(0.5, 40.0);

  EXPECT_LT(mapper.throttleToForce(0.49, 40.0), hover_force);
  EXPECT_GT(mapper.throttleToForce(0.51, 40.0), hover_force);
  EXPECT_LT(mapper.throttleToForce(0.51, 40.0) - hover_force, 1.0);
}

TEST(RcThrustMapper, IsMonotonicAcrossThrottleRange)
{
  RcThrustMapper mapper = makeMapper();
  double previous = mapper.throttleToForce(0.0, 40.0);

  for (int i = 1; i <= 100; ++i)
  {
    const double force = mapper.throttleToForce(i / 100.0, 40.0);
    EXPECT_GE(force, previous);
    previous = force;
  }
}

TEST(RcThrustMapper, CompensatesTiltAndHonorsLimit)
{
  RcThrustMapper mapper = makeMapper();
  constexpr double angle = 0.35;
  const double expected = 1.0 / (std::cos(angle) * std::cos(angle));

  EXPECT_NEAR(mapper.tiltCompensation(angle, angle), expected, 1e-12);
  EXPECT_DOUBLE_EQ(mapper.compensateTilt(80.0, angle, angle), 80.0);

  mapper.config().max_tilt_compensation = 1.05;
  EXPECT_DOUBLE_EQ(mapper.tiltCompensation(angle, angle), 1.05);
}

TEST(RcThrustMapper, CanDisableTiltCompensation)
{
  RcThrustMapper mapper = makeMapper();
  mapper.config().tilt_compensation = false;

  EXPECT_DOUBLE_EQ(mapper.tiltCompensation(0.5, 0.5), 1.0);
  EXPECT_DOUBLE_EQ(mapper.compensateTilt(40.0, 0.5, 0.5), 40.0);
}
}  // namespace aerial_robot_control

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
