#include <gtest/gtest.h>

#include <aerial_robot_msgs/RcRaw.h>
#include <nami/control/rc_mapper.h>

namespace aerial_robot_control
{
namespace
{
aerial_robot_msgs::RcRaw makeFrame(const ros::Time& stamp)
{
  aerial_robot_msgs::RcRaw frame;
  frame.header.stamp = stamp;
  for (auto& channel : frame.channels)
    channel = 368;

  frame.channels[1] = 1009;  // CH2 pitch center
  frame.channels[2] = 396;   // CH3 throttle minimum
  frame.channels[5] = 368;   // CH6 disarm
  frame.channels[6] = 368;   // CH7 kill released
  frame.channels[7] = 1680;  // CH8 takeoff enabled
  return frame;
}
}  // namespace

TEST(RcMapper, UsesConfiguredSwitchChannels)
{
  RcMapper mapper;
  ASSERT_TRUE(mapper.initialize(ros::NodeHandle("/rc_mapper_test/controller/rc")));

  const ros::Time stamp = ros::Time::now();
  aerial_robot_msgs::RcRaw frame = makeFrame(stamp);
  mapper.feed(frame);
  mapper.update(stamp);
  EXPECT_TRUE(mapper.linkValid());
  EXPECT_FALSE(mapper.armSwitch());
  EXPECT_FALSE(mapper.killLatched());
  EXPECT_TRUE(mapper.takeoffEnable());

  frame.channels[5] = 1680;  // CH6 arm
  frame.header.stamp += ros::Duration(0.01);
  mapper.feed(frame);
  mapper.update(frame.header.stamp);
  EXPECT_TRUE(mapper.armSwitch());
  EXPECT_TRUE(mapper.armRisingEdge());

  frame.channels[6] = 1680;  // CH7 kill
  frame.header.stamp += ros::Duration(0.01);
  mapper.feed(frame);
  mapper.update(frame.header.stamp);
  EXPECT_TRUE(mapper.killLatched());
}

TEST(RcMapper, LinkLossLatchesKillDisarmsAndZerosCommands)
{
  RcMapper mapper;
  ASSERT_TRUE(mapper.initialize(ros::NodeHandle("/rc_mapper_test/controller/rc")));

  const ros::Time stamp = ros::Time::now();
  aerial_robot_msgs::RcRaw frame = makeFrame(stamp);
  mapper.feed(frame);
  mapper.update(stamp);

  frame.channels[0] = 1680;
  frame.channels[2] = 1680;
  frame.channels[5] = 1680;
  frame.header.stamp += ros::Duration(0.01);
  mapper.feed(frame);
  mapper.update(frame.header.stamp);
  ASSERT_TRUE(mapper.armSwitch());
  ASSERT_GT(mapper.sticks().yaw_rate, 0.0);
  ASSERT_GT(mapper.sticks().throttle, 0.0);

  mapper.update(frame.header.stamp + ros::Duration(0.21));
  EXPECT_FALSE(mapper.linkValid());
  EXPECT_TRUE(mapper.killLatched());
  EXPECT_FALSE(mapper.armSwitch());
  EXPECT_FALSE(mapper.takeoffEnable());
  EXPECT_DOUBLE_EQ(mapper.sticks().roll, 0.0);
  EXPECT_DOUBLE_EQ(mapper.sticks().pitch, 0.0);
  EXPECT_DOUBLE_EQ(mapper.sticks().yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(mapper.sticks().throttle, 0.0);

  aerial_robot_msgs::RcRaw recovery = makeFrame(frame.header.stamp + ros::Duration(0.22));
  mapper.feed(recovery);
  mapper.update(recovery.header.stamp);
  EXPECT_TRUE(mapper.linkValid());
  EXPECT_FALSE(mapper.killLatched());
  EXPECT_FALSE(mapper.armSwitch());

  recovery.channels[5] = 1680;
  recovery.header.stamp += ros::Duration(0.01);
  mapper.feed(recovery);
  mapper.update(recovery.header.stamp);
  EXPECT_TRUE(mapper.armRisingEdge());
}
}  // namespace aerial_robot_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_rc_mapper");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
