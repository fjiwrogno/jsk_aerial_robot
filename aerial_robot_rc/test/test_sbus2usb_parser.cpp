#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "aerial_robot_rc/sbus2usb_parser.h"

namespace
{
std::array<uint8_t, aerial_robot_rc::Sbus2UsbParser::FRAME_SIZE> buildFrame(const std::array<uint16_t, 16>& channels,
                                                                            uint8_t flags)
{
  std::array<uint8_t, aerial_robot_rc::Sbus2UsbParser::FRAME_SIZE> frame{};
  frame[0] = aerial_robot_rc::Sbus2UsbParser::FRAME_HEADER;
  for (std::size_t i = 0; i < channels.size(); ++i)
  {
    frame[1 + i * 2] = static_cast<uint8_t>((channels[i] >> 8) & 0xFF);
    frame[2 + i * 2] = static_cast<uint8_t>(channels[i] & 0xFF);
  }
  frame[33] = flags;

  uint8_t checksum = 0;
  for (std::size_t i = 1; i < frame.size() - 1; ++i)
  {
    checksum ^= frame[i];
  }
  frame[34] = checksum;
  return frame;
}

/* returns true when at least one frame parsed during this feed, even if
   trailing bytes follow the completed frame */
template <std::size_t N>
bool feed(aerial_robot_rc::Sbus2UsbParser& parser, const std::array<uint8_t, N>& bytes, aerial_robot_rc::RcFrame& frame)
{
  bool parsed = false;
  for (const auto byte : bytes)
  {
    if (parser.consume(byte, frame))
      parsed = true;
  }
  return parsed;
}
}  // namespace

TEST(Sbus2UsbParser, ParseValidFrame)
{
  aerial_robot_rc::Sbus2UsbParser parser;
  aerial_robot_rc::RcFrame parsed_frame;
  const std::array<uint16_t, 16> channels = { 1000, 1000, 0,    1000, 1000, 1000, 1000, 1000,
                                              1068, 1068, 1068, 1068, 1024, 1024, 1024, 1024 };
  const auto frame = buildFrame(channels, 0x0C);

  ASSERT_TRUE(feed(parser, frame, parsed_frame));
  EXPECT_EQ(parsed_frame.channels[0], 1000);
  EXPECT_EQ(parsed_frame.channels[15], 1024);
  EXPECT_EQ(parsed_frame.flags, 0x0C);
  EXPECT_FALSE(parsed_frame.frame_lost);
  EXPECT_FALSE(parsed_frame.failsafe);
}

TEST(Sbus2UsbParser, RejectBadChecksum)
{
  aerial_robot_rc::Sbus2UsbParser parser;
  aerial_robot_rc::RcFrame parsed_frame;
  std::array<uint16_t, 16> channels{};
  auto frame = buildFrame(channels, 0x30);
  frame[34] ^= 0x01;

  EXPECT_FALSE(feed(parser, frame, parsed_frame));
  EXPECT_EQ(parser.badFrameCount(), 1u);
}

TEST(Sbus2UsbParser, ResyncAfterNoise)
{
  aerial_robot_rc::Sbus2UsbParser parser;
  aerial_robot_rc::RcFrame parsed_frame;
  std::array<uint16_t, 16> channels{};
  channels[0] = 321;
  channels[1] = 654;
  const auto frame = buildFrame(channels, 0xA0);
  const std::vector<uint8_t> stream = { 0xAA, 0xBB, 0xCC, frame[0], frame[1], frame[2], 0x01, 0x02 };

  bool parsed = false;
  for (const auto byte : stream)
  {
    parsed = parser.consume(byte, parsed_frame);
  }
  EXPECT_FALSE(parsed);

  ASSERT_TRUE(feed(parser, frame, parsed_frame));
  EXPECT_EQ(parsed_frame.channels[0], 321);
  EXPECT_EQ(parsed_frame.channels[1], 654);
  EXPECT_EQ(parsed_frame.flags, 0xA0);
  EXPECT_TRUE(parsed_frame.frame_lost);
  EXPECT_FALSE(parsed_frame.failsafe);
}

TEST(Sbus2UsbParser, ParseConsecutiveFrames)
{
  aerial_robot_rc::Sbus2UsbParser parser;
  aerial_robot_rc::RcFrame parsed_frame;
  std::size_t parsed_count = 0;

  for (uint16_t value : { 100, 1000, 2000 })
  {
    std::array<uint16_t, 16> channels{};
    channels.fill(value);
    const auto frame = buildFrame(channels, 0x00);
    if (feed(parser, frame, parsed_frame))
    {
      ++parsed_count;
      EXPECT_EQ(parsed_frame.channels[0], value);
      EXPECT_EQ(parsed_frame.channels[15], value);
    }
  }

  EXPECT_EQ(parsed_count, 3u);
  EXPECT_EQ(parser.badFrameCount(), 0u);
}

TEST(Sbus2UsbParser, ChannelLowByteEqualsHeader)
{
  /* a channel value of 15 puts 0x0F into the payload; the parser must not
     treat it as a frame boundary */
  aerial_robot_rc::Sbus2UsbParser parser;
  aerial_robot_rc::RcFrame parsed_frame;
  std::array<uint16_t, 16> channels{};
  channels.fill(0x000F);
  const auto frame = buildFrame(channels, 0x00);

  ASSERT_TRUE(feed(parser, frame, parsed_frame));
  EXPECT_EQ(parsed_frame.channels[0], 0x000F);
  EXPECT_EQ(parsed_frame.channels[15], 0x000F);

  ASSERT_TRUE(feed(parser, frame, parsed_frame));
  EXPECT_EQ(parsed_frame.channels[7], 0x000F);
}

TEST(Sbus2UsbParser, RecoverAfterCorruptedFrame)
{
  aerial_robot_rc::Sbus2UsbParser parser;
  aerial_robot_rc::RcFrame parsed_frame;

  /* corrupted frame whose payload contains no 0x0F byte, so the parser fully
     drops it and locks onto the next frame */
  std::array<uint16_t, 16> channels{};
  channels.fill(0x0301);
  auto bad_frame = buildFrame(channels, 0x00);
  bad_frame[34] ^= 0x01;
  EXPECT_FALSE(feed(parser, bad_frame, parsed_frame));
  EXPECT_EQ(parser.badFrameCount(), 1u);

  channels.fill(500);
  const auto good_frame = buildFrame(channels, 0x00);
  ASSERT_TRUE(feed(parser, good_frame, parsed_frame));
  EXPECT_EQ(parsed_frame.channels[0], 500);
}

TEST(Sbus2UsbParser, RejectOutOfRangeChannel)
{
  /* a misaligned frame can pass the 1-byte XOR by chance; the 11-bit channel
     range check (high byte <= 0x07) must reject it */
  aerial_robot_rc::Sbus2UsbParser parser;
  aerial_robot_rc::RcFrame parsed_frame;
  std::array<uint16_t, 16> channels{};
  channels[3] = 2048;
  const auto frame = buildFrame(channels, 0x00);

  EXPECT_FALSE(feed(parser, frame, parsed_frame));
  EXPECT_EQ(parser.badFrameCount(), 1u);
}

TEST(Sbus2UsbParser, CustomFlagMasks)
{
  aerial_robot_rc::Sbus2UsbParser parser;
  parser.setFlagMasks(0x08, 0x04);
  aerial_robot_rc::RcFrame parsed_frame;
  std::array<uint16_t, 16> channels{};

  ASSERT_TRUE(feed(parser, buildFrame(channels, 0x0C), parsed_frame));
  EXPECT_TRUE(parsed_frame.failsafe);
  EXPECT_TRUE(parsed_frame.frame_lost);

  ASSERT_TRUE(feed(parser, buildFrame(channels, 0x30), parsed_frame));
  EXPECT_FALSE(parsed_frame.failsafe);
  EXPECT_FALSE(parsed_frame.frame_lost);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
