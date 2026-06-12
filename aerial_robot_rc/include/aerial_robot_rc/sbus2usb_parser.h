#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace aerial_robot_rc
{
struct RcFrame
{
  std::array<uint16_t, 16> channels{};
  uint8_t flags = 0;
  bool frame_lost = false;
  bool failsafe = false;
};

class Sbus2UsbParser
{
public:
  static constexpr std::size_t FRAME_SIZE = 35;
  static constexpr uint8_t FRAME_HEADER = 0x0F;
  static constexpr uint16_t CHANNEL_MAX = 2047;
  /* flag bit assignment from the converter manual; the example frame in the
     manual (flag = 0x0C) carries bit3/bit2 set during normal operation, so
     the masks stay configurable until confirmed against real hardware */
  static constexpr uint8_t DEFAULT_FAILSAFE_MASK = 0x10;
  static constexpr uint8_t DEFAULT_FRAME_LOST_MASK = 0x20;

  void setFlagMasks(uint8_t failsafe_mask, uint8_t frame_lost_mask)
  {
    failsafe_mask_ = failsafe_mask;
    frame_lost_mask_ = frame_lost_mask;
  }

  bool consume(uint8_t byte, RcFrame& frame);
  void reset();
  std::size_t badFrameCount() const
  {
    return bad_frame_count_;
  }
  std::size_t resyncCount() const
  {
    return resync_count_;
  }

private:
  bool tryParse(RcFrame& frame) const;

  std::vector<uint8_t> buffer_;
  std::size_t bad_frame_count_ = 0;
  std::size_t resync_count_ = 0;
  uint8_t failsafe_mask_ = DEFAULT_FAILSAFE_MASK;
  uint8_t frame_lost_mask_ = DEFAULT_FRAME_LOST_MASK;
};
}  // namespace aerial_robot_rc
