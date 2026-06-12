#include "aerial_robot_rc/sbus2usb_parser.h"

#include <algorithm>

namespace aerial_robot_rc
{
bool Sbus2UsbParser::consume(uint8_t byte, RcFrame& frame)
{
  if (buffer_.empty())
  {
    if (byte != FRAME_HEADER)
      return false;
  }

  if (!buffer_.empty() && buffer_[0] != FRAME_HEADER)
  {
    buffer_.clear();
    if (byte != FRAME_HEADER)
      return false;
  }

  buffer_.push_back(byte);

  /* one byte is appended per call and every FRAME_SIZE-sized buffer is parsed
     then cleared or shrunk below FRAME_SIZE, so the buffer never grows past it */
  if (buffer_.size() < FRAME_SIZE)
    return false;

  const bool parsed = tryParse(frame);

  if (parsed)
  {
    buffer_.clear();
    return true;
  }

  ++bad_frame_count_;
  ++resync_count_;
  auto header_it = std::find(buffer_.begin() + 1, buffer_.end(), FRAME_HEADER);
  if (header_it == buffer_.end())
  {
    buffer_.clear();
  }
  else
  {
    buffer_.erase(buffer_.begin(), header_it);
  }

  return false;
}

void Sbus2UsbParser::reset()
{
  buffer_.clear();
  bad_frame_count_ = 0;
  resync_count_ = 0;
}

bool Sbus2UsbParser::tryParse(RcFrame& frame) const
{
  if (buffer_.size() != FRAME_SIZE)
    return false;
  if (buffer_[0] != FRAME_HEADER)
    return false;

  uint8_t xor_check = 0;
  for (std::size_t i = 1; i < FRAME_SIZE - 1; ++i)
  {
    xor_check ^= buffer_[i];
  }

  if (xor_check != buffer_[FRAME_SIZE - 1])
    return false;

  /* channels are 11-bit (0-2047) so every high byte must be <= 0x07; this
     rejects misaligned frames that pass the single-byte XOR by chance */
  for (std::size_t i = 0; i < frame.channels.size(); ++i)
  {
    if (buffer_[1 + i * 2] > (CHANNEL_MAX >> 8))
      return false;
  }

  for (std::size_t i = 0; i < frame.channels.size(); ++i)
  {
    const std::size_t offset = 1 + i * 2;
    frame.channels[i] = static_cast<uint16_t>((buffer_[offset] << 8) | buffer_[offset + 1]);
  }

  const uint8_t flags = buffer_[33];
  frame.flags = flags;
  frame.frame_lost = flags & frame_lost_mask_;
  frame.failsafe = flags & failsafe_mask_;
  return true;
}
}  // namespace aerial_robot_rc
