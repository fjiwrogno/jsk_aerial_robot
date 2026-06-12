#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <aerial_robot_msgs/RcRaw.h>
#include "aerial_robot_rc/sbus2usb_parser.h"

namespace aerial_robot_rc
{
class Sbus2UsbRcNode
{
public:
  Sbus2UsbRcNode()
    : nh_()
    , pnh_("~")
    , fd_(-1)
    , opened_(false)
    , failsafe_active_(false)
    , total_bytes_(0)
    , total_frames_(0)
    , decode_errors_offset_(0)
    , resyncs_offset_(0)
    , bytes_since_status_(0)
    , frames_since_status_(0)
  {
    pnh_.param<std::string>("port", port_, "/dev/ttyUSB0");
    pnh_.param<int>("baud_rate", baud_rate_, 115200);
    pnh_.param<double>("poll_rate", poll_rate_, 200.0);
    pnh_.param<double>("status_rate", status_rate_, 1.0);
    pnh_.param<double>("serial_timeout", serial_timeout_, 0.5);
    pnh_.param<double>("byte_timeout", byte_timeout_, 1.0);
    poll_rate_ = sanitizePositive("poll_rate", poll_rate_, 200.0);
    status_rate_ = sanitizePositive("status_rate", status_rate_, 1.0);
    serial_timeout_ = sanitizePositive("serial_timeout", serial_timeout_, 0.5);
    byte_timeout_ = sanitizePositive("byte_timeout", byte_timeout_, 1.0);

    int failsafe_mask, frame_lost_mask;
    pnh_.param<int>("failsafe_mask", failsafe_mask, Sbus2UsbParser::DEFAULT_FAILSAFE_MASK);
    pnh_.param<int>("frame_lost_mask", frame_lost_mask, Sbus2UsbParser::DEFAULT_FRAME_LOST_MASK);
    failsafe_mask_ = sanitizeMask("failsafe_mask", failsafe_mask, Sbus2UsbParser::DEFAULT_FAILSAFE_MASK);
    frame_lost_mask_ = sanitizeMask("frame_lost_mask", frame_lost_mask, Sbus2UsbParser::DEFAULT_FRAME_LOST_MASK);
    parser_.setFlagMasks(failsafe_mask_, frame_lost_mask_);

    raw_pub_ = nh_.advertise<aerial_robot_msgs::RcRaw>("rc/raw", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("rc/status", 10);

    const ros::Time now = ros::Time::now();
    last_byte_time_ = now;
    last_valid_frame_time_ = now;
    last_open_attempt_time_ = ros::Time(0);

    poll_timer_ = nh_.createTimer(ros::Duration(1.0 / poll_rate_), &Sbus2UsbRcNode::pollSerial, this);
    watchdog_timer_ = nh_.createTimer(ros::Duration(1.0 / WATCHDOG_RATE), &Sbus2UsbRcNode::watchdog, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_rate_), &Sbus2UsbRcNode::publishStatus, this);
  }

  ~Sbus2UsbRcNode()
  {
    closeSerial();
  }

private:
  static constexpr double WATCHDOG_RATE = 20.0;

  void pollSerial(const ros::TimerEvent&)
  {
    if (!opened_ && !openSerial())
      return;

    RcFrame frame;
    std::array<uint8_t, 256> read_buffer{};
    while (true)
    {
      const ssize_t read_size = read(fd_, read_buffer.data(), read_buffer.size());
      if (read_size > 0)
      {
        last_byte_time_ = ros::Time::now();
        total_bytes_ += read_size;
        bytes_since_status_ += read_size;
        for (ssize_t i = 0; i < read_size; ++i)
        {
          if (parser_.consume(read_buffer[i], frame))
          {
            publishFrame(frame);
          }
        }
        continue;
      }

      /* read() == 0 on a removed USB serial device looks like "no data";
         the byte_timeout watchdog recovers from that case */
      if (read_size == 0)
        break;

      if (errno == EAGAIN || errno == EWOULDBLOCK)
        break;

      ROS_WARN_THROTTLE(1.0, "Failed to read %s: %s", port_.c_str(), std::strerror(errno));
      closeSerial();
      break;
    }
  }

  void watchdog(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();

    if (opened_ && (now - last_byte_time_).toSec() > byte_timeout_)
    {
      ROS_WARN("No bytes from %s for %.1f s, reopening the device", port_.c_str(), byte_timeout_);
      closeSerial();
    }

    if ((now - last_valid_frame_time_).toSec() > serial_timeout_)
    {
      if (!failsafe_active_)
      {
        ROS_ERROR("RC failsafe: no valid frame for %.2f s", serial_timeout_);
        failsafe_active_ = true;
      }
      publishFailsafe(now);
    }
  }

  void publishStatus(const ros::TimerEvent&)
  {
    const std::size_t decode_errors_total = decode_errors_offset_ + parser_.badFrameCount();
    const std::size_t resyncs_total = resyncs_offset_ + parser_.resyncCount();

    std_msgs::String status_msg;
    std::ostringstream oss;
    oss << "port=" << port_ << " opened=" << (opened_ ? "true" : "false")
        << " failsafe=" << (failsafe_active_ ? "true" : "false") << " baud_rate=" << baud_rate_
        << " bytes_total=" << total_bytes_ << " bytes_last=" << bytes_since_status_ << " frames_total=" << total_frames_
        << " frames_last=" << frames_since_status_ << " decode_errors_total=" << decode_errors_total
        << " decode_errors_last=" << decode_errors_total - last_decode_errors_total_
        << " resyncs_total=" << resyncs_total;
    status_msg.data = oss.str();
    status_pub_.publish(status_msg);

    bytes_since_status_ = 0;
    frames_since_status_ = 0;
    last_decode_errors_total_ = decode_errors_total;
  }

  bool openSerial()
  {
    const ros::Time now = ros::Time::now();
    if (!last_open_attempt_time_.isZero() && (now - last_open_attempt_time_).toSec() < OPEN_RETRY_INTERVAL)
      return false;
    last_open_attempt_time_ = now;

    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
      ROS_WARN_THROTTLE(5.0, "Failed to open %s: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
      ROS_WARN("Failed to query serial attributes for %s: %s", port_.c_str(), std::strerror(errno));
      closeSerial();
      return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    const speed_t baud = resolveBaudRate(baud_rate_);
    if (baud == 0)
    {
      ROS_ERROR("Unsupported baud rate: %d", baud_rate_);
      closeSerial();
      return false;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
      ROS_WARN("Failed to configure serial port %s: %s", port_.c_str(), std::strerror(errno));
      closeSerial();
      return false;
    }

    /* drop bytes buffered by the kernel before the port was configured;
       a stale frame from a previous session must not be published as fresh */
    tcflush(fd_, TCIFLUSH);

    absorbParserCounters();
    parser_.reset();
    opened_ = true;
    last_byte_time_ = ros::Time::now();
    ROS_INFO("Opened SBUS2USB serial device %s at %d baud", port_.c_str(), baud_rate_);
    return true;
  }

  void closeSerial()
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
    absorbParserCounters();
    parser_.reset();
    opened_ = false;
  }

  /* parser counters restart from zero on reset(); fold them into the running
     offsets first so the totals published in rc/status stay monotonic */
  void absorbParserCounters()
  {
    decode_errors_offset_ += parser_.badFrameCount();
    resyncs_offset_ += parser_.resyncCount();
  }

  void publishFrame(const RcFrame& frame)
  {
    last_frame_ = frame;
    last_valid_frame_time_ = ros::Time::now();
    if (frame.failsafe)
    {
      if (!failsafe_active_)
      {
        ROS_ERROR("RC failsafe reported by the receiver");
        failsafe_active_ = true;
      }
    }
    else if (failsafe_active_)
    {
      ROS_INFO("RC signal recovered");
      failsafe_active_ = false;
    }

    aerial_robot_msgs::RcRaw msg;
    msg.header.stamp = last_valid_frame_time_;
    fillChannels(msg, frame);
    msg.flags = frame.flags;
    msg.frame_lost = frame.frame_lost;
    msg.failsafe = frame.failsafe;
    raw_pub_.publish(msg);

    ++total_frames_;
    ++frames_since_status_;
  }

  /* keep telling downstream consumers that the link is dead: freeze the last
     known channel values (all zero until the first valid frame ever arrives)
     and force failsafe/frame_lost in both the bools and the flags byte so
     mask-based consumers agree with the bool fields */
  void publishFailsafe(const ros::Time& stamp)
  {
    aerial_robot_msgs::RcRaw msg;
    msg.header.stamp = stamp;
    fillChannels(msg, last_frame_);
    msg.flags = last_frame_.flags | failsafe_mask_ | frame_lost_mask_;
    msg.frame_lost = true;
    msg.failsafe = true;
    raw_pub_.publish(msg);
  }

  static void fillChannels(aerial_robot_msgs::RcRaw& msg, const RcFrame& frame)
  {
    for (std::size_t i = 0; i < frame.channels.size(); ++i)
    {
      msg.channels[i] = frame.channels[i];
    }
  }

  static double sanitizePositive(const char* name, double value, double fallback)
  {
    if (value > 0.0)
      return value;
    ROS_ERROR("Invalid %s %.3f (must be > 0), falling back to %.3f", name, value, fallback);
    return fallback;
  }

  static uint8_t sanitizeMask(const char* name, int value, uint8_t fallback)
  {
    if (value >= 0x01 && value <= 0xFF)
      return static_cast<uint8_t>(value);
    ROS_ERROR("Invalid %s 0x%X (must be within 0x01-0xFF), falling back to 0x%02X", name, value, fallback);
    return fallback;
  }

  static speed_t resolveBaudRate(int baud_rate)
  {
    switch (baud_rate)
    {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:
        return 0;
    }
  }

  static constexpr double OPEN_RETRY_INTERVAL = 1.0;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher raw_pub_;
  ros::Publisher status_pub_;
  ros::Timer poll_timer_;
  ros::Timer watchdog_timer_;
  ros::Timer status_timer_;

  Sbus2UsbParser parser_;
  RcFrame last_frame_;

  std::string port_;
  int baud_rate_;
  double poll_rate_;
  double status_rate_;
  double serial_timeout_;
  double byte_timeout_;
  uint8_t failsafe_mask_;
  uint8_t frame_lost_mask_;

  int fd_;
  bool opened_;
  bool failsafe_active_;

  ros::Time last_byte_time_;
  ros::Time last_valid_frame_time_;
  ros::Time last_open_attempt_time_;

  std::size_t total_bytes_;
  std::size_t total_frames_;
  std::size_t decode_errors_offset_;
  std::size_t resyncs_offset_;
  std::size_t bytes_since_status_;
  std::size_t frames_since_status_;
  std::size_t last_decode_errors_total_ = 0;
};
}  // namespace aerial_robot_rc

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sbus2usb_rc_node");
  aerial_robot_rc::Sbus2UsbRcNode node;
  ros::spin();
  return 0;
}
