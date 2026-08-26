// 串口绝对编码器 ROS 节点（C++ 参考实现）。
//
// 节点周期性地向编码器模块发送读请求，接收并解析 "ID CMD Len Data CRC"
// 响应帧，把编码器原始值换算为角度并发布。
//
// 发布：
//   <angle>  std_msgs/Float64   编码器角度（默认单位为 deg）
//   <raw>    std_msgs/Int32     编码器原始计数值
//
// 参数（~ 私有）：
//   port        串口设备，默认 /dev/ttyUSB0
//   baudrate    波特率，默认 115200
//   slave_id    模块地址，默认 0
//   start_reg   读取起始寄存器，默认 0（按编码器寄存器定义配置）
//   reg_count   读取寄存器个数，默认 2（对应 4 字节 -> 18 位编码器值）
//   rate_hz     采样频率，默认 50
//   units       角度单位 deg|rad，默认 deg
//   angle_topic 角度话题名，默认 encoder_angle
//   raw_topic   原始话题名，默认 encoder_raw
//
// 角度换算：Angle = raw / 262144 * 360（度）。
//
// 用法：
//   运行节点：rosrun encoder_driver encoder_node _port:=/dev/ttyUSB0
//   离线自检：rosrun encoder_driver encoder_node --selftest

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <sys/select.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include "encoder_driver/encoder_frame.hpp"

namespace {

using encoder_driver::BuildReadQuery;
using encoder_driver::FormatHex;
using encoder_driver::ParseResponse;
using encoder_driver::RawToAngle;
using encoder_driver::Response;
using encoder_driver::ResponseParser;

constexpr double kDegToRad = std::acos(-1.0) / 180.0;

speed_t ToSpeedT(int baudrate) {
  switch (baudrate) {
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
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      ROS_WARN("不支持的波特率 %d，使用 115200", baudrate);
      return B115200;
  }
}

// 极简 POSIX 串口封装（8N1，无流控）。
class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  bool Open(const std::string& port, int baudrate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
      ROS_ERROR("无法打开串口 %s: %s", port.c_str(), std::strerror(errno));
      return false;
    }
    // 打开成功后恢复阻塞模式，用 select 实现有界读取。
    int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags >= 0) {
      ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (::tcgetattr(fd_, &tio) != 0) {
      ROS_ERROR("tcgetattr 失败: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    speed_t speed = ToSpeedT(baudrate);
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;          // 8 位数据
    tio.c_cflag &= ~PARENB;      // 无校验
    tio.c_cflag &= ~CSTOPB;      // 1 位停止
    tio.c_cflag &= ~CRTSCTS;     // 无硬件流控
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
      ROS_ERROR("tcsetattr 失败: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    ::tcflush(fd_, TCIFLUSH);
    ROS_INFO("串口 %s 已打开，波特率 %d", port.c_str(), baudrate);
    return true;
  }

  bool IsOpen() const { return fd_ >= 0; }

  bool Write(const std::vector<uint8_t>& data) {
    if (fd_ < 0) {
      return false;
    }
    size_t written = 0;
    while (written < data.size()) {
      ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        ROS_WARN("写串口失败: %s", std::strerror(errno));
        return false;
      }
      written += static_cast<size_t>(n);
    }
    return true;
  }

  // 在 timeout_ms 内读取最多 max_bytes 字节；返回读取到的字节数，
  // 0 表示超时/无数据，-1 表示读取错误。
  ssize_t Read(std::vector<uint8_t>* out, size_t max_bytes, int timeout_ms) {
    if (fd_ < 0) {
      return -1;
    }
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int sel = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (sel <= 0) {
      return 0;  // 超时或被打断
    }
    std::vector<uint8_t> tmp(max_bytes);
    ssize_t n = ::read(fd_, tmp.data(), tmp.size());
    if (n > 0) {
      out->assign(tmp.begin(), tmp.begin() + n);
    }
    return n;
  }

 private:
  int fd_ = -1;
};

class EncoderNode {
 public:
  EncoderNode() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string port = pnh.param("port", std::string("/dev/ttyUSB0"));
    int baudrate = pnh.param("baudrate", 115200);
    slave_id_ = static_cast<uint8_t>(pnh.param("slave_id", 0));
    uint16_t start_reg = static_cast<uint16_t>(pnh.param("start_reg", 0));
    uint16_t reg_count = static_cast<uint16_t>(pnh.param("reg_count", 2));
    rate_hz_ = pnh.param("rate_hz", 50.0);
    units_ = pnh.param("units", std::string("deg"));
    std::string angle_topic = pnh.param("angle_topic", std::string("encoder_angle"));
    std::string raw_topic = pnh.param("raw_topic", std::string("encoder_raw"));

    if (units_ != "deg" && units_ != "rad") {
      ROS_WARN("~units=%s 无效，使用 deg", units_.c_str());
      units_ = "deg";
    }
    if (rate_hz_ <= 0.0) {
      ROS_WARN("~rate_hz=%f 无效，使用 50", rate_hz_);
      rate_hz_ = 50.0;
    }
    // 与 Python 版本一致：单次读取窗口不超过 1.5 个周期且不大于 500 ms。
    read_timeout_ms_ = static_cast<int>(
        std::min(500.0, 1500.0 / rate_hz_));

    parser_.reset(new ResponseParser(slave_id_));
    query_ = BuildReadQuery(slave_id_, start_reg, reg_count);

    pub_angle_ = nh.advertise<std_msgs::Float64>(angle_topic, 1);
    pub_raw_ = nh.advertise<std_msgs::Int32>(raw_topic, 1);

    serial_.reset(new SerialPort());
    if (!serial_->Open(port, baudrate)) {
      ROS_FATAL("编码器串口打开失败，节点退出");
      ros::shutdown();
    }

    ROS_INFO(
        "编码器节点就绪: port=%s baud=%d slave=0x%02X reg=%d@%d units=%s",
        port.c_str(), baudrate, slave_id_, reg_count, start_reg, units_.c_str());
  }

  void Spin() {
    ros::Rate rate(rate_hz_);
    while (ros::ok()) {
      ReadOnce();
      rate.sleep();
    }
  }

 private:
  void ReadOnce() {
    if (!serial_ || !serial_->IsOpen()) {
      return;
    }
    if (!serial_->Write(query_)) {
      return;
    }
    while (ros::ok()) {
      std::vector<uint8_t> data;
      ssize_t n = serial_->Read(&data, 64, read_timeout_ms_);
      if (n <= 0) {
        break;
      }
      for (const std::vector<uint8_t>& frame : parser_->Feed(data)) {
        HandleFrame(frame);
      }
    }
  }

  void HandleFrame(const std::vector<uint8_t>& frame) {
    Response info;
    try {
      info = ParseResponse(frame);
    } catch (const std::invalid_argument&) {
      return;
    }
    double angle = RawToAngle(info.raw);
    if (units_ == "rad") {
      angle *= kDegToRad;
    }
    std_msgs::Float64 angle_msg;
    angle_msg.data = angle;
    pub_angle_.publish(angle_msg);
    std_msgs::Int32 raw_msg;
    raw_msg.data = static_cast<int32_t>(info.raw);
    pub_raw_.publish(raw_msg);
    ROS_DEBUG("raw=%u angle=%.6f (slave=0x%02X data=%s)", info.raw, angle,
              info.slave_id, FormatHex(info.data).c_str());
  }

  uint8_t slave_id_ = 0;
  double rate_hz_ = 50.0;
  int read_timeout_ms_ = 30;
  std::string units_ = "deg";
  std::vector<uint8_t> query_;
  std::unique_ptr<ResponseParser> parser_;
  std::unique_ptr<SerialPort> serial_;
  ros::Publisher pub_angle_;
  ros::Publisher pub_raw_;
};

}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--selftest") {
      return encoder_driver::SelfTest() ? 0 : 1;
    }
  }
  ros::init(argc, argv, "encoder_node", ros::init_options::AnonymousName);
  EncoderNode node;
  node.Spin();
  return 0;
}
