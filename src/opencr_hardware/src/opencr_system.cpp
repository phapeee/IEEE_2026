#include "opencr_hardware/opencr_system.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/logging.hpp"

namespace opencr_hardware
{
namespace
{

speed_t baud_to_flag(int baud)
{
  switch (baud) {
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

constexpr std::size_t SERIAL_BUFFER_SIZE = 128;

}  // namespace

uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte)
{
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (int i = 0; i < 8; ++i) {
    if (crc & 0x8000) {
      crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
    } else {
      crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baudrate)
{
  close();
  int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    ::close(fd);
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  speed_t speed_flag = baud_to_flag(baudrate);
  if (speed_flag == 0) {
    ::close(fd);
    return false;
  }

  cfsetispeed(&tty, speed_flag);
  cfsetospeed(&tty, speed_flag);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    ::close(fd);
    return false;
  }

  fd_ = fd;
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

std::size_t SerialPort::read(uint8_t * dst, std::size_t max_len)
{
  if (fd_ < 0 || dst == nullptr || max_len == 0) {
    return 0;
  }

  ssize_t ret = ::read(fd_, dst, max_len);
  if (ret < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return 0;
    }
    return std::numeric_limits<std::size_t>::max();
  }
  return static_cast<std::size_t>(ret);
}

bool SerialPort::write_all(const uint8_t * data, std::size_t len)
{
  if (fd_ < 0 || data == nullptr || len == 0) {
    return false;
  }

  std::size_t written = 0;
  while (written < len) {
    ssize_t ret = ::write(fd_, data + written, len - written);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(ret);
  }
  return true;
}

void FrameParser::reset()
{
  state_ = State::HEADER_1;
  msg_type_ = 0;
  msg_len_ = 0;
  msg_seq_ = 0;
  payload_idx_ = 0;
  crc_low_ = 0;
  crc_calc_ = 0;
}

bool FrameParser::consume(uint8_t byte, Frame & frame)
{
  switch (state_) {
    case State::HEADER_1:
      if (byte == MSG_HEADER_1) {
        state_ = State::HEADER_2;
      }
      break;

    case State::HEADER_2:
      if (byte == MSG_HEADER_2) {
        state_ = State::TYPE;
      } else if (byte == MSG_HEADER_1) {
        state_ = State::HEADER_2;
      } else {
        state_ = State::HEADER_1;
      }
      break;

    case State::TYPE:
      msg_type_ = byte;
      crc_calc_ = crc16_ccitt_update(0xFFFF, byte);
      state_ = State::LEN;
      break;

    case State::LEN:
      msg_len_ = byte;
      crc_calc_ = crc16_ccitt_update(crc_calc_, byte);
      if (msg_len_ > MAX_PAYLOAD_LEN) {
        reset();
      } else {
        state_ = State::SEQ;
      }
      break;

    case State::SEQ:
      msg_seq_ = byte;
      crc_calc_ = crc16_ccitt_update(crc_calc_, byte);
      payload_idx_ = 0;
      state_ = (msg_len_ == 0) ? State::CRC_LO : State::PAYLOAD;
      break;

    case State::PAYLOAD:
      payload_[payload_idx_++] = byte;
      crc_calc_ = crc16_ccitt_update(crc_calc_, byte);
      if (payload_idx_ >= msg_len_) {
        state_ = State::CRC_LO;
      }
      break;

    case State::CRC_LO:
      crc_low_ = byte;
      state_ = State::CRC_HI;
      break;

    case State::CRC_HI: {
      uint16_t crc_rx = static_cast<uint16_t>(crc_low_) | (static_cast<uint16_t>(byte) << 8);
      if (crc_rx == crc_calc_) {
        frame.type = msg_type_;
        frame.seq = msg_seq_;
        frame.payload.assign(payload_.begin(), payload_.begin() + msg_len_);
        reset();
        return true;
      }
      reset();
      break;
    }
  }

  return false;
}

hardware_interface::CallbackReturn OpenCRSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 4) {
    RCLCPP_ERROR(logger_, "OpenCR interface expects 4 joints but received %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    auto has_velocity_cmd = std::any_of(
      joint.command_interfaces.begin(), joint.command_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_VELOCITY;
      });
    auto has_velocity_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_VELOCITY;
      });
    auto has_position_state = std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
      [](const auto & interface) {
        return interface.name == hardware_interface::HW_IF_POSITION;
      });

    if (!has_velocity_cmd) {
      RCLCPP_ERROR(logger_, "Joint %s is missing velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!has_velocity_state || !has_position_state) {
      RCLCPP_ERROR(
        logger_, "Joint %s must expose position and velocity state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), 0.0);

  serial_port_ = get_param("serial_port", "/dev/ttyACM0");
  baud_rate_ = get_param_int("baud_rate", 115200);
  status_timeout_sec_ = get_param_double("status_timeout_sec", 0.5);
  max_read_iterations_ = std::max(1, get_param_int("max_read_iterations", 16));

  last_status_time_ = steady_clock_.now();

  RCLCPP_INFO(
    logger_, "Configured OpenCR hardware on %s @ %d baud", serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenCRSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  serial_.flush_input();
  last_status_time_ = steady_clock_.now();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenCRSystem::safe_close(
  const rclcpp_lifecycle::State &, const char * context)
{
  (void)context;
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenCRSystem::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  return safe_close(state, "cleanup");
}

hardware_interface::CallbackReturn OpenCRSystem::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return safe_close(state, "shutdown");
}

hardware_interface::CallbackReturn OpenCRSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  watchdog_tripped_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenCRSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  send_velocity_command();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OpenCRSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 2);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]));
    interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> OpenCRSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size());
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return interfaces;
}

hardware_interface::return_type OpenCRSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!serial_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  pump_serial();

  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] += hw_velocities_[i] * period.seconds();
  }

  const auto age = (steady_clock_.now() - last_status_time_).seconds();
  if (age > status_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "No status frame received for %.2f s on %s", age, serial_port_.c_str());
  }

  if (watchdog_tripped_) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "OpenCR watchdog triggered (seq %u); check command timing", last_status_seq_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenCRSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!serial_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!send_velocity_command()) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool OpenCRSystem::open_serial()
{
  if (serial_.is_open()) {
    return true;
  }

  if (!serial_.open(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(
      logger_, "Failed to open serial port %s (baud %d)", serial_port_.c_str(), baud_rate_);
    return false;
  }

  RCLCPP_INFO(logger_, "Opened serial port %s", serial_port_.c_str());
  return true;
}

void OpenCRSystem::close_serial()
{
  if (serial_.is_open()) {
    serial_.close();
    RCLCPP_INFO(logger_, "Closed serial port %s", serial_port_.c_str());
  }
}

bool OpenCRSystem::send_velocity_command()
{
  if (hw_commands_.size() != WHEEL_COUNT) {
    RCLCPP_ERROR(
      logger_, "CMD_VEL expects %zu joints but found %zu",
      WHEEL_COUNT, hw_commands_.size());
    return false;
  }

  const std::size_t payload_len = hw_commands_.size() * sizeof(float);
  std::array<uint8_t, 2 + 3 + WHEEL_COUNT * sizeof(float) + 2> frame{};

  frame[0] = MSG_HEADER_1;
  frame[1] = MSG_HEADER_2;
  frame[2] = MSG_TYPE_CMD_VEL;
  frame[3] = static_cast<uint8_t>(payload_len);
  frame[4] = next_seq_++;

  std::size_t offset = 5;
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    float value = static_cast<float>(hw_commands_[i]);
    std::memcpy(frame.data() + offset, &value, sizeof(float));
    offset += sizeof(float);
  }

  const std::size_t body_start = 2;
  const std::size_t body_end = body_start + 3 + payload_len;
  uint16_t crc = 0xFFFF;
  for (std::size_t idx = body_start; idx < body_end; ++idx) {
    crc = crc16_ccitt_update(crc, frame[idx]);
  }

  frame[offset++] = static_cast<uint8_t>(crc & 0xFF);
  frame[offset++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  if (!serial_.write_all(frame.data(), offset)) {
    RCLCPP_ERROR(logger_, "Failed to write CMD_VEL to %s", serial_port_.c_str());
    return false;
  }

  return true;
}

void OpenCRSystem::pump_serial()
{
  std::array<uint8_t, SERIAL_BUFFER_SIZE> buffer{};
  for (int iter = 0; iter < max_read_iterations_; ++iter) {
    auto read_len = serial_.read(buffer.data(), buffer.size());
    if (read_len == std::numeric_limits<std::size_t>::max()) {
      RCLCPP_ERROR_THROTTLE(
        logger_, steady_clock_, 2000, "Serial read failure on %s", serial_port_.c_str());
      break;
    }
    if (read_len == 0) {
      break;
    }

    for (std::size_t i = 0; i < read_len; ++i) {
      Frame frame;
      if (parser_.consume(buffer[i], frame)) {
        handle_frame(frame);
      }
    }

    if (read_len < buffer.size()) {
      break;
    }
  }
}

void OpenCRSystem::handle_frame(const Frame & frame)
{
  if (frame.type == MSG_TYPE_STATUS) {
    handle_status_payload(frame.payload);
    last_status_seq_ = frame.seq;
    return;
  }

  if (frame.type == MSG_TYPE_PING || frame.type == MSG_TYPE_ECHO) {
    // Ignore responses for now; useful for diagnostics.
    return;
  }
}

void OpenCRSystem::handle_status_payload(const std::vector<uint8_t> & payload)
{
  if (payload.size() != STATUS_PAYLOAD_LEN) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "Unexpected status payload length %zu (expected %zu)",
      payload.size(), STATUS_PAYLOAD_LEN);
    return;
  }

  watchdog_tripped_ = (payload[0] & 0x01) != 0;

  for (std::size_t i = 0; i < hw_velocities_.size(); ++i) {
    float value = 0.0f;
    std::memcpy(&value, payload.data() + 1 + i * sizeof(float), sizeof(float));
    hw_velocities_[i] = static_cast<double>(value);
  }

  last_status_time_ = steady_clock_.now();
}

std::string OpenCRSystem::get_param(const std::string & key, const std::string & default_value) const
{
  auto iter = info_.hardware_parameters.find(key);
  if (iter == info_.hardware_parameters.end()) {
    return default_value;
  }
  return iter->second;
}

int OpenCRSystem::get_param_int(const std::string & key, int default_value) const
{
  auto iter = info_.hardware_parameters.find(key);
  if (iter == info_.hardware_parameters.end()) {
    return default_value;
  }
  try {
    return std::stoi(iter->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      logger_, "Parameter %s value '%s' is not an integer, using %d",
      key.c_str(), iter->second.c_str(), default_value);
  }
  return default_value;
}

double OpenCRSystem::get_param_double(const std::string & key, double default_value) const
{
  auto iter = info_.hardware_parameters.find(key);
  if (iter == info_.hardware_parameters.end()) {
    return default_value;
  }
  try {
    return std::stod(iter->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      logger_, "Parameter %s value '%s' is not numeric, using %.3f",
      key.c_str(), iter->second.c_str(), default_value);
  }
  return default_value;
}

}  // namespace opencr_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opencr_hardware::OpenCRSystem, hardware_interface::SystemInterface)
