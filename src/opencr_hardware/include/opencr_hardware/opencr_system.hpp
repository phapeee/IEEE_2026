#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace opencr_hardware
{

constexpr uint8_t MSG_HEADER_1 = 0xAA;
constexpr uint8_t MSG_HEADER_2 = 0x55;
constexpr uint8_t MSG_TYPE_CMD_VEL = 0x01;
constexpr uint8_t MSG_TYPE_PING = 0x02;
constexpr uint8_t MSG_TYPE_ECHO = 0x03;
constexpr uint8_t MSG_TYPE_STATUS = 0x10;
constexpr std::size_t WHEEL_COUNT = 4;
constexpr std::size_t STATUS_PAYLOAD_LEN = 1 + WHEEL_COUNT * sizeof(float);
constexpr std::size_t MAX_PAYLOAD_LEN = 32;

uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte);

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  bool open(const std::string & device, int baudrate);
  void close();
  bool is_open() const { return fd_ >= 0; }
  void flush_input();

  std::size_t read(uint8_t * dst, std::size_t max_len);
  bool write_all(const uint8_t * data, std::size_t len);

private:
  int fd_{-1};
};

struct Frame
{
  uint8_t type{0};
  uint8_t seq{0};
  std::vector<uint8_t> payload;
};

class FrameParser
{
public:
  FrameParser() = default;
  bool consume(uint8_t byte, Frame & frame);

private:
  enum class State
  {
    HEADER_1,
    HEADER_2,
    TYPE,
    LEN,
    SEQ,
    PAYLOAD,
    CRC_LO,
    CRC_HI
  };

  void reset();

  State state_{State::HEADER_1};
  uint8_t msg_type_{0};
  uint8_t msg_len_{0};
  uint8_t msg_seq_{0};
  uint8_t payload_idx_{0};
  uint8_t crc_low_{0};
  uint16_t crc_calc_{0};
  std::array<uint8_t, MAX_PAYLOAD_LEN> payload_{};
};

class OpenCRSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OpenCRSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool open_serial();
  void close_serial();
  hardware_interface::CallbackReturn safe_close(
    const rclcpp_lifecycle::State & previous_state, const char * context);

  bool send_velocity_command();
  void pump_serial();
  void handle_frame(const Frame & frame);
  void handle_status_payload(const std::vector<uint8_t> & payload);

  std::string get_param(const std::string & key, const std::string & default_value) const;
  int get_param_int(const std::string & key, int default_value) const;
  double get_param_double(const std::string & key, double default_value) const;

  rclcpp::Logger logger_{rclcpp::get_logger("opencr_hardware")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  SerialPort serial_;
  FrameParser parser_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  std::string serial_port_;
  int baud_rate_{115200};
  double status_timeout_sec_{0.5};
  int max_read_iterations_{16};

  uint8_t next_seq_{0};
  uint8_t last_status_seq_{0};
  bool watchdog_tripped_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_STEADY_TIME};
};

}  // namespace opencr_hardware
