#include "i2c_velocity_hardware/i2c_velocity_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace i2c_velocity_hardware
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int kInt16Min = -32768;
constexpr int kInt16Max = 32767;

int get_int_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key, int default_value,
  const rclcpp::Logger & logger)
{
  const auto param_it = info.hardware_parameters.find(key);
  if (param_it == info.hardware_parameters.end())
  {
    return default_value;
  }

  try
  {
    size_t consumed = 0;
    int value = std::stoi(param_it->second, &consumed, 0);
    if (consumed != param_it->second.size())
    {
      throw std::invalid_argument("Unexpected characters");
    }
    return value;
  }
  catch (const std::exception &)
  {
    RCLCPP_WARN(logger, "Failed to parse parameter '%s', falling back to default.", key.c_str());
    return default_value;
  }
}

double get_double_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key, double default_value,
  const rclcpp::Logger & logger)
{
  const auto param_it = info.hardware_parameters.find(key);
  if (param_it == info.hardware_parameters.end())
  {
    return default_value;
  }

  try
  {
    size_t consumed = 0;
    double value = std::stod(param_it->second, &consumed);
    if (consumed != param_it->second.size())
    {
      throw std::invalid_argument("Unexpected characters");
    }
    return value;
  }
  catch (const std::exception &)
  {
    RCLCPP_WARN(logger, "Failed to parse parameter '%s', falling back to default.", key.c_str());
    return default_value;
  }
}

size_t get_size_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key, size_t default_value,
  const rclcpp::Logger & logger)
{
  const auto param_it = info.hardware_parameters.find(key);
  if (param_it == info.hardware_parameters.end())
  {
    return default_value;
  }

  try
  {
    size_t consumed = 0;
    size_t value = std::stoul(param_it->second, &consumed, 0);
    if (consumed != param_it->second.size())
    {
      throw std::invalid_argument("Unexpected characters");
    }
    return value;
  }
  catch (const std::exception &)
  {
    RCLCPP_WARN(logger, "Failed to parse parameter '%s', falling back to default.", key.c_str());
    return default_value;
  }
}

bool get_bool_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key, bool default_value,
  const rclcpp::Logger & logger)
{
  const auto param_it = info.hardware_parameters.find(key);
  if (param_it == info.hardware_parameters.end())
  {
    return default_value;
  }

  std::string value = param_it->second;
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (value == "true" || value == "1" || value == "yes" || value == "on")
  {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off")
  {
    return false;
  }

  RCLCPP_WARN(logger, "Failed to parse boolean parameter '%s', falling back to default.",
    key.c_str());
  return default_value;
}

bool parse_bool(const std::string & value, bool * output)
{
  if (!output)
  {
    return false;
  }
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on")
  {
    *output = true;
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off")
  {
    *output = false;
    return true;
  }
  return false;
}

bool parse_double(const std::string & value, double * output)
{
  if (!output)
  {
    return false;
  }
  try
  {
    size_t consumed = 0;
    double parsed = std::stod(value, &consumed);
    if (consumed != value.size())
    {
      return false;
    }
    *output = parsed;
    return true;
  }
  catch (const std::exception &)
  {
    return false;
  }
}
}  // namespace

hardware_interface::CallbackReturn I2cVelocityHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.empty())
  {
    RCLCPP_ERROR(logger_, "No joints were defined for the I2C hardware.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  expected_joints_ = get_size_parameter(info_, "expected_joints", expected_joints_, logger_);
  if (expected_joints_ != info_.joints.size())
  {
    RCLCPP_ERROR(
      logger_, "Expected %zu joints but got %zu.", expected_joints_, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints)
  {
    if (!validate_joint(joint))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  if (!parse_joint_order(info_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_states_.assign(joint_names_.size(), 0.0);
  velocity_states_.assign(joint_names_.size(), 0.0);
  velocity_commands_.assign(joint_names_.size(), 0.0);

  i2c_address_ = get_int_parameter(info_, "i2c_address", i2c_address_, logger_);
  command_register_ = static_cast<uint8_t>(
    get_int_parameter(info_, "command_register", command_register_, logger_));
  state_register_ = static_cast<uint8_t>(
    get_int_parameter(info_, "state_register", state_register_, logger_));
  centi_rps_scale_ = get_double_parameter(info_, "centi_rps_scale", centi_rps_scale_, logger_);
  max_rps_ = get_double_parameter(info_, "max_rps", max_rps_, logger_);
  service_timeout_sec_ =
    get_double_parameter(info_, "service_timeout_sec", service_timeout_sec_, logger_);
  log_measured_velocities_ =
    get_bool_parameter(info_, "log_measured_velocities", log_measured_velocities_, logger_);
  log_measured_velocities_ms_ = std::max(
    1, get_int_parameter(info_, "log_measured_velocities_ms", log_measured_velocities_ms_, logger_));

  const auto service_it = info_.hardware_parameters.find("i2c_service");
  if (service_it != info_.hardware_parameters.end() && !service_it->second.empty())
  {
    i2c_service_ = service_it->second;
  }
  const auto node_name_it = info_.hardware_parameters.find("node_name");
  if (node_name_it != info_.hardware_parameters.end() && !node_name_it->second.empty())
  {
    node_name_ = node_name_it->second;
  }
  const auto node_ns_it = info_.hardware_parameters.find("node_namespace");
  if (node_ns_it != info_.hardware_parameters.end() && !node_ns_it->second.empty())
  {
    node_namespace_ = node_ns_it->second;
  }

  if (centi_rps_scale_ <= 0.0)
  {
    RCLCPP_ERROR(logger_, "centi_rps_scale must be positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> I2cVelocityHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 2);
  for (size_t idx = 0; idx < joint_names_.size(); ++idx)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[idx], hardware_interface::HW_IF_POSITION,
      &position_states_[idx]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[idx], hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[idx]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> I2cVelocityHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());
  for (size_t idx = 0; idx < joint_names_.size(); ++idx)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_names_[idx], hardware_interface::HW_IF_VELOCITY, &velocity_commands_[idx]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn I2cVelocityHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  node_ = std::make_shared<rclcpp::Node>(node_name_, node_namespace_, options);
  executor_.add_node(node_);

  client_ = node_->create_client<i2c_manager::srv::I2cTransfer>(i2c_service_);
  const double connect_timeout_sec = std::max(service_timeout_sec_, 1.0);
  const auto timeout = std::chrono::duration<double>(connect_timeout_sec);
  if (!client_->wait_for_service(timeout))
  {
    RCLCPP_ERROR(
      logger_, "I2C manager service '%s' not available.", i2c_service_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  last_read_time_ = rclcpp::Time(0, 0, steady_clock_.get_clock_type());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn I2cVelocityHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (node_)
  {
    executor_.remove_node(node_);
    node_.reset();
  }
  client_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn I2cVelocityHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  last_read_time_ = rclcpp::Time(0, 0, steady_clock_.get_clock_type());
  if (!write_target_velocities())
  {
    RCLCPP_WARN(logger_, "Failed to send zero-velocity command on activate.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn I2cVelocityHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  if (!write_target_velocities())
  {
    RCLCPP_WARN(logger_, "Failed to send zero-velocity command on deactivate.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type I2cVelocityHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!client_)
  {
    RCLCPP_ERROR(logger_, "I2C manager client is not available.");
    return hardware_interface::return_type::ERROR;
  }

  if (!read_measured_velocities())
  {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "Failed to read measured velocities over I2C.");
    return hardware_interface::return_type::ERROR;
  }

  double dt = period.seconds();
  if (dt <= 0.0 && last_read_time_.nanoseconds() != 0)
  {
    dt = (time - last_read_time_).seconds();
  }
  if (dt > 0.0)
  {
    for (size_t i = 0; i < position_states_.size(); ++i)
    {
      position_states_[i] += velocity_states_[i] * dt;
    }
  }
  last_read_time_ = time;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type I2cVelocityHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!client_)
  {
    RCLCPP_ERROR(logger_, "I2C manager client is not available.");
    return hardware_interface::return_type::ERROR;
  }

  if (!write_target_velocities())
  {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "Failed to write target velocities over I2C.");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool I2cVelocityHardware::read_measured_velocities()
{
  const size_t motor_count = joint_names_.size();
  std::vector<uint8_t> tx_buf(1, state_register_);
  std::vector<uint8_t> rx_buf;
  if (!call_transfer(tx_buf, motor_count * 2, &rx_buf))
  {
    return false;
  }
  if (rx_buf.size() != motor_count * 2)
  {
    RCLCPP_ERROR(logger_, "I2C read returned %zu bytes, expected %zu.", rx_buf.size(),
      motor_count * 2);
    return false;
  }

  for (size_t hw_index = 0; hw_index < motor_count; ++hw_index)
  {
    const uint8_t lo = rx_buf[(hw_index * 2) + 0];
    const uint8_t hi = rx_buf[(hw_index * 2) + 1];
    const int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(lo) |
      (static_cast<uint16_t>(hi) << 8));
    const double rps = static_cast<double>(raw) / centi_rps_scale_;
    const double rad_s = rps * kTwoPi;
    const size_t joint_index = hw_to_joint_index_[hw_index];
    velocity_states_[joint_index] = rad_s * velocity_signs_[joint_index];
  }

  if (log_measured_velocities_)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Measured velocities (rps): ";
    for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx)
    {
      if (joint_idx)
      {
        oss << ", ";
      }
      const double rps = velocity_states_[joint_idx] / kTwoPi;
      oss << joint_names_[joint_idx] << '=' << rps;
    }
    RCLCPP_INFO_THROTTLE(logger_, steady_clock_, log_measured_velocities_ms_, "%s",
      oss.str().c_str());
  }

  return true;
}

bool I2cVelocityHardware::write_target_velocities()
{
  const size_t motor_count = joint_names_.size();
  std::vector<uint8_t> buf(1 + (motor_count * 2), 0);
  buf[0] = command_register_;

  for (size_t joint_idx = 0; joint_idx < motor_count; ++joint_idx)
  {
    const size_t hw_index = joint_to_hw_index_[joint_idx];
    double rps = (velocity_commands_[joint_idx] * velocity_signs_[joint_idx]) / kTwoPi;
    if (max_rps_ > 0.0)
    {
      rps = std::clamp(rps, -max_rps_, max_rps_);
    }
    const double scaled = rps * centi_rps_scale_;
    int32_t value = static_cast<int32_t>(std::llround(scaled));
    value = std::clamp(value, kInt16Min, kInt16Max);
    const int16_t packed = static_cast<int16_t>(value);
    buf[1 + (hw_index * 2) + 0] = static_cast<uint8_t>(packed & 0xFF);
    buf[1 + (hw_index * 2) + 1] = static_cast<uint8_t>((packed >> 8) & 0xFF);
  }

  if (log_measured_velocities_)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Desired velocities (rps): ";
    for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx)
    {
      if (joint_idx)
      {
        oss << ", ";
      }
      double rps = velocity_commands_[joint_idx] / kTwoPi;
      if (max_rps_ > 0.0)
      {
        rps = std::clamp(rps, -max_rps_, max_rps_);
      }
      oss << joint_names_[joint_idx] << '=' << rps;
    }
    RCLCPP_INFO_THROTTLE(logger_, steady_clock_, log_measured_velocities_ms_, "%s",
      oss.str().c_str());
  }

  return call_transfer(buf, 0, nullptr);
}

bool I2cVelocityHardware::validate_joint(const hardware_interface::ComponentInfo & joint) const
{
  const auto has_position = std::find_if(
    joint.state_interfaces.begin(), joint.state_interfaces.end(),
    [](const hardware_interface::InterfaceInfo & state)
    {
      return state.name == hardware_interface::HW_IF_POSITION;
    }) != joint.state_interfaces.end();

  const auto has_velocity = std::find_if(
    joint.state_interfaces.begin(), joint.state_interfaces.end(),
    [](const hardware_interface::InterfaceInfo & state)
    {
      return state.name == hardware_interface::HW_IF_VELOCITY;
    }) != joint.state_interfaces.end();

  const auto has_velocity_command = std::find_if(
    joint.command_interfaces.begin(), joint.command_interfaces.end(),
    [](const hardware_interface::InterfaceInfo & command)
    {
      return command.name == hardware_interface::HW_IF_VELOCITY;
    }) != joint.command_interfaces.end();

  if (!has_position || !has_velocity || !has_velocity_command)
  {
    RCLCPP_ERROR(
      logger_, "Joint '%s' lacks the required state/command interfaces.", joint.name.c_str());
    return false;
  }

  return true;
}

bool I2cVelocityHardware::parse_joint_order(const hardware_interface::HardwareInfo & info)
{
  const size_t joint_count = info.joints.size();
  joint_to_hw_index_.assign(joint_count, 0);
  hw_to_joint_index_.assign(joint_count, 0);
  velocity_signs_.assign(joint_count, 1.0);
  std::vector<bool> used(joint_count, false);

  for (size_t i = 0; i < joint_count; ++i)
  {
    const auto & joint = info.joints[i];
    size_t hw_index = i;
    double velocity_sign = 1.0;
    bool has_velocity_sign = false;
    bool has_invert_direction = false;
    const auto param_it = joint.parameters.find("i2c_index");
    if (param_it != joint.parameters.end() && !param_it->second.empty())
    {
      try
      {
        size_t consumed = 0;
        hw_index = std::stoul(param_it->second, &consumed, 0);
        if (consumed != param_it->second.size())
        {
          throw std::invalid_argument("Unexpected characters");
        }
      }
      catch (const std::exception &)
      {
        RCLCPP_ERROR(
          logger_, "Failed to parse i2c_index for joint '%s'.", joint.name.c_str());
        return false;
      }
    }

    const auto sign_it = joint.parameters.find("velocity_sign");
    if (sign_it != joint.parameters.end() && !sign_it->second.empty())
    {
      if (!parse_double(sign_it->second, &velocity_sign))
      {
        RCLCPP_ERROR(
          logger_, "Failed to parse velocity_sign for joint '%s'.", joint.name.c_str());
        return false;
      }
      if (velocity_sign == 0.0)
      {
        RCLCPP_ERROR(
          logger_, "velocity_sign for joint '%s' cannot be 0.", joint.name.c_str());
        return false;
      }
      has_velocity_sign = true;
    }

    const auto invert_it = joint.parameters.find("invert_direction");
    if (invert_it != joint.parameters.end() && !invert_it->second.empty())
    {
      bool invert = false;
      if (!parse_bool(invert_it->second, &invert))
      {
        RCLCPP_ERROR(
          logger_, "Failed to parse invert_direction for joint '%s'.", joint.name.c_str());
        return false;
      }
      if (invert)
      {
        velocity_sign *= -1.0;
      }
      has_invert_direction = true;
    }

    if (has_velocity_sign && has_invert_direction)
    {
      RCLCPP_WARN(
        logger_,
        "Joint '%s' sets both velocity_sign and invert_direction; velocity_sign is applied first.",
        joint.name.c_str());
    }

    if (hw_index >= joint_count)
    {
      RCLCPP_ERROR(
        logger_, "i2c_index %zu for joint '%s' is out of range.", hw_index, joint.name.c_str());
      return false;
    }
    if (used[hw_index])
    {
      RCLCPP_ERROR(logger_, "Duplicate i2c_index %zu detected.", hw_index);
      return false;
    }

    used[hw_index] = true;
    joint_to_hw_index_[i] = hw_index;
    hw_to_joint_index_[hw_index] = i;
    velocity_signs_[i] = velocity_sign;
  }

  return true;
}

bool I2cVelocityHardware::call_transfer(
  const std::vector<uint8_t> & write_data, size_t read_length, std::vector<uint8_t> * read_data)
{
  if (!client_ || !node_)
  {
    return false;
  }

  auto request = std::make_shared<i2c_manager::srv::I2cTransfer::Request>();
  request->address = static_cast<uint8_t>(i2c_address_);
  request->write_data = write_data;
  request->read_length = static_cast<uint32_t>(read_length);

  auto future = client_->async_send_request(request);
  const auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  const auto result = executor_.spin_until_future_complete(future, timeout);
  if (result != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger_, "I2C transfer timed out.");
    return false;
  }

  const auto response = future.get();
  if (!response->success)
  {
    RCLCPP_ERROR(
      logger_, "I2C transfer failed (error_code=%d): %s", response->error_code,
      response->error.c_str());
    return false;
  }

  if (read_data)
  {
    *read_data = response->read_data;
  }

  return true;
}
}  // namespace i2c_velocity_hardware

PLUGINLIB_EXPORT_CLASS(i2c_velocity_hardware::I2cVelocityHardware, hardware_interface::SystemInterface)
