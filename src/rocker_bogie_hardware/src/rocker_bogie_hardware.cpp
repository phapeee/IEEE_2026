#include "rocker_bogie_hardware/rocker_bogie_hardware.hpp"

#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace rocker_bogie_hardware
{
namespace
{
template<typename T>
T get_numeric_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & key, T default_value,
  const rclcpp::Logger & logger)
{
  const auto param_it = info.hardware_parameters.find(key);
  if (param_it == info.hardware_parameters.end())
  {
    return default_value;
  }

  std::stringstream ss(param_it->second);
  T parsed_value{};
  ss >> parsed_value;
  if (ss.fail())
  {
    RCLCPP_WARN(logger, "Failed to parse parameter '%s', falling back to default.", key.c_str());
    return default_value;
  }

  return parsed_value;
}
}  // namespace

hardware_interface::CallbackReturn RockerBogieHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.empty())
  {
    RCLCPP_ERROR(logger_, "No joints were defined for the rocker bogie hardware.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(info_.joints.size());
  hardware_joint_names_.reserve(info_.joints.size());
  joint_command_modes_.reserve(info_.joints.size());
  joint_command_indices_.reserve(info_.joints.size());
  velocity_commands_.clear();
  position_commands_.clear();
  for (const auto & joint : info_.joints)
  {
    if (!validate_joint(joint))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto hw_name_it = joint.parameters.find("hardware_joint_name");
    std::string hardware_name = (hw_name_it != joint.parameters.end() && !hw_name_it->second.empty()) ?
      hw_name_it->second : joint.name;
    if (joint_index_.count(hardware_name) != 0)
    {
      RCLCPP_ERROR(logger_, "Duplicate hardware joint name '%s' detected.", hardware_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_index_[hardware_name] = joint_names_.size();
    joint_names_.push_back(joint.name);
    hardware_joint_names_.push_back(hardware_name);
    const std::string command_interface =
      (!joint.command_interfaces.empty() && !joint.command_interfaces.front().name.empty()) ?
        joint.command_interfaces.front().name :
        hardware_interface::HW_IF_VELOCITY;
    if (command_interface == hardware_interface::HW_IF_POSITION)
    {
      joint_command_modes_.push_back(CommandMode::POSITION);
      joint_command_indices_.push_back(position_commands_.size());
      position_commands_.push_back(0.0);
    }
    else
    {
      joint_command_modes_.push_back(CommandMode::VELOCITY);
      joint_command_indices_.push_back(velocity_commands_.size());
      velocity_commands_.push_back(0.0);
    }
  }

  position_states_.assign(joint_names_.size(), 0.0);
  velocity_states_.assign(joint_names_.size(), 0.0);

  const auto state_topic_it = info_.hardware_parameters.find("state_topic");
  if (state_topic_it == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(logger_, "Parameter 'state_topic' is required.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  state_topic_ = state_topic_it->second;

  const auto command_topic_it = info_.hardware_parameters.find("command_topic");
  if (command_topic_it == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(logger_, "Parameter 'command_topic' is required.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  command_topic_ = command_topic_it->second;

  const auto namespace_it = info_.hardware_parameters.find("parameter_namespace");
  if (namespace_it != info_.hardware_parameters.end() && !namespace_it->second.empty())
  {
    node_name_ = namespace_it->second;
  }

  state_qos_depth_ = get_numeric_parameter<size_t>(info_, "state_qos_depth", state_qos_depth_, logger_);
  command_qos_depth_ =
    get_numeric_parameter<size_t>(info_, "command_qos_depth", command_qos_depth_, logger_);
  state_timeout_ = get_numeric_parameter<double>(info_, "state_timeout", state_timeout_, logger_);
  command_publish_period_ = get_numeric_parameter<double>(
    info_, "command_publish_period", command_publish_period_, logger_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RockerBogieHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> RockerBogieHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());
  for (size_t idx = 0; idx < joint_names_.size(); ++idx)
  {
    const auto mode = joint_command_modes_[idx];
    const auto command_index = joint_command_indices_[idx];
    if (mode == CommandMode::POSITION)
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint_names_[idx], hardware_interface::HW_IF_POSITION, &position_commands_[command_index]));
    }
    else
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint_names_[idx], hardware_interface::HW_IF_VELOCITY, &velocity_commands_[command_index]));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RockerBogieHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  node_ = std::make_shared<rclcpp::Node>(node_name_, options);
  executor_.add_node(node_);

  auto state_qos = rclcpp::QoS(rclcpp::KeepLast(state_qos_depth_)).best_effort();
  auto command_qos = rclcpp::QoS(rclcpp::KeepLast(command_qos_depth_)).reliable();

  state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, state_qos,
    std::bind(&RockerBogieHardware::handle_state_message, this, std::placeholders::_1));
  command_publisher_ =
    node_->create_publisher<sensor_msgs::msg::JointState>(command_topic_, command_qos);

  last_state_time_ = node_->now();
  last_command_publish_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

  RCLCPP_INFO(node_->get_logger(),
    "Rocker bogie hardware configured (state topic: %s, command topic: %s).",
    state_topic_.c_str(), command_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RockerBogieHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  state_subscription_.reset();
  command_publisher_.reset();

  if (node_)
  {
    executor_.remove_node(node_);
    node_.reset();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RockerBogieHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
  last_command_publish_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RockerBogieHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RockerBogieHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (node_)
  {
    executor_.spin_some();
  }

  const double elapsed_since_state = (time - last_state_time_).seconds();
  if (elapsed_since_state > state_timeout_)
  {
    if (node_)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "No joint state update received for %.2f seconds.", elapsed_since_state);
    }
    else
    {
      RCLCPP_WARN(logger_, "No joint state update received for %.2f seconds.", elapsed_since_state);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RockerBogieHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  publish_command(time);
  return hardware_interface::return_type::OK;
}

void RockerBogieHardware::handle_state_message(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!node_)
  {
    return;
  }

  last_state_time_ = node_->now();

  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    const auto name_it = joint_index_.find(msg->name[i]);
    if (name_it == joint_index_.end())
    {
      continue;
    }
    const size_t idx = name_it->second;

    if (i < msg->position.size())
    {
      position_states_[idx] = msg->position[i];
    }
    if (i < msg->velocity.size())
    {
      velocity_states_[idx] = msg->velocity[i];
    }
  }
}

void RockerBogieHardware::publish_command(const rclcpp::Time & now)
{
  if (!command_publisher_)
  {
    return;
  }

  const bool first_publish = last_command_publish_time_.nanoseconds() == 0;
  const double elapsed = first_publish ? std::numeric_limits<double>::infinity() :
    (now - last_command_publish_time_).seconds();

  if (!first_publish && elapsed < command_publish_period_)
  {
    return;
  }

  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = now;
  cmd_msg.name = hardware_joint_names_;
  cmd_msg.position.assign(cmd_msg.name.size(), 0.0);
  cmd_msg.velocity.assign(cmd_msg.name.size(), 0.0);
  for (size_t idx = 0; idx < joint_names_.size(); ++idx)
  {
    const auto mode = joint_command_modes_[idx];
    const auto command_index = joint_command_indices_[idx];
    if (mode == CommandMode::POSITION)
    {
      cmd_msg.position[idx] = position_commands_[command_index];
    }
    else
    {
      cmd_msg.velocity[idx] = velocity_commands_[command_index];
    }
  }

  command_publisher_->publish(cmd_msg);
  last_command_publish_time_ = now;
}

bool RockerBogieHardware::validate_joint(const hardware_interface::ComponentInfo & joint) const
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

  const auto has_position_command = std::find_if(
    joint.command_interfaces.begin(), joint.command_interfaces.end(),
    [](const hardware_interface::InterfaceInfo & command)
    {
      return command.name == hardware_interface::HW_IF_POSITION;
    }) != joint.command_interfaces.end();

  if (!has_position || !has_velocity || (!has_velocity_command && !has_position_command))
  {
    RCLCPP_ERROR(
      logger_, "Joint '%s' lacks the required state/command interfaces.", joint.name.c_str());
    return false;
  }

  return true;
}
}  // namespace rocker_bogie_hardware

PLUGINLIB_EXPORT_CLASS(rocker_bogie_hardware::RockerBogieHardware, hardware_interface::SystemInterface)
