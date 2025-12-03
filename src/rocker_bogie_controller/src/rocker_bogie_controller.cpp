#include "rocker_bogie_controller/rocker_bogie_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace rocker_bogie_controller
{
controller_interface::CallbackReturn RockerBogieController::on_init()
{
  wheel_joint_names_ = auto_declare<std::vector<std::string>>("wheel_joints", {});
  if (wheel_joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'wheel_joints' must list wheel joint names.");
    return controller_interface::CallbackReturn::ERROR;
  }

  servo_joint_names_ = auto_declare<std::vector<std::string>>("servo_joints", {});
  auto servo_direction_param =
    auto_declare<std::vector<std::string>>("servo_directions", std::vector<std::string>{});
  servo_direction_overrides_.clear();
  for (const auto & entry : servo_direction_param)
  {
    const auto colon = entry.find(':');
    if (colon == std::string::npos)
    {
      continue;
    }
    auto trim = [](const std::string & input)
    {
      const auto start = input.find_first_not_of(" \t");
      if (start == std::string::npos)
      {
        return std::string{};
      }
      const auto end = input.find_last_not_of(" \t");
      return input.substr(start, end - start + 1);
    };
    const std::string name = trim(entry.substr(0, colon));
    const std::string value_str = trim(entry.substr(colon + 1));
    try
    {
      const double value = std::stod(value_str);
      servo_direction_overrides_[name] = value;
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to parse servo direction entry '%s'.", entry.c_str());
    }
  }
  const double min_angle_deg = auto_declare<double>("min_steering_angle_deg", 3.0);
  const double max_angle_deg = auto_declare<double>("max_steering_angle_deg", 10.0);
  min_steering_angle_ = min_angle_deg * M_PI / 180.0;
  max_steering_angle_ = max_angle_deg * M_PI / 180.0;
  const double servo_limit_deg = auto_declare<double>("servo_max_angle_deg", 90.0);
  servo_max_angle_ = std::clamp(servo_limit_deg, 1.0, 180.0) * M_PI / 180.0;
  strafe_deadband_ = auto_declare<double>("strafe_deadband", 0.01);
  wheel_distance_x_ = auto_declare<double>("wheel_distance_x", 0.1);
  wheel_distance_y_ = auto_declare<double>("wheel_distance_y", 0.1);
  yaw_deadband_ = auto_declare<double>("yaw_deadband", 0.01);

  auto default_state_joints = wheel_joint_names_;
  default_state_joints.insert(
    default_state_joints.end(), servo_joint_names_.begin(), servo_joint_names_.end());
  state_joint_names_ =
    auto_declare<std::vector<std::string>>("state_joints", default_state_joints);
  if (state_joint_names_.empty())
  {
    state_joint_names_ = default_state_joints;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RockerBogieController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(wheel_joint_names_.size() + servo_joint_names_.size());
  for (const auto & joint : wheel_joint_names_)
  {
    conf.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint : servo_joint_names_)
  {
    conf.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration RockerBogieController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(state_joint_names_.size() * 2);
  for (const auto & joint : state_joint_names_)
  {
    conf.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

controller_interface::CallbackReturn RockerBogieController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  cmd_vel_topic_ = auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
  publish_joint_states_ = auto_declare<bool>("publish_joint_states", true);
  publish_rate_ = auto_declare<double>("publish_rate", 50.0);
  cmd_vel_timeout_ = auto_declare<double>("cmd_vel_timeout", 0.5);
  left_forward_direction_ = auto_declare<double>("left_wheel_direction", 1.0);
  right_forward_direction_ = auto_declare<double>("right_wheel_direction", -1.0);

  publish_period_ = publish_rate_ > 0.0 ?
    rclcpp::Duration::from_seconds(1.0 / publish_rate_) : rclcpp::Duration(0, 0);
  last_linear_cmd_ = 0.0;
  last_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_state_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  rclcpp::QoS cmd_vel_qos{rclcpp::SystemDefaultsQoS()};
  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic_, cmd_vel_qos,
    std::bind(&RockerBogieController::cmd_vel_callback, this, std::placeholders::_1));

  if (publish_joint_states_)
  {
    joint_state_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_states", rclcpp::SystemDefaultsQoS());
  }
  else
  {
    joint_state_publisher_.reset();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RockerBogieController::on_activate(
  const rclcpp_lifecycle::State &)
{
  wheel_handles_.clear();
  state_handles_.clear();
  servo_handles_.clear();
  steering_ready_ = true;

  for (const auto & joint : wheel_joint_names_)
  {
    auto * cmd = find_command_handle(joint);
    auto * pos_state = find_state_handle(joint, hardware_interface::HW_IF_POSITION);
    auto * vel_state = find_state_handle(joint, hardware_interface::HW_IF_VELOCITY);
    if (!cmd || !pos_state || !vel_state)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to acquire interfaces for wheel joint '%s'.",
        joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    const bool is_left = joint.find("Left") != std::string::npos;
    WheelHandle handle;
    handle.name = joint;
    handle.command = cmd;
    handle.position_state = pos_state;
    handle.velocity_state = vel_state;
    handle.direction = is_left ? left_forward_direction_ : right_forward_direction_;
    wheel_handles_.push_back(handle);
  }

  for (const auto & joint : state_joint_names_)
  {
    StateHandle handle;
    handle.position = find_state_handle(joint, hardware_interface::HW_IF_POSITION);
    handle.velocity = find_state_handle(joint, hardware_interface::HW_IF_VELOCITY);
    state_handles_.emplace(joint, handle);
  }

  for (const auto & joint : servo_joint_names_)
  {
    auto * cmd = find_command_handle(joint, hardware_interface::HW_IF_POSITION);
    auto * pos_state = find_state_handle(joint, hardware_interface::HW_IF_POSITION);
    if (!cmd || !pos_state)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to acquire interfaces for servo joint '%s'.",
        joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    ServoHandle servo;
    servo.name = joint;
    servo.command = cmd;
    servo.position_state = pos_state;
    const auto dir_it = servo_direction_overrides_.find(joint);
    servo.direction = (dir_it != servo_direction_overrides_.end() && dir_it->second != 0.0) ?
      dir_it->second : 1.0;
    servo_handles_.push_back(servo);
  }

  for (auto & wheel : wheel_handles_)
  {
    wheel.command->set_value(0.0);
  }
  for (auto & servo : servo_handles_)
  {
    servo.command->set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RockerBogieController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & wheel : wheel_handles_)
  {
    wheel.command->set_value(0.0);
  }
  wheel_handles_.clear();
  state_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RockerBogieController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  double desired_linear_x = 0.0;
  double desired_linear_y = 0.0;
  double desired_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (cmd_vel_timeout_ <= 0.0 || (time - last_cmd_time_).seconds() <= cmd_vel_timeout_)
    {
      desired_linear_x = last_linear_cmd_;
      desired_linear_y = last_lateral_cmd_;
      desired_yaw = last_yaw_cmd_;
    }
  }

  const double yaw_magnitude = std::abs(desired_yaw);
  if (yaw_magnitude > yaw_deadband_)
  {
    const double diag_distance = std::hypot(wheel_distance_x_, wheel_distance_y_);
    const double offset_angle = std::atan2(wheel_distance_y_, wheel_distance_x_);
    const double front_target = -M_PI_2 + offset_angle;
    const double rear_target = M_PI_2 - offset_angle;
    std::unordered_map<std::string, double> servo_targets{
      {"Left_Front_Servo", front_target},
      {"Right_Rear_Servo", front_target},
      {"Left_Rear_Servo", rear_target},
      {"Right_Front_Servo", rear_target}};

    double error_sum = 0.0;
    size_t targeted_servos = 0;
    for (auto & servo : servo_handles_)
    {
      auto target_it = servo_targets.find(servo.name);
      if (target_it == servo_targets.end())
      {
        // Hold current angle for untargeted servos
        const double current_angle = servo.direction * servo.position_state->get_value();
        const double direction = std::abs(servo.direction) < 1e-6 ? 1.0 : servo.direction;
        servo.command->set_value(current_angle / direction);
        continue;
      }
      double servo_command_angle =
        std::clamp(target_it->second, -servo_max_angle_, servo_max_angle_);
      const double direction = std::abs(servo.direction) < 1e-6 ? 1.0 : servo.direction;
      servo.command->set_value(servo_command_angle / direction);
      const double current_angle = servo.direction * servo.position_state->get_value();
      const double servo_error = std::abs(shortest_angular_distance(current_angle, servo_command_angle));
      error_sum += servo_error;
      targeted_servos++;
    }
    if (targeted_servos > 0)
    {
      const double avg_error = error_sum / static_cast<double>(targeted_servos);
      if (avg_error > max_steering_angle_)
      {
        steering_ready_ = false;
      }
      else if (avg_error < min_steering_angle_)
      {
        steering_ready_ = true;
      }
    }

    const double wheel_speed = steering_ready_ ? (diag_distance * desired_yaw) : 0.0;
    const double mid_wheel_speed = steering_ready_ ? (wheel_distance_y_ * desired_yaw) : 0.0;
    const std::unordered_map<std::string, double> wheel_targets{
      {"Left_Front_Wheel", -wheel_speed},
      {"Left_Rear_Wheel", -wheel_speed},
      {"Left_Mid_Wheel", -mid_wheel_speed},
      {"Right_Front_Wheel", wheel_speed},
      {"Right_Rear_Wheel", wheel_speed},
      {"Right_Mid_Wheel", mid_wheel_speed}};

    for (auto & wheel : wheel_handles_)
    {
      auto wheel_it = wheel_targets.find(wheel.name);
      double command = 0.0;
      if (wheel_it != wheel_targets.end())
      {
        command = wheel.direction * wheel_it->second;
      }
      wheel.command->set_value(command);
    }

    publish_joint_states(time);
    return controller_interface::return_type::OK;
  }

  const double magnitude = std::hypot(desired_linear_x, desired_linear_y);
  double desired_speed = magnitude;
  double desired_angle = 0.0;
  double forward_sign = (desired_linear_x >= 0.0) ? 1.0 : -1.0;
  if (std::abs(desired_linear_y) > strafe_deadband_)
  {
    // Steering angle follows atan(abs(vx)/vy), clamped to limits
    desired_angle = std::atan2(desired_linear_y, std::abs(desired_linear_x));
    desired_angle = std::clamp(desired_angle, -servo_max_angle_, servo_max_angle_);
    if (forward_sign < 0.0)
    {
      desired_angle = -desired_angle;
    }
  }
  else
  {
    desired_angle = 0.0;
  }

  double avg_servo_angle = 0.0;
  if (!servo_handles_.empty())
  {
    double sum_angles = 0.0;
    for (auto & servo : servo_handles_)
    {
      const double current_angle = servo.direction * servo.position_state->get_value();
      double servo_command_angle = desired_angle;
      servo_command_angle = std::clamp(servo_command_angle, -servo_max_angle_, servo_max_angle_);
      const double direction = std::abs(servo.direction) < 1e-6 ? 1.0 : servo.direction;
      servo.command->set_value(servo_command_angle / direction);
      sum_angles += current_angle;
    }
    avg_servo_angle = sum_angles / static_cast<double>(servo_handles_.size());
  }

  const double steering_error = shortest_angular_distance(avg_servo_angle, desired_angle);
  if (std::abs(steering_error) > max_steering_angle_)
  {
    steering_ready_ = false;
  }
  else if (std::abs(steering_error) < min_steering_angle_)
  {
    steering_ready_ = true;
  }

  const double wheel_command_speed = steering_ready_ ? desired_speed : 0.0;

  for (auto & wheel : wheel_handles_)
  {
    wheel.command->set_value(wheel.direction * forward_sign * wheel_command_speed);
  }

  publish_joint_states(time);

  return controller_interface::return_type::OK;
}

void RockerBogieController::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  last_linear_cmd_ = msg->twist.linear.x;
  last_lateral_cmd_ = msg->twist.linear.y;
  last_yaw_cmd_ = msg->twist.angular.z;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
  {
    last_cmd_time_ = get_node()->now();
  }
  else
  {
    last_cmd_time_ = rclcpp::Time(msg->header.stamp);
  }
}

hardware_interface::LoanedCommandInterface * RockerBogieController::find_command_handle(
  const std::string & joint_name, const std::string & interface_name)
{
  auto it = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&](hardware_interface::LoanedCommandInterface & interface)
    {
      return interface.get_prefix_name() == joint_name && interface.get_interface_name() == interface_name;
    });
  return it != command_interfaces_.end() ? &(*it) : nullptr;
}

hardware_interface::LoanedStateInterface * RockerBogieController::find_state_handle(
  const std::string & joint_name, const std::string & interface_name)
{
  auto it = std::find_if(
    state_interfaces_.begin(), state_interfaces_.end(),
    [&](hardware_interface::LoanedStateInterface & interface)
    {
      return interface.get_prefix_name() == joint_name &&
             interface.get_interface_name() == interface_name;
    });
  return it != state_interfaces_.end() ? &(*it) : nullptr;
}

double RockerBogieController::normalized_angle(double angle) const
{
  double result = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (result < 0.0)
  {
    result += 2.0 * M_PI;
  }
  return result - M_PI;
}

double RockerBogieController::shortest_angular_distance(double from, double to) const
{
  return normalized_angle(to - from);
}

void RockerBogieController::publish_joint_states(const rclcpp::Time & time)
{
  if (!publish_joint_states_ || !joint_state_publisher_)
  {
    return;
  }
  if (publish_period_.nanoseconds() > 0 &&
    last_state_publish_time_.nanoseconds() != 0 &&
    (time - last_state_publish_time_) < publish_period_)
  {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = time;
  msg.name.reserve(state_handles_.size());
  msg.position.reserve(state_handles_.size());
  msg.velocity.reserve(state_handles_.size());

  for (const auto & joint : state_joint_names_)
  {
    msg.name.push_back(joint);
    const auto handle_it = state_handles_.find(joint);
    double position = 0.0;
    double velocity = 0.0;
    if (handle_it != state_handles_.end())
    {
      if (handle_it->second.position)
      {
        position = handle_it->second.position->get_value();
      }
      if (handle_it->second.velocity)
      {
        velocity = handle_it->second.velocity->get_value();
      }
    }
    msg.position.push_back(position);
    msg.velocity.push_back(velocity);
  }

  joint_state_publisher_->publish(msg);
  last_state_publish_time_ = time;
}
}  // namespace rocker_bogie_controller

PLUGINLIB_EXPORT_CLASS(
  rocker_bogie_controller::RockerBogieController, controller_interface::ControllerInterface)
