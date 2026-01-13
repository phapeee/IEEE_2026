#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <smacc2_msgs/msg/smacc_event.hpp>

namespace isaac_sim_gpio
{

constexpr double kEqualityEpsilon = 1e-6;

enum class Operation
{
  kGreater,
  kLess,
  kEqual,
  kGreaterEqual,
  kLessEqual,
  kInvalid
};

struct JointRule
{
  std::string id;
  std::string joint_name;
  std::string gpio_name;
  double threshold{0.0};
  bool absolute_value{false};
  std::string operation_label;
  Operation operation{Operation::kInvalid};
  std::optional<bool> last_state;
};

Operation parse_operation(const std::string & op)
{
  if (op == ">") {
    return Operation::kGreater;
  }
  if (op == "<") {
    return Operation::kLess;
  }
  if (op == "=") {
    return Operation::kEqual;
  }
  if (op == ">=") {
    return Operation::kGreaterEqual;
  }
  if (op == "<=") {
    return Operation::kLessEqual;
  }
  return Operation::kInvalid;
}

bool evaluate_operation(Operation op, double value, double threshold)
{
  switch (op) {
    case Operation::kGreater:
      return value > threshold;
    case Operation::kLess:
      return value < threshold;
    case Operation::kEqual:
      return std::fabs(value - threshold) <= kEqualityEpsilon;
    case Operation::kGreaterEqual:
      return value >= threshold;
    case Operation::kLessEqual:
      return value <= threshold;
    case Operation::kInvalid:
    default:
      return false;
  }
}

class IsaacSimGpioNode : public rclcpp::Node
{
public:
  IsaacSimGpioNode()
  : rclcpp::Node("isaac_sim_gpio")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "joint_states");
    output_topic_ = declare_parameter<std::string>("output_topic", "smacc2/isaac_sim_gpio");
    event_source_ = declare_parameter<std::string>("event_source", "isaac_sim_gpio_node");

    const auto joint_ids = declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    configure_joints(joint_ids);

    event_pub_ = create_publisher<smacc2_msgs::msg::SmaccEvent>(output_topic_, 10);

    auto qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, qos,
      std::bind(&IsaacSimGpioNode::handle_joint_state, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "isaac_sim_gpio node ready: input=%s output_base=%s joints=%zu",
      input_topic_.c_str(), output_topic_.c_str(), joints_.size());
  }

private:
  void configure_joints(const std::vector<std::string> & joint_ids)
  {
    if (joint_ids.empty()) {
      RCLCPP_WARN(get_logger(), "No joints configured; events will not be emitted.");
      return;
    }

    joints_.reserve(joint_ids.size());
    for (const auto & id : joint_ids) {
      const std::string prefix = "joints." + id;
      JointRule rule;
      rule.id = id;
      rule.joint_name = declare_parameter<std::string>(prefix + ".joint_name", id);
      rule.gpio_name = declare_parameter<std::string>(prefix + ".gpio_name", id);
      rule.threshold = declare_parameter<double>(prefix + ".threshold", 0.0);
      rule.absolute_value = declare_parameter<bool>(prefix + ".absolute_value", false);
      rule.operation_label = declare_parameter<std::string>(prefix + ".operation", ">");
      rule.operation = parse_operation(rule.operation_label);
      if (rule.operation == Operation::kInvalid) {
        RCLCPP_ERROR(
          get_logger(),
          "Joint '%s' has invalid operation '%s'; expected one of >, <, =, >=, <=. Skipping.",
          id.c_str(), rule.operation_label.c_str());
        continue;
      }

      if (rule.gpio_name.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Joint '%s' has empty gpio_name; defaulting to '%s'.",
          id.c_str(), id.c_str());
        rule.gpio_name = id;
      }

      joints_.push_back(std::move(rule));
      log_joint_configuration(joints_.back());
    }
  }

  void handle_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (joints_.empty()) {
      return;
    }

    std::unordered_map<std::string, size_t> name_index;
    name_index.reserve(msg->name.size());
    for (size_t idx = 0; idx < msg->name.size(); ++idx) {
      name_index.emplace(msg->name[idx], idx);
    }

    for (auto & rule : joints_) {
      auto it = name_index.find(rule.joint_name);
      if (it == name_index.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Joint '%s' not found in JointState message.", rule.joint_name.c_str());
        continue;
      }

      const size_t position_index = it->second;
      if (position_index >= msg->position.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Joint '%s' index %zu has no position value.", rule.joint_name.c_str(), position_index);
        continue;
      }

      double position = msg->position[position_index];
      if (rule.absolute_value) {
        position = std::fabs(position);
      }

      const bool active = evaluate_operation(rule.operation, position, rule.threshold);
      if (!rule.last_state.has_value() || rule.last_state.value() != active) {
        rule.last_state = active;
        publish_event(rule, active, position);
      }
    }
  }

  void publish_event(const JointRule & rule, bool active, double value)
  {
    if (!event_pub_) {
      return;
    }

    smacc2_msgs::msg::SmaccEvent event;
    event.event_type = active ? "GPIO_ACTIVE" : "GPIO_INACTIVE";
    event.event_object_tag = rule.gpio_name;
    event.event_source = event_source_;
    std::ostringstream label;
    label << rule.gpio_name << ":" << (active ? "active" : "inactive")
          << " value=" << value << " threshold=" << rule.threshold;
    event.label = label.str();
    event_pub_->publish(event);

    RCLCPP_INFO(
      get_logger(),
      "GPIO '%s' from joint '%s' %s (value=%.3f op %s %.3f)",
      rule.gpio_name.c_str(), rule.joint_name.c_str(), active ? "active" : "inactive",
      value, rule.operation_label.c_str(), rule.threshold);
  }

  void log_joint_configuration(const JointRule & rule)
  {
    RCLCPP_INFO(
      get_logger(),
      "Joint '%s': joint_name=%s gpio_name=%s threshold=%.3f abs=%s op=%s",
      rule.id.c_str(), rule.joint_name.c_str(), rule.gpio_name.c_str(), rule.threshold,
      rule.absolute_value ? "true" : "false", rule.operation_label.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string event_source_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccEvent>::SharedPtr event_pub_;
  std::vector<JointRule> joints_;
};

}  // namespace isaac_sim_gpio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<isaac_sim_gpio::IsaacSimGpioNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
