#ifndef ISAAC_TOPIC_HARDWARE__ISAAC_TOPIC_HARDWARE_HPP_
#define ISAAC_TOPIC_HARDWARE__ISAAC_TOPIC_HARDWARE_HPP_

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaac_topic_hardware
{
class IsaacTopicHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacTopicHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void handle_state_message(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_command(const rclcpp::Time & now);
  bool validate_joint(const hardware_interface::ComponentInfo & joint) const;

  rclcpp::Logger logger_{rclcpp::get_logger("isaac_topic_hardware")};
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> hardware_joint_names_;
  std::unordered_map<std::string, size_t> joint_index_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> velocity_commands_;

  std::string state_topic_;
  std::string command_topic_;
  std::string node_name_{"isaac_topic_hardware"};
  size_t state_qos_depth_{50};
  size_t command_qos_depth_{10};
  double state_timeout_{0.5};
  double command_publish_period_{0.02};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_state_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_command_publish_time_{0, 0, RCL_STEADY_TIME};
};
}  // namespace isaac_topic_hardware

#endif  // ISAAC_TOPIC_HARDWARE__ISAAC_TOPIC_HARDWARE_HPP_
