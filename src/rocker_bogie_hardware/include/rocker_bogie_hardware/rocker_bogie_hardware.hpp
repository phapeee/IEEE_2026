#ifndef ROCKER_BOGIE_HARDWARE__ROCKER_BOGIE_HARDWARE_HPP_
#define ROCKER_BOGIE_HARDWARE__ROCKER_BOGIE_HARDWARE_HPP_

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

namespace rocker_bogie_hardware
{
class RockerBogieHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RockerBogieHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
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

  rclcpp::Logger logger_{rclcpp::get_logger("rocker_bogie_hardware")};
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
  std::vector<double> position_commands_;
  enum class CommandMode { VELOCITY, POSITION };
  std::vector<CommandMode> joint_command_modes_;
  std::vector<size_t> joint_command_indices_;

  std::string state_topic_;
  std::string command_topic_;
  std::string node_name_{"rocker_bogie_hardware"};
  size_t state_qos_depth_{50};
  size_t command_qos_depth_{10};
  double state_timeout_{0.5};
  double command_publish_period_{0.02};

  rclcpp::Time last_state_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_command_publish_time_{0, 0, RCL_SYSTEM_TIME};
};
}  // namespace rocker_bogie_hardware

#endif  // ROCKER_BOGIE_HARDWARE__ROCKER_BOGIE_HARDWARE_HPP_
