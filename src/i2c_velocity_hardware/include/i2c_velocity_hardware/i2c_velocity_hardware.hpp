#ifndef I2C_VELOCITY_HARDWARE__I2C_VELOCITY_HARDWARE_HPP_
#define I2C_VELOCITY_HARDWARE__I2C_VELOCITY_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "i2c_manager/srv/i2c_transfer.hpp"

namespace i2c_velocity_hardware
{
class I2cVelocityHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(I2cVelocityHardware);

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
  bool read_measured_velocities();
  bool write_target_velocities();
  bool validate_joint(const hardware_interface::ComponentInfo & joint) const;
  bool parse_joint_order(const hardware_interface::HardwareInfo & info);
  bool call_transfer(
    const std::vector<uint8_t> & write_data, size_t read_length, std::vector<uint8_t> * read_data);

  rclcpp::Logger logger_{rclcpp::get_logger("i2c_velocity_hardware")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_read_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<i2c_manager::srv::I2cTransfer>::SharedPtr client_;

  std::vector<std::string> joint_names_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> velocity_commands_;
  std::vector<double> velocity_signs_;
  std::vector<size_t> joint_to_hw_index_;
  std::vector<size_t> hw_to_joint_index_;

  int i2c_address_{0x10};
  uint8_t command_register_{0x01};
  uint8_t state_register_{0x10};
  double centi_rps_scale_{100.0};
  double max_rps_{3.0};
  size_t expected_joints_{4};
  std::string i2c_service_{"i2c_manager/transfer"};
  std::string node_name_{"i2c_velocity_hardware"};
  std::string node_namespace_{""};
  double service_timeout_sec_{0.05};
  bool log_measured_velocities_{false};
  int log_measured_velocities_ms_{1000};
};
}  // namespace i2c_velocity_hardware

#endif  // I2C_VELOCITY_HARDWARE__I2C_VELOCITY_HARDWARE_HPP_
