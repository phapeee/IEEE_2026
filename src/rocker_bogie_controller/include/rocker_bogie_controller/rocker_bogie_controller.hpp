#ifndef ROCKER_BOGIE_CONTROLLER__ROCKER_BOGIE_CONTROLLER_HPP_
#define ROCKER_BOGIE_CONTROLLER__ROCKER_BOGIE_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rocker_bogie_controller
{
class RockerBogieController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct WheelHandle
  {
    std::string name;
    hardware_interface::LoanedCommandInterface * command{nullptr};
    hardware_interface::LoanedStateInterface * position_state{nullptr};
    hardware_interface::LoanedStateInterface * velocity_state{nullptr};
    double direction{1.0};
  };

  struct StateHandle
  {
    hardware_interface::LoanedStateInterface * position{nullptr};
    hardware_interface::LoanedStateInterface * velocity{nullptr};
  };

  struct ServoHandle
  {
    std::string name;
    hardware_interface::LoanedCommandInterface * command{nullptr};
    hardware_interface::LoanedStateInterface * position_state{nullptr};
    double direction{1.0};
  };

  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  hardware_interface::LoanedCommandInterface * find_command_handle(
    const std::string & joint_name, const std::string & interface_name = hardware_interface::HW_IF_VELOCITY);
  hardware_interface::LoanedStateInterface * find_state_handle(
    const std::string & joint_name, const std::string & interface_name);
  void publish_joint_states(const rclcpp::Time & time);
  void publish_twist(
    const rclcpp::Time & time, double linear_x, double linear_y, double angular_z);
  double normalized_angle(double angle) const;
  double shortest_angular_distance(double from, double to) const;

  std::vector<std::string> wheel_joint_names_;
  std::vector<std::string> servo_joint_names_;
  std::vector<std::string> state_joint_names_;
  std::vector<WheelHandle> wheel_handles_;
  std::vector<ServoHandle> servo_handles_;
  std::unordered_map<std::string, StateHandle> state_handles_;
  std::unordered_map<std::string, double> servo_direction_overrides_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;

  std::string cmd_vel_topic_{"/cmd_vel"};
  bool publish_joint_states_{true};
  bool publish_twist_{false};
  double publish_rate_{50.0};
  double cmd_vel_timeout_{0.5};
  double left_forward_direction_{1.0};
  double right_forward_direction_{-1.0};
  double min_steering_angle_{0.05};
  double max_steering_angle_{0.2};
  double servo_max_angle_{M_PI_2};
  double strafe_deadband_{0.01};
  double wheel_radius_{0.05};
  double wheel_radius_inv_{20.0};
  double wheel_distance_x_{0.1};
  double wheel_distance_y_{0.1};
  double yaw_deadband_{0.01};
  std::string twist_frame_id_{"odom"};
  std::array<double, 36> twist_covariance_{};
  double twist_linear_deadband_{1e-3};

  double last_linear_cmd_{0.0};
  double last_lateral_cmd_{0.0};
  double last_yaw_cmd_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_period_{0, 1};
  rclcpp::Time last_state_publish_time_{0, 0, RCL_ROS_TIME};
  std::mutex cmd_vel_mutex_;
  bool steering_ready_{true};
};
}  // namespace rocker_bogie_controller

#endif  // ROCKER_BOGIE_CONTROLLER__ROCKER_BOGIE_CONTROLLER_HPP_
