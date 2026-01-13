#pragma once

#include <chrono>
#include <cmath>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <limit_switch_calibration_msgs/action/limit_switch_calibration.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <smacc2_msgs/msg/smacc_event.hpp>

namespace smacc_button_nav
{

enum class TaskType
{
  NONE,
  WAIT,
  SERVICE_CALL,
  EXTERNAL_STATE_MACHINE,
  LIMIT_SWITCH_CALIBRATION
};

struct TaskDefinition
{
  TaskType type{TaskType::NONE};

  // Common
  std::string name;

  // For wait
  double default_duration_sec{0.0};

  // For service_call
  std::string service_name;
  std::string service_type;   // informational only (C++ is compiled)
  double service_timeout_sec{0.0};

  // For external_state_machine
  std::string start_topic;
  std::string start_command;  // e.g. "antenna_button_cali"
  std::string result_event_topic;
  std::string success_event_type;
  std::string failure_event_type;
  double result_timeout_sec{0.0};

  // For limit_switch_calibration action
  std::string action_name;
  std::vector<std::string> directions;
  std::string pose_frame_id;
  double pose_x{0.0};
  double pose_y{0.0};
  double pose_yaw_deg{0.0};
  double action_timeout_sec{0.0};
};

struct WaypointTaskSpec
{
  geometry_msgs::msg::PoseStamped pose;
  std::string name;                 // logical name for debugging (optional)
  std::string task_name;            // key into TaskDefinition map
  std::optional<double> duration;   // override for wait tasks, if provided
};

namespace sc = boost::statechart;
namespace mpl = boost::mpl;
using namespace std::chrono_literals;

struct EvButtonPressed : sc::event<EvButtonPressed>
{
};

struct EvNavigationSucceeded : sc::event<EvNavigationSucceeded>
{
};

struct EvNavigationFailed : sc::event<EvNavigationFailed>
{
};

struct EvWaitOver : sc::event<EvWaitOver>
{
};

struct EvMissionCompleted : sc::event<EvMissionCompleted>
{
};

struct EvResetFinished : sc::event<EvResetFinished>
{
};

class SmButtonNav;
struct StIdle;
struct StNavigateWaypoint;
struct StWaypointWait;
struct StReset;

class ClButtonInterface : public smacc2::ISmaccClient
{
public:
  std::string getName() const override { return "button_interface"; }
};

class ClNavActionClient : public smacc2::ISmaccClient
{
public:
  std::string getName() const override { return "nav2_navigate_to_pose"; }
};

class CpButtonWatcher : public smacc2::ISmaccComponent
{
public:
  void onInitialize() override;

private:
  void handleButton(const std_msgs::msg::Bool::SharedPtr msg);

  std::string topic_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  bool last_state_{false};
  bool initialized_{false};
  bool armed_{false};
};

class CpWaypointNavigator : public smacc2::ISmaccComponent
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void onInitialize() override;

  void sendGoal(const geometry_msgs::msg::PoseStamped & goal);

  void cancelGoal();

private:
  void handleResult(const GoalHandle::WrappedResult & result);

  std::string action_name_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::mutex mutex_;
};

struct OrButtonInterface : smacc2::Orthogonal<OrButtonInterface>
{
  void onInitialize() override
  {
    auto client = this->createClient<ClButtonInterface>();
    client->template createComponent<CpButtonWatcher>();
  }
};

struct OrNavigation : smacc2::Orthogonal<OrNavigation>
{
  void onInitialize() override
  {
    auto client = this->createClient<ClNavActionClient>();
    client->template createComponent<CpWaypointNavigator>();
  }
};

class SmButtonNav : public smacc2::SmaccStateMachineBase<SmButtonNav, StIdle>
{
public:
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override;

  void resetMission();

  std::optional<geometry_msgs::msg::PoseStamped> nextWaypoint();

  bool hasPendingWaypoints() const;

  double waitDurationDefault() const { return wait_duration_sec_; }
  size_t lastWaypointIndex() const;
  const std::string & controllerManagerService() const { return controller_manager_service_; }
  const std::string & configureControllerService() const { return configure_controller_service_; }
  const std::string & controllerName() const { return controller_name_; }
  const std::string & resetPoseService() const { return reset_pose_service_; }
  const geometry_msgs::msg::PoseWithCovarianceStamped & resetPose() const { return reset_pose_; }
  const WaypointTaskSpec * waypointSpec(size_t index) const
  {
    if (index >= waypoints_.size()) return nullptr;
    return &waypoints_[index];
  }

  const WaypointTaskSpec * lastWaypointSpec() const
  {
    if (next_waypoint_index_ == 0) return nullptr;
    return waypointSpec(next_waypoint_index_ - 1);
  }

  // new API used by StWaypointTask instead of waitDurationForWaypoint:
  void loadMissionFromFile(const std::string & mission_file);
  void loadTasksFromFile(const std::string & task_file);
  TaskDefinition getTaskDefinition(const std::string & task_name) const;
  std::chrono::milliseconds controllerSwitchTimeout() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(controller_switch_timeout_sec_));
  }
  std::chrono::milliseconds poseServiceTimeout() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(pose_service_timeout_sec_));
  }
  std::chrono::milliseconds serviceWaitTimeout() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(service_wait_timeout_sec_));
  }
  double resetPoseYaw() const { return reset_pose_yaw_; }
  void triggerImuCalibration();

private:
  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
  double maybeConvertYaw(double value) const;

  double wait_duration_sec_{3.0};
  std::string waypoint_frame_id_{"map"};
  bool waypoint_angles_in_degrees_{false};
  std::vector<WaypointTaskSpec> waypoints_;
  std::map<std::string, TaskDefinition> task_defs_;
  size_t next_waypoint_index_{0};
  std::string controller_manager_service_{"/controller_manager/switch_controller"};
  std::string configure_controller_service_{"/controller_manager/configure_controller"};
  std::string controller_name_{"mecanum_controller"};
  std::string reset_pose_service_{"/set_pose"};
  std::string reset_pose_frame_id_{"map"};
  geometry_msgs::msg::PoseWithCovarianceStamped reset_pose_;
  double controller_switch_timeout_sec_{2.0};
  double pose_service_timeout_sec_{2.0};
  double service_wait_timeout_sec_{2.0};
  double reset_pose_yaw_{1.5707963268};  // default 90 deg (pi/2)
  std::string imu_calibration_service_{"trigger_imu_calibration"};
  double imu_calibration_service_wait_sec_{2.0};
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr imu_calibration_client_;
};

struct StIdle : smacc2::SmaccState<StIdle, SmButtonNav>
{
  using SmaccState::SmaccState;

  using reactions = mpl::list<smacc2::Transition<EvButtonPressed, StNavigateWaypoint>>;

  void onEntry();
  void onExit();
};

struct StNavigateWaypoint : smacc2::SmaccState<StNavigateWaypoint, SmButtonNav>
{
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvNavigationSucceeded, StWaypointWait>,
    smacc2::Transition<EvNavigationFailed, StReset>,
    smacc2::Transition<EvMissionCompleted, StIdle>,
    smacc2::Transition<EvButtonPressed, StReset>>;

  void onEntry();
  void onExit();
};

struct StWaypointWait : smacc2::SmaccState<StWaypointWait, SmButtonNav>
{
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvWaitOver, StNavigateWaypoint>,
    smacc2::Transition<EvMissionCompleted, StIdle>,
    smacc2::Transition<EvButtonPressed, StReset>>;

  void onEntry();
  void onExit();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ext_sm_start_pub_;
  rclcpp::Subscription<smacc2_msgs::msg::SmaccEvent>::SharedPtr ext_sm_result_sub_;
  rclcpp::TimerBase::SharedPtr ext_timeout_timer_;
  bool ext_waiting_{false};
  std::string ext_success_event_type_;
  std::string ext_failure_event_type_;

  rclcpp_action::Client<limit_switch_calibration_msgs::action::LimitSwitchCalibration>::SharedPtr
    calibration_action_client_;
  rclcpp_action::ClientGoalHandle<limit_switch_calibration_msgs::action::LimitSwitchCalibration>::SharedPtr
    calibration_goal_handle_;
  rclcpp::TimerBase::SharedPtr calibration_timeout_timer_;
  bool calibration_waiting_{false};

  void completeExternalTask(bool success);
  void completeCalibrationTask(bool success);
};

struct StReset : smacc2::SmaccState<StReset, SmButtonNav>
{
  using SmaccState::SmaccState;

  using reactions = mpl::list<smacc2::Transition<EvResetFinished, StIdle>>;

  void onEntry();
  void onExit();

private:
  void startResetWorker();
  void resetSequence();
  bool restartController(const SmButtonNav & sm);
  bool resetPose(const SmButtonNav & sm);
  bool waitForService(
    const rclcpp::ClientBase::SharedPtr & client, const std::string & service_name,
    std::chrono::milliseconds timeout);

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_client_;
  rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
  std::thread reset_thread_;
};

}  // namespace smacc_button_nav
