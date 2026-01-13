#include "smacc_button_nav/smacc_button_nav.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <functional>

namespace smacc_button_nav
{

void SmButtonNav::loadTasksFromFile(const std::string & task_file)
{
  YAML::Node root = YAML::LoadFile(task_file);
  YAML::Node tasks = root["tasks"];
  if (!tasks || !tasks.IsMap())
  {
    RCLCPP_ERROR(this->getLogger(), "tasks.yaml must contain a 'tasks' map");
    return;
  }

  task_defs_.clear();
  for (auto it : tasks)
  {
    const std::string name = it.first.as<std::string>();
    YAML::Node cfg = it.second;
    TaskDefinition def;
    def.name = name;
    const std::string type = cfg["type"].as<std::string>("");

    if (type == "wait")
    {
      def.type = TaskType::WAIT;
      def.default_duration_sec = cfg["default_duration_sec"].as<double>(0.0);
    }
    else if (type == "service_call")
    {
      def.type = TaskType::SERVICE_CALL;
      def.service_name = cfg["service_name"].as<std::string>("");
      def.service_type = cfg["service_type"].as<std::string>("");
      def.service_timeout_sec = cfg["timeout_sec"].as<double>(0.0);
    }
    else if (type == "external_state_machine")
    {
      def.type = TaskType::EXTERNAL_STATE_MACHINE;
      YAML::Node start = cfg["start"];
      YAML::Node result = cfg["result"];
      def.start_topic = start["topic"].as<std::string>("");
      def.start_command = start["command"].as<std::string>("");
      def.result_event_topic = result["event_topic"].as<std::string>("");
      def.success_event_type = result["success_event_type"].as<std::string>("");
      def.failure_event_type = result["failure_event_type"].as<std::string>("");
      def.result_timeout_sec = result["timeout_sec"].as<double>(0.0);
    }
    else
    {
      RCLCPP_ERROR(this->getLogger(), "Unknown task type '%s' for task '%s'",
                   type.c_str(), name.c_str());
      continue;
    }

    task_defs_[name] = def;
  }

  RCLCPP_INFO(this->getLogger(), "Loaded %zu task definition(s) from %s",
              task_defs_.size(), task_file.c_str());
}

TaskDefinition SmButtonNav::getTaskDefinition(const std::string & task_name) const
{
  auto it = task_defs_.find(task_name);
  if (it != task_defs_.end())
    return it->second;

  // Fallback: simple built-in wait if unknown
  TaskDefinition def;
  def.name = task_name;
  def.type = TaskType::WAIT;
  def.default_duration_sec = wait_duration_sec_;
  return def;
}

void SmButtonNav::loadMissionFromFile(const std::string & mission_file)
{
  YAML::Node root = YAML::LoadFile(mission_file);
  YAML::Node mission = root["mission"];
  if (!mission)
    throw std::runtime_error("mission.yaml must contain 'mission' root");

  YAML::Node wps = mission["waypoints"];
  if (!wps || !wps.IsSequence())
    throw std::runtime_error("'mission.waypoints' must be a sequence");

  waypoints_.clear();
  waypoints_.reserve(wps.size());

  for (auto wp_node : wps)
  {
    WaypointTaskSpec spec;

    spec.name = wp_node["name"].as<std::string>("");

    YAML::Node coord = wp_node["coord"];
    if (!coord)
      throw std::runtime_error("Waypoint missing 'coord'");

    const std::string frame = coord["frame"].as<std::string>(waypoint_frame_id_);
    const double x = coord["x"].as<double>();
    const double y = coord["y"].as<double>();
    double yaw = 0.0;
    if (coord["yaw_deg"])
      yaw = maybeConvertYaw(coord["yaw_deg"].as<double>());
    else if (coord["yaw"])
      yaw = coord["yaw"].as<double>();

    spec.pose.header.frame_id = frame;
    spec.pose.pose.position.x = x;
    spec.pose.pose.position.y = y;
    spec.pose.pose.position.z = 0.0;
    spec.pose.pose.orientation = yawToQuaternion(yaw);

    YAML::Node task = wp_node["task"];
    if (task && task["name"])
    {
      spec.task_name = task["name"].as<std::string>();
      if (task["duration_sec"])
        spec.duration = task["duration_sec"].as<double>();
    }
    else
    {
      spec.task_name = "wait";
    }

    waypoints_.push_back(spec);
  }

  RCLCPP_INFO(this->getLogger(), "Mission file '%s' has %zu waypoint(s)",
              mission_file.c_str(), waypoints_.size());
}

void CpButtonWatcher::onInitialize()
{
  auto node = this->getNode();
  topic_ = node->declare_parameter<std::string>("button_state_topic", "smacc2/button_state");
  sub_ = node->create_subscription<std_msgs::msg::Bool>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&CpButtonWatcher::handleButton, this, std::placeholders::_1));
  initialized_ = true;
  RCLCPP_INFO(this->getLogger(), "Listening for button state on %s", topic_.c_str());
}

void CpButtonWatcher::handleButton(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!initialized_) {
    return;
  }

  const bool pressed = msg->data;
  if (!armed_) {
    armed_ = true;
    RCLCPP_DEBUG(this->getLogger(), "Button watcher armed (initial state=%s)", pressed ? "true" : "false");
    // Treat the first message as a potential rising edge from an assumed "released" state.
    if (pressed && !last_state_) {
      RCLCPP_INFO(this->getLogger(), "Detected button press rising edge");
      this->postEvent<EvButtonPressed>();
    }
    last_state_ = pressed;
    return;
  }

  if (pressed && !last_state_) {
    RCLCPP_INFO(this->getLogger(), "Detected button press rising edge");
    this->postEvent<EvButtonPressed>();
  }
  last_state_ = pressed;
}

void CpWaypointNavigator::onInitialize()
{
  auto node = this->getNode();
  action_name_ = node->declare_parameter<std::string>("navigate_action_name", "navigate_to_pose");
  client_ = rclcpp_action::create_client<NavigateToPose>(node, action_name_);
  RCLCPP_INFO(this->getLogger(), "Waiting for %s action server...", action_name_.c_str());
  client_->wait_for_action_server();
}

void CpWaypointNavigator::sendGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (!client_) {
    throw std::runtime_error("NavigateToPose client is not initialized");
  }

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.result_callback =
    std::bind(&CpWaypointNavigator::handleResult, this, std::placeholders::_1);
  options.goal_response_callback = [this](const GoalHandle::SharedPtr & handle) {
    if (!handle) {
      RCLCPP_ERROR(this->getLogger(), "NavigateToPose goal was rejected");
      this->postEvent<EvNavigationFailed>();
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = handle;
  };
  options.feedback_callback = [](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback>) {};

  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal;
  if (!client_->wait_for_action_server(1s)) {
    RCLCPP_WARN(
      this->getLogger(), "navigate_to_pose action not available, retrying while sending goal");
  }
  client_->async_send_goal(goal_msg, options);
  RCLCPP_INFO(
    this->getLogger(), "Sent NavigateToPose goal to (%.2f, %.2f)", goal.pose.position.x,
    goal.pose.position.y);
}

void CpWaypointNavigator::cancelGoal()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_) {
    RCLCPP_INFO(this->getLogger(), "Cancelling active NavigateToPose goal");
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
}

void CpWaypointNavigator::handleResult(const GoalHandle::WrappedResult & result)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_.reset();
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->getLogger(), "NavigateToPose goal reached");
    this->postEvent<EvNavigationSucceeded>();
  } else {
    RCLCPP_WARN(
      this->getLogger(), "NavigateToPose failed with code %d", static_cast<int>(result.code));
    this->postEvent<EvNavigationFailed>();
  }
}

void SmButtonNav::onInitialize()
{
  this->createOrthogonal<OrButtonInterface>();
  this->createOrthogonal<OrNavigation>();

  auto node = this->getNode();

  // --- File-based mission / task config ---
  const std::string mission_file =
    node->declare_parameter<std::string>("mission_file", "");
  const std::string task_file =
    node->declare_parameter<std::string>("task_file", "");

  // --- Existing scalar / reset parameters ---
  wait_duration_sec_ = node->declare_parameter<double>("wait_duration_sec", 3.0);
  waypoint_frame_id_ = node->declare_parameter<std::string>("waypoint_frame_id", "map");
  waypoint_angles_in_degrees_ =
    node->declare_parameter<bool>("waypoint_angles_in_degrees", false);
  controller_manager_service_ = node->declare_parameter<std::string>(
    "controller_manager_service", "/controller_manager/switch_controller");
  configure_controller_service_ = node->declare_parameter<std::string>(
    "configure_controller_service", "/controller_manager/configure_controller");
  controller_name_ = node->declare_parameter<std::string>(
    "reset_controller_name", "mecanum_controller");
  reset_pose_service_ =
    node->declare_parameter<std::string>("reset_pose_service", "/set_pose");
  reset_pose_frame_id_ =
    node->declare_parameter<std::string>("reset_pose_frame_id", "map");
  controller_switch_timeout_sec_ =
    std::max(0.0, node->declare_parameter<double>("controller_switch_timeout_sec", 2.0));
  pose_service_timeout_sec_ =
    std::max(0.0, node->declare_parameter<double>("reset_pose_timeout_sec", 2.0));
  service_wait_timeout_sec_ =
    std::max(0.0, node->declare_parameter<double>("service_wait_timeout_sec", 2.0));
  imu_calibration_service_ =
    node->declare_parameter<std::string>("imu_calibration_service", "/trigger_imu_calibration");
  imu_calibration_service_wait_sec_ =
    std::max(0.0, node->declare_parameter<double>("imu_calibration_service_wait_sec", 2.0));

  const double reset_pose_x =
    node->declare_parameter<double>("reset_pose_x", 0.1778);
  const double reset_pose_y =
    node->declare_parameter<double>("reset_pose_y", 0.1778);
  const double reset_pose_yaw_param =
    node->declare_parameter<double>("reset_pose_yaw", 1.5707963268);

  reset_pose_yaw_ = maybeConvertYaw(reset_pose_yaw_param);
  reset_pose_.header.frame_id = reset_pose_frame_id_;
  reset_pose_.pose.pose.position.x = reset_pose_x;
  reset_pose_.pose.pose.position.y = reset_pose_y;
  reset_pose_.pose.pose.position.z = 0.0;
  reset_pose_.pose.pose.orientation = yawToQuaternion(reset_pose_yaw_);
  reset_pose_.pose.covariance.fill(0.0);
  // Match desired covariance: high uncertainty on z/roll/pitch, zeros elsewhere.
  reset_pose_.pose.covariance[14] = 9999.0;  // z
  reset_pose_.pose.covariance[21] = 9999.0;  // roll
  reset_pose_.pose.covariance[28] = 9999.0;  // pitch
  imu_calibration_client_ =
    node->create_client<std_srvs::srv::Empty>(imu_calibration_service_);

  // --- Load tasks ---
  if (!task_file.empty())
  {
    loadTasksFromFile(task_file);
  }
  else
  {
    RCLCPP_WARN(
      this->getLogger(),
      "No task_file specified; only built-in tasks will be available");
  }

  // --- Load mission (waypoints + task per waypoint) ---
  if (!mission_file.empty())
  {
    try
    {
      loadMissionFromFile(mission_file);
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(
        this->getLogger(),
        "Failed to load mission file '%s': %s",
        mission_file.c_str(), ex.what());
    }
  }
  else
  {
    RCLCPP_WARN(
      this->getLogger(),
      "No mission_file specified; no waypoints loaded");
  }

  // --- Summary ---
  if (waypoints_.empty())
  {
    RCLCPP_WARN(
      this->getLogger(),
      "No waypoints in mission; navigation will immediately complete");
  }
  else
  {
    RCLCPP_INFO(
      this->getLogger(),
      "Loaded %zu waypoint(s) from mission file (default wait=%.2f s)",
      waypoints_.size(), wait_duration_sec_);
  }

  RCLCPP_INFO(
    this->getLogger(),
    "Reset will toggle controller '%s' via '%s' (configure via '%s') and set pose to "
    "(%.2f, %.2f, %.2f) in frame '%s' using service '%s'",
    controller_name_.c_str(), controller_manager_service_.c_str(),
    configure_controller_service_.c_str(),
    reset_pose_x, reset_pose_y, reset_pose_yaw_,
    reset_pose_frame_id_.c_str(), reset_pose_service_.c_str());

  RCLCPP_INFO(
    this->getLogger(),
    "Button press will trigger IMU calibration via service '%s'",
    imu_calibration_service_.c_str());
}

void SmButtonNav::triggerImuCalibration()
{
  if (!imu_calibration_client_) {
    RCLCPP_WARN(this->getLogger(), "IMU calibration client not initialized");
    return;
  }

  // 1) Wait for the service to be available
  const auto service_wait = std::chrono::duration<double>(service_wait_timeout_sec_);
  if (!imu_calibration_client_->wait_for_service(service_wait)) {
    RCLCPP_WARN(
      this->getLogger(),
      "IMU calibration service '%s' not available after waiting %.2f seconds; skipping trigger",
      imu_calibration_service_.c_str(), service_wait_timeout_sec_);
    return;
  }

  // 2) Call the service and BLOCK until calibration node finishes
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = imu_calibration_client_->async_send_request(request);

  // How long we’re willing to wait for the calibration to finish.
  // This must be >= calibration_timeout_sec (in the Python node) + a bit of margin.
  const auto call_timeout = std::chrono::duration<double>(imu_calibration_service_wait_sec_);

  RCLCPP_INFO(
    this->getLogger(),
    "Waiting up to %.2f s for IMU calibration service '%s' to finish",
    imu_calibration_service_wait_sec_, imu_calibration_service_.c_str());

  if (future.wait_for(call_timeout) != std::future_status::ready) {
    RCLCPP_WARN(
      this->getLogger(),
      "Timed out waiting %.2f s for IMU calibration to finish; continuing anyway",
      imu_calibration_service_wait_sec_);
    return;
  }

  // If you ever add fields to the response, you could inspect them here
  try {
    (void)future.get();
    RCLCPP_INFO(
      this->getLogger(),
      "IMU calibration service '%s' completed; proceeding with mission",
      imu_calibration_service_.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      this->getLogger(),
      "IMU calibration service '%s' returned with error: %s",
      imu_calibration_service_.c_str(), ex.what());
  }
}

void SmButtonNav::resetMission()
{
  next_waypoint_index_ = 0;
}

std::optional<geometry_msgs::msg::PoseStamped> SmButtonNav::nextWaypoint()
{
  if (next_waypoint_index_ >= waypoints_.size()) {
    return std::nullopt;
  }
  auto pose = waypoints_[next_waypoint_index_].pose;
  pose.header.stamp = this->getNode()->now();
  ++next_waypoint_index_;
  return pose;
}

bool SmButtonNav::hasPendingWaypoints() const
{
  return next_waypoint_index_ < waypoints_.size();
}

size_t SmButtonNav::lastWaypointIndex() const
{
  if (next_waypoint_index_ == 0) {
    return 0;
  }
  return next_waypoint_index_ - 1;
}

geometry_msgs::msg::Quaternion SmButtonNav::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double SmButtonNav::maybeConvertYaw(double value) const
{
  if (!waypoint_angles_in_degrees_) {
    return value;
  }
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  return value * kDegToRad;
}

void StIdle::onEntry()
{
  RCLCPP_INFO(this->getLogger(), "State: Idle -> waiting for button press");
  this->context<SmButtonNav>().resetMission();
  CpWaypointNavigator * navigator = nullptr;
  this->context<SmButtonNav>().requiresComponent(
    navigator, smacc2::ComponentRequirement::SOFT);
  if (navigator != nullptr) {
    navigator->cancelGoal();
  }
}

void StIdle::onExit()
{
  RCLCPP_INFO(this->getLogger(), "Leaving Idle");
}

void StNavigateWaypoint::onEntry()
{
  RCLCPP_INFO(this->getLogger(), "State: NavigateWaypoint");

  this->context<SmButtonNav>().triggerImuCalibration();

  auto maybe_goal = this->context<SmButtonNav>().nextWaypoint();
  if (!maybe_goal) {
    RCLCPP_INFO(this->getLogger(), "No waypoints remaining; mission complete");
    this->postEvent<EvMissionCompleted>();
    return;
  }

  CpWaypointNavigator * navigator = nullptr;
  this->context<SmButtonNav>().requiresComponent(
    navigator, smacc2::ComponentRequirement::SOFT);
  if (navigator == nullptr) {
    RCLCPP_ERROR(this->getLogger(), "Waypoint navigator component not available");
    this->postEvent<EvNavigationFailed>();
    return;
  }
  navigator->sendGoal(*maybe_goal);
}

void StNavigateWaypoint::onExit()
{
  RCLCPP_INFO(this->getLogger(), "Exiting NavigateWaypoint");
}

void StWaypointWait::onExit()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  if (ext_timeout_timer_) {
    ext_timeout_timer_->cancel();
    ext_timeout_timer_.reset();
  }

  ext_sm_result_sub_.reset();
  ext_sm_start_pub_.reset();
  ext_waiting_ = false;

  RCLCPP_INFO(this->getLogger(), "Exiting WaypointWait");
}

void StWaypointWait::onEntry()
{
  auto & sm = this->context<SmButtonNav>();
  const auto * spec = sm.lastWaypointSpec();
  if (!spec)
  {
    RCLCPP_WARN(this->getLogger(), "No last waypoint spec; finishing mission");
    this->postEvent<EvMissionCompleted>();
    return;
  }

  const auto def = sm.getTaskDefinition(spec->task_name);
  RCLCPP_INFO(this->getLogger(),
              "State: WaypointTask at '%s' (task=%s)",
              spec->name.c_str(), spec->task_name.c_str());

  switch (def.type)
  {
    case TaskType::WAIT:
    {
      const double duration =
        spec->duration.value_or(def.default_duration_sec);
      const double wait_sec = std::max(0.0, duration);

      auto & sm     = this->context<SmButtonNav>();   // state machine (long-lived)
      auto logger   = this->getLogger();              // copy, safe to keep
      auto node     = this->getNode();                // node outlives the state

      RCLCPP_INFO(
        logger,
        "WAIT task at waypoint '%s' for %.2f s",
        spec->name.c_str(), wait_sec);

      timer_ = node->create_wall_timer(
        std::chrono::duration<double>(wait_sec),
        [logger, &sm]()
        {
          if (sm.hasPendingWaypoints())
          {
            RCLCPP_INFO(logger,
              "Task done; proceeding to next waypoint");
            sm.postEvent<EvWaitOver>();          // <- post on the *SM*, not the state
          }
          else
          {
            RCLCPP_INFO(logger,
              "Final waypoint task complete; mission finished");
            sm.postEvent<EvMissionCompleted>();
          }
        });
      break;
    }

    case TaskType::SERVICE_CALL:
    {
      RCLCPP_INFO(this->getLogger(),
                  "SERVICE_CALL task '%s' at waypoint '%s'",
                  spec->task_name.c_str(), spec->name.c_str());

      // For imu_calibration_service you can just reuse the existing helper:
      if (spec->task_name == "imu_calibration_service")
      {
        sm.triggerImuCalibration();
      }
      // You could generalize later and look up service_name etc. from def.

      // For now, treat it as instantaneous and proceed as if wait is over:
      if (sm.hasPendingWaypoints())
        this->postEvent<EvWaitOver>();
      else
        this->postEvent<EvMissionCompleted>();
      break;
    }

    case TaskType::EXTERNAL_STATE_MACHINE:
    {
      RCLCPP_INFO(
        this->getLogger(),
        "EXTERNAL_STATE_MACHINE task '%s' at waypoint '%s'",
        spec->task_name.c_str(), spec->name.c_str());

      const auto & start_topic       = def.start_topic;
      const auto & start_command     = def.start_command;
      const auto & result_topic      = def.result_event_topic;
      ext_success_event_type_        = def.success_event_type;
      ext_failure_event_type_        = def.failure_event_type;
      const double timeout_sec       = def.result_timeout_sec;

      if (start_topic.empty() || start_command.empty() || result_topic.empty())
      {
        RCLCPP_WARN(
          this->getLogger(),
          "External task '%s' is missing start_topic / command / result_topic; skipping",
          spec->task_name.c_str());
        if (sm.hasPendingWaypoints())
          this->postEvent<EvWaitOver>();
        else
          this->postEvent<EvMissionCompleted>();
        break;
      }

      auto node = this->getNode();

      // Publisher to start the external calibration SM
      ext_sm_start_pub_ =
        node->create_publisher<std_msgs::msg::String>(start_topic, 10);

      // Reset flag and (re)create result subscriber
      ext_waiting_ = true;

      ext_sm_result_sub_ =
        node->create_subscription<smacc2_msgs::msg::SmaccEvent>(
          result_topic, 10,
          [this](const smacc2_msgs::msg::SmaccEvent::SharedPtr msg)
          {
            if (!ext_waiting_) {
              return;
            }

            // Filter on event_type; you can also check event_object_tag if you want
            if (!ext_success_event_type_.empty() &&
                msg->event_type == ext_success_event_type_)
            {
              RCLCPP_INFO(
                this->getLogger(),
                "Received external SM success event '%s' (label='%s')",
                msg->event_type.c_str(), msg->label.c_str());
              this->completeExternalTask(true);
            }
            else if (!ext_failure_event_type_.empty() &&
                    msg->event_type == ext_failure_event_type_)
            {
              RCLCPP_WARN(
                this->getLogger(),
                "Received external SM failure event '%s' (label='%s')",
                msg->event_type.c_str(), msg->label.c_str());
              this->completeExternalTask(false);
            }
          });

      // Send the start command (antenna_button_cali)
      std_msgs::msg::String cmd;
      cmd.data = start_command;
      ext_sm_start_pub_->publish(cmd);

      RCLCPP_INFO(
        this->getLogger(),
        "Sent external SM command '%s' on '%s'; waiting for events on '%s' "
        "(success='%s', failure='%s', timeout=%.2f s)",
        start_command.c_str(), start_topic.c_str(), result_topic.c_str(),
        ext_success_event_type_.c_str(), ext_failure_event_type_.c_str(),
        timeout_sec);

      // Optional timeout: if no success/failure event arrives in time
      if (timeout_sec > 0.0)
      {
        ext_timeout_timer_ =
          node->create_wall_timer(
            std::chrono::duration<double>(timeout_sec),
            [this, timeout_sec]()
            {
              if (!ext_waiting_) {
                return;
              }
              RCLCPP_WARN(
                this->getLogger(),
                "External state machine timed out after %.2f seconds; treating as failure",
                timeout_sec);
              this->completeExternalTask(false);
            });
      }

      // NOTE: do NOT post EvWaitOver/EvMissionCompleted here.
      // We stay in this state until success/failure/timeout.
      break;
    }

    default:
    {
      RCLCPP_WARN(this->getLogger(),
                  "Unknown task type for '%s'; skipping",
                  spec->task_name.c_str());
      if (sm.hasPendingWaypoints())
        this->postEvent<EvWaitOver>();
      else
        this->postEvent<EvMissionCompleted>();
      break;
    }
  }
}

void StReset::onEntry()
{
  RCLCPP_INFO(this->getLogger(),
              "State: Reset - restarting controller and resetting pose");

  auto node = this->getNode();
  auto & sm = this->context<SmButtonNav>();

  // Create service clients if they don't exist yet
  if (!switch_controller_client_) {
    switch_controller_client_ =
      node->create_client<controller_manager_msgs::srv::SwitchController>(
        sm.controllerManagerService());
  }

  if (!configure_controller_client_) {
    configure_controller_client_ =
      node->create_client<controller_manager_msgs::srv::ConfigureController>(
        sm.configureControllerService());
  }

  if (!set_pose_client_) {
    set_pose_client_ =
      node->create_client<robot_localization::srv::SetPose>(
        sm.resetPoseService());
  }

  // Kick off the reset sequence in the background thread
  startResetWorker();
}

void StReset::onExit()
{
  if (reset_thread_.joinable()) {
    reset_thread_.join();
  }
  RCLCPP_INFO(this->getLogger(), "Reset complete");
}

void StReset::startResetWorker()
{
  if (reset_thread_.joinable()) {
    reset_thread_.join();
  }

  reset_thread_ = std::thread([this]() { this->resetSequence(); });
}

void StReset::resetSequence()
{
  auto & sm = this->context<SmButtonNav>();
  const bool controller_ok = restartController(sm);
  const bool pose_ok = resetPose(sm);

  if (!controller_ok || !pose_ok) {
    RCLCPP_WARN(
      this->getLogger(), "Reset sequence finished with errors (controller_ok=%s, pose_ok=%s)",
      controller_ok ? "true" : "false", pose_ok ? "true" : "false");
  } else {
    RCLCPP_INFO(this->getLogger(), "Reset sequence finished successfully");
  }

  this->postEvent<EvResetFinished>();
}

bool StReset::restartController(const SmButtonNav & sm)
{
  const auto logger = this->getLogger();
  if (!switch_controller_client_) {
    RCLCPP_ERROR(logger, "Switch controller client is not initialized");
    return false;
  }
  if (!configure_controller_client_) {
    RCLCPP_ERROR(logger, "Configure controller client is not initialized");
    return false;
  }
  if (
    !waitForService(
      switch_controller_client_, sm.controllerManagerService(), sm.serviceWaitTimeout())) {
    return false;
  }
  if (
    !waitForService(
      configure_controller_client_, sm.configureControllerService(), sm.serviceWaitTimeout())) {
    return false;
  }

  const auto timeout_ms = sm.controllerSwitchTimeout();

  auto send_switch_request =
    [&](const std::vector<std::string> & activate, const std::vector<std::string> & deactivate,
        const char * action) -> bool {
      auto request =
        std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
      request->activate_controllers = activate;
      request->deactivate_controllers = deactivate;
      request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
      request->activate_asap = false;
      request->timeout.sec = 0;
      request->timeout.nanosec = 0;

      auto future = switch_controller_client_->async_send_request(request);
      if (future.wait_for(timeout_ms) != std::future_status::ready) {
        RCLCPP_ERROR(
          logger, "Timed out %s controller '%s' via '%s'", action, sm.controllerName().c_str(),
          sm.controllerManagerService().c_str());
        return false;
      }

      auto response = future.get();
      if (!response->ok) {
        RCLCPP_ERROR(
          logger, "Controller manager failed to %s '%s' through '%s'", action,
          sm.controllerName().c_str(), sm.controllerManagerService().c_str());
        return false;
      }
      return true;
    };

  auto send_configure_request = [&]() -> bool {
    auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    request->name = sm.controllerName();
    auto future = configure_controller_client_->async_send_request(request);
    if (future.wait_for(timeout_ms) != std::future_status::ready) {
      RCLCPP_ERROR(
        logger, "Timed out configuring controller '%s' via '%s'", sm.controllerName().c_str(),
        sm.configureControllerService().c_str());
      return false;
    }
    auto response = future.get();
    if (!response->ok) {
      RCLCPP_ERROR(
        logger, "Configure controller '%s' via '%s' failed",
        sm.controllerName().c_str(), sm.configureControllerService().c_str());
      return false;
    }
    return true;
  };

  if (!send_switch_request({}, {sm.controllerName()}, "deactivating")) {
    return false;
  }

  if (!send_configure_request()) {
    return false;
  }

  if (!send_switch_request({sm.controllerName()}, {}, "activating")) {
    return false;
  }

  RCLCPP_INFO(
    logger, "Restarted controller '%s' through '%s'", sm.controllerName().c_str(),
    sm.controllerManagerService().c_str());
  return true;
}

bool StReset::resetPose(const SmButtonNav & sm)
{
  const auto logger = this->getLogger();
  if (!set_pose_client_) {
    RCLCPP_ERROR(logger, "SetPose client is not initialized");
    return false;
  }
  if (!waitForService(set_pose_client_, sm.resetPoseService(), sm.serviceWaitTimeout())) {
    return false;
  }

  auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
  request->pose = sm.resetPose();
  request->pose.header.stamp = this->getNode()->now();

  auto future = set_pose_client_->async_send_request(request);
  if (future.wait_for(sm.poseServiceTimeout()) != std::future_status::ready) {
    RCLCPP_ERROR(
      logger, "Timed out setting pose via '%s'", sm.resetPoseService().c_str());
    return false;
  }

  // Response contains a placeholder byte; reaching here means the call returned.
  (void)future.get();
  RCLCPP_INFO(
    logger, "Reset robot pose to (%.2f, %.2f, %.2f) using '%s'",
    request->pose.pose.pose.position.x, request->pose.pose.pose.position.y, sm.resetPoseYaw(),
    sm.resetPoseService().c_str());
  return true;
}

bool StReset::waitForService(
  const rclcpp::ClientBase::SharedPtr & client, const std::string & service_name,
  std::chrono::milliseconds timeout)
{
  if (!client) {
    RCLCPP_ERROR(this->getLogger(), "Service client for '%s' is not available", service_name.c_str());
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!client->wait_for_service(250ms)) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(
        this->getLogger(), "Interrupted while waiting for service '%s'", service_name.c_str());
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        this->getLogger(), "Timed out waiting for service '%s'", service_name.c_str());
      return false;
    }
    RCLCPP_WARN(this->getLogger(), "Waiting for service '%s'...", service_name.c_str());
  }

  return true;
}

void StWaypointWait::completeExternalTask(bool success)
{
  if (!ext_waiting_) {
    return;
  }

  ext_waiting_ = false;

  if (ext_timeout_timer_) {
    ext_timeout_timer_->cancel();
    ext_timeout_timer_.reset();
  }

  auto & sm   = this->context<SmButtonNav>();
  auto logger = this->getLogger();

  if (success && sm.hasPendingWaypoints())
  {
    RCLCPP_INFO(
      logger,
      "External state machine completed successfully; proceeding to next waypoint");
    sm.postEvent<EvWaitOver>();
  }
  else
  {
    if (!success)
    {
      RCLCPP_WARN(
        logger,
        "External state machine reported FAILURE; ending mission");
    }
    else
    {
      RCLCPP_INFO(
        logger,
        "External state machine success at final waypoint; mission complete");
    }
    sm.postEvent<EvMissionCompleted>();
  }
}

}  // namespace smacc_button_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<smacc_button_nav::SmButtonNav>();
  rclcpp::shutdown();
  return 0;
}
