#include <ieee_fleet_msgs/action/execute_command.hpp>
#include <ieee_fleet_msgs/msg/robot_state.hpp>

#include <rmf_fleet_adapter/agv/Adapter.hpp>
#include <rmf_fleet_adapter/agv/EasyFullControl.hpp>
#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>

#include <rmf_fleet_msgs/msg/closed_lanes.hpp>
#include <rmf_fleet_msgs/msg/lane_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/speed_limit_request.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

using ExecuteCommand = ieee_fleet_msgs::action::ExecuteCommand;
using RobotStateMsg = ieee_fleet_msgs::msg::RobotState;

struct RobotUpdateData
{
  rclcpp::Time stamp;
  std::string robot_name;
  std::string map_name;
  Eigen::Vector3d position;
  double battery_soc;
  bool battery_soc_valid;
  std::uint64_t last_request_completed;
  bool requires_replan;

  bool is_command_completed(const std::uint64_t cmd_id) const
  {
    return last_request_completed == cmd_id;
  }
};

using RequiredActionKeys = std::unordered_map<std::string, std::vector<std::string>>;

struct AdapterConfig
{
  double state_stale_threshold_sec = 2.0;
  double battery_fallback_soc = 1.0;
  double low_battery_threshold = 0.2;
  bool enable_battery_gating = true;
  std::unordered_set<std::string> critical_action_categories;
  std::unordered_map<std::string, double> action_timeouts;
  double default_action_timeout_sec = 30.0;
  double navigate_timeout_base_sec = 5.0;
  double navigate_timeout_per_meter_sec = 3.0;
  double navigate_timeout_min_sec = 10.0;
  double navigate_timeout_max_sec = 300.0;
  double retry_backoff_sec = 1.0;
  std::string health_topic = "/ieee_fleet/adapter_health";
  double health_publish_period_sec = 2.0;
  double metrics_publish_period_sec = 30.0;
  double nominal_linear_speed = 0.5;
  std::unordered_set<std::string> allowed_actions;
  RequiredActionKeys required_action_keys;
};

struct ActiveCommand
{
  std::uint64_t cmd_id;
  std::string category;
  std::chrono::steady_clock::time_point start_time;
  double timeout_sec;
};

struct ActiveExecution
{
  std::function<void()> finished;
  std::function<bool()> okay;
  std::function<rmf_fleet_adapter::agv::EasyFullControl::ConstActivityIdentifierPtr()> identifier;
};

template<typename ExecutionT>
static ActiveExecution make_active_execution(ExecutionT execution)
{
  auto shared_execution = std::make_shared<ExecutionT>(std::move(execution));
  return ActiveExecution{
    [shared_execution]()
    {
      shared_execution->finished();
    },
    [shared_execution]()
    {
      return shared_execution->okay();
    },
    [shared_execution]()
    {
      return shared_execution->identifier();
    },
  };
}

static RequiredActionKeys default_required_action_keys()
{
  return {
    {"deploy", {"site"}},
    {"activate_antenna", {"antenna_id"}},
    {"pickup_duck", {"pickup_zone"}},
    {"drop_duck", {"drop_zone"}},
    {"crater_lap_leader", {"lap_count"}},
    {"crater_lap_follower", {"leader_id"}},
    {"dock", {"dock"}},
    {"takeoff", {"altitude_m"}},
    {"scan_leds", {"pattern"}},
    {"send_ir", {"payload"}},
  };
}

static std::string trim_slashes(std::string value)
{
  while (!value.empty() && value.front() == '/')
    value.erase(value.begin());

  while (!value.empty() && value.back() == '/')
    value.pop_back();

  return value;
}

template<typename T>
static T yaml_scalar_or(const YAML::Node& node, const std::string& key, const T& fallback)
{
  if (!node || !node[key])
    return fallback;

  try
  {
    return node[key].as<T>();
  }
  catch (const YAML::Exception&)
  {
    return fallback;
  }
}

static std::unordered_set<std::string> yaml_string_set(
  const YAML::Node& node,
  const std::string& key)
{
  std::unordered_set<std::string> result;
  if (!node || !node[key] || !node[key].IsSequence())
    return result;

  for (const auto& entry : node[key])
  {
    try
    {
      result.insert(entry.as<std::string>());
    }
    catch (const YAML::Exception&)
    {
      continue;
    }
  }

  return result;
}

static std::unordered_map<std::string, double> yaml_double_map(
  const YAML::Node& node,
  const std::string& key)
{
  std::unordered_map<std::string, double> result;
  if (!node || !node[key] || !node[key].IsMap())
    return result;

  for (const auto& it : node[key])
  {
    if (!it.first || !it.second)
      continue;

    try
    {
      result[it.first.as<std::string>()] = it.second.as<double>();
    }
    catch (const YAML::Exception&)
    {
      continue;
    }
  }

  return result;
}

static RequiredActionKeys yaml_required_action_keys(
  const YAML::Node& node,
  const std::string& key,
  const RequiredActionKeys& fallback)
{
  if (!node || !node[key] || !node[key].IsMap())
    return fallback;

  RequiredActionKeys result;
  for (const auto& it : node[key])
  {
    if (!it.first || !it.second || !it.second.IsSequence())
      continue;

    std::vector<std::string> keys;
    for (const auto& key_node : it.second)
    {
      try
      {
        keys.push_back(key_node.as<std::string>());
      }
      catch (const YAML::Exception&)
      {
        continue;
      }
    }

    try
    {
      result[it.first.as<std::string>()] = std::move(keys);
    }
    catch (const YAML::Exception&)
    {
      continue;
    }
  }

  if (result.empty())
    return fallback;

  return result;
}

class MetricsTracker
{
public:
  void record(const std::string& category, const std::string& outcome, const double duration_sec)
  {
    std::lock_guard<std::mutex> lock(_mutex);

    auto& entry = _stats[category];
    entry[outcome] += 1.0;
    entry["total_duration"] += duration_sec;
    entry["count_duration"] += 1.0;
  }

  void record_replan()
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _replans += 1.0;
  }

  nlohmann::json snapshot() const
  {
    std::lock_guard<std::mutex> lock(_mutex);

    nlohmann::json result = nlohmann::json::object();
    for (const auto& [category, entry] : _stats)
    {
      auto category_json = nlohmann::json::object();
      for (const auto& [key, value] : entry)
        category_json[key] = value;

      const auto count_duration_it = entry.find("count_duration");
      const auto total_duration_it = entry.find("total_duration");
      const double count_duration =
        count_duration_it == entry.end() ? 0.0 : count_duration_it->second;
      const double total_duration =
        total_duration_it == entry.end() ? 0.0 : total_duration_it->second;

      category_json["avg_duration"] =
        count_duration > 0.0 ? total_duration / count_duration : 0.0;
      result[category] = std::move(category_json);
    }

    result["__replans__"] = { {"count", _replans} };
    return result;
  }

private:
  mutable std::mutex _mutex;
  std::unordered_map<std::string, std::unordered_map<std::string, double>> _stats;
  double _replans = 0.0;
};

class RobotAPI
{
public:
  using GoalHandleExecute = rclcpp_action::ClientGoalHandle<ExecuteCommand>;
  using ActionClient = rclcpp_action::Client<ExecuteCommand>;

  RobotAPI(
    rclcpp::Node::SharedPtr node,
    std::string action_name,
    std::string state_topic,
    const double command_timeout,
    const double server_wait_timeout,
    const bool use_robot_namespace,
    const bool debug)
  : _node(std::move(node)),
    _action_name(trim_slashes(std::move(action_name))),
    _command_timeout(command_timeout),
    _server_wait_timeout(server_wait_timeout),
    _use_robot_namespace(use_robot_namespace),
    _debug(debug)
  {
    _state_sub = _node->create_subscription<RobotStateMsg>(
      std::move(state_topic),
      rclcpp::SystemDefaultsQoS(),
      [this](const RobotStateMsg::ConstSharedPtr msg)
      {
        if (msg->robot_name.empty())
        {
          RCLCPP_WARN(_node->get_logger(), "RobotState missing robot_name; ignoring");
          return;
        }

        std::lock_guard<std::mutex> lock(_state_mutex);
        _states[msg->robot_name] = *msg;
      });
  }

  std::optional<RobotUpdateData> get_data(const std::string& robot_name)
  {
    RobotStateMsg state;
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      const auto it = _states.find(robot_name);
      if (it == _states.end())
        return std::nullopt;

      state = it->second;
    }

    const double battery_soc = static_cast<double>(state.battery_soc);
    const bool battery_valid =
      std::isfinite(battery_soc) && battery_soc >= 0.0 && battery_soc <= 1.0;

    return RobotUpdateData{
      rclcpp::Time(state.stamp),
      state.robot_name,
      state.map_name,
      Eigen::Vector3d(state.x, state.y, state.yaw),
      battery_soc,
      battery_valid,
      state.last_completed_request,
      state.requires_replan,
    };
  }

  bool navigate(
    const std::string& robot_name,
    const std::uint64_t cmd_id,
    const Eigen::Vector3d& pose,
    const std::string& map_name,
    const double speed_limit,
    const std::optional<std::string>& dock = std::nullopt)
  {
    nlohmann::json payload = {
      {"map_name", map_name},
      {"x", pose.x()},
      {"y", pose.y()},
      {"yaw", pose.z()},
      {"speed_limit", speed_limit},
    };

    std::string category = "navigate";
    if (dock.has_value())
    {
      payload["dock"] = *dock;
      category = "dock";
    }

    return send_command(robot_name, cmd_id, category, payload);
  }

  bool perform_action(
    const std::string& robot_name,
    const std::uint64_t cmd_id,
    const std::string& category,
    const nlohmann::json& description)
  {
    const nlohmann::json payload = {
      {"category", category},
      {"description", description},
    };

    return send_command(robot_name, cmd_id, category, payload);
  }

  bool stop(const std::string& robot_name)
  {
    ActionClient::SharedPtr client;
    GoalHandleExecute::SharedPtr goal_handle;

    {
      std::lock_guard<std::mutex> lock(_clients_mutex);
      const auto client_it = _clients.find(robot_name);
      if (client_it == _clients.end())
        return true;

      client = client_it->second;

      const auto goal_it = _goal_handles.find(robot_name);
      if (goal_it == _goal_handles.end() || !goal_it->second)
        return true;

      goal_handle = goal_it->second;
    }

    try
    {
      auto cancel_future = client->async_cancel_goal(goal_handle);
      const auto status =
        cancel_future.wait_for(std::chrono::duration<double>(_command_timeout));

      if (status != std::future_status::ready)
        return false;

      const auto cancel_response = cancel_future.get();
      const bool success =
        cancel_response && !cancel_response->goals_canceling.empty();

      if (success)
      {
        std::lock_guard<std::mutex> lock(_clients_mutex);
        _goal_handles.erase(robot_name);
      }

      return success;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Failed to cancel goal for [%s]: %s",
        robot_name.c_str(),
        e.what());
      return false;
    }
  }

private:
  std::pair<ActionClient::SharedPtr, std::string> client_for(const std::string& robot_name)
  {
    std::lock_guard<std::mutex> lock(_clients_mutex);

    const auto existing = _clients.find(robot_name);
    if (existing != _clients.end())
      return {existing->second, _client_action_names[robot_name]};

    const std::string clean_name = trim_slashes(robot_name);
    const std::string action_full_name =
      _use_robot_namespace
      ? "/" + clean_name + "/" + _action_name
      : "/" + _action_name;

    auto client = rclcpp_action::create_client<ExecuteCommand>(_node, action_full_name);
    _clients[robot_name] = client;
    _client_action_names[robot_name] = action_full_name;
    return {std::move(client), action_full_name};
  }

  bool send_command(
    const std::string& robot_name,
    const std::uint64_t cmd_id,
    const std::string& category,
    const nlohmann::json& payload)
  {
    const auto [client, action_name] = client_for(robot_name);

    if (!client->wait_for_action_server(std::chrono::duration<double>(_server_wait_timeout)))
    {
      if (_debug)
      {
        RCLCPP_WARN(
          _node->get_logger(),
          "Action server for [%s] not available on [%s]",
          robot_name.c_str(),
          action_name.c_str());
      }
      return false;
    }

    ExecuteCommand::Goal goal;
    goal.command_id = cmd_id;
    goal.category = category;
    goal.description_json = payload.dump();

    try
    {
      auto goal_future = client->async_send_goal(goal);
      const auto status =
        goal_future.wait_for(std::chrono::duration<double>(_command_timeout));

      if (status != std::future_status::ready)
      {
        if (_debug)
        {
          RCLCPP_WARN(
            _node->get_logger(),
            "Command timeout waiting for [%s] to accept goal",
            robot_name.c_str());
        }
        return false;
      }

      auto goal_handle = goal_future.get();
      if (!goal_handle)
        return false;

      {
        std::lock_guard<std::mutex> lock(_clients_mutex);
        _goal_handles[robot_name] = std::move(goal_handle);
      }

      return true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "Failed to send goal to [%s]: %s",
        robot_name.c_str(),
        e.what());
      return false;
    }
  }

  rclcpp::Node::SharedPtr _node;
  std::string _action_name;
  double _command_timeout;
  double _server_wait_timeout;
  bool _use_robot_namespace;
  bool _debug;

  std::mutex _clients_mutex;
  std::unordered_map<std::string, ActionClient::SharedPtr> _clients;
  std::unordered_map<std::string, std::string> _client_action_names;
  std::unordered_map<std::string, GoalHandleExecute::SharedPtr> _goal_handles;

  std::mutex _state_mutex;
  std::unordered_map<std::string, RobotStateMsg> _states;
  rclcpp::Subscription<RobotStateMsg>::SharedPtr _state_sub;
};

class RobotAdapter : public std::enable_shared_from_this<RobotAdapter>
{
public:
  RobotAdapter(
    std::string name,
    rmf_fleet_adapter::agv::EasyFullControl::RobotConfiguration configuration,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<RobotAPI> api,
    std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl> fleet_handle,
    std::shared_ptr<const AdapterConfig> adapter_config,
    std::shared_ptr<MetricsTracker> metrics)
  : _name(std::move(name)),
    _configuration(std::move(configuration)),
    _node(std::move(node)),
    _api(std::move(api)),
    _fleet_handle(std::move(fleet_handle)),
    _adapter_config(std::move(adapter_config)),
    _metrics(std::move(metrics))
  {
    // Do nothing
  }

  ~RobotAdapter()
  {
    shutdown();
  }

  const std::string& name() const
  {
    return _name;
  }

  const rmf_fleet_adapter::agv::EasyFullControl::RobotConfiguration& configuration() const
  {
    return _configuration;
  }

  std::shared_ptr<RobotAPI> api() const
  {
    return _api;
  }

  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl> fleet_handle() const
  {
    return _fleet_handle;
  }

  const AdapterConfig& adapter_config() const
  {
    return *_adapter_config;
  }

  void set_update_handle(
    std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle> handle)
  {
    std::lock_guard<std::mutex> lock(_state_mutex);
    _update_handle = std::move(handle);
  }

  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle>
  update_handle() const
  {
    std::lock_guard<std::mutex> lock(_state_mutex);
    return _update_handle;
  }

  std::string last_map_name() const
  {
    std::lock_guard<std::mutex> lock(_state_mutex);
    return _last_map_name;
  }

  std::optional<double> last_state_age_sec(const rclcpp::Time& now) const
  {
    std::lock_guard<std::mutex> lock(_state_mutex);
    if (!_last_state_time.has_value())
      return std::nullopt;

    const auto age_ns = now.nanoseconds() - _last_state_time->nanoseconds();
    return static_cast<double>(age_ns) / 1e9;
  }

  rmf_fleet_adapter::agv::EasyFullControl::RobotCallbacks make_callbacks()
  {
    return rmf_fleet_adapter::agv::EasyFullControl::RobotCallbacks(
      [self = shared_from_this()](
        rmf_fleet_adapter::agv::EasyFullControl::Destination destination,
        rmf_fleet_adapter::agv::EasyFullControl::CommandExecution execution)
      {
        self->navigate(std::move(destination), std::move(execution));
      },
      [self = shared_from_this()](
        rmf_fleet_adapter::agv::EasyFullControl::ConstActivityIdentifierPtr activity)
      {
        self->stop(std::move(activity));
      },
      [self = shared_from_this()](
        const std::string& category,
        const nlohmann::json& description,
        rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution execution)
      {
        self->execute_action(category, description, std::move(execution));
      });
  }

  void update(const RobotUpdateData& data)
  {
    const auto now = _node->get_clock()->now();

    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      _last_state_time = data.stamp;
      _last_position = data.position;

      if (!data.map_name.empty())
      {
        _last_map_name = data.map_name;
      }
      else if (_last_map_name.empty())
      {
        _last_map_name = "unknown";
        RCLCPP_ERROR(
          _node->get_logger(),
          "[%s] RobotState missing map_name",
          _name.c_str());
      }
    }

    if (_is_state_stale(now))
      _set_unavailable("stale");
    else
      _clear_unavailable("stale");

    double battery_soc = data.battery_soc;
    if (!data.battery_soc_valid)
    {
      bool should_log = false;
      {
        std::lock_guard<std::mutex> lock(_state_mutex);
        should_log = !_battery_missing_logged;
        _battery_missing_logged = true;
      }

      if (should_log)
      {
        RCLCPP_WARN(
          _node->get_logger(),
          "[%s] Battery SOC missing/invalid; using fallback %.3f",
          _name.c_str(),
          _adapter_config->battery_fallback_soc);
      }

      battery_soc = _adapter_config->battery_fallback_soc;
    }
    else
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      _battery_missing_logged = false;
    }

    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      _last_battery_soc = battery_soc;
    }

    if (_adapter_config->enable_battery_gating)
    {
      if (battery_soc < _adapter_config->low_battery_threshold)
        _set_unavailable("low_battery");
      else
        _clear_unavailable("low_battery");
    }

    bool should_replan = false;
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      if (data.requires_replan && !_last_requires_replan)
        should_replan = true;

      _last_requires_replan = data.requires_replan;
    }

    if (should_replan)
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[%s] requires_replan=true; triggering RMF replan",
        _name.c_str());
      _request_replan();
      _fleet_handle->more()->reassign_dispatched_tasks();
      _metrics->record_replan();
    }

    std::optional<ActiveExecution> execution;
    std::optional<ActiveCommand> active;
    std::uint64_t cmd_id = 0;
    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      execution = _execution;
      active = _active_command;
      cmd_id = _cmd_id;
    }

    rmf_fleet_adapter::agv::EasyFullControl::ConstActivityIdentifierPtr activity_identifier;
    if (execution.has_value())
    {
      if (data.is_command_completed(cmd_id))
      {
        _finish_active_command(true, "completed");
        std::lock_guard<std::mutex> lock(_cmd_mutex);
        _execution.reset();
      }
      else
      {
        activity_identifier = execution->identifier();
      }
    }

    if (active.has_value())
      _check_timeout();

    std::string map_name;
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      if (!_last_map_name.empty())
        map_name = _last_map_name;
      else if (!data.map_name.empty())
        map_name = data.map_name;
      else
        map_name = "unknown";
    }

    auto handle = update_handle();
    if (!handle)
      return;

    rmf_fleet_adapter::agv::EasyFullControl::RobotState state(
      map_name,
      data.position,
      battery_soc);

    handle->update(std::move(state), activity_identifier);
  }

  void finish_action()
  {
    std::optional<ActiveExecution> execution;
    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      execution = _execution;
      _execution.reset();
    }

    if (execution.has_value())
    {
      execution->finished();
      _finish_active_command(true, "manual_finish");
    }
  }

  void shutdown()
  {
    cancel_cmd_attempt();
  }

private:
  bool _is_state_stale(const rclcpp::Time& now) const
  {
    const auto age = last_state_age_sec(now);
    if (!age.has_value())
      return true;

    if (*age < 0.0)
      return true;

    return *age > _adapter_config->state_stale_threshold_sec;
  }

  std::shared_ptr<rmf_fleet_adapter::agv::RobotUpdateHandle> _rmf_update_handle() const
  {
    auto easy_handle = update_handle();
    if (!easy_handle)
      return nullptr;

    return easy_handle->more();
  }

  bool _set_commissioned_state(const bool commissioned)
  {
    auto handle = _rmf_update_handle();
    if (!handle)
      return false;

    if (commissioned)
    {
      rmf_fleet_adapter::agv::RobotUpdateHandle::Commission commission;
      handle->set_commission(std::move(commission));
    }
    else
    {
      handle->set_commission(
        rmf_fleet_adapter::agv::RobotUpdateHandle::Commission::decommission());
    }

    return true;
  }

  bool _override_status(const std::optional<std::string>& new_status)
  {
    auto handle = _rmf_update_handle();
    if (!handle)
      return false;

    handle->override_status(new_status);
    return true;
  }

  void _request_replan()
  {
    auto handle = _rmf_update_handle();
    if (!handle)
      return;

    handle->replan();
  }

  std::string _unavailable_status_override() const
  {
    std::lock_guard<std::mutex> lock(_state_mutex);
    if (_decommission_reasons.count("stale") != 0)
      return "offline";

    return "error";
  }

  void _set_unavailable(const std::string& reason)
  {
    bool already_set = false;
    bool has_update_handle = false;
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      already_set = _decommission_reasons.count(reason) != 0;
      if (!already_set)
      {
        _decommission_reasons.insert(reason);
        has_update_handle = static_cast<bool>(_update_handle);
      }
    }

    if (already_set || !has_update_handle)
      return;

    if (!_set_commissioned_state(false))
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[%s] Could not decommission robot via RMF API",
        _name.c_str());
    }

    _override_status(_unavailable_status_override());

    RCLCPP_WARN(
      _node->get_logger(),
      "[%s] decommissioned (%s)",
      _name.c_str(),
      reason.c_str());
  }

  void _clear_unavailable(const std::string& reason)
  {
    bool existed = false;
    bool has_update_handle = false;
    bool has_remaining_reasons = false;

    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      const auto it = _decommission_reasons.find(reason);
      if (it != _decommission_reasons.end())
      {
        existed = true;
        _decommission_reasons.erase(it);
      }

      has_update_handle = static_cast<bool>(_update_handle);
      has_remaining_reasons = !_decommission_reasons.empty();
    }

    if (!existed || !has_update_handle)
      return;

    if (!has_remaining_reasons)
    {
      if (!_set_commissioned_state(true))
      {
        RCLCPP_WARN(
          _node->get_logger(),
          "[%s] Could not recommission robot via RMF API",
          _name.c_str());
      }

      _override_status(std::nullopt);
      RCLCPP_INFO(_node->get_logger(), "[%s] recommissioned", _name.c_str());
      return;
    }

    _override_status(_unavailable_status_override());
  }

  void navigate(
    rmf_fleet_adapter::agv::EasyFullControl::Destination destination,
    rmf_fleet_adapter::agv::EasyFullControl::CommandExecution execution)
  {
    _preempt_active("navigate");

    std::uint64_t cmd_id = 0;
    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      _cmd_id += 1;
      cmd_id = _cmd_id;
      _execution = make_active_execution(std::move(execution));
    }

    const auto dock = destination.dock();
    const std::string category = dock.has_value() ? "dock" : "navigate";
    const double timeout_sec = _compute_navigation_timeout(destination);

    _start_command_tracking(cmd_id, category, timeout_sec);
    _log_command_event(
      "sent",
      cmd_id,
      category,
      {
        {"map", destination.map()},
        {"dock", dock.has_value() ? nlohmann::json(*dock) : nlohmann::json(nullptr)},
        {"timeout_sec", timeout_sec},
      });

    const auto pose = destination.position();
    const auto map_name = destination.map();
    const double speed_limit = destination.speed_limit().value_or(0.0);

    attempt_cmd_until_success(
      [this, cmd_id, pose, map_name, speed_limit, dock]()
      {
        return _api->navigate(
          _name,
          cmd_id,
          pose,
          map_name,
          speed_limit,
          dock);
      },
      cmd_id,
      category);
  }

  void stop(
    rmf_fleet_adapter::agv::EasyFullControl::ConstActivityIdentifierPtr activity)
  {
    std::optional<ActiveExecution> execution;
    std::uint64_t cmd_id = 0;

    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      cmd_id = _cmd_id;

      if (_execution.has_value())
      {
        const auto exec_identifier = _execution->identifier();
        const bool same_activity =
          exec_identifier && activity && (*exec_identifier == *activity);

        if (same_activity)
        {
          execution = _execution;
          _execution.reset();
        }
      }
    }

    if (execution.has_value())
    {
      _log_command_event("stop", cmd_id, "stop", nlohmann::json::object());
      execution->finished();
      _finish_active_command(false, "stopped");
    }

    attempt_cmd_until_success(
      [this]()
      {
        return _api->stop(_name);
      },
      cmd_id,
      "stop");
  }

  void execute_action(
    const std::string& category,
    const nlohmann::json& description,
    rmf_fleet_adapter::agv::RobotUpdateHandle::ActionExecution execution)
  {
    if (_adapter_config->allowed_actions.count(category) == 0)
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "[%s] Rejected unsupported action category [%s]",
        _name.c_str(),
        category.c_str());
      execution.finished();
      return;
    }

    if (!description.is_object())
    {
      RCLCPP_ERROR(
        _node->get_logger(),
        "[%s] Invalid action payload for [%s]",
        _name.c_str(),
        category.c_str());
      execution.finished();
      _metrics->record(category, "rejected_invalid", 0.0);
      return;
    }

    const auto required_it = _adapter_config->required_action_keys.find(category);
    if (required_it != _adapter_config->required_action_keys.end())
    {
      std::vector<std::string> missing;
      for (const auto& key : required_it->second)
      {
        if (!description.contains(key))
          missing.push_back(key);
      }

      if (!missing.empty())
      {
        nlohmann::json missing_json = missing;
        RCLCPP_ERROR(
          _node->get_logger(),
          "[%s] Missing keys for [%s]: %s",
          _name.c_str(),
          category.c_str(),
          missing_json.dump().c_str());
        execution.finished();
        _metrics->record(category, "rejected_invalid", 0.0);
        return;
      }
    }

    std::optional<double> battery_soc;
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      battery_soc = _last_battery_soc;
    }

    if (
      _adapter_config->enable_battery_gating
      && battery_soc.has_value()
      && *battery_soc < _adapter_config->low_battery_threshold
      && _adapter_config->critical_action_categories.count(category) == 0)
    {
      RCLCPP_WARN(
        _node->get_logger(),
        "[%s] Rejecting action [%s] due to low battery",
        _name.c_str(),
        category.c_str());
      execution.finished();
      _metrics->record(category, "rejected_low_battery", 0.0);
      return;
    }

    _preempt_active(category);

    std::uint64_t cmd_id = 0;
    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      _cmd_id += 1;
      cmd_id = _cmd_id;
      _execution = make_active_execution(std::move(execution));
    }

    const double timeout_sec = _compute_action_timeout(category, description);
    _start_command_tracking(cmd_id, category, timeout_sec);

    _log_command_event(
      "sent",
      cmd_id,
      category,
      {{"timeout_sec", timeout_sec}});

    attempt_cmd_until_success(
      [this, cmd_id, category, description]()
      {
        return _api->perform_action(_name, cmd_id, category, description);
      },
      cmd_id,
      category);
  }

  void attempt_cmd_until_success(
    std::function<bool()> command,
    const std::uint64_t cmd_id,
    const std::string& category)
  {
    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      if (
        _pending_cmd_id.has_value()
        && _pending_category.has_value()
        && *_pending_cmd_id == cmd_id
        && *_pending_category == category)
      {
        return;
      }
    }

    cancel_cmd_attempt();

    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      _pending_cmd_id = cmd_id;
      _pending_category = category;
      _cancel_cmd_event.store(false);
    }

    _issue_cmd_thread = std::thread(
      [this, command = std::move(command), cmd_id, category]() mutable
      {
        while (rclcpp::ok() && !_cancel_cmd_event.load())
        {
          if (command())
          {
            _log_command_event("accepted", cmd_id, category, nlohmann::json::object());
            break;
          }

          RCLCPP_WARN(
            _node->get_logger(),
            "Failed to contact command endpoint for robot %s (cmd_id=%lu, category=%s)",
            _name.c_str(),
            static_cast<unsigned long>(cmd_id),
            category.c_str());

          const auto backoff = std::chrono::duration<double>(_adapter_config->retry_backoff_sec);
          std::this_thread::sleep_for(backoff);
        }

        std::lock_guard<std::mutex> lock(_cmd_mutex);
        _pending_cmd_id.reset();
        _pending_category.reset();
      });
  }

  void cancel_cmd_attempt()
  {
    _cancel_cmd_event.store(true);

    if (_issue_cmd_thread.joinable())
    {
      if (_issue_cmd_thread.get_id() == std::this_thread::get_id())
        _issue_cmd_thread.detach();
      else
        _issue_cmd_thread.join();
    }

    _cancel_cmd_event.store(false);
  }

  void _start_command_tracking(
    const std::uint64_t cmd_id,
    const std::string& category,
    const double timeout_sec)
  {
    std::lock_guard<std::mutex> lock(_cmd_mutex);
    _active_command = ActiveCommand{
      cmd_id,
      category,
      std::chrono::steady_clock::now(),
      timeout_sec,
    };
  }

  void _finish_active_command(const bool success, const std::string& reason)
  {
    std::optional<ActiveCommand> active;
    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      active = _active_command;
      _active_command.reset();
    }

    if (!active.has_value())
      return;

    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - active->start_time).count();

    std::string outcome = success ? "success" : reason;
    if (reason == "timeout" || reason == "preempted" || reason == "stopped")
      outcome = reason;

    _metrics->record(active->category, outcome, elapsed);

    _log_command_event(
      "finished",
      active->cmd_id,
      active->category,
      {
        {"success", success},
        {"reason", reason},
        {"duration_sec", elapsed},
      });

    if (!success)
      _fleet_handle->more()->reassign_dispatched_tasks();
  }

  void _check_timeout()
  {
    std::optional<ActiveCommand> active;
    std::optional<ActiveExecution> execution;

    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      active = _active_command;
      execution = _execution;
    }

    if (!active.has_value())
      return;

    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - active->start_time).count();

    if (elapsed <= active->timeout_sec)
      return;

    RCLCPP_WARN(
      _node->get_logger(),
      "[%s] Command %lu timed out after %.1fs",
      _name.c_str(),
      static_cast<unsigned long>(active->cmd_id),
      elapsed);

    _api->stop(_name);

    if (execution.has_value())
      execution->finished();

    _finish_active_command(false, "timeout");

    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      _execution.reset();
    }
  }

  double _compute_navigation_timeout(
    const rmf_fleet_adapter::agv::EasyFullControl::Destination& destination) const
  {
    const double base = _adapter_config->navigate_timeout_base_sec;
    const double per_meter = _adapter_config->navigate_timeout_per_meter_sec;

    double distance = 0.0;
    {
      std::lock_guard<std::mutex> lock(_state_mutex);
      if (_last_position.has_value())
      {
        const auto dx = destination.position().x() - _last_position->x();
        const auto dy = destination.position().y() - _last_position->y();
        distance = std::sqrt(dx * dx + dy * dy);
      }
    }

    double timeout = base + (distance * per_meter);
    timeout = std::max(timeout, _adapter_config->navigate_timeout_min_sec);
    timeout = std::min(timeout, _adapter_config->navigate_timeout_max_sec);
    return timeout;
  }

  double _compute_action_timeout(
    const std::string& category,
    const nlohmann::json& description) const
  {
    const auto timeout_it = _adapter_config->action_timeouts.find(category);
    if (timeout_it != _adapter_config->action_timeouts.end())
      return timeout_it->second;

    if (description.is_object())
    {
      if (description.contains("duration_ms") && description["duration_ms"].is_number())
        return description["duration_ms"].get<double>() / 1000.0;

      if (description.contains("duration_sec") && description["duration_sec"].is_number())
        return description["duration_sec"].get<double>();
    }

    return _adapter_config->default_action_timeout_sec;
  }

  void _preempt_active(const std::string& new_category)
  {
    std::optional<ActiveCommand> active;
    std::optional<ActiveExecution> execution;

    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      active = _active_command;
      execution = _execution;
    }

    if (!active.has_value() || !execution.has_value())
      return;

    RCLCPP_WARN(
      _node->get_logger(),
      "[%s] Preempting cmd_id=%lu for new command category=%s",
      _name.c_str(),
      static_cast<unsigned long>(active->cmd_id),
      new_category.c_str());

    _api->stop(_name);
    execution->finished();
    _finish_active_command(false, "preempted");

    {
      std::lock_guard<std::mutex> lock(_cmd_mutex);
      _execution.reset();
    }
  }

  void _log_command_event(
    const std::string& event,
    const std::uint64_t cmd_id,
    const std::string& category,
    const nlohmann::json& extra)
  {
    nlohmann::json payload = {
      {"event", event},
      {"robot", _name},
      {"cmd_id", cmd_id},
      {"category", category},
    };

    if (extra.is_object())
      payload.update(extra);

    RCLCPP_INFO(_node->get_logger(), "CMD %s", payload.dump().c_str());
  }

  std::string _name;
  rmf_fleet_adapter::agv::EasyFullControl::RobotConfiguration _configuration;
  rclcpp::Node::SharedPtr _node;
  std::shared_ptr<RobotAPI> _api;
  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl> _fleet_handle;
  std::shared_ptr<const AdapterConfig> _adapter_config;
  std::shared_ptr<MetricsTracker> _metrics;

  mutable std::mutex _state_mutex;
  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl::EasyRobotUpdateHandle> _update_handle;
  std::optional<rclcpp::Time> _last_state_time;
  std::optional<Eigen::Vector3d> _last_position;
  std::string _last_map_name;
  std::optional<double> _last_battery_soc;
  std::unordered_set<std::string> _decommission_reasons;
  bool _battery_missing_logged = false;
  bool _last_requires_replan = false;

  mutable std::mutex _cmd_mutex;
  std::uint64_t _cmd_id = 0;
  std::optional<ActiveExecution> _execution;
  std::optional<ActiveCommand> _active_command;
  std::optional<std::uint64_t> _pending_cmd_id;
  std::optional<std::string> _pending_category;
  std::thread _issue_cmd_thread;
  std::atomic_bool _cancel_cmd_event{false};
};

static void update_robot(const std::shared_ptr<RobotAdapter>& robot)
{
  const auto data = robot->api()->get_data(robot->name());
  if (!data.has_value())
    return;

  if (!robot->update_handle())
  {
    double battery_soc = data->battery_soc;
    if (!data->battery_soc_valid)
      battery_soc = robot->adapter_config().battery_fallback_soc;

    std::string map_name = data->map_name;
    if (map_name.empty())
    {
      map_name = robot->last_map_name();
      if (map_name.empty())
        map_name = "unknown";
    }

    rmf_fleet_adapter::agv::EasyFullControl::RobotState state(
      map_name,
      data->position,
      battery_soc);

    auto handle = robot->fleet_handle()->add_robot(
      robot->name(),
      std::move(state),
      robot->configuration(),
      robot->make_callbacks());

    if (!handle)
      return;

    robot->set_update_handle(std::move(handle));
    return;
  }

  robot->update(*data);
}

class RuntimeContext : public std::enable_shared_from_this<RuntimeContext>
{
public:
  RuntimeContext(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl> fleet_handle,
    std::shared_ptr<const AdapterConfig> adapter_config,
    std::shared_ptr<MetricsTracker> metrics,
    std::unordered_map<std::string, std::shared_ptr<RobotAdapter>> robots,
    const double reassign_task_interval_sec)
  : _node(std::move(node)),
    _fleet_handle(std::move(fleet_handle)),
    _adapter_config(std::move(adapter_config)),
    _metrics(std::move(metrics)),
    _robots(std::move(robots)),
    _reassign_task_interval_sec(reassign_task_interval_sec),
    _last_task_replan(_node->get_clock()->now()),
    _last_health_publish(std::chrono::steady_clock::now()),
    _last_metrics_publish(std::chrono::steady_clock::now())
  {
    _health_pub = _node->create_publisher<std_msgs::msg::String>(
      _adapter_config->health_topic,
      10);
  }

  void start(const double update_period_sec)
  {
    setup_ros_connections();

    const auto update_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(update_period_sec));

    _update_timer = _node->create_wall_timer(
      update_period,
      [self = shared_from_this()]()
      {
        self->on_update_timer();
      });
  }

  void shutdown()
  {
    if (_update_timer)
      _update_timer->cancel();

    for (auto& [_, robot] : _robots)
      robot->shutdown();
  }

private:
  void on_update_timer()
  {
    const auto now = _node->get_clock()->now();

    for (const auto& [_, robot] : _robots)
      update_robot(robot);

    const auto replan_elapsed_sec =
      static_cast<double>(now.nanoseconds() - _last_task_replan.nanoseconds()) / 1e9;
    if (replan_elapsed_sec > _reassign_task_interval_sec)
    {
      _fleet_handle->more()->reassign_dispatched_tasks();
      _last_task_replan = now;
    }

    const auto wall_now = std::chrono::steady_clock::now();

    if (
      std::chrono::duration<double>(wall_now - _last_health_publish).count()
      >= _adapter_config->health_publish_period_sec)
    {
      nlohmann::json health_payload = nlohmann::json::object();
      for (const auto& [name, robot] : _robots)
      {
        const auto age = robot->last_state_age_sec(now);
        health_payload[name] = {
          {"age_sec", age.has_value() ? nlohmann::json(*age) : nlohmann::json(nullptr)},
          {
            "stale",
            !age.has_value() || *age > _adapter_config->state_stale_threshold_sec,
          },
        };
      }

      std_msgs::msg::String msg;
      msg.data = health_payload.dump();
      _health_pub->publish(msg);
      _last_health_publish = wall_now;
    }

    if (
      std::chrono::duration<double>(wall_now - _last_metrics_publish).count()
      >= _adapter_config->metrics_publish_period_sec)
    {
      const auto snapshot = _metrics->snapshot();
      RCLCPP_INFO(
        _node->get_logger(),
        "Adapter metrics: %s",
        snapshot.dump().c_str());
      _last_metrics_publish = wall_now;
    }
  }

  void setup_ros_connections()
  {
    const auto fleet_name = _fleet_handle->more()->fleet_name();

    const auto transient_qos =
      rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local();

    _closed_lanes_pub = _node->create_publisher<rmf_fleet_msgs::msg::ClosedLanes>(
      "closed_lanes",
      transient_qos);

    _lane_request_sub = _node->create_subscription<rmf_fleet_msgs::msg::LaneRequest>(
      "lane_closure_requests",
      rclcpp::SystemDefaultsQoS(),
      [self = shared_from_this(), fleet_name](
        const rmf_fleet_msgs::msg::LaneRequest::SharedPtr msg)
      {
        if (msg->fleet_name.empty() || msg->fleet_name != fleet_name)
          return;

        self->_fleet_handle->more()->open_lanes(msg->open_lanes);
        self->_fleet_handle->more()->close_lanes(msg->close_lanes);

        for (const auto lane_idx : msg->close_lanes)
          self->_closed_lanes.insert(lane_idx);

        for (const auto lane_idx : msg->open_lanes)
          self->_closed_lanes.erase(lane_idx);

        rmf_fleet_msgs::msg::ClosedLanes state_msg;
        state_msg.fleet_name = fleet_name;
        state_msg.closed_lanes.insert(
          state_msg.closed_lanes.end(),
          self->_closed_lanes.begin(),
          self->_closed_lanes.end());

        self->_closed_lanes_pub->publish(state_msg);
      });

    _speed_limit_sub = _node->create_subscription<rmf_fleet_msgs::msg::SpeedLimitRequest>(
      "speed_limit_requests",
      rclcpp::SystemDefaultsQoS(),
      [self = shared_from_this(), fleet_name](
        const rmf_fleet_msgs::msg::SpeedLimitRequest::SharedPtr msg)
      {
        if (msg->fleet_name.empty() || msg->fleet_name != fleet_name)
          return;

        std::vector<rmf_fleet_adapter::agv::FleetUpdateHandle::SpeedLimitRequest> requests;
        requests.reserve(msg->speed_limits.size());

        for (const auto& limit : msg->speed_limits)
        {
          requests.emplace_back(limit.lane_index, limit.speed_limit);
        }

        self->_fleet_handle->more()->limit_lane_speeds(requests);
        self->_fleet_handle->more()->remove_speed_limits(msg->remove_limits);
      });

    _action_execution_notice_sub = _node->create_subscription<rmf_fleet_msgs::msg::ModeRequest>(
      "action_execution_notice",
      rclcpp::SystemDefaultsQoS(),
      [self = shared_from_this(), fleet_name](
        const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg)
      {
        if (
          msg->fleet_name.empty()
          || msg->fleet_name != fleet_name
          || msg->robot_name.empty())
        {
          return;
        }

        if (msg->mode.mode == rmf_fleet_msgs::msg::RobotMode::MODE_IDLE)
        {
          const auto it = self->_robots.find(msg->robot_name);
          if (it == self->_robots.end())
            return;

          it->second->finish_action();
        }
      });
  }

  rclcpp::Node::SharedPtr _node;
  std::shared_ptr<rmf_fleet_adapter::agv::EasyFullControl> _fleet_handle;
  std::shared_ptr<const AdapterConfig> _adapter_config;
  std::shared_ptr<MetricsTracker> _metrics;
  std::unordered_map<std::string, std::shared_ptr<RobotAdapter>> _robots;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _health_pub;
  rclcpp::Publisher<rmf_fleet_msgs::msg::ClosedLanes>::SharedPtr _closed_lanes_pub;

  rclcpp::Subscription<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr _lane_request_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::SpeedLimitRequest>::SharedPtr _speed_limit_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr _action_execution_notice_sub;

  rclcpp::TimerBase::SharedPtr _update_timer;

  std::unordered_set<std::size_t> _closed_lanes;

  double _reassign_task_interval_sec;
  rclcpp::Time _last_task_replan;
  std::chrono::steady_clock::time_point _last_health_publish;
  std::chrono::steady_clock::time_point _last_metrics_publish;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto args_without_ros = rclcpp::remove_ros_arguments(argc, argv);

  std::string config_path;
  std::string nav_graph_path;
  bool use_sim_time = false;

  for (std::size_t i = 1; i < args_without_ros.size(); ++i)
  {
    const auto& arg = args_without_ros[i];

    if ((arg == "-c" || arg == "--config_file") && i + 1 < args_without_ros.size())
    {
      config_path = args_without_ros[++i];
      continue;
    }

    if ((arg == "-n" || arg == "--nav_graph") && i + 1 < args_without_ros.size())
    {
      nav_graph_path = args_without_ros[++i];
      continue;
    }

    if (arg == "-sim" || arg == "--use_sim_time")
    {
      use_sim_time = true;
      continue;
    }

    if (arg == "-h" || arg == "--help")
    {
      std::cout
        << "Usage: fleet_adapter -c <config.yaml> -n <nav_graph.yaml> [-sim|--use_sim_time]"
        << std::endl;
      rclcpp::shutdown();
      return 0;
    }

    std::cerr << "Unknown argument: " << arg << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  if (config_path.empty() || nav_graph_path.empty())
  {
    std::cerr
      << "Missing required arguments. Usage: fleet_adapter -c <config.yaml> -n <nav_graph.yaml>"
      << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("fleet_adapter"), "Starting IEEE fleet adapter...");

  auto fleet_config_opt =
    rmf_fleet_adapter::agv::EasyFullControl::FleetConfiguration::from_config_files(
    config_path,
    nav_graph_path);

  if (!fleet_config_opt.has_value())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("fleet_adapter"),
      "Failed to parse config file [%s]",
      config_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  rmf_fleet_adapter::agv::EasyFullControl::FleetConfiguration fleet_config =
    *fleet_config_opt;

  YAML::Node config_yaml;
  try
  {
    config_yaml = YAML::LoadFile(config_path);
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("fleet_adapter"),
      "Failed to load YAML config [%s]: %s",
      config_path.c_str(),
      e.what());
    rclcpp::shutdown();
    return 1;
  }

  const std::string fleet_name = fleet_config.fleet_name();
  auto node = rclcpp::Node::make_shared(fleet_name + "_command_handle");

  auto adapter = rmf_fleet_adapter::agv::Adapter::make(
    fleet_name + "_fleet_adapter");

  if (!adapter)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Unable to initialize fleet adapter. Ensure RMF schedule node is running");
    rclcpp::shutdown();
    return 1;
  }

  if (use_sim_time)
  {
    if (!node->has_parameter("use_sim_time"))
      node->declare_parameter<bool>("use_sim_time", true);

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    auto adapter_node = adapter->node();
    if (!adapter_node->has_parameter("use_sim_time"))
      adapter_node->declare_parameter<bool>("use_sim_time", true);

    adapter_node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  }

  adapter->start();
  std::this_thread::sleep_for(1s);

  const auto server_uri_value = node->declare_parameter<std::string>("server_uri", "");
  if (server_uri_value.empty())
    fleet_config.set_server_uri(std::nullopt);
  else
    fleet_config.set_server_uri(server_uri_value);

  auto fleet_handle = adapter->add_easy_fleet(fleet_config);
  if (!fleet_handle)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to add easy fleet to adapter");
    adapter->stop();
    adapter->wait();
    rclcpp::shutdown();
    return 1;
  }

  const auto fleet_mgr_yaml = config_yaml["fleet_manager"];
  const auto rmf_fleet_yaml = config_yaml["rmf_fleet"];
  const auto limits_yaml = rmf_fleet_yaml ? rmf_fleet_yaml["limits"] : YAML::Node();

  double nominal_speed = 0.5;
  if (limits_yaml && limits_yaml["linear"] && limits_yaml["linear"].IsSequence())
  {
    const auto linear_limits = limits_yaml["linear"];
    if (linear_limits.size() > 0)
    {
      try
      {
        nominal_speed = linear_limits[0].as<double>();
      }
      catch (const YAML::Exception&)
      {
        nominal_speed = 0.5;
      }
    }
  }

  const double update_frequency =
    yaml_scalar_or<double>(fleet_mgr_yaml, "robot_state_update_frequency", 10.0);
  const double update_period = 1.0 / std::max(update_frequency, 1e-3);

  auto adapter_config = std::make_shared<AdapterConfig>();
  adapter_config->state_stale_threshold_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "state_stale_threshold_sec", 2.0);
  adapter_config->battery_fallback_soc =
    yaml_scalar_or<double>(fleet_mgr_yaml, "battery_fallback_soc", 1.0);
  adapter_config->low_battery_threshold =
    yaml_scalar_or<double>(fleet_mgr_yaml, "low_battery_threshold", 0.2);
  adapter_config->enable_battery_gating =
    yaml_scalar_or<bool>(fleet_mgr_yaml, "enable_battery_gating", true);
  adapter_config->critical_action_categories =
    yaml_string_set(fleet_mgr_yaml, "critical_action_categories");
  adapter_config->action_timeouts = yaml_double_map(fleet_mgr_yaml, "action_timeouts");
  adapter_config->default_action_timeout_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "default_action_timeout_sec", 30.0);
  adapter_config->navigate_timeout_base_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "navigate_timeout_base_sec", 5.0);
  adapter_config->navigate_timeout_per_meter_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "navigate_timeout_per_meter_sec", 3.0);
  adapter_config->navigate_timeout_min_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "navigate_timeout_min_sec", 10.0);
  adapter_config->navigate_timeout_max_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "navigate_timeout_max_sec", 300.0);
  adapter_config->retry_backoff_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "retry_backoff_sec", 1.0);
  adapter_config->health_topic =
    yaml_scalar_or<std::string>(fleet_mgr_yaml, "health_topic", "/ieee_fleet/adapter_health");
  adapter_config->health_publish_period_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "health_publish_period_sec", 2.0);
  adapter_config->metrics_publish_period_sec =
    yaml_scalar_or<double>(fleet_mgr_yaml, "metrics_publish_period_sec", 30.0);
  adapter_config->nominal_linear_speed = nominal_speed;
  adapter_config->allowed_actions = yaml_string_set(rmf_fleet_yaml, "actions");
  adapter_config->required_action_keys = yaml_required_action_keys(
    fleet_mgr_yaml,
    "required_action_keys",
    default_required_action_keys());

  if (adapter_config->allowed_actions.empty())
  {
    RCLCPP_WARN(
      node->get_logger(),
      "No action categories configured; perform_action will be rejected");
  }

  auto api = std::make_shared<RobotAPI>(
    node,
    yaml_scalar_or<std::string>(fleet_mgr_yaml, "command_action", "execute_command"),
    yaml_scalar_or<std::string>(fleet_mgr_yaml, "robot_state_topic", "/ieee_fleet/robot_state"),
    yaml_scalar_or<double>(fleet_mgr_yaml, "command_timeout", 5.0),
    yaml_scalar_or<double>(fleet_mgr_yaml, "action_server_wait_timeout", 2.0),
    yaml_scalar_or<bool>(fleet_mgr_yaml, "use_robot_namespace", true),
    yaml_scalar_or<bool>(fleet_mgr_yaml, "debug", false));

  auto metrics = std::make_shared<MetricsTracker>();

  std::unordered_map<std::string, std::shared_ptr<RobotAdapter>> robots;
  for (const auto& robot_name : fleet_config.known_robots())
  {
    const auto robot_config = fleet_config.get_known_robot_configuration(robot_name);
    if (!robot_config.has_value())
    {
      RCLCPP_WARN(
        node->get_logger(),
        "Missing robot configuration for known robot [%s]",
        robot_name.c_str());
      continue;
    }

    robots[robot_name] = std::make_shared<RobotAdapter>(
      robot_name,
      *robot_config,
      node,
      api,
      fleet_handle,
      adapter_config,
      metrics);
  }

  const double reassign_task_interval =
    yaml_scalar_or<double>(rmf_fleet_yaml, "reassign_task_interval", 60.0);

  auto context = std::make_shared<RuntimeContext>(
    node,
    fleet_handle,
    adapter_config,
    metrics,
    std::move(robots),
    reassign_task_interval);

  context->start(update_period);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  context->shutdown();
  adapter->stop();
  adapter->wait();
  rclcpp::shutdown();
  return 0;
}
