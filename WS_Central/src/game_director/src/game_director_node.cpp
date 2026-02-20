#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ieee_fleet_msgs/msg/robot_state.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <rmf_task_msgs/msg/dispatch_state.hpp>
#include <rmf_task_msgs/msg/dispatch_states.hpp>
#include <rmf_task_msgs/msg/task_summary.hpp>
#include <rmf_task_msgs/srv/cancel_task.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

namespace game_director
{
using json = nlohmann::json;

static const std::regex kPlaceholderRegex(R"(\{\{\s*([a-zA-Z0-9_.]+)\s*\}\})");
static const std::regex kWholePlaceholderRegex(R"(^\{\{\s*([a-zA-Z0-9_.]+)\s*\}\}$)");

static const std::vector<std::string> kDefaultBootValidationRequirements = {
  "dispatch_states_publisher",
  "task_summaries_publisher",
  "task_api_requests_subscriber",
  "expected_robots_seen",
  "expected_robots_online",
  "valid_robot_maps",
  "finite_robot_pose",
  "sane_battery_values",
};

std::string trim(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\n\r");
  return input.substr(begin, end - begin + 1);
}

std::string to_upper(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}

std::string to_lower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

bool contains(const std::vector<std::string> & values, const std::string & needle)
{
  return std::find(values.begin(), values.end(), needle) != values.end();
}

std::string join(const std::vector<std::string> & values, const std::string & separator)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << separator;
    }
    oss << values[i];
  }
  return oss.str();
}

std::string random_hex(std::size_t length)
{
  static thread_local std::mt19937 rng{std::random_device{}()};
  static const char * digits = "0123456789abcdef";
  std::uniform_int_distribution<int> dist(0, 15);

  std::string out;
  out.reserve(length);
  for (std::size_t i = 0; i < length; ++i) {
    out.push_back(digits[dist(rng)]);
  }
  return out;
}

std::string expand_environment(const std::string & input)
{
  std::string out;
  out.reserve(input.size());

  for (std::size_t i = 0; i < input.size(); ++i) {
    if (input[i] != '$') {
      out.push_back(input[i]);
      continue;
    }

    if (i + 1 >= input.size()) {
      out.push_back(input[i]);
      continue;
    }

    std::string var_name;
    if (input[i + 1] == '{') {
      const auto end = input.find('}', i + 2);
      if (end == std::string::npos) {
        out.push_back(input[i]);
        continue;
      }
      var_name = input.substr(i + 2, end - (i + 2));
      i = end;
    } else {
      std::size_t j = i + 1;
      while (j < input.size()) {
        const char c = input[j];
        const bool valid = (c == '_') || std::isalnum(static_cast<unsigned char>(c));
        if (!valid) {
          break;
        }
        ++j;
      }
      if (j == i + 1) {
        out.push_back(input[i]);
        continue;
      }
      var_name = input.substr(i + 1, j - (i + 1));
      i = j - 1;
    }

    const char * env_value = std::getenv(var_name.c_str());
    if (env_value != nullptr) {
      out += env_value;
    }
  }

  return out;
}

std::filesystem::path expand_path(const std::string & raw)
{
  std::string expanded = expand_environment(raw);
  if (!expanded.empty() && expanded[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      if (expanded.size() == 1) {
        expanded = home;
      } else if (expanded[1] == '/') {
        expanded = std::string(home) + expanded.substr(1);
      }
    }
  }
  return std::filesystem::path(expanded);
}

std::filesystem::path executable_path()
{
  std::array<char, 4096> buffer{};
  const auto length = readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
  if (length <= 0) {
    return {};
  }
  buffer[static_cast<std::size_t>(length)] = '\0';
  return std::filesystem::path(buffer.data());
}

double safe_to_double(const json & value, double default_value)
{
  try {
    double parsed = default_value;
    if (value.is_number()) {
      parsed = value.get<double>();
    } else if (value.is_string()) {
      parsed = std::stod(value.get<std::string>());
    } else {
      return default_value;
    }
    if (std::isfinite(parsed)) {
      return parsed;
    }
  } catch (...) {
  }
  return default_value;
}

bool json_truthy(const json & value, bool default_value)
{
  if (value.is_null()) {
    return default_value;
  }
  if (value.is_boolean()) {
    return value.get<bool>();
  }
  if (value.is_number()) {
    return value.get<double>() != 0.0;
  }
  if (value.is_string()) {
    return !value.get<std::string>().empty();
  }
  if (value.is_array()) {
    return !value.empty();
  }
  if (value.is_object()) {
    return !value.empty();
  }
  return default_value;
}

std::string json_string_value(const json & value)
{
  if (value.is_string()) {
    return value.get<std::string>();
  }
  if (value.is_null()) {
    return "";
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? "True" : "False";
  }
  if (value.is_number_integer()) {
    return std::to_string(value.get<long long>());
  }
  if (value.is_number_unsigned()) {
    return std::to_string(value.get<unsigned long long>());
  }
  if (value.is_number_float()) {
    std::ostringstream oss;
    oss << value.get<double>();
    return oss.str();
  }
  return value.dump();
}

std::string yaml_scalar_to_string(const YAML::Node & node)
{
  try {
    return node.as<std::string>();
  } catch (...) {
    return "";
  }
}

bool yaml_bool_or(const YAML::Node & node, bool default_value)
{
  if (!node) {
    return default_value;
  }
  try {
    return node.as<bool>();
  } catch (...) {
  }
  try {
    return node.as<int>() != 0;
  } catch (...) {
  }
  try {
    return !node.as<std::string>().empty();
  } catch (...) {
  }
  return default_value;
}

int yaml_int_or(const YAML::Node & node, int default_value)
{
  if (!node) {
    return default_value;
  }
  try {
    return node.as<int>();
  } catch (...) {
    return default_value;
  }
}

double yaml_double_or(const YAML::Node & node, double default_value)
{
  if (!node) {
    return default_value;
  }
  try {
    return node.as<double>();
  } catch (...) {
    return default_value;
  }
}

std::string yaml_string_or(const YAML::Node & node, const std::string & default_value)
{
  if (!node) {
    return default_value;
  }
  try {
    return node.as<std::string>();
  } catch (...) {
    return default_value;
  }
}

std::vector<std::string> yaml_string_list(const YAML::Node & node)
{
  std::vector<std::string> out;
  if (!node || !node.IsSequence()) {
    return out;
  }
  out.reserve(node.size());
  for (const auto & entry : node) {
    out.push_back(yaml_scalar_to_string(entry));
  }
  return out;
}

json yaml_to_json(const YAML::Node & node)
{
  if (!node || node.IsNull()) {
    return nullptr;
  }

  if (node.IsMap()) {
    json out = json::object();
    for (const auto & entry : node) {
      out[yaml_scalar_to_string(entry.first)] = yaml_to_json(entry.second);
    }
    return out;
  }

  if (node.IsSequence()) {
    json out = json::array();
    for (const auto & entry : node) {
      out.push_back(yaml_to_json(entry));
    }
    return out;
  }

  const std::string raw = yaml_scalar_to_string(node);
  const std::string lowered = to_lower(raw);
  if (lowered == "true") {
    return true;
  }
  if (lowered == "false") {
    return false;
  }

  try {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(raw, &consumed, 10);
    if (consumed == raw.size()) {
      return parsed;
    }
  } catch (...) {
  }

  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(raw, &consumed);
    if (consumed == raw.size() && std::isfinite(parsed)) {
      return parsed;
    }
  } catch (...) {
  }

  return raw;
}

std::vector<json> yaml_json_list(const YAML::Node & node)
{
  std::vector<json> out;
  if (!node || !node.IsSequence()) {
    return out;
  }
  out.reserve(node.size());
  for (const auto & entry : node) {
    out.push_back(yaml_to_json(entry));
  }
  return out;
}

enum class MissionPhase
{
  BOOT,
  SAFE_HOLD,
  READY,
  GAME_START,
  COMPLETE,
};

enum class TaskLifecycle
{
  PENDING,
  READY,
  SUBMITTED,
  RUNNING,
  CANCELING,
  DONE,
  FAILED,
  SKIPPED,
  CANCELED,
};

enum class AntennaState
{
  UNKNOWN,
  ACTIVATED,
  ATTEMPTED_FAILED,
  BLOCKED,
};

enum class DuckState
{
  UNKNOWN,
  DETECTED,
  CLAIMED,
  CARRIED,
  IN_DROP_ZONE,
  LOST,
};

enum class DroneState
{
  ON_GROUND,
  AIRBORNE,
  SCANNING,
  REPORTED,
};

std::string phase_to_string(MissionPhase phase)
{
  switch (phase) {
    case MissionPhase::BOOT:
      return "BOOT";
    case MissionPhase::SAFE_HOLD:
      return "SAFE_HOLD";
    case MissionPhase::READY:
      return "READY";
    case MissionPhase::GAME_START:
      return "GAME_START";
    case MissionPhase::COMPLETE:
      return "COMPLETE";
  }
  return "BOOT";
}

std::optional<MissionPhase> parse_phase(const std::string & raw)
{
  std::string text = to_upper(trim(raw));
  if (text.empty()) {
    return std::nullopt;
  }

  if (
    text == "ANTENNA" || text == "DUCK" || text == "CRATER" || text == "DRONE" ||
    text == "ENDGAME")
  {
    text = "GAME_START";
  }

  if (text == "BOOT") {
    return MissionPhase::BOOT;
  }
  if (text == "SAFE_HOLD") {
    return MissionPhase::SAFE_HOLD;
  }
  if (text == "READY") {
    return MissionPhase::READY;
  }
  if (text == "GAME_START") {
    return MissionPhase::GAME_START;
  }
  if (text == "COMPLETE") {
    return MissionPhase::COMPLETE;
  }

  return std::nullopt;
}

std::string lifecycle_to_string(TaskLifecycle state)
{
  switch (state) {
    case TaskLifecycle::PENDING:
      return "PENDING";
    case TaskLifecycle::READY:
      return "READY";
    case TaskLifecycle::SUBMITTED:
      return "SUBMITTED";
    case TaskLifecycle::RUNNING:
      return "RUNNING";
    case TaskLifecycle::CANCELING:
      return "CANCELING";
    case TaskLifecycle::DONE:
      return "DONE";
    case TaskLifecycle::FAILED:
      return "FAILED";
    case TaskLifecycle::SKIPPED:
      return "SKIPPED";
    case TaskLifecycle::CANCELED:
      return "CANCELED";
  }
  return "PENDING";
}

bool is_terminal(TaskLifecycle state)
{
  return state == TaskLifecycle::DONE || state == TaskLifecycle::FAILED ||
         state == TaskLifecycle::SKIPPED || state == TaskLifecycle::CANCELED;
}

AntennaState to_antenna_state(const std::string & raw)
{
  const std::string text = to_upper(raw);
  if (text == "ACTIVATED") {
    return AntennaState::ACTIVATED;
  }
  if (text == "ATTEMPTED_FAILED") {
    return AntennaState::ATTEMPTED_FAILED;
  }
  if (text == "BLOCKED") {
    return AntennaState::BLOCKED;
  }
  return AntennaState::UNKNOWN;
}

std::string antenna_state_to_string(AntennaState state)
{
  switch (state) {
    case AntennaState::UNKNOWN:
      return "UNKNOWN";
    case AntennaState::ACTIVATED:
      return "ACTIVATED";
    case AntennaState::ATTEMPTED_FAILED:
      return "ATTEMPTED_FAILED";
    case AntennaState::BLOCKED:
      return "BLOCKED";
  }
  return "UNKNOWN";
}

DuckState to_duck_state(const std::string & raw)
{
  const std::string text = to_upper(raw);
  if (text == "DETECTED") {
    return DuckState::DETECTED;
  }
  if (text == "CLAIMED") {
    return DuckState::CLAIMED;
  }
  if (text == "CARRIED") {
    return DuckState::CARRIED;
  }
  if (text == "IN_DROP_ZONE") {
    return DuckState::IN_DROP_ZONE;
  }
  if (text == "LOST") {
    return DuckState::LOST;
  }
  return DuckState::UNKNOWN;
}

std::string duck_state_to_string(DuckState state)
{
  switch (state) {
    case DuckState::UNKNOWN:
      return "UNKNOWN";
    case DuckState::DETECTED:
      return "DETECTED";
    case DuckState::CLAIMED:
      return "CLAIMED";
    case DuckState::CARRIED:
      return "CARRIED";
    case DuckState::IN_DROP_ZONE:
      return "IN_DROP_ZONE";
    case DuckState::LOST:
      return "LOST";
  }
  return "UNKNOWN";
}

DroneState to_drone_state(const std::string & raw)
{
  const std::string text = to_upper(raw);
  if (text == "AIRBORNE") {
    return DroneState::AIRBORNE;
  }
  if (text == "SCANNING") {
    return DroneState::SCANNING;
  }
  if (text == "REPORTED") {
    return DroneState::REPORTED;
  }
  return DroneState::ON_GROUND;
}

std::string drone_state_to_string(DroneState state)
{
  switch (state) {
    case DroneState::ON_GROUND:
      return "ON_GROUND";
    case DroneState::AIRBORNE:
      return "AIRBORNE";
    case DroneState::SCANNING:
      return "SCANNING";
    case DroneState::REPORTED:
      return "REPORTED";
  }
  return "ON_GROUND";
}

struct RetryPolicy
{
  int max_attempts = 2;
  double backoff_sec = 2.0;
};

struct TaskTimeouts
{
  double queued_sec = 30.0;
  double running_sec = 90.0;
};

struct TaskPriority
{
  double base = 1.0;
  double deadline_boost_per_sec = 0.0;
  double opportunistic_boost = 0.0;
};

struct DispatchTemplate
{
  std::string mode = "best_available";
  std::string fleet;
  std::string robot;
  json request = json::object();
};

struct TaskDefinition
{
  std::string task_id;
  std::string task_type;
  MissionPhase phase = MissionPhase::GAME_START;
  bool enabled = true;
  bool major = true;
  json target = json::object();
  std::vector<json> preconditions;
  std::vector<json> dismiss_conditions;
  std::vector<json> success_conditions;
  std::vector<std::string> lock_keys;
  TaskPriority priority;
  RetryPolicy retry;
  TaskTimeouts timeouts;
  DispatchTemplate dispatch;
  std::vector<std::string> allowed_robots;
  std::vector<std::string> preferred_robots;
  std::vector<std::string> required_capabilities;
  std::vector<std::string> excluded_robots;
  std::vector<std::string> depends_on;
  std::string dependency_policy = "require_done";
  std::string on_dependency_failure = "skip";
  json metadata = json::object();
};

struct TaskRuntime
{
  TaskDefinition definition;
  TaskLifecycle state = TaskLifecycle::PENDING;
  int attempts = 0;
  std::string last_error;
  double blocked_until_unix_s = 0.0;
  double submitted_at_unix_s = 0.0;
  double running_since_unix_s = 0.0;
  std::string request_id;
  std::string rmf_task_id;
  std::string dispatch_status;
  std::string assigned_robot;
  std::string assigned_fleet;
  std::vector<std::string> held_locks;
  std::string status_note;
  std::string cancel_reason;
  std::string cancel_outcome;
  double cancel_since_unix_s = 0.0;
  double cancel_requested_at_unix_s = 0.0;

  bool can_retry_now(double now_unix_s) const
  {
    if (attempts >= definition.retry.max_attempts) {
      return false;
    }
    return now_unix_s >= blocked_until_unix_s;
  }

  void mark_retry_backoff(double now_unix_s)
  {
    blocked_until_unix_s = now_unix_s + definition.retry.backoff_sec;
  }
};

struct RobotSnapshot
{
  std::string robot_name;
  std::string map_name;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double battery_soc = 1.0;
  bool battery_valid = false;
  bool requires_replan = false;
  std::uint64_t last_completed_request = 0;
  double last_update_unix_s = 0.0;
  bool online = false;
  std::vector<std::string> fault_flags;
  std::string carrying_duck_id;
};

struct DuckSnapshot
{
  std::string duck_id;
  DuckState state = DuckState::UNKNOWN;
  double x = 0.0;
  double y = 0.0;
  double confidence = 0.0;
  std::string claimed_by;
  std::string carried_by;
};

struct CraterSnapshot
{
  std::string token_owner_task_id;
  bool lap_in_progress = false;
  bool entry_clear = true;
  bool exit_clear = true;
  std::vector<std::string> paired_robots;
};

struct DispatchSnapshot
{
  std::string task_id;
  int status = 0;
  std::string status_name;
  std::string fleet_name;
  std::string robot_name;
  std::vector<std::string> errors;
  double updated_at_unix_s = 0.0;
};

struct SummarySnapshot
{
  std::string task_id;
  int state = 0;
  std::string state_name;
  std::string fleet_name;
  std::string robot_name;
  std::string status;
  double updated_at_unix_s = 0.0;
};

struct CapabilityHealth
{
  std::unordered_map<std::string, int> failures_by_type;
  std::unordered_set<std::string> degraded_types;
};

struct DirectorSettings
{
  std::string requester = "game_director";
  double tick_period_sec = 0.5;
  double boot_wait_sec = 10.0;
  double stale_robot_timeout_sec = 2.5;
  double min_battery_soc = 0.2;
  int major_inflight_limit_per_robot = 1;
  int micro_inflight_limit_per_robot = 1;
  int failure_degrade_threshold = 3;
  double match_duration_sec = 600.0;
  double queued_timeout_default_sec = 30.0;
  double running_timeout_default_sec = 120.0;
  double api_response_timeout_sec = 8.0;
  double cancel_retry_period_sec = 5.0;
  double cancel_timeout_sec = 45.0;
};

struct WorldSeed
{
  std::map<std::string, std::string> antennas;
  std::vector<std::string> ducks;
  std::vector<std::string> crater_pair;
  std::string drop_zone = "drop_zone";
  std::vector<std::string> expected_ground_robots;
  std::vector<std::string> expected_drone_robots;
};

struct TaskPoolSpec
{
  DirectorSettings settings;
  WorldSeed world;
  std::vector<TaskDefinition> tasks;
};

DirectorSettings parse_settings(const YAML::Node & raw)
{
  DirectorSettings settings;
  settings.requester = yaml_string_or(raw["requester"], "game_director");
  settings.tick_period_sec = yaml_double_or(raw["tick_period_sec"], 0.5);
  settings.boot_wait_sec = yaml_double_or(raw["boot_wait_sec"], 10.0);
  settings.stale_robot_timeout_sec = yaml_double_or(raw["stale_robot_timeout_sec"], 2.5);
  settings.min_battery_soc = yaml_double_or(raw["min_battery_soc"], 0.2);
  settings.major_inflight_limit_per_robot = yaml_int_or(raw["major_inflight_limit_per_robot"], 1);
  settings.micro_inflight_limit_per_robot = yaml_int_or(raw["micro_inflight_limit_per_robot"], 1);
  settings.failure_degrade_threshold = yaml_int_or(raw["failure_degrade_threshold"], 3);
  settings.match_duration_sec = yaml_double_or(raw["match_duration_sec"], 600.0);
  settings.queued_timeout_default_sec = yaml_double_or(raw["queued_timeout_default_sec"], 30.0);
  settings.running_timeout_default_sec = yaml_double_or(raw["running_timeout_default_sec"], 120.0);
  settings.api_response_timeout_sec = yaml_double_or(raw["api_response_timeout_sec"], 8.0);
  settings.cancel_retry_period_sec = yaml_double_or(raw["cancel_retry_period_sec"], 5.0);
  settings.cancel_timeout_sec = yaml_double_or(raw["cancel_timeout_sec"], 45.0);
  return settings;
}

WorldSeed parse_world(const YAML::Node & raw)
{
  WorldSeed world;

  const YAML::Node antennas_node = raw["antennas"];
  if (antennas_node && antennas_node.IsSequence()) {
    for (const auto & value : antennas_node) {
      world.antennas[yaml_scalar_to_string(value)] = "UNKNOWN";
    }
  } else if (antennas_node && antennas_node.IsMap()) {
    for (const auto & entry : antennas_node) {
      world.antennas[yaml_scalar_to_string(entry.first)] = yaml_scalar_to_string(entry.second);
    }
  }

  world.ducks = yaml_string_list(raw["ducks"]);
  world.crater_pair = yaml_string_list(raw["crater_pair"]);
  world.drop_zone = yaml_string_or(raw["drop_zone"], "drop_zone");
  world.expected_ground_robots = yaml_string_list(raw["expected_ground_robots"]);
  world.expected_drone_robots = yaml_string_list(raw["expected_drone_robots"]);

  return world;
}

TaskDefinition parse_task(const YAML::Node & raw, const DirectorSettings & settings)
{
  if (!raw["id"] || !raw["type"]) {
    throw std::runtime_error("task must contain id and type");
  }

  TaskDefinition task;
  task.task_id = yaml_scalar_to_string(raw["id"]);
  task.task_type = yaml_scalar_to_string(raw["type"]);

  std::string raw_phase = to_upper(yaml_string_or(raw["phase"], "GAME_START"));
  if (
    raw_phase == "ANTENNA" || raw_phase == "DUCK" || raw_phase == "CRATER" ||
    raw_phase == "DRONE" || raw_phase == "ENDGAME")
  {
    raw_phase = "GAME_START";
  }

  const auto phase = parse_phase(raw_phase);
  if (!phase.has_value()) {
    throw std::runtime_error("task [" + task.task_id + "] has invalid phase [" + raw_phase + "]");
  }
  task.phase = *phase;

  task.enabled = yaml_bool_or(raw["enabled"], true);
  task.major = yaml_bool_or(raw["major"], true);
  task.target = raw["target"] ? yaml_to_json(raw["target"]) : json::object();
  if (!task.target.is_object()) {
    task.target = json::object();
  }

  task.preconditions = yaml_json_list(raw["preconditions"]);
  task.dismiss_conditions = yaml_json_list(raw["dismiss_conditions"]);
  task.success_conditions = yaml_json_list(raw["success_conditions"]);
  task.lock_keys = yaml_string_list(raw["lock_keys"]);

  const YAML::Node priority_raw = raw["priority"];
  task.priority.base = yaml_double_or(priority_raw["base"], 1.0);
  task.priority.deadline_boost_per_sec = yaml_double_or(priority_raw["deadline_boost_per_sec"], 0.0);
  task.priority.opportunistic_boost = yaml_double_or(priority_raw["opportunistic_boost"], 0.0);

  const YAML::Node retry_raw = raw["retry"];
  task.retry.max_attempts = yaml_int_or(retry_raw["max_attempts"], 2);
  task.retry.backoff_sec = yaml_double_or(retry_raw["backoff_sec"], 2.0);

  const YAML::Node timeout_raw = raw["timeouts"];
  task.timeouts.queued_sec =
    yaml_double_or(timeout_raw["queued_sec"], settings.queued_timeout_default_sec);
  task.timeouts.running_sec =
    yaml_double_or(timeout_raw["running_sec"], settings.running_timeout_default_sec);

  const YAML::Node dispatch_raw = raw["dispatch"];
  task.dispatch.mode = trim(yaml_string_or(dispatch_raw["mode"], "best_available"));
  if (task.dispatch.mode != "best_available" && task.dispatch.mode != "robot_targeted") {
    throw std::runtime_error(
            "task [" + task.task_id + "] has invalid dispatch mode [" + task.dispatch.mode + "]");
  }
  task.dispatch.fleet = yaml_string_or(dispatch_raw["fleet"], "");
  task.dispatch.robot = yaml_string_or(dispatch_raw["robot"], "");
  task.dispatch.request = dispatch_raw["request"] ? yaml_to_json(dispatch_raw["request"]) : json::object();

  task.allowed_robots = yaml_string_list(raw["allowed_robots"]);
  task.preferred_robots = yaml_string_list(raw["preferred_robots"]);
  task.required_capabilities = yaml_string_list(raw["required_capabilities"]);
  task.excluded_robots = yaml_string_list(raw["excluded_robots"]);
  task.depends_on = yaml_string_list(raw["depends_on"]);

  task.dependency_policy = trim(yaml_string_or(raw["dependency_policy"], "require_done"));
  if (task.dependency_policy != "require_done" && task.dependency_policy != "require_terminal") {
    throw std::runtime_error(
            "task [" + task.task_id + "] has invalid dependency_policy [" +
            task.dependency_policy + "]");
  }

  task.on_dependency_failure = trim(yaml_string_or(raw["on_dependency_failure"], "skip"));
  if (
    task.on_dependency_failure != "skip" && task.on_dependency_failure != "fail" &&
    task.on_dependency_failure != "run_anyway")
  {
    throw std::runtime_error(
            "task [" + task.task_id + "] has invalid on_dependency_failure [" +
            task.on_dependency_failure + "]");
  }

  task.metadata = raw["metadata"] ? yaml_to_json(raw["metadata"]) : json::object();

  return task;
}

TaskPoolSpec load_task_pool(const std::filesystem::path & path)
{
  YAML::Node raw = YAML::LoadFile(path.string());
  if (!raw || !raw.IsMap()) {
    raw = YAML::Node(YAML::NodeType::Map);
  }

  TaskPoolSpec spec;
  spec.settings = parse_settings(raw["settings"]);
  spec.world = parse_world(raw["world"]);

  const YAML::Node raw_tasks = raw["tasks"];
  if (raw_tasks && !raw_tasks.IsSequence()) {
    throw std::runtime_error("task_pool field 'tasks' must be a list");
  }

  std::unordered_set<std::string> seen_ids;
  if (raw_tasks) {
    for (const auto & raw_task : raw_tasks) {
      if (!raw_task.IsMap()) {
        throw std::runtime_error("each entry in 'tasks' must be a mapping");
      }
      TaskDefinition task = parse_task(raw_task, spec.settings);
      if (seen_ids.find(task.task_id) != seen_ids.end()) {
        throw std::runtime_error("duplicate task id in task pool: [" + task.task_id + "]");
      }
      seen_ids.insert(task.task_id);
      spec.tasks.push_back(task);
    }
  }

  return spec;
}

std::pair<std::size_t, std::map<std::string, int>> summarize_task_pool(
  const std::vector<TaskDefinition> & tasks)
{
  std::map<std::string, int> by_type;
  for (const auto & task : tasks) {
    by_type[task.task_type] += 1;
  }
  return {tasks.size(), by_type};
}

class WorldModel
{
public:
  WorldModel()
  : WorldModel(WorldSeed{})
  {
  }

  explicit WorldModel(const WorldSeed & seed)
  {
    for (const auto & [antenna_id, state] : seed.antennas) {
      antennas[antenna_id] = to_antenna_state(state);
    }
    for (const auto & duck_id : seed.ducks) {
      ducks[duck_id] = DuckSnapshot{duck_id};
    }

    expected_ground_robots = seed.expected_ground_robots;
    expected_drone_robots = seed.expected_drone_robots;
    drop_zone = seed.drop_zone;
  }

  void set_match_start(double now_unix_s)
  {
    match_start_unix_s = now_unix_s;
  }

  double elapsed_match_time(double now_unix_s) const
  {
    if (match_start_unix_s <= 0.0) {
      return 0.0;
    }
    return std::max(0.0, now_unix_s - match_start_unix_s);
  }

  void update_robot_state(const ieee_fleet_msgs::msg::RobotState & msg, double now_unix_s)
  {
    const double battery_soc = static_cast<double>(msg.battery_soc);
    const bool battery_valid = std::isfinite(battery_soc) && battery_soc >= 0.0 && battery_soc <= 1.0;

    auto it = robots.find(msg.robot_name);
    if (it == robots.end()) {
      it = robots.emplace(msg.robot_name, RobotSnapshot{msg.robot_name}).first;
    }
    RobotSnapshot & robot = it->second;

    robot.map_name = msg.map_name;
    robot.x = msg.x;
    robot.y = msg.y;
    robot.yaw = msg.yaw;
    robot.battery_soc = battery_soc;
    robot.battery_valid = battery_valid;
    robot.requires_replan = msg.requires_replan;
    robot.last_completed_request = msg.last_completed_request;
    robot.last_update_unix_s = now_unix_s;
    robot.online = true;
  }

  void refresh_heartbeats(double now_unix_s, double stale_threshold_sec)
  {
    for (auto & [_, robot] : robots) {
      const double age = now_unix_s - robot.last_update_unix_s;
      robot.online = robot.last_update_unix_s > 0.0 && age <= stale_threshold_sec;
    }
  }

  bool all_expected_robots_seen() const
  {
    std::unordered_set<std::string> expected;
    expected.insert(expected_ground_robots.begin(), expected_ground_robots.end());
    expected.insert(expected_drone_robots.begin(), expected_drone_robots.end());

    if (expected.empty()) {
      return !robots.empty();
    }

    for (const auto & name : expected) {
      if (robots.find(name) == robots.end()) {
        return false;
      }
    }
    return true;
  }

  bool valid_robot_maps() const
  {
    for (const auto & [_, robot] : robots) {
      if (robot.map_name.empty()) {
        return false;
      }
    }
    return true;
  }

  bool all_expected_robots_online() const
  {
    std::vector<std::string> expected = expected_ground_robots;
    expected.insert(expected.end(), expected_drone_robots.begin(), expected_drone_robots.end());

    if (expected.empty()) {
      for (const auto & [_, robot] : robots) {
        if (robot.online) {
          return true;
        }
      }
      return false;
    }

    for (const auto & name : expected) {
      const auto it = robots.find(name);
      if (it == robots.end() || !it->second.online) {
        return false;
      }
    }
    return true;
  }

  bool sane_battery_values() const
  {
    for (const auto & [_, robot] : robots) {
      if (!std::isfinite(robot.battery_soc)) {
        return false;
      }
      if (robot.battery_soc < 0.0 || robot.battery_soc > 1.0) {
        return false;
      }
    }
    return true;
  }

  std::vector<std::string> antenna_remaining() const
  {
    std::vector<std::string> remaining;
    for (const auto & [antenna_id, state] : antennas) {
      if (state != AntennaState::ACTIVATED && state != AntennaState::ATTEMPTED_FAILED) {
        remaining.push_back(antenna_id);
      }
    }
    return remaining;
  }

  std::vector<std::string> unclaimed_ducks() const
  {
    std::vector<std::string> result;
    for (const auto & [duck_id, duck] : ducks) {
      if (
        (duck.state == DuckState::DETECTED || duck.state == DuckState::UNKNOWN ||
        duck.state == DuckState::LOST) &&
        duck.claimed_by.empty())
      {
        result.push_back(duck_id);
      }
    }
    return result;
  }

  double robot_health_score(
    const std::string & robot_name,
    double min_soc,
    const std::unordered_map<std::string, int> * failure_counts = nullptr) const
  {
    const auto it = robots.find(robot_name);
    if (it == robots.end() || !it->second.online) {
      return 0.0;
    }
    const RobotSnapshot & robot = it->second;

    double score = 1.0;
    if (robot.battery_valid) {
      if (robot.battery_soc < min_soc) {
        return 0.0;
      }
      score *= 0.7 + 0.3 * std::clamp(robot.battery_soc, 0.0, 1.0);
    }
    if (robot.requires_replan) {
      score *= 0.7;
    }
    if (!robot.fault_flags.empty()) {
      score *= 0.5;
    }
    if (failure_counts != nullptr) {
      int total_failures = 0;
      for (const auto & [_, count] : *failure_counts) {
        total_failures += count;
      }
      score *= 1.0 / (1.0 + 0.2 * static_cast<double>(total_failures));
    }

    return std::max(0.0, score);
  }

  bool all_antennas_terminal() const
  {
    return antenna_remaining().empty();
  }

  bool all_ducks_in_drop_zone() const
  {
    for (const auto & [_, duck] : ducks) {
      if (duck.state != DuckState::IN_DROP_ZONE) {
        return false;
      }
    }
    return true;
  }

  bool crater_idle() const
  {
    return !crater.lap_in_progress;
  }

  bool all_drones_terminal() const
  {
    if (drone_states.empty()) {
      return false;
    }
    for (const auto & [_, state] : drone_states) {
      if (state != DroneState::ON_GROUND && state != DroneState::REPORTED) {
        return false;
      }
    }
    return true;
  }

  bool any_antenna_done() const
  {
    for (const auto & [_, state] : antennas) {
      if (state == AntennaState::ACTIVATED || state == AntennaState::ATTEMPTED_FAILED) {
        return true;
      }
    }
    return false;
  }

  void mark_antenna_state(const std::string & antenna_id, AntennaState state)
  {
    antennas[antenna_id] = state;
  }

  DuckSnapshot & ensure_duck(const std::string & duck_id)
  {
    auto it = ducks.find(duck_id);
    if (it == ducks.end()) {
      it = ducks.emplace(duck_id, DuckSnapshot{duck_id}).first;
    }
    return it->second;
  }

  void on_task_success(const std::string & task_type, const json & target, const std::string & assigned_robot)
  {
    if (task_type == "ACTIVATE_ANTENNA" || task_type == "CLEAR_BLOCKING_DUCK") {
      const std::string antenna_id =
        (target.is_object() && target.contains("antenna_id")) ?
        json_string_value(target["antenna_id"]) : "";
      if (!antenna_id.empty()) {
        mark_antenna_state(antenna_id, AntennaState::ACTIVATED);
      }
    }

    if (task_type == "COLLECT_DUCK" || task_type == "CLEAR_BLOCKING_DUCK") {
      const std::string duck_id =
        (target.is_object() && target.contains("duck_id")) ?
        json_string_value(target["duck_id"]) : "";
      if (!duck_id.empty()) {
        DuckSnapshot & duck = ensure_duck(duck_id);
        duck.state = DuckState::IN_DROP_ZONE;
        duck.claimed_by.clear();
        duck.carried_by.clear();
        clear_duck_from_robots(duck_id, "");
      }
    }

    if (task_type == "CRATER_LAP_PAIR") {
      crater.lap_in_progress = false;
      crater.token_owner_task_id.clear();
    }

    if (task_type == "DRONE_SCAN_LEDS") {
      for (auto & [_, state] : drone_states) {
        state = DroneState::SCANNING;
      }
    }

    if (task_type == "DRONE_SEND_IR") {
      for (auto & [_, state] : drone_states) {
        state = DroneState::REPORTED;
      }
    }

    if (task_type == "DRONE_LAND") {
      for (auto & [_, state] : drone_states) {
        state = DroneState::ON_GROUND;
      }
    }

    if (!assigned_robot.empty()) {
      auto robot_it = robots.find(assigned_robot);
      if (robot_it != robots.end() && task_type == "COLLECT_DUCK") {
        robot_it->second.carrying_duck_id.clear();
      }
    }
  }

  void on_task_failure(
    const std::string & task_type,
    const json & target,
    bool attempts_exhausted,
    const std::string & reason)
  {
    (void)reason;

    if (task_type == "ACTIVATE_ANTENNA" && attempts_exhausted) {
      const std::string antenna_id =
        (target.is_object() && target.contains("antenna_id")) ?
        json_string_value(target["antenna_id"]) : "";
      if (!antenna_id.empty()) {
        mark_antenna_state(antenna_id, AntennaState::ATTEMPTED_FAILED);
      }
    }

    if (task_type == "COLLECT_DUCK") {
      const std::string duck_id =
        (target.is_object() && target.contains("duck_id")) ?
        json_string_value(target["duck_id"]) : "";
      if (!duck_id.empty()) {
        DuckSnapshot & duck = ensure_duck(duck_id);
        if (duck.state != DuckState::IN_DROP_ZONE) {
          duck.state = DuckState::LOST;
          duck.claimed_by.clear();
          duck.carried_by.clear();
          clear_duck_from_robots(duck_id, "");
        }
      }
    }

    if (task_type == "CRATER_LAP_PAIR") {
      crater.lap_in_progress = false;
      crater.token_owner_task_id.clear();
    }
  }

  std::string apply_external_event(const std::string & data)
  {
    json payload;
    try {
      payload = json::parse(data);
    } catch (...) {
      return "invalid_json";
    }

    const std::string event =
      payload.contains("event") ? json_string_value(payload["event"]) : "";

    if (event == "antenna_state") {
      const std::string antenna_id =
        payload.contains("antenna_id") ? json_string_value(payload["antenna_id"]) : "";
      if (!antenna_id.empty()) {
        const std::string state =
          payload.contains("state") ? json_string_value(payload["state"]) : "UNKNOWN";
        mark_antenna_state(antenna_id, to_antenna_state(state));
      }
      return event;
    }

    if (event == "duck_detected") {
      const std::string duck_id =
        payload.contains("duck_id") ? json_string_value(payload["duck_id"]) : "";
      if (!duck_id.empty()) {
        DuckSnapshot & duck = ensure_duck(duck_id);
        duck.state = DuckState::DETECTED;
        duck.x = payload.contains("x") ? safe_to_double(payload["x"], duck.x) : duck.x;
        duck.y = payload.contains("y") ? safe_to_double(payload["y"], duck.y) : duck.y;
        duck.confidence = payload.contains("confidence") ?
          safe_to_double(payload["confidence"], duck.confidence) :
          duck.confidence;
      }
      return event;
    }

    if (event == "duck_claimed") {
      const std::string duck_id =
        payload.contains("duck_id") ? json_string_value(payload["duck_id"]) : "";
      const std::string robot_name =
        payload.contains("robot_name") ? json_string_value(payload["robot_name"]) : "";
      if (!duck_id.empty()) {
        DuckSnapshot & duck = ensure_duck(duck_id);
        duck.state = DuckState::CLAIMED;
        duck.claimed_by = robot_name;
        duck.carried_by.clear();
      }
      return event;
    }

    if (event == "duck_carried") {
      const std::string duck_id =
        payload.contains("duck_id") ? json_string_value(payload["duck_id"]) : "";
      const std::string robot_name =
        payload.contains("robot_name") ? json_string_value(payload["robot_name"]) : "";
      if (!duck_id.empty()) {
        DuckSnapshot & duck = ensure_duck(duck_id);
        duck.state = DuckState::CARRIED;
        duck.claimed_by = robot_name;
        duck.carried_by = robot_name;
        clear_duck_from_robots(duck_id, robot_name);
        auto robot_it = robots.find(robot_name);
        if (robot_it != robots.end()) {
          robot_it->second.carrying_duck_id = duck_id;
        }
      }
      return event;
    }

    if (event == "duck_in_drop_zone") {
      const std::string duck_id =
        payload.contains("duck_id") ? json_string_value(payload["duck_id"]) : "";
      if (!duck_id.empty()) {
        DuckSnapshot & duck = ensure_duck(duck_id);
        duck.state = DuckState::IN_DROP_ZONE;
        duck.claimed_by.clear();
        duck.carried_by.clear();
        clear_duck_from_robots(duck_id, "");
      }
      return event;
    }

    if (event == "duck_lost") {
      const std::string duck_id =
        payload.contains("duck_id") ? json_string_value(payload["duck_id"]) : "";
      if (!duck_id.empty()) {
        DuckSnapshot & duck = ensure_duck(duck_id);
        duck.state = DuckState::LOST;
        duck.claimed_by.clear();
        duck.carried_by.clear();
        clear_duck_from_robots(duck_id, "");
      }
      return event;
    }

    if (event == "crater_state") {
      crater.entry_clear = payload.contains("entry_clear") ?
        json_truthy(payload["entry_clear"], crater.entry_clear) : crater.entry_clear;
      crater.exit_clear = payload.contains("exit_clear") ?
        json_truthy(payload["exit_clear"], crater.exit_clear) : crater.exit_clear;
      crater.lap_in_progress = payload.contains("lap_in_progress") ?
        json_truthy(payload["lap_in_progress"], crater.lap_in_progress) : crater.lap_in_progress;
      if (payload.contains("paired_robots") && payload["paired_robots"].is_array()) {
        crater.paired_robots.clear();
        for (const auto & value : payload["paired_robots"]) {
          crater.paired_robots.push_back(json_string_value(value));
        }
      }
      return event;
    }

    if (event == "drone_state") {
      const std::string drone_name = payload.contains("drone_name") ?
        json_string_value(payload["drone_name"]) :
        "drone1";
      const std::string state = payload.contains("state") ?
        json_string_value(payload["state"]) :
        "ON_GROUND";
      drone_states[drone_name] = to_drone_state(state);
      return event;
    }

    if (event == "robot_fault") {
      const std::string robot_name = payload.contains("robot_name") ?
        json_string_value(payload["robot_name"]) :
        "";
      const std::string fault = trim(
        payload.contains("fault") ? json_string_value(payload["fault"]) : "");
      const bool set_fault = payload.contains("set") ? json_truthy(payload["set"], true) : true;

      auto robot_it = robots.find(robot_name);
      if (robot_it != robots.end() && !fault.empty()) {
        auto & faults = robot_it->second.fault_flags;
        const auto existing = std::find(faults.begin(), faults.end(), fault);
        if (set_fault && existing == faults.end()) {
          faults.push_back(fault);
        }
        if (!set_fault && existing != faults.end()) {
          faults.erase(existing);
        }
      }
      return event;
    }

    if (event == "robot_capability") {
      const std::string robot_name = payload.contains("robot_name") ?
        json_string_value(payload["robot_name"]) :
        "";
      const std::string capability = trim(
        payload.contains("capability") ? json_string_value(payload["capability"]) : "");
      const bool set_capability = payload.contains("set") ? json_truthy(payload["set"], true) : true;

      if (!robot_name.empty() && !capability.empty()) {
        auto & caps = robot_capabilities[robot_name];
        if (set_capability) {
          caps.insert(capability);
        } else {
          caps.erase(capability);
        }
      }
      return event;
    }

    return "ignored";
  }

  double match_start_unix_s = 0.0;
  MissionPhase phase = MissionPhase::BOOT;

  std::unordered_map<std::string, RobotSnapshot> robots;
  std::unordered_map<std::string, AntennaState> antennas;
  std::unordered_map<std::string, DuckSnapshot> ducks;
  CraterSnapshot crater;
  std::unordered_map<std::string, DroneState> drone_states;
  std::unordered_map<std::string, std::unordered_set<std::string>> robot_capabilities;

  std::vector<std::string> expected_ground_robots;
  std::vector<std::string> expected_drone_robots;
  std::string drop_zone;

private:
  void clear_duck_from_robots(const std::string & duck_id, const std::string & keep_robot)
  {
    for (auto & [robot_name, robot] : robots) {
      if (!keep_robot.empty() && robot_name == keep_robot) {
        continue;
      }
      if (robot.carrying_duck_id == duck_id) {
        robot.carrying_duck_id.clear();
      }
    }
  }
};

std::string dispatch_state_to_str(int value)
{
  using rmf_task_msgs::msg::DispatchState;
  switch (value) {
    case DispatchState::STATUS_UNINITIALIZED:
      return "uninitialized";
    case DispatchState::STATUS_QUEUED:
      return "queued";
    case DispatchState::STATUS_SELECTED:
      return "selected";
    case DispatchState::STATUS_DISPATCHED:
      return "dispatched";
    case DispatchState::STATUS_FAILED_TO_ASSIGN:
      return "failed_to_assign";
    case DispatchState::STATUS_CANCELED_IN_FLIGHT:
      return "canceled_in_flight";
    default:
      return "unknown_" + std::to_string(value);
  }
}

std::string summary_state_to_str(int value)
{
  using rmf_task_msgs::msg::TaskSummary;
  switch (value) {
    case TaskSummary::STATE_QUEUED:
      return "queued";
    case TaskSummary::STATE_ACTIVE:
      return "active";
    case TaskSummary::STATE_COMPLETED:
      return "completed";
    case TaskSummary::STATE_FAILED:
      return "failed";
    case TaskSummary::STATE_CANCELED:
      return "canceled";
    case TaskSummary::STATE_PENDING:
      return "pending";
    default:
      return "unknown_" + std::to_string(value);
  }
}

class DispatchTracker
{
public:
  void on_dispatch_states(const rmf_task_msgs::msg::DispatchStates & msg, double now_unix_s)
  {
    for (const auto & state : msg.active) {
      consume_dispatch_state(state, now_unix_s);
    }
    for (const auto & state : msg.finished) {
      consume_dispatch_state(state, now_unix_s);
    }
  }

  void on_task_summary(const rmf_task_msgs::msg::TaskSummary & msg, double now_unix_s)
  {
    SummarySnapshot snapshot;
    snapshot.task_id = msg.task_id;
    snapshot.state = static_cast<int>(msg.state);
    snapshot.state_name = summary_state_to_str(snapshot.state);
    snapshot.fleet_name = msg.fleet_name;
    snapshot.robot_name = msg.robot_name;
    snapshot.status = msg.status;
    snapshot.updated_at_unix_s = now_unix_s;

    summary_by_task_id[msg.task_id] = snapshot;
  }

  std::unordered_map<std::string, DispatchSnapshot> dispatch_by_task_id;
  std::unordered_map<std::string, SummarySnapshot> summary_by_task_id;

private:
  void consume_dispatch_state(const rmf_task_msgs::msg::DispatchState & state, double now_unix_s)
  {
    DispatchSnapshot snapshot;
    snapshot.task_id = state.task_id;
    snapshot.status = static_cast<int>(state.status);
    snapshot.status_name = dispatch_state_to_str(snapshot.status);
    if (state.assignment.is_assigned) {
      snapshot.fleet_name = state.assignment.fleet_name;
      snapshot.robot_name = state.assignment.expected_robot_name;
    }
    for (const auto & error : state.errors) {
      snapshot.errors.push_back(error);
    }
    snapshot.updated_at_unix_s = now_unix_s;

    dispatch_by_task_id[state.task_id] = snapshot;
  }
};

struct PendingRequest
{
  std::string request_id;
  std::string game_task_id;
  std::string request_kind;
  double sent_at_unix_s = 0.0;
};

class DispatchClient
{
public:
  explicit DispatchClient(rclcpp::Node * node, double response_timeout_sec = 8.0)
  : node_(node), response_timeout_sec_(response_timeout_sec)
  {
    auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    transient_qos.reliable();
    transient_qos.transient_local();

    pub_ = node_->create_publisher<rmf_task_msgs::msg::ApiRequest>("task_api_requests", transient_qos);
    sub_ = node_->create_subscription<rmf_task_msgs::msg::ApiResponse>(
      "task_api_responses",
      transient_qos,
      std::bind(&DispatchClient::on_response, this, std::placeholders::_1));
    cancel_service_ = node_->create_client<rmf_task_msgs::srv::CancelTask>("cancel_task");
  }

  std::string send_dispatch_request(
    const std::string & game_task_id,
    const json & request,
    const std::string & mode,
    const std::string & fleet,
    const std::string & robot,
    double now_unix_s)
  {
    const std::string request_id = game_task_id + "." + random_hex(10);

    json payload;
    if (mode == "robot_targeted") {
      if (fleet.empty() || robot.empty()) {
        throw std::runtime_error("robot_targeted dispatch requires fleet and robot");
      }
      payload = {
        {"type", "robot_task_request"},
        {"fleet", fleet},
        {"robot", robot},
        {"request", request},
      };
    } else {
      payload = {
        {"type", "dispatch_task_request"},
        {"request", request},
      };
    }

    rmf_task_msgs::msg::ApiRequest msg;
    msg.request_id = request_id;
    msg.json_msg = payload.dump();
    pub_->publish(msg);

    pending_[request_id] = PendingRequest{request_id, game_task_id, payload["type"].get<std::string>(), now_unix_s};
    return request_id;
  }

  std::string send_cancel_api_request(
    const std::string & rmf_task_id,
    const std::vector<std::string> & labels = {})
  {
    const std::string request_id = "cancel." + rmf_task_id + "." + random_hex(8);
    json payload = {
      {"type", "cancel_task_request"},
      {"task_id", rmf_task_id},
    };
    if (!labels.empty()) {
      payload["labels"] = labels;
    }

    rmf_task_msgs::msg::ApiRequest msg;
    msg.request_id = request_id;
    msg.json_msg = payload.dump();
    pub_->publish(msg);

    pending_[request_id] = PendingRequest{
      request_id,
      rmf_task_id,
      "cancel_task_request",
      static_cast<double>(node_->get_clock()->now().nanoseconds()) / 1e9};
    return request_id;
  }

  void send_cancel_service_request(const std::string & rmf_task_id)
  {
    using namespace std::chrono_literals;

    if (!cancel_service_->wait_for_service(100ms)) {
      RCLCPP_WARN(node_->get_logger(), "cancel_task service unavailable");
      return;
    }

    auto request = std::make_shared<rmf_task_msgs::srv::CancelTask::Request>();
    request->requester = node_->get_name();
    request->task_id = rmf_task_id;
    (void)cancel_service_->async_send_request(request);
  }

  std::optional<json> consume_response(const std::string & request_id)
  {
    const auto it = responses_.find(request_id);
    if (it == responses_.end()) {
      return std::nullopt;
    }
    json payload = it->second;
    responses_.erase(it);
    return payload;
  }

  std::vector<PendingRequest> consume_pending_timeouts(double now_unix_s)
  {
    std::vector<PendingRequest> timed_out;
    std::vector<std::string> remove_ids;

    for (const auto & [request_id, pending] : pending_) {
      if (now_unix_s - pending.sent_at_unix_s > response_timeout_sec_) {
        timed_out.push_back(pending);
        remove_ids.push_back(request_id);
      }
    }

    for (const auto & request_id : remove_ids) {
      pending_.erase(request_id);
      responses_.erase(request_id);
    }

    return timed_out;
  }

  bool has_pending(const std::string & request_id) const
  {
    return pending_.find(request_id) != pending_.end();
  }

private:
  void on_response(const rmf_task_msgs::msg::ApiResponse::SharedPtr msg)
  {
    const auto pending_it = pending_.find(msg->request_id);
    if (pending_it == pending_.end()) {
      return;
    }

    PendingRequest pending = pending_it->second;
    pending_.erase(pending_it);

    json payload;
    try {
      payload = msg->json_msg.empty() ? json::object() : json::parse(msg->json_msg);
    } catch (const std::exception & exc) {
      payload = {
        {"success", false},
        {
          "errors",
          json::array({{{"category", "json_parse"}, {"detail", exc.what()}}}),
        },
      };
    }

    if (pending.request_kind == "cancel_task_request") {
      const bool success = payload.value("success", false);
      if (!success) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Cancel request [%s] rejected: %s",
          msg->request_id.c_str(),
          payload.contains("errors") ? payload["errors"].dump().c_str() : "unknown");
      }
      return;
    }

    payload["__response_type"] = static_cast<int>(msg->type);
    payload["__request_kind"] = pending.request_kind;
    payload["__game_task_id"] = pending.game_task_id;
    responses_[msg->request_id] = payload;
  }

  rclcpp::Node * node_;
  double response_timeout_sec_;

  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr pub_;
  rclcpp::Subscription<rmf_task_msgs::msg::ApiResponse>::SharedPtr sub_;
  rclcpp::Client<rmf_task_msgs::srv::CancelTask>::SharedPtr cancel_service_;

  std::unordered_map<std::string, PendingRequest> pending_;
  std::unordered_map<std::string, json> responses_;
};

class GameDirector : public rclcpp::Node
{
public:
  GameDirector()
  : rclcpp::Node("game_director"), world_(WorldSeed{})
  {
    this->declare_parameter<std::string>("task_pool_path", "");
    this->declare_parameter<std::vector<std::string>>(
      "boot_validation_requirements", kDefaultBootValidationRequirements);
    this->declare_parameter<std::string>("robot_state_topic", "/ieee_fleet/robot_state");
    this->declare_parameter<std::string>("external_event_topic", "/game_director/events");
    this->declare_parameter<std::string>("status_topic", "/game_director/status");
    this->declare_parameter<std::string>("final_report_topic", "/game_director/final_report");
    this->declare_parameter<std::string>("start_service_name", "/game_director/start_game");
    this->declare_parameter<double>("match_duration_sec", 0.0);
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", false);
    }

    spec_ = load_task_pool(resolve_task_pool_path());
    settings_ = spec_.settings;

    const double match_duration_override = this->get_parameter("match_duration_sec").as_double();
    if (match_duration_override > 0.0) {
      settings_.match_duration_sec = match_duration_override;
    }

    world_ = WorldModel(spec_.world);
    phase_ = MissionPhase::BOOT;
    world_.phase = phase_;
    boot_validation_requirements_ = load_boot_validation_requirements();

    tasks_.clear();
    for (const auto & task : spec_.tasks) {
      if (!task.enabled) {
        continue;
      }
      tasks_.emplace(task.task_id, TaskRuntime{task});
    }

    dispatch_client_ = std::make_unique<DispatchClient>(this, settings_.api_response_timeout_sec);

    boot_started_unix_s_ = now_unix_s();

    const std::string robot_state_topic = this->get_parameter("robot_state_topic").as_string();
    const std::string event_topic = this->get_parameter("external_event_topic").as_string();
    const std::string start_service_name = this->get_parameter("start_service_name").as_string();

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("status_topic").as_string(), 10);

    auto final_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    final_qos.reliable();
    final_qos.transient_local();
    final_report_pub_ = this->create_publisher<std_msgs::msg::String>(
      this->get_parameter("final_report_topic").as_string(),
      final_qos);

    robot_state_sub_ = this->create_subscription<ieee_fleet_msgs::msg::RobotState>(
      robot_state_topic,
      100,
      std::bind(&GameDirector::on_robot_state, this, std::placeholders::_1));
    dispatch_states_sub_ = this->create_subscription<rmf_task_msgs::msg::DispatchStates>(
      "dispatch_states",
      services_qos(),
      std::bind(&GameDirector::on_dispatch_states, this, std::placeholders::_1));
    task_summary_sub_ = this->create_subscription<rmf_task_msgs::msg::TaskSummary>(
      "task_summaries",
      100,
      std::bind(&GameDirector::on_task_summary, this, std::placeholders::_1));
    event_sub_ = this->create_subscription<std_msgs::msg::String>(
      event_topic,
      20,
      std::bind(&GameDirector::on_external_event, this, std::placeholders::_1));

    start_service_ = this->create_service<std_srvs::srv::Trigger>(
      start_service_name,
      std::bind(
        &GameDirector::on_start_game_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto tick_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(settings_.tick_period_sec));
    timer_ = this->create_wall_timer(tick_period, std::bind(&GameDirector::tick, this));

    std::vector<TaskDefinition> enabled_definitions;
    enabled_definitions.reserve(tasks_.size());
    for (const auto & [_, runtime] : tasks_) {
      enabled_definitions.push_back(runtime.definition);
    }

    const auto [total_tasks, by_type] = summarize_task_pool(enabled_definitions);
    json by_type_json = json::object();
    for (const auto & [task_type, count] : by_type) {
      by_type_json[task_type] = count;
    }

    std::vector<std::string> checks(boot_validation_requirements_.begin(), boot_validation_requirements_.end());
    std::sort(checks.begin(), checks.end());
    const std::string by_type_text = by_type_json.dump();
    const std::string checks_text = join(checks, ", ");

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded task pool with %zu tasks (%s); boot_wait=%.1fs tick=%.2fs boot_checks=[%s]",
      total_tasks,
      by_type_text.c_str(),
      settings_.boot_wait_sec,
      settings_.tick_period_sec,
      checks_text.c_str());
  }

private:
  std::set<std::string> load_boot_validation_requirements()
  {
    std::vector<std::string> raw;
    try {
      raw = this->get_parameter("boot_validation_requirements").as_string_array();
    } catch (const std::exception &) {
      RCLCPP_WARN(
        this->get_logger(),
        "boot_validation_requirements must be a string list; using defaults");
      return std::set<std::string>(
        kDefaultBootValidationRequirements.begin(), kDefaultBootValidationRequirements.end());
    }

    std::set<std::string> enabled;
    std::vector<std::string> invalid;
    bool disable_all = false;

    for (const auto & value : raw) {
      const std::string key = trim(value);
      if (key.empty()) {
        continue;
      }
      const std::string lowered = to_lower(key);
      if (lowered == "none" || lowered == "off" || lowered == "disable_all") {
        disable_all = true;
        continue;
      }
      if (!contains(kDefaultBootValidationRequirements, key)) {
        invalid.push_back(key);
        continue;
      }
      enabled.insert(key);
    }

    if (!invalid.empty()) {
      std::sort(invalid.begin(), invalid.end());
      invalid.erase(std::unique(invalid.begin(), invalid.end()), invalid.end());
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring unknown boot_validation_requirements entries: [%s]",
        join(invalid, ", ").c_str());
    }

    if (disable_all) {
      if (!enabled.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "boot_validation_requirements includes disable-all sentinel and explicit checks; disabling all checks");
      }
      return {};
    }

    if (!raw.empty() && enabled.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "No valid boot_validation_requirements left after filtering; using defaults");
      return std::set<std::string>(
        kDefaultBootValidationRequirements.begin(), kDefaultBootValidationRequirements.end());
    }

    return enabled;
  }

  std::filesystem::path resolve_task_pool_path()
  {
    std::vector<std::filesystem::path> candidates;

    const std::string param_path = trim(this->get_parameter("task_pool_path").as_string());
    if (!param_path.empty()) {
      candidates.push_back(expand_path(param_path));
    }

    candidates.push_back(std::filesystem::current_path() / "config" / "task_pool.yaml");

    try {
      const auto share_dir = ament_index_cpp::get_package_share_directory("game_director");
      candidates.push_back(std::filesystem::path(share_dir) / "config" / "task_pool.yaml");
    } catch (const std::exception &) {
    }

    const std::filesystem::path exe_path = executable_path();
    if (!exe_path.empty()) {
      std::filesystem::path parent = exe_path.parent_path();
      while (!parent.empty()) {
        candidates.push_back(parent / "config" / "task_pool.yaml");
        if (parent == parent.root_path()) {
          break;
        }
        const auto next = parent.parent_path();
        if (next == parent) {
          break;
        }
        parent = next;
      }
    }

    std::set<std::string> seen;
    std::vector<std::string> checked;

    for (const auto & candidate : candidates) {
      const auto expanded = candidate;
      const std::string key = expanded.string();
      if (seen.find(key) != seen.end()) {
        continue;
      }
      seen.insert(key);
      checked.push_back(key);
      if (std::filesystem::exists(expanded)) {
        return expanded;
      }
    }

    throw std::runtime_error(
            "Unable to locate task_pool.yaml; checked: [" + join(checked, ", ") + "]");
  }

  rclcpp::QoS services_qos() const
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(20));
    qos.reliable();
    qos.durability_volatile();
    return qos;
  }

  void tick()
  {
    const double now = now_unix_s();

    world_.refresh_heartbeats(now, settings_.stale_robot_timeout_sec);
    cleanup_expired_priority_boosts(now);

    if (phase_ == MissionPhase::BOOT) {
      run_boot_stage(now);
      publish_status(now);
      return;
    }

    if (phase_ == MissionPhase::SAFE_HOLD) {
      enforce_safe_hold(now);
      if (boot_checks_passed(now)) {
        safe_hold_reason_.clear();
        set_phase(MissionPhase::READY, "safe hold checks recovered");
      }
      publish_status(now);
      return;
    }

    if (phase_ == MissionPhase::READY) {
      if (!boot_checks_passed(now)) {
        start_requested_ = false;
        safe_hold_reason_ = "ready validation failed";
        set_phase(MissionPhase::SAFE_HOLD, safe_hold_reason_);
        publish_status(now);
        return;
      }

      if (start_requested_) {
        if (world_.match_start_unix_s <= 0.0) {
          world_.set_match_start(now);
        }
        start_requested_ = false;
        set_phase(MissionPhase::GAME_START, "start_game service called");
      }
      publish_status(now);
      return;
    }

    consume_api_responses(now);
    sync_rmf_state(now);
    enforce_watchdogs(now);
    recompute_task_states(now);
    advance_phase(now);

    dispatch_ready_tasks(now);

    if (phase_ == MissionPhase::COMPLETE) {
      publish_final_report_if_needed(now);
    }

    publish_status(now);
  }

  void run_boot_stage(double now)
  {
    if (boot_checks_passed(now)) {
      safe_hold_reason_.clear();
      set_phase(MissionPhase::READY, "boot validation passed");
      return;
    }

    const double elapsed = now - boot_started_unix_s_;
    if (elapsed > settings_.boot_wait_sec) {
      safe_hold_reason_ = "boot validation timeout";
      set_phase(MissionPhase::SAFE_HOLD, safe_hold_reason_);
    }
  }

  bool boot_checks_passed(double now)
  {
    (void)now;

    if (
      boot_validation_requirements_.count("dispatch_states_publisher") > 0 &&
      this->count_publishers("dispatch_states") <= 0)
    {
      return false;
    }

    if (
      boot_validation_requirements_.count("task_summaries_publisher") > 0 &&
      this->count_publishers("task_summaries") <= 0)
    {
      return false;
    }

    if (
      boot_validation_requirements_.count("task_api_requests_subscriber") > 0 &&
      this->count_subscribers("task_api_requests") <= 0)
    {
      return false;
    }

    if (
      boot_validation_requirements_.count("expected_robots_seen") > 0 &&
      !world_.all_expected_robots_seen())
    {
      return false;
    }

    if (
      boot_validation_requirements_.count("expected_robots_online") > 0 &&
      !world_.all_expected_robots_online())
    {
      return false;
    }

    if (
      boot_validation_requirements_.count("valid_robot_maps") > 0 &&
      !world_.valid_robot_maps())
    {
      return false;
    }

    if (boot_validation_requirements_.count("finite_robot_pose") > 0) {
      for (const auto & [_, robot] : world_.robots) {
        if (!std::isfinite(robot.x) || !std::isfinite(robot.y) || !std::isfinite(robot.yaw)) {
          return false;
        }
      }
    }

    if (
      boot_validation_requirements_.count("sane_battery_values") > 0 &&
      !world_.sane_battery_values())
    {
      return false;
    }

    return true;
  }

  void enforce_safe_hold(double now)
  {
    for (auto & [_, runtime] : tasks_) {
      if (
        runtime.state == TaskLifecycle::RUNNING || runtime.state == TaskLifecycle::SUBMITTED ||
        runtime.state == TaskLifecycle::CANCELING)
      {
        begin_cancel(runtime, "safe_hold", now, "hold");
      }
    }
  }

  void advance_phase(double now)
  {
    (void)now;
    if (phase_ == MissionPhase::GAME_START && all_tasks_terminal()) {
      set_phase(MissionPhase::COMPLETE, "all mission tasks terminal");
    }
  }

  void set_phase(MissionPhase phase, const std::string & reason)
  {
    if (phase == phase_) {
      return;
    }
    phase_ = phase;
    world_.phase = phase;
    RCLCPP_INFO(this->get_logger(), "Phase -> %s (%s)", phase_to_string(phase).c_str(), reason.c_str());
  }

  bool all_tasks_terminal() const
  {
    for (const auto & [_, runtime] : tasks_) {
      if (!is_terminal(runtime.state)) {
        return false;
      }
    }
    return true;
  }

  bool task_terminal(const std::string & task_id) const
  {
    const auto it = tasks_.find(task_id);
    if (it == tasks_.end()) {
      return true;
    }
    return is_terminal(it->second.state);
  }

  void recompute_task_states(double now)
  {
    for (auto & [_, runtime] : tasks_) {
      if (
        runtime.state == TaskLifecycle::SUBMITTED || runtime.state == TaskLifecycle::RUNNING ||
        runtime.state == TaskLifecycle::CANCELING)
      {
        continue;
      }

      if (success_conditions_met(runtime)) {
        if (runtime.state != TaskLifecycle::DONE) {
          mark_done(runtime);
        }
        continue;
      }

      if (dismiss_conditions_met(runtime)) {
        if (runtime.state != TaskLifecycle::SKIPPED) {
          runtime.state = TaskLifecycle::SKIPPED;
          release_task_locks(runtime);
        }
        continue;
      }

      if (
        runtime.state == TaskLifecycle::DONE || runtime.state == TaskLifecycle::SKIPPED ||
        runtime.state == TaskLifecycle::CANCELED)
      {
        continue;
      }

      if (runtime.state == TaskLifecycle::FAILED && !runtime.can_retry_now(now)) {
        continue;
      }

      if (!phase_allows_task(runtime)) {
        runtime.state = TaskLifecycle::PENDING;
        continue;
      }

      if (runtime.attempts >= runtime.definition.retry.max_attempts) {
        runtime.state = TaskLifecycle::FAILED;
        continue;
      }

      const std::string dependency_state = dependency_resolution_state(runtime);
      if (dependency_state == "block") {
        runtime.state = TaskLifecycle::PENDING;
        continue;
      }
      if (dependency_state == "skip") {
        runtime.state = TaskLifecycle::SKIPPED;
        runtime.status_note = "dependency failed";
        release_task_locks(runtime);
        continue;
      }
      if (dependency_state == "fail") {
        runtime.state = TaskLifecycle::FAILED;
        runtime.last_error = "dependency failed";
        runtime.status_note = "dependency failed";
        release_task_locks(runtime);
        continue;
      }

      if (preconditions_met(runtime)) {
        runtime.state = TaskLifecycle::READY;
      } else {
        runtime.state = TaskLifecycle::PENDING;
      }
    }
  }

  std::string dependency_resolution_state(const TaskRuntime & runtime) const
  {
    if (runtime.definition.depends_on.empty()) {
      return "ready";
    }

    const bool require_terminal = runtime.definition.dependency_policy == "require_terminal";
    const std::string on_failure = runtime.definition.on_dependency_failure;

    for (const auto & task_id : runtime.definition.depends_on) {
      const auto other_it = tasks_.find(task_id);
      if (other_it == tasks_.end()) {
        return "fail";
      }
      const TaskRuntime & other = other_it->second;

      if (other.state == TaskLifecycle::DONE) {
        continue;
      }

      if (require_terminal) {
        if (!is_terminal(other.state)) {
          return "block";
        }
        if (on_failure == "run_anyway") {
          continue;
        }
        if (other.state != TaskLifecycle::DONE) {
          return on_failure;
        }
        continue;
      }

      if (is_terminal(other.state)) {
        if (on_failure == "run_anyway") {
          continue;
        }
        return other.state == TaskLifecycle::DONE ? "ready" : on_failure;
      }
      return "block";
    }

    return "ready";
  }

  bool phase_allows_task(const TaskRuntime & runtime) const
  {
    (void)runtime;
    return phase_ == MissionPhase::GAME_START;
  }

  bool preconditions_met(const TaskRuntime & runtime)
  {
    return conditions_match(runtime.definition.preconditions, runtime);
  }

  bool dismiss_conditions_met(const TaskRuntime & runtime)
  {
    if (runtime.definition.dismiss_conditions.empty()) {
      return false;
    }
    return conditions_match(runtime.definition.dismiss_conditions, runtime);
  }

  bool success_conditions_met(const TaskRuntime & runtime)
  {
    if (runtime.definition.success_conditions.empty()) {
      return false;
    }
    return conditions_match(runtime.definition.success_conditions, runtime);
  }

  bool conditions_match(const std::vector<json> & conditions, const TaskRuntime & runtime)
  {
    for (const auto & condition : conditions) {
      if (!condition_holds(condition, runtime)) {
        return false;
      }
    }
    return true;
  }

  bool condition_holds(const json & condition, const TaskRuntime & runtime)
  {
    if (!condition.is_object()) {
      return false;
    }

    const std::string kind = trim(condition.value("kind", ""));
    if (kind == "always") {
      return true;
    }

    if (kind == "phase_is") {
      const auto target_phase = parse_phase(condition.value("phase", ""));
      if (!target_phase.has_value()) {
        return false;
      }
      return phase_ == target_phase.value();
    }

    if (kind == "phase_in") {
      if (!condition.contains("phases") || !condition["phases"].is_array()) {
        return false;
      }
      for (const auto & value : condition["phases"]) {
        const auto target_phase = parse_phase(json_string_value(value));
        if (target_phase.has_value() && phase_ == target_phase.value()) {
          return true;
        }
      }
      return false;
    }

    if (kind == "any_antenna_done") {
      return world_.any_antenna_done();
    }

    if (kind == "all_antennas_terminal") {
      return world_.all_antennas_terminal();
    }

    if (kind == "all_ducks_in_drop_zone") {
      return world_.all_ducks_in_drop_zone();
    }

    if (kind == "antenna_state_in") {
      std::string antenna_id;
      if (condition.contains("antenna_id")) {
        antenna_id = json_string_value(condition["antenna_id"]);
      } else if (runtime.definition.target.contains("antenna_id")) {
        antenna_id = json_string_value(runtime.definition.target["antenna_id"]);
      }

      std::set<std::string> states;
      if (condition.contains("states") && condition["states"].is_array()) {
        for (const auto & state : condition["states"]) {
          states.insert(to_upper(json_string_value(state)));
        }
      }

      const auto it = world_.antennas.find(antenna_id);
      return it != world_.antennas.end() &&
             states.find(antenna_state_to_string(it->second)) != states.end();
    }

    if (kind == "duck_state_in") {
      std::string duck_id;
      if (condition.contains("duck_id")) {
        duck_id = json_string_value(condition["duck_id"]);
      } else if (runtime.definition.target.contains("duck_id")) {
        duck_id = json_string_value(runtime.definition.target["duck_id"]);
      }

      std::set<std::string> states;
      if (condition.contains("states") && condition["states"].is_array()) {
        for (const auto & state : condition["states"]) {
          states.insert(to_upper(json_string_value(state)));
        }
      }

      const auto it = world_.ducks.find(duck_id);
      return it != world_.ducks.end() && states.find(duck_state_to_string(it->second.state)) != states.end();
    }

    if (kind == "task_done") {
      const std::string task_id = condition.value("task_id", "");
      const auto it = tasks_.find(task_id);
      return it != tasks_.end() && it->second.state == TaskLifecycle::DONE;
    }

    if (kind == "task_terminal") {
      const std::string task_id = condition.value("task_id", "");
      const auto it = tasks_.find(task_id);
      return it != tasks_.end() && is_terminal(it->second.state);
    }

    if (kind == "crater_token_free") {
      const auto owner_it = locks_.find("crater_token");
      return owner_it == locks_.end() || owner_it->second == runtime.definition.task_id;
    }

    if (kind == "lock_free") {
      const std::string key_template = condition.value("lock", "");
      const json key_value = render_string(
        key_template,
        template_context(runtime, "", "", now_unix_s()));
      const std::string key = json_string_value(key_value);
      const auto owner_it = locks_.find(key);
      return owner_it == locks_.end() || owner_it->second == runtime.definition.task_id;
    }

    if (kind == "robot_online") {
      const std::string robot_name = condition.value("robot", "");
      const auto it = world_.robots.find(robot_name);
      return it != world_.robots.end() && it->second.online;
    }

    if (kind == "any_ground_robot_available") {
      for (const auto & robot_name : candidate_ground_robots(runtime)) {
        if (robot_available_for_task(robot_name, runtime)) {
          return true;
        }
      }
      return false;
    }

    if (kind == "any_drone_available") {
      for (const auto & robot_name : candidate_drone_robots(runtime)) {
        if (robot_available_for_task(robot_name, runtime)) {
          return true;
        }
      }
      return false;
    }

    if (kind == "match_time_before_sec") {
      const double max_sec = condition.contains("value") ? safe_to_double(condition["value"], 0.0) : 0.0;
      return world_.elapsed_match_time(now_unix_s()) <= max_sec;
    }

    if (kind == "match_time_after_sec") {
      const double min_sec = condition.contains("value") ? safe_to_double(condition["value"], 0.0) : 0.0;
      return world_.elapsed_match_time(now_unix_s()) >= min_sec;
    }

    return false;
  }

  void dispatch_ready_tasks(double now)
  {
    std::vector<TaskRuntime *> candidates;
    for (auto & [_, runtime] : tasks_) {
      if (runtime.state == TaskLifecycle::READY) {
        candidates.push_back(&runtime);
      }
    }

    if (candidates.empty()) {
      return;
    }

    std::sort(
      candidates.begin(),
      candidates.end(),
      [this, now](const TaskRuntime * lhs, const TaskRuntime * rhs) {
        return priority_score(*lhs, now) > priority_score(*rhs, now);
      });

    int submissions = 0;
    static constexpr int kMaxSubmissionsPerTick = 4;

    for (TaskRuntime * runtime : candidates) {
      if (submissions >= kMaxSubmissionsPerTick) {
        break;
      }

      if (!runtime->can_retry_now(now)) {
        continue;
      }

      if (runtime->definition.task_type == "CRATER_LAP_PAIR" && !crater_pair_dispatchable()) {
        continue;
      }

      const auto [selected_robot, selected_fleet] = select_robot(*runtime);
      if (runtime->definition.dispatch.mode == "robot_targeted" && selected_robot.empty()) {
        continue;
      }

      const std::vector<std::string> lock_keys = render_lock_keys(*runtime, selected_robot, selected_fleet, now);
      if (!acquire_locks(runtime->definition.task_id, lock_keys)) {
        continue;
      }

      json request;
      try {
        request = render_request(*runtime, selected_robot, selected_fleet, now);
      } catch (const std::exception & exc) {
        release_keys(lock_keys);
        runtime->last_error = std::string("dispatch submit failed: ") + exc.what();
        runtime->state = TaskLifecycle::PENDING;
        continue;
      }

      std::string request_id;
      try {
        request_id = dispatch_client_->send_dispatch_request(
          runtime->definition.task_id,
          request,
          runtime->definition.dispatch.mode,
          selected_fleet,
          selected_robot,
          now);
      } catch (const std::exception & exc) {
        release_keys(lock_keys);
        runtime->last_error = std::string("dispatch submit failed: ") + exc.what();
        runtime->state = TaskLifecycle::PENDING;
        continue;
      }

      runtime->state = TaskLifecycle::SUBMITTED;
      runtime->attempts += 1;
      runtime->request_id = request_id;
      runtime->submitted_at_unix_s = now;
      runtime->assigned_robot = selected_robot;
      runtime->assigned_fleet = selected_fleet;
      runtime->held_locks = lock_keys;
      runtime->status_note = "waiting_api_response";
      runtime->cancel_reason.clear();
      runtime->cancel_outcome.clear();
      runtime->cancel_since_unix_s = 0.0;
      runtime->cancel_requested_at_unix_s = 0.0;

      if (runtime->definition.task_type == "CRATER_LAP_PAIR") {
        world_.crater.lap_in_progress = true;
      }

      ++submissions;
      RCLCPP_INFO(
        this->get_logger(),
        "Submitted %s (attempt=%d, mode=%s, robot=%s, fleet=%s)",
        runtime->definition.task_id.c_str(),
        runtime->attempts,
        runtime->definition.dispatch.mode.c_str(),
        selected_robot.empty() ? "<rmf>" : selected_robot.c_str(),
        selected_fleet.empty() ? "<any>" : selected_fleet.c_str());
    }
  }

  bool crater_pair_dispatchable()
  {
    std::vector<TaskRuntime *> crater_tasks;
    for (auto & [_, runtime] : tasks_) {
      if (runtime.definition.task_type == "CRATER_LAP_PAIR") {
        crater_tasks.push_back(&runtime);
      }
    }

    if (crater_tasks.size() <= 1) {
      return true;
    }

    for (TaskRuntime * runtime : crater_tasks) {
      if (runtime->state == TaskLifecycle::SUBMITTED || runtime->state == TaskLifecycle::RUNNING) {
        continue;
      }
      if (runtime->state != TaskLifecycle::READY) {
        return false;
      }

      const auto [selected_robot, _selected_fleet] = select_robot(*runtime);
      if (runtime->definition.dispatch.mode == "robot_targeted" && selected_robot.empty()) {
        return false;
      }
    }

    return true;
  }

  double priority_score(const TaskRuntime & runtime, double now)
  {
    double score = runtime.definition.priority.base;

    const double remaining =
      std::max(0.0, settings_.match_duration_sec - world_.elapsed_match_time(now));
    score += runtime.definition.priority.deadline_boost_per_sec *
      (settings_.match_duration_sec - remaining);

    const auto dynamic_it = dynamic_priority_boost_.find(runtime.definition.task_id);
    if (dynamic_it != dynamic_priority_boost_.end() && now <= dynamic_it->second.second) {
      score += dynamic_it->second.first;
    }

    if (runtime.definition.phase == phase_) {
      score += 10.0;
    }

    if (runtime.definition.task_type == "CLEAR_BLOCKING_DUCK") {
      score += 30.0;
    }

    return score;
  }

  void cleanup_expired_priority_boosts(double now)
  {
    std::vector<std::string> expired;
    for (const auto & [task_id, boost] : dynamic_priority_boost_) {
      if (now > boost.second) {
        expired.push_back(task_id);
      }
    }

    for (const auto & task_id : expired) {
      dynamic_priority_boost_.erase(task_id);
    }
  }

  std::pair<std::string, std::string> select_robot(const TaskRuntime & runtime)
  {
    const std::string mode = runtime.definition.dispatch.mode;
    if (mode == "robot_targeted") {
      const std::string desired_robot = runtime.definition.dispatch.robot;
      if (!desired_robot.empty() && robot_available_for_task(desired_robot, runtime)) {
        return {
          desired_robot,
          runtime.definition.dispatch.fleet.empty() ?
          infer_fleet(runtime, desired_robot) : runtime.definition.dispatch.fleet,
        };
      }

      const std::vector<std::string> preferred =
        runtime.definition.preferred_robots.empty() ?
        runtime.definition.allowed_robots : runtime.definition.preferred_robots;
      for (const auto & robot_name : preferred) {
        if (robot_available_for_task(robot_name, runtime)) {
          return {
            robot_name,
            runtime.definition.dispatch.fleet.empty() ?
            infer_fleet(runtime, robot_name) : runtime.definition.dispatch.fleet,
          };
        }
      }

      return {
        "",
        runtime.definition.dispatch.fleet.empty() ? infer_fleet(runtime, "") : runtime.definition.dispatch.fleet,
      };
    }

    std::vector<std::string> candidates = candidate_robots_for_task(runtime);
    std::string best_robot;
    double best_score = -1.0;

    for (const auto & robot_name : candidates) {
      if (!robot_available_for_task(robot_name, runtime)) {
        continue;
      }

      const auto health_it = capability_health_.find(robot_name);
      const std::unordered_map<std::string, int> * failure_counts = nullptr;
      if (health_it != capability_health_.end()) {
        failure_counts = &health_it->second.failures_by_type;
      }

      const double health = world_.robot_health_score(
        robot_name,
        settings_.min_battery_soc,
        failure_counts);
      if (health <= 0.0) {
        continue;
      }

      double score = health;
      if (contains(runtime.definition.preferred_robots, robot_name)) {
        score += 0.4;
      }

      const auto [inflight_major, inflight_micro] = inflight_counts_for_robot(robot_name);
      score -= 0.2 * static_cast<double>(inflight_major);
      score -= 0.1 * static_cast<double>(inflight_micro);

      if (starts_with(runtime.definition.task_type, "DRONE_")) {
        if (contains(world_.expected_drone_robots, robot_name)) {
          score += 0.6;
        } else {
          score -= 0.6;
        }
      }

      if (score > best_score) {
        best_score = score;
        best_robot = robot_name;
      }
    }

    std::string fleet = runtime.definition.dispatch.fleet;
    if (fleet.empty() && !best_robot.empty()) {
      fleet = infer_fleet(runtime, best_robot);
    }

    return {best_robot, fleet};
  }

  std::vector<std::string> candidate_robots_for_task(const TaskRuntime & runtime)
  {
    if (!runtime.definition.allowed_robots.empty()) {
      return runtime.definition.allowed_robots;
    }

    if (starts_with(runtime.definition.task_type, "DRONE_")) {
      if (!world_.expected_drone_robots.empty()) {
        return world_.expected_drone_robots;
      }
      std::vector<std::string> candidates;
      for (const auto & [name, _] : world_.robots) {
        if (starts_with(name, "drone")) {
          candidates.push_back(name);
        }
      }
      return candidates;
    }

    if (!world_.expected_ground_robots.empty()) {
      return world_.expected_ground_robots;
    }

    std::vector<std::string> candidates;
    for (const auto & [name, _] : world_.robots) {
      if (!starts_with(name, "drone")) {
        candidates.push_back(name);
      }
    }
    return candidates;
  }

  std::vector<std::string> candidate_ground_robots(const TaskRuntime & runtime)
  {
    (void)runtime;
    if (!world_.expected_ground_robots.empty()) {
      return world_.expected_ground_robots;
    }

    std::vector<std::string> candidates;
    for (const auto & [name, _] : world_.robots) {
      if (!starts_with(name, "drone")) {
        candidates.push_back(name);
      }
    }
    return candidates;
  }

  std::vector<std::string> candidate_drone_robots(const TaskRuntime & runtime)
  {
    (void)runtime;
    if (!world_.expected_drone_robots.empty()) {
      return world_.expected_drone_robots;
    }

    std::vector<std::string> candidates;
    for (const auto & [name, _] : world_.robots) {
      if (starts_with(name, "drone")) {
        candidates.push_back(name);
      }
    }
    return candidates;
  }

  bool robot_available_for_task(const std::string & robot_name, const TaskRuntime & runtime)
  {
    if (robot_name.empty()) {
      return false;
    }

    const auto snapshot_it = world_.robots.find(robot_name);
    if (snapshot_it == world_.robots.end() || !snapshot_it->second.online) {
      return false;
    }

    const RobotSnapshot & snapshot = snapshot_it->second;

    if (
      !runtime.definition.excluded_robots.empty() &&
      contains(runtime.definition.excluded_robots, robot_name))
    {
      return false;
    }

    if (
      !runtime.definition.allowed_robots.empty() &&
      !contains(runtime.definition.allowed_robots, robot_name))
    {
      return false;
    }

    if (!runtime.definition.required_capabilities.empty()) {
      const auto cap_it = world_.robot_capabilities.find(robot_name);
      if (cap_it == world_.robot_capabilities.end()) {
        return false;
      }

      const auto & capabilities = cap_it->second;
      for (const auto & capability : runtime.definition.required_capabilities) {
        if (capabilities.find(capability) == capabilities.end()) {
          return false;
        }
      }
    }

    if (snapshot.battery_valid && snapshot.battery_soc < settings_.min_battery_soc) {
      return false;
    }

    if (capability_health_[robot_name].degraded_types.count(runtime.definition.task_type) > 0) {
      return false;
    }

    const auto [inflight_major, inflight_micro] = inflight_counts_for_robot(robot_name);
    if (runtime.definition.major && inflight_major >= settings_.major_inflight_limit_per_robot) {
      return false;
    }
    if (!runtime.definition.major && inflight_micro >= settings_.micro_inflight_limit_per_robot) {
      return false;
    }

    if (
      (runtime.definition.task_type == "COLLECT_DUCK" ||
      runtime.definition.task_type == "CLEAR_BLOCKING_DUCK") &&
      !snapshot.carrying_duck_id.empty())
    {
      return false;
    }

    return true;
  }

  std::pair<int, int> inflight_counts_for_robot(const std::string & robot_name) const
  {
    int major = 0;
    int micro = 0;

    for (const auto & [_, runtime] : tasks_) {
      if (runtime.assigned_robot != robot_name) {
        continue;
      }

      if (
        runtime.state != TaskLifecycle::SUBMITTED && runtime.state != TaskLifecycle::RUNNING &&
        runtime.state != TaskLifecycle::CANCELING)
      {
        continue;
      }

      if (runtime.definition.major) {
        major += 1;
      } else {
        micro += 1;
      }
    }

    return {major, micro};
  }

  std::string infer_fleet(const TaskRuntime & runtime, const std::string & robot_name) const
  {
    if (!runtime.definition.dispatch.fleet.empty()) {
      return runtime.definition.dispatch.fleet;
    }

    if (starts_with(runtime.definition.task_type, "DRONE_")) {
      return "drone";
    }

    if (!robot_name.empty() && contains(world_.expected_drone_robots, robot_name)) {
      return "drone";
    }

    return "ground";
  }

  std::vector<std::string> render_lock_keys(
    const TaskRuntime & runtime,
    const std::string & robot,
    const std::string & fleet,
    double now)
  {
    std::vector<std::string> keys;
    const json context = template_context(runtime, robot, fleet, now);

    keys.reserve(runtime.definition.lock_keys.size());
    for (const auto & templated_key : runtime.definition.lock_keys) {
      keys.push_back(json_string_value(render_string(templated_key, context)));
    }

    return keys;
  }

  bool acquire_locks(const std::string & task_id, const std::vector<std::string> & keys)
  {
    for (const auto & key : keys) {
      const auto owner_it = locks_.find(key);
      if (owner_it != locks_.end() && owner_it->second != task_id) {
        return false;
      }
    }

    for (const auto & key : keys) {
      locks_[key] = task_id;
      if (key == "crater_token") {
        world_.crater.token_owner_task_id = task_id;
      }
    }

    return true;
  }

  void release_keys(const std::vector<std::string> & keys)
  {
    for (const auto & key : keys) {
      locks_.erase(key);
      if (key == "crater_token" && locks_.find("crater_token") == locks_.end()) {
        world_.crater.token_owner_task_id.clear();
      }
    }
  }

  void release_task_locks(TaskRuntime & runtime)
  {
    release_keys(runtime.held_locks);
    runtime.held_locks.clear();
  }

  json template_context(
    const TaskRuntime & runtime,
    const std::string & robot,
    const std::string & fleet,
    double now) const
  {
    json target = runtime.definition.target;
    if (!target.is_object()) {
      target = json::object();
    }

    json context = {
      {"task_id", runtime.definition.task_id},
      {"task_type", runtime.definition.task_type},
      {"selected_robot", robot},
      {"selected_fleet", fleet},
      {"drop_zone", world_.drop_zone},
      {"now_unix_ms", static_cast<long long>(now * 1000.0)},
      {"target", target},
    };

    for (auto it = target.begin(); it != target.end(); ++it) {
      context[it.key()] = it.value();
    }

    return context;
  }

  json render_request(
    const TaskRuntime & runtime,
    const std::string & robot,
    const std::string & fleet,
    double now)
  {
    json request = runtime.definition.dispatch.request;
    request = render_template(request, template_context(runtime, robot, fleet, now));

    if (!request.is_object()) {
      throw std::runtime_error(
              "task " + runtime.definition.task_id + " dispatch.request must be an object");
    }

    if (!request.contains("category") || !request.contains("description")) {
      throw std::runtime_error(
              "task " + runtime.definition.task_id + " dispatch.request requires category+description");
    }

    const long long start_ms = static_cast<long long>(now * 1000.0);
    if (!request.contains("unix_millis_request_time")) {
      request["unix_millis_request_time"] = start_ms;
    }
    if (!request.contains("unix_millis_earliest_start_time")) {
      request["unix_millis_earliest_start_time"] = start_ms;
    }
    if (!request.contains("requester")) {
      request["requester"] = settings_.requester;
    }

    if (runtime.definition.dispatch.mode == "best_available" && !fleet.empty()) {
      if (!request.contains("fleet_name")) {
        request["fleet_name"] = fleet;
      }
    }

    std::set<std::string> labels;
    if (request.contains("labels") && request["labels"].is_array()) {
      for (const auto & label : request["labels"]) {
        labels.insert(json_string_value(label));
      }
    }

    labels.insert("game_task:" + runtime.definition.task_id);
    labels.insert("game_type:" + runtime.definition.task_type);
    labels.insert("game_phase:" + phase_to_string(runtime.definition.phase));

    json label_list = json::array();
    for (const auto & label : labels) {
      label_list.push_back(label);
    }
    request["labels"] = label_list;

    return request;
  }

  json render_template(const json & value, const json & context)
  {
    if (value.is_object()) {
      json out = json::object();
      for (auto it = value.begin(); it != value.end(); ++it) {
        out[it.key()] = render_template(it.value(), context);
      }
      return out;
    }

    if (value.is_array()) {
      json out = json::array();
      for (const auto & entry : value) {
        out.push_back(render_template(entry, context));
      }
      return out;
    }

    if (value.is_string()) {
      return render_string(value.get<std::string>(), context);
    }

    return value;
  }

  json render_string(const std::string & text, const json & context)
  {
    std::smatch whole_match;
    if (std::regex_match(text, whole_match, kWholePlaceholderRegex)) {
      return resolve_context_path(whole_match[1].str(), context);
    }

    const auto begin = std::sregex_iterator(text.begin(), text.end(), kPlaceholderRegex);
    const auto end = std::sregex_iterator();
    if (begin == end) {
      return text;
    }

    std::string rendered;
    std::size_t cursor = 0;

    for (auto it = begin; it != end; ++it) {
      const auto & match = *it;
      const std::size_t pos = static_cast<std::size_t>(match.position());
      rendered.append(text.substr(cursor, pos - cursor));
      rendered.append(json_string_value(resolve_context_path(match[1].str(), context)));
      cursor = pos + static_cast<std::size_t>(match.length());
    }

    rendered.append(text.substr(cursor));
    return rendered;
  }

  json resolve_context_path(const std::string & key, const json & context)
  {
    json current = context;
    std::stringstream ss(key);
    std::string token;

    while (std::getline(ss, token, '.')) {
      if (current.is_object() && current.contains(token)) {
        current = current[token];
      } else {
        return "";
      }
    }

    return current;
  }

  void consume_api_responses(double now)
  {
    for (auto & [_, runtime] : tasks_) {
      if (runtime.request_id.empty()) {
        continue;
      }

      const auto response = dispatch_client_->consume_response(runtime.request_id);
      if (!response.has_value()) {
        continue;
      }

      const std::string request_id = runtime.request_id;
      (void)request_id;
      runtime.request_id.clear();

      if (response->value("success", false)) {
        const std::string rmf_task_id = extract_rmf_task_id(*response);
        if (rmf_task_id.empty()) {
          mark_failed(runtime, "accepted response missing booking.id", now);
          continue;
        }

        runtime.rmf_task_id = rmf_task_id;
        game_to_rmf_[runtime.definition.task_id] = rmf_task_id;
        rmf_to_game_[rmf_task_id] = runtime.definition.task_id;
        runtime.dispatch_status = "queued";
        runtime.status_note = "accepted";
        continue;
      }

      const std::string reason = response_error_text(*response);
      mark_failed(runtime, reason, now);
    }

    for (const auto & pending : dispatch_client_->consume_pending_timeouts(now)) {
      const auto runtime_it = tasks_.find(pending.game_task_id);
      if (runtime_it == tasks_.end()) {
        continue;
      }

      TaskRuntime & runtime = runtime_it->second;
      if (runtime.request_id != pending.request_id) {
        continue;
      }

      runtime.request_id.clear();
      mark_failed(runtime, "api response timeout", now);
    }
  }

  std::string extract_rmf_task_id(const json & response)
  {
    if (!response.contains("state") || !response["state"].is_object()) {
      return "";
    }

    const json & state = response["state"];
    if (!state.contains("booking") || !state["booking"].is_object()) {
      return "";
    }

    const json & booking = state["booking"];
    if (!booking.contains("id")) {
      return "";
    }

    return json_string_value(booking["id"]);
  }

  std::string response_error_text(const json & response)
  {
    if (response.contains("errors") && response["errors"].is_array() && !response["errors"].empty()) {
      const json & first = response["errors"][0];
      if (first.is_object()) {
        const std::string category = first.value("category", "");
        const std::string detail = first.value("detail", "");
        std::string reason = category;
        if (!detail.empty()) {
          if (!reason.empty()) {
            reason += ": ";
          }
          reason += detail;
        }
        return reason.empty() ? "request rejected" : reason;
      }
      return json_string_value(first);
    }

    return "request rejected";
  }

  void sync_rmf_state(double now)
  {
    std::vector<std::pair<std::string, std::string>> mappings;
    mappings.reserve(rmf_to_game_.size());
    for (const auto & mapping : rmf_to_game_) {
      mappings.push_back(mapping);
    }

    for (const auto & [rmf_task_id, game_task_id] : mappings) {
      const auto runtime_it = tasks_.find(game_task_id);
      if (runtime_it == tasks_.end()) {
        continue;
      }
      TaskRuntime & runtime = runtime_it->second;

      const auto dispatch_it = dispatch_tracker_.dispatch_by_task_id.find(rmf_task_id);
      if (dispatch_it != dispatch_tracker_.dispatch_by_task_id.end()) {
        const DispatchSnapshot & dispatch = dispatch_it->second;

        runtime.dispatch_status = dispatch.status_name;
        if (!dispatch.robot_name.empty()) {
          runtime.assigned_robot = dispatch.robot_name;
        }
        if (!dispatch.fleet_name.empty()) {
          runtime.assigned_fleet = dispatch.fleet_name;
        }

        if (
          dispatch.status == rmf_task_msgs::msg::DispatchState::STATUS_FAILED_TO_ASSIGN ||
          dispatch.status == rmf_task_msgs::msg::DispatchState::STATUS_CANCELED_IN_FLIGHT)
        {
          std::string reason;
          if (!dispatch.errors.empty()) {
            reason = join(dispatch.errors, "; ");
          } else {
            reason = dispatch.status_name;
          }

          if (runtime.state == TaskLifecycle::CANCELING && runtime.cancel_outcome == "hold") {
            mark_canceled_hold(runtime, now, reason);
          } else {
            mark_failed(runtime, "dispatch " + reason, now);
          }
          continue;
        }
      }

      const auto summary_it = dispatch_tracker_.summary_by_task_id.find(rmf_task_id);
      if (summary_it == dispatch_tracker_.summary_by_task_id.end()) {
        continue;
      }
      const SummarySnapshot & summary = summary_it->second;

      if (!summary.robot_name.empty()) {
        runtime.assigned_robot = summary.robot_name;
      }
      if (!summary.fleet_name.empty()) {
        runtime.assigned_fleet = summary.fleet_name;
      }

      if (
        summary.state == rmf_task_msgs::msg::TaskSummary::STATE_PENDING ||
        summary.state == rmf_task_msgs::msg::TaskSummary::STATE_QUEUED)
      {
        if (
          runtime.state != TaskLifecycle::SUBMITTED && runtime.state != TaskLifecycle::RUNNING &&
          runtime.state != TaskLifecycle::CANCELING)
        {
          runtime.state = TaskLifecycle::SUBMITTED;
        }
        runtime.status_note = summary.status;
        continue;
      }

      if (summary.state == rmf_task_msgs::msg::TaskSummary::STATE_ACTIVE) {
        if (runtime.state != TaskLifecycle::RUNNING && runtime.state != TaskLifecycle::CANCELING) {
          runtime.state = TaskLifecycle::RUNNING;
          runtime.running_since_unix_s = summary.updated_at_unix_s;
        }
        runtime.status_note = summary.status;
        continue;
      }

      if (summary.state == rmf_task_msgs::msg::TaskSummary::STATE_COMPLETED) {
        mark_done(runtime);
        continue;
      }

      if (
        summary.state == rmf_task_msgs::msg::TaskSummary::STATE_FAILED ||
        summary.state == rmf_task_msgs::msg::TaskSummary::STATE_CANCELED)
      {
        const std::string reason = summary.status.empty() ? summary.state_name : summary.status;
        if (runtime.state == TaskLifecycle::CANCELING && runtime.cancel_outcome == "hold") {
          mark_canceled_hold(runtime, now, reason);
        } else {
          const std::string cancel_reason =
            runtime.state == TaskLifecycle::CANCELING ? runtime.cancel_reason : "";
          mark_failed(runtime, cancel_reason.empty() ? reason : cancel_reason, now);
        }
        continue;
      }
    }
  }

  void enforce_watchdogs(double now)
  {
    for (auto & [_, runtime] : tasks_) {
      if (runtime.state == TaskLifecycle::SUBMITTED && !runtime.rmf_task_id.empty()) {
        const double elapsed = now - runtime.submitted_at_unix_s;
        if (elapsed > runtime.definition.timeouts.queued_sec) {
          begin_cancel(runtime, "queued_timeout", now, "retry");
          continue;
        }
      }

      if (runtime.state == TaskLifecycle::RUNNING) {
        const double elapsed = now - runtime.running_since_unix_s;
        if (elapsed > runtime.definition.timeouts.running_sec) {
          begin_cancel(runtime, "run_timeout", now, "retry");
          continue;
        }
      }

      if (
        (runtime.state == TaskLifecycle::SUBMITTED || runtime.state == TaskLifecycle::RUNNING) &&
        !runtime.assigned_robot.empty())
      {
        const auto robot_it = world_.robots.find(runtime.assigned_robot);
        if (robot_it == world_.robots.end() || !robot_it->second.online) {
          begin_cancel(runtime, "robot_offline", now, "retry");
          continue;
        }
      }

      if (runtime.state == TaskLifecycle::CANCELING) {
        if (
          runtime.cancel_requested_at_unix_s > 0.0 &&
          now - runtime.cancel_requested_at_unix_s >= settings_.cancel_retry_period_sec)
        {
          request_cancel(runtime, runtime.cancel_reason.empty() ? "canceling" : runtime.cancel_reason);
          runtime.cancel_requested_at_unix_s = now;
        }

        if (
          runtime.cancel_since_unix_s > 0.0 &&
          now - runtime.cancel_since_unix_s >= settings_.cancel_timeout_sec)
        {
          if (runtime.cancel_outcome == "hold") {
            mark_canceled_hold(runtime, now, "cancel timeout");
          } else {
            mark_failed(runtime, "cancel timeout", now, true, true);
          }
        }
      }
    }
  }

  void request_cancel(TaskRuntime & runtime, const std::string & reason)
  {
    if (runtime.rmf_task_id.empty()) {
      return;
    }

    if (runtime.dispatch_status == "queued" || runtime.dispatch_status == "selected") {
      dispatch_client_->send_cancel_service_request(runtime.rmf_task_id);
    }

    dispatch_client_->send_cancel_api_request(
      runtime.rmf_task_id,
      {
        "game_task:" + runtime.definition.task_id,
        "reason:" + reason,
      });
  }

  void begin_cancel(TaskRuntime & runtime, const std::string & reason, double now, const std::string & outcome)
  {
    if (
      runtime.state != TaskLifecycle::SUBMITTED && runtime.state != TaskLifecycle::RUNNING &&
      runtime.state != TaskLifecycle::CANCELING)
    {
      return;
    }

    if (runtime.state != TaskLifecycle::CANCELING) {
      runtime.state = TaskLifecycle::CANCELING;
      runtime.cancel_since_unix_s = now;
      runtime.cancel_reason = reason;
      runtime.cancel_outcome = outcome;
      runtime.status_note = "canceling:" + reason;
    } else if (outcome == "hold") {
      runtime.cancel_outcome = "hold";
      if (runtime.cancel_reason.empty()) {
        runtime.cancel_reason = reason;
      }
    }

    const bool should_send =
      runtime.cancel_requested_at_unix_s <= 0.0 ||
      now - runtime.cancel_requested_at_unix_s >= settings_.cancel_retry_period_sec;

    if (should_send) {
      request_cancel(runtime, reason);
      runtime.cancel_requested_at_unix_s = now;
    }
  }

  void mark_canceled_hold(TaskRuntime & runtime, double now, const std::string & reason)
  {
    runtime.status_note = "canceled:" + reason;
    runtime.last_error.clear();
    if (runtime.attempts > 0) {
      runtime.attempts -= 1;
    }
    runtime.blocked_until_unix_s = now;

    if (runtime.definition.task_type == "CRATER_LAP_PAIR") {
      world_.crater.lap_in_progress = false;
      world_.crater.token_owner_task_id.clear();
    }

    runtime.state = TaskLifecycle::PENDING;
    clear_runtime_tracking(runtime, true);
  }

  void mark_done(TaskRuntime & runtime)
  {
    runtime.state = TaskLifecycle::DONE;
    runtime.status_note = "completed";
    world_.on_task_success(runtime.definition.task_type, runtime.definition.target, runtime.assigned_robot);
    record_success(runtime);
    clear_runtime_tracking(runtime, true);
  }

  void mark_failed(
    TaskRuntime & runtime,
    const std::string & reason,
    double now,
    bool propagate = true,
    bool force_terminal = false)
  {
    runtime.last_error = reason;

    if (propagate && runtime.definition.task_type == "CRATER_LAP_PAIR") {
      abort_crater_partner(runtime, reason, now);
    }

    record_failure(runtime);
    const bool attempts_exhausted = force_terminal ||
      runtime.attempts >= runtime.definition.retry.max_attempts;

    world_.on_task_failure(
      runtime.definition.task_type,
      runtime.definition.target,
      attempts_exhausted,
      reason);

    const std::string lowered = to_lower(reason);
    if (
      runtime.definition.task_type == "ACTIVATE_ANTENNA" &&
      lowered.find("duck") != std::string::npos &&
      lowered.find("block") != std::string::npos)
    {
      boost_duck_clearance(now);
    }

    runtime.status_note = reason;
    clear_runtime_tracking(runtime, true);

    if (attempts_exhausted) {
      runtime.state = TaskLifecycle::FAILED;
      return;
    }

    runtime.state = TaskLifecycle::PENDING;
    runtime.mark_retry_backoff(now);
  }

  void abort_crater_partner(TaskRuntime & failed_runtime, const std::string & reason, double now)
  {
    (void)reason;

    for (auto & [_, runtime] : tasks_) {
      if (runtime.definition.task_type != "CRATER_LAP_PAIR") {
        continue;
      }
      if (runtime.definition.task_id == failed_runtime.definition.task_id) {
        continue;
      }
      if (
        runtime.state != TaskLifecycle::SUBMITTED && runtime.state != TaskLifecycle::RUNNING &&
        runtime.state != TaskLifecycle::CANCELING)
      {
        continue;
      }

      begin_cancel(runtime, "crater_partner_failed", now, "retry");
    }
  }

  void clear_runtime_tracking(TaskRuntime & runtime, bool release_locks)
  {
    if (!runtime.rmf_task_id.empty()) {
      rmf_to_game_.erase(runtime.rmf_task_id);
    }

    game_to_rmf_.erase(runtime.definition.task_id);
    runtime.request_id.clear();
    runtime.rmf_task_id.clear();
    runtime.dispatch_status.clear();
    runtime.running_since_unix_s = 0.0;
    runtime.submitted_at_unix_s = 0.0;
    runtime.cancel_reason.clear();
    runtime.cancel_outcome.clear();
    runtime.cancel_since_unix_s = 0.0;
    runtime.cancel_requested_at_unix_s = 0.0;

    if (release_locks) {
      release_task_locks(runtime);
    }
  }

  void record_success(TaskRuntime & runtime)
  {
    if (runtime.assigned_robot.empty()) {
      return;
    }

    CapabilityHealth & health = capability_health_[runtime.assigned_robot];
    auto it = health.failures_by_type.find(runtime.definition.task_type);
    if (it == health.failures_by_type.end()) {
      return;
    }

    const int failures = std::max(0, it->second - 1);
    it->second = failures;
    if (failures < settings_.failure_degrade_threshold) {
      health.degraded_types.erase(runtime.definition.task_type);
    }
  }

  void record_failure(TaskRuntime & runtime)
  {
    if (runtime.assigned_robot.empty()) {
      return;
    }

    CapabilityHealth & health = capability_health_[runtime.assigned_robot];
    int failures = health.failures_by_type[runtime.definition.task_type] + 1;
    health.failures_by_type[runtime.definition.task_type] = failures;

    if (failures >= settings_.failure_degrade_threshold) {
      health.degraded_types.insert(runtime.definition.task_type);
      RCLCPP_WARN(
        this->get_logger(),
        "Degraded capability: robot=%s type=%s (failures=%d)",
        runtime.assigned_robot.c_str(),
        runtime.definition.task_type.c_str(),
        failures);
    }
  }

  void boost_duck_clearance(double now)
  {
    for (const auto & [_, runtime] : tasks_) {
      if (
        runtime.definition.task_type == "CLEAR_BLOCKING_DUCK" ||
        runtime.definition.task_type == "COLLECT_DUCK")
      {
        double boost = runtime.definition.priority.opportunistic_boost;
        if (boost <= 0.0) {
          boost = 50.0;
        }
        dynamic_priority_boost_[runtime.definition.task_id] = {boost, now + 30.0};
      }
    }
  }

  void publish_status(double now)
  {
    std::map<std::string, int> counts;
    for (const auto & [_, runtime] : tasks_) {
      counts[lifecycle_to_string(runtime.state)] += 1;
    }

    json robot_status = json::object();
    for (const auto & [name, robot] : world_.robots) {
      robot_status[name] = {
        {"online", robot.online},
        {"map", robot.map_name},
        {"soc", robot.battery_valid ? json(robot.battery_soc) : json(nullptr)},
        {"faults", robot.fault_flags},
      };
    }

    json payload = {
      {"phase", phase_to_string(phase_)},
      {"elapsed_match_sec", std::round(world_.elapsed_match_time(now) * 100.0) / 100.0},
      {"tasks", counts},
      {"locks", locks_},
      {"robots", robot_status},
      {"safe_hold_reason", safe_hold_reason_},
    };

    std_msgs::msg::String msg;
    msg.data = payload.dump();
    status_pub_->publish(msg);
  }

  void publish_final_report_if_needed(double now)
  {
    const std::string phase_text = phase_to_string(phase_);
    if (last_final_report_phase_ == phase_text) {
      return;
    }

    json antennas = json::object();
    for (const auto & [antenna_id, state] : world_.antennas) {
      antennas[antenna_id] = antenna_state_to_string(state);
    }

    json ducks = json::object();
    for (const auto & [duck_id, duck] : world_.ducks) {
      ducks[duck_id] = duck_state_to_string(duck.state);
    }

    json drones = json::object();
    for (const auto & [drone_name, drone_state] : world_.drone_states) {
      drones[drone_name] = drone_state_to_string(drone_state);
    }

    json task_results = json::object();
    for (const auto & [task_id, runtime] : tasks_) {
      task_results[task_id] = {
        {"state", lifecycle_to_string(runtime.state)},
        {"attempts", runtime.attempts},
        {"last_error", runtime.last_error},
        {"rmf_task_id", runtime.rmf_task_id},
      };
    }

    json payload = {
      {"timestamp_unix_s", now},
      {"phase", phase_text},
      {"antennas", antennas},
      {"ducks", ducks},
      {
        "crater",
        {
          {"lap_in_progress", world_.crater.lap_in_progress},
          {"entry_clear", world_.crater.entry_clear},
          {"exit_clear", world_.crater.exit_clear},
          {"paired_robots", world_.crater.paired_robots},
        },
      },
      {"drone", drones},
      {"task_results", task_results},
    };

    std_msgs::msg::String msg;
    msg.data = payload.dump();
    final_report_pub_->publish(msg);

    last_final_report_phase_ = phase_text;
    if (phase_ == MissionPhase::COMPLETE) {
      final_report_published_ = true;
    }

    if (!final_report_published_) {
      RCLCPP_INFO(this->get_logger(), "Published final mission report");
    }
  }

  void on_robot_state(const ieee_fleet_msgs::msg::RobotState::SharedPtr msg)
  {
    world_.update_robot_state(*msg, now_unix_s());
  }

  void on_dispatch_states(const rmf_task_msgs::msg::DispatchStates::SharedPtr msg)
  {
    dispatch_tracker_.on_dispatch_states(*msg, now_unix_s());
  }

  void on_task_summary(const rmf_task_msgs::msg::TaskSummary::SharedPtr msg)
  {
    dispatch_tracker_.on_task_summary(*msg, now_unix_s());
  }

  void on_external_event(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string event = world_.apply_external_event(msg->data);
    if (event == "invalid_json") {
      RCLCPP_WARN(this->get_logger(), "Ignored malformed /game_director/events payload");
    }
  }

  void on_start_game_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (phase_ == MissionPhase::READY) {
      start_requested_ = true;
      response->success = true;
      response->message = "accepted: GAME_START requested";
      return;
    }

    if (phase_ == MissionPhase::GAME_START) {
      response->success = true;
      response->message = "already in GAME_START";
      return;
    }

    response->success = false;
    response->message = "cannot start from phase=" + phase_to_string(phase_) + "; wait for READY";
  }

  double now_unix_s() const
  {
    return static_cast<double>(this->get_clock()->now().nanoseconds()) / 1e9;
  }

  TaskPoolSpec spec_;
  DirectorSettings settings_;
  WorldModel world_;
  MissionPhase phase_ = MissionPhase::BOOT;

  std::set<std::string> boot_validation_requirements_;

  std::map<std::string, TaskRuntime> tasks_;
  std::map<std::string, std::string> game_to_rmf_;
  std::map<std::string, std::string> rmf_to_game_;
  std::map<std::string, std::string> locks_;

  DispatchTracker dispatch_tracker_;
  std::unique_ptr<DispatchClient> dispatch_client_;

  std::unordered_map<std::string, CapabilityHealth> capability_health_;
  std::unordered_map<std::string, std::pair<double, double>> dynamic_priority_boost_;

  bool final_report_published_ = false;
  std::string last_final_report_phase_;
  bool start_requested_ = false;

  double boot_started_unix_s_ = 0.0;
  std::string safe_hold_reason_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr final_report_pub_;

  rclcpp::Subscription<ieee_fleet_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<rmf_task_msgs::msg::DispatchStates>::SharedPtr dispatch_states_sub_;
  rclcpp::Subscription<rmf_task_msgs::msg::TaskSummary>::SharedPtr task_summary_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace game_director

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<game_director::GameDirector>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "game_director failed: %s\n", exc.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
