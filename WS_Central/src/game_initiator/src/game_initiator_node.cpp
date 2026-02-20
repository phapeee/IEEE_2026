#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <functional>
#include <limits.h>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

double monotonic_seconds()
{
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

std::string join_args(const std::vector<std::string> & args)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < args.size(); ++i) {
    if (i > 0) {
      oss << ' ';
    }
    oss << args[i];
  }
  return oss.str();
}

std::string trimmed(const std::string & text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::filesystem::path expand_user_path(const std::string & raw)
{
  if (raw.empty() || raw[0] != '~') {
    return std::filesystem::path(raw);
  }

  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return std::filesystem::path(raw);
  }

  if (raw.size() == 1) {
    return std::filesystem::path(home);
  }

  if (raw[1] == '/') {
    return std::filesystem::path(home) / raw.substr(2);
  }

  return std::filesystem::path(raw);
}

std::optional<std::filesystem::path> current_executable_path()
{
  std::array<char, PATH_MAX> buffer{};
  const ssize_t count = ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
  if (count <= 0) {
    return std::nullopt;
  }

  buffer[static_cast<std::size_t>(count)] = '\0';
  return std::filesystem::path(buffer.data());
}

struct CommandResult
{
  int exit_code{0};
  std::string stdout_text;
  std::string stderr_text;
};

void set_non_blocking(int fd)
{
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags == -1) {
    throw std::runtime_error("fcntl(F_GETFL) failed: " + std::string(std::strerror(errno)));
  }
  if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
    throw std::runtime_error("fcntl(F_SETFL) failed: " + std::string(std::strerror(errno)));
  }
}

void read_available(int fd, std::string & buffer, bool & open)
{
  std::array<char, 512> chunk{};
  while (open) {
    const ssize_t bytes = ::read(fd, chunk.data(), chunk.size());
    if (bytes > 0) {
      buffer.append(chunk.data(), static_cast<std::size_t>(bytes));
      continue;
    }
    if (bytes == 0) {
      open = false;
      ::close(fd);
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    throw std::runtime_error("read() failed: " + std::string(std::strerror(errno)));
  }
}

CommandResult run_command(const std::vector<std::string> & args, double timeout_seconds)
{
  if (args.empty()) {
    throw std::runtime_error("Cannot execute an empty command.");
  }

  int stdout_pipe[2] = {-1, -1};
  int stderr_pipe[2] = {-1, -1};
  if (::pipe(stdout_pipe) != 0 || ::pipe(stderr_pipe) != 0) {
    if (stdout_pipe[0] != -1) {
      ::close(stdout_pipe[0]);
      ::close(stdout_pipe[1]);
    }
    if (stderr_pipe[0] != -1) {
      ::close(stderr_pipe[0]);
      ::close(stderr_pipe[1]);
    }
    throw std::runtime_error("pipe() failed: " + std::string(std::strerror(errno)));
  }

  const pid_t pid = ::fork();
  if (pid < 0) {
    ::close(stdout_pipe[0]);
    ::close(stdout_pipe[1]);
    ::close(stderr_pipe[0]);
    ::close(stderr_pipe[1]);
    throw std::runtime_error("fork() failed: " + std::string(std::strerror(errno)));
  }

  if (pid == 0) {
    ::dup2(stdout_pipe[1], STDOUT_FILENO);
    ::dup2(stderr_pipe[1], STDERR_FILENO);

    ::close(stdout_pipe[0]);
    ::close(stdout_pipe[1]);
    ::close(stderr_pipe[0]);
    ::close(stderr_pipe[1]);

    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (const auto & arg : args) {
      argv.push_back(const_cast<char *>(arg.c_str()));
    }
    argv.push_back(nullptr);
    ::execvp(argv[0], argv.data());

    const std::string error_msg =
      "execvp failed for '" + args[0] + "': " + std::string(std::strerror(errno)) + "\n";
    (void)::write(STDERR_FILENO, error_msg.c_str(), error_msg.size());
    _exit(127);
  }

  ::close(stdout_pipe[1]);
  ::close(stderr_pipe[1]);

  set_non_blocking(stdout_pipe[0]);
  set_non_blocking(stderr_pipe[0]);

  CommandResult result;
  bool stdout_open = true;
  bool stderr_open = true;
  bool child_exited = false;
  int status = 0;
  const auto timeout = std::chrono::duration<double>(std::max(timeout_seconds, 0.01));
  const auto deadline = Clock::now() + timeout;

  while (stdout_open || stderr_open || !child_exited) {
    std::array<pollfd, 2> pollfds{};
    nfds_t fds_count = 0;
    if (stdout_open) {
      pollfds[fds_count++] = pollfd{stdout_pipe[0], POLLIN | POLLHUP, 0};
    }
    if (stderr_open) {
      pollfds[fds_count++] = pollfd{stderr_pipe[0], POLLIN | POLLHUP, 0};
    }

    if (fds_count > 0) {
      const int poll_result = ::poll(pollfds.data(), fds_count, 20);
      if (poll_result < 0 && errno != EINTR) {
        ::kill(pid, SIGKILL);
        (void)::waitpid(pid, nullptr, 0);
        throw std::runtime_error("poll() failed: " + std::string(std::strerror(errno)));
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (stdout_open) {
      read_available(stdout_pipe[0], result.stdout_text, stdout_open);
    }
    if (stderr_open) {
      read_available(stderr_pipe[0], result.stderr_text, stderr_open);
    }

    if (!child_exited) {
      const pid_t wait_result = ::waitpid(pid, &status, WNOHANG);
      if (wait_result == pid) {
        child_exited = true;
      } else if (wait_result < 0) {
        ::kill(pid, SIGKILL);
        (void)::waitpid(pid, nullptr, 0);
        throw std::runtime_error("waitpid() failed: " + std::string(std::strerror(errno)));
      }
    }

    if (Clock::now() > deadline && !child_exited) {
      ::kill(pid, SIGKILL);
      (void)::waitpid(pid, &status, 0);
      if (stdout_open) {
        read_available(stdout_pipe[0], result.stdout_text, stdout_open);
      }
      if (stderr_open) {
        read_available(stderr_pipe[0], result.stderr_text, stderr_open);
      }
      throw std::runtime_error(
              "Command timed out after " + std::to_string(timeout_seconds) + "s: " + join_args(args));
    }
  }

  if (WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    result.exit_code = 128 + WTERMSIG(status);
  } else {
    result.exit_code = -1;
  }

  return result;
}

std::string line_list_string(const std::vector<int> & lines)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    oss << lines[i];
  }
  return oss.str();
}

}  // namespace

struct InputState
{
  std::string name;
  int gpio_line{0};
  bool active_low{false};
  bool pullup{false};
  std::string trigger_on{"press"};
  std::optional<bool> stable_state;
  std::optional<bool> pending_state;
  double pending_since{0.0};
};

class GameInitiatorNode : public rclcpp::Node
{
public:
  GameInitiatorNode()
  : Node("game_initiator")
  {
    const std::string default_chip =
      std::filesystem::exists("/dev/gpiochip4") ? "gpiochip4" : "gpiochip0";

    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", false);
    }

    backend_ = this->declare_parameter<std::string>("backend", "gpioget");
    gpio_chip_ = this->declare_parameter<std::string>("gpio_chip", default_chip);
    default_gpio_line_ = static_cast<int>(this->declare_parameter<int64_t>("gpio_line", 4));
    default_active_low_ = this->declare_parameter<bool>("active_low", true);
    default_pullup_ = this->declare_parameter<bool>("use_internal_pullup", false);
    default_trigger_on_ = this->declare_parameter<std::string>("trigger_on", "press");

    polling_frequency_hz_ = this->declare_parameter<double>("polling_frequency_hz", 50.0);
    const double debounce_ms = this->declare_parameter<double>("debounce_duration_ms", 30.0);
    debounce_duration_sec_ = std::max(debounce_ms / 1000.0, 0.0);
    gpioget_timeout_sec_ = std::max(this->declare_parameter<double>("gpioget_timeout_sec", 0.2), 0.01);

    start_service_name_ =
      this->declare_parameter<std::string>("start_service_name", "/game_director/start_game");
    service_wait_timeout_sec_ =
      std::max(this->declare_parameter<double>("service_wait_timeout_sec", 1.0), 0.0);
    service_call_cooldown_sec_ =
      std::max(this->declare_parameter<double>("service_call_cooldown_sec", 1.0), 0.0);
    call_once_after_success_ = this->declare_parameter<bool>("call_once_after_success", true);
    shutdown_on_success_ = this->declare_parameter<bool>("shutdown_on_success", false);

    prime_pullups_ = this->declare_parameter<bool>("prime_pullups", true);
    prime_pullups_script_ =
      this->declare_parameter<std::string>("prime_pullups_script", "scripts/prime_gpio_pullups.sh");

    const auto input_names =
      this->declare_parameter<std::vector<std::string>>("inputs", std::vector<std::string>{});
    inputs_ = configure_inputs(input_names);
    if (inputs_.empty()) {
      throw std::runtime_error("No GPIO inputs configured.");
    }

    start_client_ = this->create_client<std_srvs::srv::Trigger>(start_service_name_);
    prime_pullup_lines_if_requested();

    const double period = polling_frequency_hz_ > 0.0 ? 1.0 / polling_frequency_hz_ : 0.02;
    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period));
    timer_ = this->create_wall_timer(timer_period, std::bind(&GameInitiatorNode::poll_inputs, this));

    RCLCPP_INFO(
      this->get_logger(),
      "game_initiator started: backend=%s chip=%s inputs=%zu poll=%.3fs service=%s shutdown_on_success=%s",
      backend_.c_str(),
      gpio_chip_.c_str(),
      inputs_.size(),
      period,
      start_service_name_.c_str(),
      shutdown_on_success_ ? "true" : "false");

    for (const auto & [name, state] : inputs_) {
      (void)name;
      RCLCPP_INFO(
        this->get_logger(),
        "Input '%s': line=%d active_low=%s pullup=%s trigger_on=%s",
        state.name.c_str(),
        state.gpio_line,
        state.active_low ? "true" : "false",
        state.pullup ? "true" : "false",
        state.trigger_on.c_str());
    }
  }

private:
  std::map<std::string, InputState> configure_inputs(const std::vector<std::string> & input_names)
  {
    std::map<std::string, InputState> states;

    if (input_names.empty()) {
      InputState state;
      state.name = "default";
      state.gpio_line = default_gpio_line_;
      state.active_low = default_active_low_;
      state.pullup = default_pullup_;
      state.trigger_on = normalize_trigger_on(default_trigger_on_, "default");
      states.emplace(
        "default",
        state);
      return states;
    }

    for (const auto & name : input_names) {
      const std::string prefix = "inputs." + name;
      const int gpio_line = static_cast<int>(
        this->declare_parameter<int64_t>(prefix + ".gpio_line", default_gpio_line_));
      const bool active_low = this->declare_parameter<bool>(prefix + ".active_low", default_active_low_);
      const bool pullup = this->declare_parameter<bool>(prefix + ".pullup", default_pullup_);
      const std::string trigger_on = normalize_trigger_on(
        this->declare_parameter<std::string>(prefix + ".trigger_on", default_trigger_on_),
        name);

      InputState state;
      state.name = name;
      state.gpio_line = gpio_line;
      state.active_low = active_low;
      state.pullup = pullup;
      state.trigger_on = trigger_on;
      states.emplace(name, state);
    }

    return states;
  }

  std::string normalize_trigger_on(const std::string & trigger_on, const std::string & input_name)
  {
    static const std::map<std::string, std::string> mapping = {
      {"press", "press"},
      {"pressed", "press"},
      {"active", "press"},
      {"rise", "press"},
      {"rising", "press"},
      {"release", "release"},
      {"released", "release"},
      {"inactive", "release"},
      {"fall", "release"},
      {"falling", "release"},
      {"both", "both"},
      {"any", "both"},
    };

    const auto normalized = to_lower(trimmed(trigger_on));
    const auto it = mapping.find(normalized);
    if (it != mapping.end()) {
      return it->second;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Input '%s' has unsupported trigger_on='%s'; defaulting to 'press'",
      input_name.c_str(),
      trigger_on.c_str());
    return "press";
  }

  void poll_inputs()
  {
    for (auto & [name, state] : inputs_) {
      (void)name;
      try {
        const bool active_now = read_gpio_active(state);
        apply_debounce_and_process(state, active_now);
      } catch (const std::exception & exc) {
        log_read_error_throttled(state.name, state.gpio_line, exc.what());
      }
    }
  }

  bool read_gpio_active(const InputState & state)
  {
    const std::string backend = to_lower(trimmed(backend_));
    if (backend != "gpioget" && backend != "auto") {
      throw std::runtime_error("Unsupported backend '" + backend_ + "'");
    }

    std::vector<std::string> cmd{"gpioget"};
    if (state.pullup) {
      cmd.emplace_back("--bias=pull-up");
    }
    cmd.emplace_back(gpio_chip_);
    cmd.emplace_back(std::to_string(state.gpio_line));

    const auto result = run_command(cmd, gpioget_timeout_sec_);
    if (result.exit_code != 0) {
      std::string details = trimmed(result.stderr_text);
      if (details.empty()) {
        details = trimmed(result.stdout_text);
      }
      throw std::runtime_error(
              "gpioget failed (exit " + std::to_string(result.exit_code) + "): " + details);
    }

    const auto raw_level = parse_gpioget_stdout(result.stdout_text);
    if (!raw_level.has_value()) {
      throw std::runtime_error("Unexpected gpioget output: '" + trimmed(result.stdout_text) + "'");
    }

    return state.active_low ? !raw_level.value() : raw_level.value();
  }

  static std::optional<bool> parse_gpioget_stdout(const std::string & stdout_text)
  {
    std::istringstream stream(stdout_text);
    std::string token;
    if (!(stream >> token)) {
      return std::nullopt;
    }

    const auto value = to_lower(trimmed(token));
    if (value == "1" || value == "true" || value == "high" || value == "active") {
      return true;
    }
    if (value == "0" || value == "false" || value == "low" || value == "inactive") {
      return false;
    }
    return std::nullopt;
  }

  void apply_debounce_and_process(InputState & state, bool active_now)
  {
    const double now = monotonic_seconds();

    if (!state.pending_state.has_value() || state.pending_state.value() != active_now) {
      state.pending_state = active_now;
      state.pending_since = now;
      return;
    }

    if (now - state.pending_since < debounce_duration_sec_) {
      return;
    }

    if (!state.stable_state.has_value()) {
      state.stable_state = active_now;
      return;
    }

    if (state.stable_state.value() == active_now) {
      return;
    }

    const bool previous = state.stable_state.value();
    state.stable_state = active_now;
    RCLCPP_INFO(
      this->get_logger(),
      "Input '%s' transitioned %s -> %s",
      state.name.c_str(),
      previous ? "active" : "inactive",
      active_now ? "active" : "inactive");

    if (is_trigger_event(state.trigger_on, previous, active_now)) {
      request_start_game(state.name, previous, active_now);
    }
  }

  static bool is_trigger_event(const std::string & trigger_on, bool previous, bool current)
  {
    if (trigger_on == "both") {
      return previous != current;
    }
    if (trigger_on == "release") {
      return previous && !current;
    }
    return !previous && current;
  }

  void request_start_game(const std::string & source, bool previous, bool current)
  {
    if (call_once_after_success_ && success_latched_) {
      return;
    }

    if (request_in_flight_) {
      return;
    }

    const double now = monotonic_seconds();
    if (now - last_service_call_time_ < service_call_cooldown_sec_) {
      return;
    }

    if (!start_client_->service_is_ready()) {
      const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(service_wait_timeout_sec_));
      const bool ready = start_client_->wait_for_service(timeout);
      if (!ready) {
        RCLCPP_WARN(
          this->get_logger(),
          "Input '%s' triggered (%s -> %s) but service %s is unavailable",
          source.c_str(),
          previous ? "active" : "inactive",
          current ? "active" : "inactive",
          start_service_name_.c_str());
        return;
      }
    }

    request_in_flight_ = true;
    last_service_call_time_ = now;

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_client_->async_send_request(
      request,
      [this, source](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        this->on_start_response(source, future);
      });

    RCLCPP_INFO(
      this->get_logger(), "Input '%s' triggered; calling %s", source.c_str(), start_service_name_.c_str());
  }

  void on_start_response(
    const std::string & source,
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
  {
    request_in_flight_ = false;

    std_srvs::srv::Trigger::Response::SharedPtr response;
    try {
      response = future.get();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(
        this->get_logger(), "start_game call from input '%s' failed: %s", source.c_str(), exc.what());
      return;
    }

    if (!response) {
      RCLCPP_ERROR(
        this->get_logger(), "start_game call from input '%s' returned no response", source.c_str());
      return;
    }

    if (response->success) {
      RCLCPP_INFO(
        this->get_logger(),
        "start_game accepted from input '%s': %s",
        source.c_str(),
        response->message.c_str());

      if (call_once_after_success_) {
        success_latched_ = true;
      }

      if (shutdown_on_success_) {
        RCLCPP_INFO(this->get_logger(), "shutdown_on_success=true; shutting down game_initiator node.");
        if (timer_) {
          timer_->cancel();
        }
        rclcpp::shutdown();
      }
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "start_game rejected from input '%s': %s",
      source.c_str(),
      response->message.c_str());
  }

  void prime_pullup_lines_if_requested()
  {
    if (!prime_pullups_) {
      return;
    }

    std::set<int> line_set;
    for (const auto & [name, state] : inputs_) {
      (void)name;
      if (state.pullup) {
        line_set.insert(state.gpio_line);
      }
    }

    if (line_set.empty()) {
      return;
    }

    std::vector<int> pullup_lines(line_set.begin(), line_set.end());

    const auto script_path = resolve_existing_path(prime_pullups_script_);
    if (!script_path.has_value()) {
      RCLCPP_WARN(
        this->get_logger(),
        "prime_pullups enabled but script '%s' not found; using inline gpioget priming",
        prime_pullups_script_.c_str());
      prime_with_gpioget(pullup_lines);
      return;
    }

    std::vector<std::string> cmd{"bash", script_path->string(), gpio_chip_};
    for (const int line : pullup_lines) {
      cmd.emplace_back(std::to_string(line));
    }

    try {
      const auto result = run_command(cmd, 5.0);
      if (result.exit_code != 0) {
        std::string details = trimmed(result.stderr_text);
        if (details.empty()) {
          details = trimmed(result.stdout_text);
        }
        throw std::runtime_error(
                "script exited with code " + std::to_string(result.exit_code) + ": " + details);
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Primed GPIO pull-ups via %s for chip=%s lines=%s",
        script_path->c_str(),
        gpio_chip_.c_str(),
        line_list_string(pullup_lines).c_str());
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to run pull-up priming script (%s). Falling back to inline priming.",
        exc.what());
      prime_with_gpioget(pullup_lines);
    }
  }

  void prime_with_gpioget(const std::vector<int> & pullup_lines)
  {
    std::vector<int> primed_lines;
    const double timeout = std::max(gpioget_timeout_sec_, 0.5);

    for (const int line : pullup_lines) {
      std::vector<std::string> cmd{
        "gpioget", "--bias=pull-up", gpio_chip_, std::to_string(line)};
      try {
        const auto result = run_command(cmd, timeout);
        if (result.exit_code != 0) {
          std::string details = trimmed(result.stderr_text);
          if (details.empty()) {
            details = trimmed(result.stdout_text);
          }
          throw std::runtime_error(
                  "gpioget exited with code " + std::to_string(result.exit_code) + ": " + details);
        }
        primed_lines.push_back(line);
      } catch (const std::exception & exc) {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed to prime pull-up for line %d on %s: %s",
          line,
          gpio_chip_.c_str(),
          exc.what());
      }
    }

    if (!primed_lines.empty()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Primed GPIO pull-ups with gpioget for chip=%s lines=%s",
        gpio_chip_.c_str(),
        line_list_string(primed_lines).c_str());
    }
  }

  std::optional<std::filesystem::path> resolve_existing_path(const std::string & path_text) const
  {
    if (path_text.empty()) {
      return std::nullopt;
    }

    const auto raw = expand_user_path(path_text);
    std::vector<std::filesystem::path> candidates;
    candidates.push_back(raw);

    if (!raw.is_absolute()) {
      candidates.push_back(std::filesystem::current_path() / raw);
    }

    if (const auto exe = current_executable_path()) {
      std::filesystem::path parent = exe->parent_path();
      while (!parent.empty()) {
        candidates.push_back(parent / raw);
        candidates.push_back(parent / "scripts" / "prime_gpio_pullups.sh");
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

    try {
      const auto share_dir =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("game_initiator"));
      candidates.push_back(share_dir / raw);
      candidates.push_back(share_dir / "scripts" / "prime_gpio_pullups.sh");
    } catch (const std::exception &) {
      // Ignore lookup failure; other candidates may still work.
    }

    std::set<std::string> seen;
    for (const auto & candidate : candidates) {
      if (candidate.empty()) {
        continue;
      }
      const auto normalized = candidate.lexically_normal();
      const auto key = normalized.string();
      if (!seen.insert(key).second) {
        continue;
      }
      std::error_code ec;
      if (std::filesystem::is_regular_file(normalized, ec) && !ec) {
        return normalized;
      }
    }

    return std::nullopt;
  }

  void log_read_error_throttled(const std::string & name, int line, const std::string & error_text)
  {
    const double now = monotonic_seconds();
    const auto it = last_read_error_time_.find(name);
    const double last_time = it == last_read_error_time_.end() ? 0.0 : it->second;
    if (now - last_time < 2.0) {
      return;
    }

    last_read_error_time_[name] = now;
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed reading input '%s' (line %d on %s): %s",
      name.c_str(),
      line,
      gpio_chip_.c_str(),
      error_text.c_str());
  }

  std::string backend_;
  std::string gpio_chip_;
  int default_gpio_line_{4};
  bool default_active_low_{true};
  bool default_pullup_{false};
  std::string default_trigger_on_{"press"};
  double polling_frequency_hz_{50.0};
  double debounce_duration_sec_{0.03};
  double gpioget_timeout_sec_{0.2};
  std::string start_service_name_{"/game_director/start_game"};
  double service_wait_timeout_sec_{1.0};
  double service_call_cooldown_sec_{1.0};
  bool call_once_after_success_{true};
  bool shutdown_on_success_{false};
  bool prime_pullups_{true};
  std::string prime_pullups_script_{"scripts/prime_gpio_pullups.sh"};

  std::map<std::string, InputState> inputs_;
  std::map<std::string, double> last_read_error_time_;
  bool request_in_flight_{false};
  bool success_latched_{false};
  double last_service_call_time_{0.0};

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<GameInitiatorNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "game_initiator failed: %s\n", exc.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
