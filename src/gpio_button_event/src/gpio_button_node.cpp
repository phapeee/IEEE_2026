#include <chrono>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <algorithm>
#include <filesystem>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <smacc2_msgs/msg/smacc_event.hpp>

#ifdef HAVE_LIBGPIOD
extern "C" {
#include <gpiod.h>
}
#endif

namespace gpio_button_event
{
class BaseGPIOReader
{
public:
  virtual ~BaseGPIOReader() = default;
  virtual bool read() = 0;
  virtual void close() {}
};

class GpiodReader : public BaseGPIOReader
{
public:
  GpiodReader(
    const std::string & chip_name, int line, bool active_low, bool use_pull_up, rclcpp::Logger logger)
  : logger_(logger)
  {
    chip_ = gpiod_chip_open_by_name(chip_name.c_str());
    if (!chip_) {
      throw std::runtime_error("Unable to open gpiod chip " + chip_name);
    }

    line_ = gpiod_chip_get_line(chip_, line);
    if (!line_) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
      std::ostringstream oss;
      oss << "Unable to get line " << line << " on chip " << chip_name;
      throw std::runtime_error(oss.str());
    }

    unsigned int flags = 0U;
#ifdef GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW
    if (active_low) {
      flags |= GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW;
      invert_state_ = false;
    } else {
      invert_state_ = false;
    }
#else
    invert_state_ = active_low;
#endif

#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
    if (use_pull_up) {
      flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    }
#else
    if (use_pull_up) {
      RCLCPP_WARN(
        logger_,
        "libgpiod does not expose internal pull-up configuration; external resistor required");
    }
#endif

    const int ret = gpiod_line_request_input_flags(line_, "smacc2_gpio_button", flags);
    if (ret < 0) {
      gpiod_line_release(line_);
      gpiod_chip_close(chip_);
      chip_ = nullptr;
      line_ = nullptr;
      throw std::runtime_error("Failed to request gpiod line");
    }

    RCLCPP_INFO(
      logger_,
      "Initialized gpiod backend on %s line %d (active_low=%s pull_up=%s)", chip_name.c_str(),
      line, active_low ? "true" : "false", use_pull_up ? "true" : "false");
  }

  ~GpiodReader() override
  {
    close();
  }

  bool read() override
  {
    const int value = gpiod_line_get_value(line_);
    if (value < 0) {
      throw std::runtime_error("Failed to read gpiod line");
    }
    const bool logical = value == 1;
    return invert_state_ ? !logical : logical;
  }

  void close() override
  {
    if (line_) {
      gpiod_line_release(line_);
      line_ = nullptr;
    }
    if (chip_) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
  }

private:
  rclcpp::Logger logger_;
  bool invert_state_{false};
  gpiod_chip * chip_{nullptr};
  gpiod_line * line_{nullptr};
};

struct SwitchHandle
{
  std::string name;
  int gpio_line;
  bool pull_up;
  bool active_low;
  std::string event_object_tag;
  std::string pressed_event_type;
  std::string released_event_type;
  std::string state_topic;
  std::unique_ptr<BaseGPIOReader> reader;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub;
  std::optional<bool> logical_state;
  std::optional<bool> pending_state;
  std::optional<std::chrono::steady_clock::time_point> pending_since;
};

class SmaccGpioButtonNode : public rclcpp::Node
{
public:
  SmaccGpioButtonNode()
  : rclcpp::Node("smacc_gpio_button_node")
  {
    const std::string default_gpio_chip =
      std::filesystem::exists("/dev/gpiochip4") ? "gpiochip4" : "gpiochip0";
    backend_ = declare_parameter<std::string>("backend", "gpiod");
    gpio_chip_ = declare_parameter<std::string>("gpio_chip", default_gpio_chip);
    gpio_line_ = declare_parameter<int>("gpio_line", 4);
    active_low_ = declare_parameter<bool>("active_low", false);
    default_pull_up_ = declare_parameter<bool>("use_internal_pullup", false);
    polling_frequency_hz_ = declare_parameter<double>("polling_frequency_hz", 50.0);
    debounce_duration_ms_ = declare_parameter<double>("debounce_duration_ms", 30.0);
    smacc_event_topic_ = declare_parameter<std::string>("smacc_event_topic", "smacc2/button_event");
    button_state_topic_ = declare_parameter<std::string>("button_state_topic", "smacc2/button_state");
    event_object_tag_ = declare_parameter<std::string>("event_object_tag", "GPIO4Button");
    event_source_ = declare_parameter<std::string>("event_source", "gpio_button_event_node");
    pressed_event_type_ = declare_parameter<std::string>("pressed_event_type", "BUTTON_PRESSED");
    released_event_type_ = declare_parameter<std::string>("released_event_type", "BUTTON_RELEASED");

    debounce_duration_ = std::max(debounce_duration_ms_ / 1000.0, 0.0);

    auto switch_names =
      declare_parameter<std::vector<std::string>>("switches", std::vector<std::string>{});
    configure_switches(switch_names);

    event_pub_ = create_publisher<smacc2_msgs::msg::SmaccEvent>(smacc_event_topic_, 10);

    const double period = polling_frequency_hz_ > 0.0 ? (1.0 / polling_frequency_hz_) : 0.02;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), [this]() {
        this->poll_gpio();
      });

    RCLCPP_INFO(
      get_logger(),
      "gpio_button_event node started: backend=%s chip=%s switches=%zu timer=%.3fs",
      backend_.c_str(), gpio_chip_.c_str(), switches_.size(), period);
  }

  ~SmaccGpioButtonNode() override
  {
    if (timer_) {
      timer_->cancel();
    }
    for (auto & handle : switches_) {
      if (handle.reader) {
        handle.reader->close();
      }
    }
  }

private:
  void configure_switches(const std::vector<std::string> & switch_names)
  {
    if (switch_names.empty()) {
      SwitchHandle handle;
      handle.name = event_object_tag_;
      handle.gpio_line = gpio_line_;
      handle.active_low = active_low_;
      handle.pull_up = default_pull_up_;
      handle.event_object_tag = event_object_tag_;
      handle.pressed_event_type = pressed_event_type_;
      handle.released_event_type = released_event_type_;
      handle.state_topic = button_state_topic_;
      handle.reader = create_reader(backend_, handle.gpio_line, handle.active_low, handle.pull_up);
      if (!handle.state_topic.empty()) {
        handle.state_pub = create_publisher<std_msgs::msg::Bool>(handle.state_topic, 10);
      }
      switches_.push_back(std::move(handle));
      log_switch_configuration(switches_.back());
      return;
    }

    switches_.reserve(switch_names.size());
    for (size_t i = 0; i < switch_names.size(); ++i) {
      const std::string & name = switch_names[i];
      const std::string prefix = "switches." + name;

      SwitchHandle handle;
      handle.name = name;
      handle.gpio_line = declare_parameter<int>(prefix + ".gpio_line", gpio_line_);
      handle.active_low = declare_parameter<bool>(prefix + ".active_low", active_low_);
      handle.pull_up = declare_parameter<bool>(prefix + ".pullup", default_pull_up_);
      handle.event_object_tag =
        declare_parameter<std::string>(prefix + ".event_object_tag", name);
      handle.pressed_event_type = declare_parameter<std::string>(
        prefix + ".pressed_event_type", name + "_pressed");
      handle.released_event_type = declare_parameter<std::string>(
        prefix + ".released_event_type", name + "_released");
      const std::string default_state_topic = button_state_topic_.empty() ? std::string{} :
        (i == 0 ? button_state_topic_ : button_state_topic_ + "/" + name);
      handle.state_topic = declare_parameter<std::string>(prefix + ".state_topic", default_state_topic);
      handle.reader = create_reader(backend_, handle.gpio_line, handle.active_low, handle.pull_up);
      if (!handle.state_topic.empty()) {
        handle.state_pub = create_publisher<std_msgs::msg::Bool>(handle.state_topic, 10);
      }
      switches_.push_back(std::move(handle));
      log_switch_configuration(switches_.back());
    }
  }

  std::unique_ptr<BaseGPIOReader> create_reader(
    const std::string & backend, int line, bool active_low, bool pull_up)
  {
    std::string normalized = backend;
    std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (normalized != "auto" && normalized != "gpiod" && normalized != "sysfs") {
      RCLCPP_WARN(get_logger(), "Unsupported backend '%s', defaulting to auto", backend.c_str());
      normalized = "auto";
    }

    if (normalized == "sysfs") {
      RCLCPP_WARN(
        get_logger(), "sysfs backend is no longer supported; forcing libgpiod backend instead");
      normalized = "gpiod";
    }

    (void)normalized;
    return std::make_unique<GpiodReader>(gpio_chip_, line, active_low, pull_up, get_logger());
  }

  void poll_gpio()
  {
    for (auto & handle : switches_) {
      bool raw_state = false;
      try {
        raw_state = handle.reader->read();
      } catch (const std::exception & ex) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to read GPIO line %d (%s): %s",
          handle.gpio_line, handle.event_object_tag.c_str(), ex.what());
        continue;
      }
      handle_switch_state(handle, raw_state);
    }
  }

  void handle_switch_state(SwitchHandle & handle, bool raw_state)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!handle.pending_state.has_value() || handle.pending_state.value() != raw_state) {
      handle.pending_state = raw_state;
      handle.pending_since = now;
      return;
    }

    if (handle.logical_state.has_value() && handle.logical_state.value() == raw_state) {
      publish_state_only(handle, raw_state);
      return;
    }

    if (
      debounce_duration_ == 0.0 ||
      (handle.pending_since.has_value() &&
      std::chrono::duration<double>(now - handle.pending_since.value()).count() >= debounce_duration_))
    {
      handle.logical_state = raw_state;
      publish_transition(handle, raw_state);
    }
  }

  void publish_state_only(SwitchHandle & handle, bool state)
  {
    if (!handle.state_pub) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = state;
    handle.state_pub->publish(msg);
  }

  void publish_transition(SwitchHandle & handle, bool state)
  {
    publish_state_only(handle, state);

    smacc2_msgs::msg::SmaccEvent event;
    event.event_type = state ? handle.pressed_event_type : handle.released_event_type;
    event.event_object_tag = handle.event_object_tag;
    event.event_source = event_source_;
    event.label = handle.event_object_tag + std::string(":") + (state ? "pressed" : "released");

    event_pub_->publish(event);
    RCLCPP_INFO(
      get_logger(), "Switch '%s' %s; emitted %s", handle.event_object_tag.c_str(),
      state ? "pressed" : "released", event.event_type.c_str());
  }

  void log_switch_configuration(const SwitchHandle & handle)
  {
    const char * state_topic = handle.state_topic.empty() ? "(disabled)" : handle.state_topic.c_str();
    RCLCPP_INFO(
      get_logger(),
      "Switch '%s': chip=%s line=%d active_low=%s pullup=%s state_topic=%s pressed_event=%s released_event=%s",
      handle.event_object_tag.c_str(), gpio_chip_.c_str(), handle.gpio_line,
      handle.active_low ? "true" : "false", handle.pull_up ? "true" : "false", state_topic,
      handle.pressed_event_type.c_str(), handle.released_event_type.c_str());
  }

  std::string backend_;
  std::string gpio_chip_;
  int gpio_line_;
  bool active_low_;
  bool default_pull_up_;
  double polling_frequency_hz_;
  double debounce_duration_ms_;
  std::string smacc_event_topic_;
  std::string button_state_topic_;
  std::string event_object_tag_;
  std::string event_source_;
  std::string pressed_event_type_;
  std::string released_event_type_;

  double debounce_duration_;
  rclcpp::Publisher<smacc2_msgs::msg::SmaccEvent>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<SwitchHandle> switches_;
};

}  // namespace gpio_button_event

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gpio_button_event::SmaccGpioButtonNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
