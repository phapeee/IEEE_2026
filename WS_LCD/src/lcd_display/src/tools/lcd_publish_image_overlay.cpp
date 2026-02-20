#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "st7789_ros_wrapper/msg/image_overlay.hpp"

namespace {

struct CliConfig {
  std::string image_path;
  std::string topic{"/st7789_display_node/image_overlay"};
  std::string frame_id;
  std::uint16_t x{0U};
  std::uint16_t y{0U};
  float scale{1.0F};
  std::int64_t wait_for_subscriber_ms{2000};
  bool show_help{false};
};

bool parseInt64(
  const std::string & input, const char * const name, std::int64_t & output, std::string & error)
{
  try {
    std::size_t consumed = 0;
    const auto value = std::stoll(input, &consumed, 0);
    if (consumed != input.size()) {
      error = std::string(name) + " must be an integer";
      return false;
    }
    output = value;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " must be an integer";
    return false;
  }
}

bool parseFloat(
  const std::string & input, const char * const name, float & output, std::string & error)
{
  try {
    std::size_t consumed = 0;
    const auto value = std::stof(input, &consumed);
    if (consumed != input.size() || !std::isfinite(value)) {
      error = std::string(name) + " must be a finite number";
      return false;
    }
    output = value;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " must be a finite number";
    return false;
  }
}

bool parseArgs(const int argc, char ** argv, CliConfig & output, std::string & error) {
  error.clear();
  output = CliConfig{};

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "-h" || arg == "--help") {
      output.show_help = true;
      return true;
    }

    const auto requireValue = [&](const char * const name, std::string & value_out) -> bool {
      if (i + 1 >= argc) {
        error = std::string("Missing value for ") + name;
        return false;
      }

      value_out = argv[++i];
      return true;
    };

    std::string value;
    if (arg == "--image") {
      if (!requireValue("--image", value)) {
        return false;
      }
      output.image_path = value;
    } else if (arg == "--topic") {
      if (!requireValue("--topic", value)) {
        return false;
      }
      output.topic = value;
    } else if (arg == "--frame-id") {
      if (!requireValue("--frame-id", value)) {
        return false;
      }
      output.frame_id = value;
    } else if (arg == "--x") {
      std::int64_t parsed_value = 0;
      if (!requireValue("--x", value) || !parseInt64(value, "--x", parsed_value, error)) {
        return false;
      }
      if (parsed_value < 0 ||
        parsed_value > static_cast<std::int64_t>(std::numeric_limits<std::uint16_t>::max()))
      {
        error = "--x must be in [0, 65535]";
        return false;
      }
      output.x = static_cast<std::uint16_t>(parsed_value);
    } else if (arg == "--y") {
      std::int64_t parsed_value = 0;
      if (!requireValue("--y", value) || !parseInt64(value, "--y", parsed_value, error)) {
        return false;
      }
      if (parsed_value < 0 ||
        parsed_value > static_cast<std::int64_t>(std::numeric_limits<std::uint16_t>::max()))
      {
        error = "--y must be in [0, 65535]";
        return false;
      }
      output.y = static_cast<std::uint16_t>(parsed_value);
    } else if (arg == "--scale") {
      if (!requireValue("--scale", value) || !parseFloat(value, "--scale", output.scale, error)) {
        return false;
      }
    } else if (arg == "--wait-for-subscriber-ms") {
      if (!requireValue("--wait-for-subscriber-ms", value) ||
          !parseInt64(
            value, "--wait-for-subscriber-ms",
            output.wait_for_subscriber_ms, error))
      {
        return false;
      }
    } else {
      error = "Unknown argument: " + arg;
      return false;
    }
  }

  return true;
}

bool validateConfig(const CliConfig & config, std::string & error) {
  error.clear();

  if (config.show_help) {
    return true;
  }

  if (config.image_path.empty()) {
    error = "--image is required";
    return false;
  }
  if (config.topic.empty()) {
    error = "--topic cannot be empty";
    return false;
  }
  if (!std::isfinite(config.scale) || config.scale <= 0.0F) {
    error = "--scale must be a positive finite value";
    return false;
  }
  if (config.wait_for_subscriber_ms < 0) {
    error = "--wait-for-subscriber-ms must be >= 0";
    return false;
  }

  return true;
}

std::string usage(const std::string & program_name) {
  return
    "Usage: " + program_name + " [options]\n"
    "Options:\n"
    "  --image <path>                  Image file path (required)\n"
    "  --x <int>                       Overlay X position in pixels (default: 0)\n"
    "  --y <int>                       Overlay Y position in pixels (default: 0)\n"
    "  --scale <float>                 Overlay scale factor (default: 1.0)\n"
    "  --topic <name>                  Target topic (default: /st7789_display_node/image_overlay)\n"
    "  --frame-id <id>                 Optional Image header frame_id\n"
    "  --wait-for-subscriber-ms <int>  Wait for a subscriber before publishing (default: 2000)\n"
    "  -h, --help                      Show this help message\n";
}

bool waitForSubscriber(
  const rclcpp::Publisher<st7789_ros_wrapper::msg::ImageOverlay>::SharedPtr & publisher,
  const std::int64_t wait_ms)
{
  if (wait_ms <= 0) {
    return true;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (publisher->get_subscription_count() > 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return publisher->get_subscription_count() > 0;
}

}  // namespace

int main(int argc, char ** argv) {
  CliConfig cli_config;
  std::string error;
  if (!parseArgs(argc, argv, cli_config, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << usage(argv[0]);
    return 2;
  }

  if (cli_config.show_help) {
    std::cout << usage(argv[0]);
    return 0;
  }

  if (!validateConfig(cli_config, error)) {
    std::cerr << "Invalid configuration: " << error << "\n";
    return 2;
  }

  cv::Mat image = cv::imread(cli_config.image_path, cv::IMREAD_COLOR);
  if (image.empty()) {
    std::cerr << "Failed to load image: " << cli_config.image_path << "\n";
    return 3;
  }
  if (!image.isContinuous()) {
    image = image.clone();
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lcd_image_overlay_demo");
  auto publisher = node->create_publisher<st7789_ros_wrapper::msg::ImageOverlay>(
    cli_config.topic, rclcpp::QoS(10));

  const auto has_subscriber = waitForSubscriber(publisher, cli_config.wait_for_subscriber_ms);
  if (!has_subscriber) {
    RCLCPP_WARN(
      node->get_logger(),
      "No subscribers detected on %s before publish timeout; publishing once anyway.",
      cli_config.topic.c_str());
  }

  sensor_msgs::msg::Image ros_image;
  ros_image.header.stamp = node->now();
  ros_image.header.frame_id = cli_config.frame_id;
  ros_image.height = static_cast<std::uint32_t>(image.rows);
  ros_image.width = static_cast<std::uint32_t>(image.cols);
  ros_image.encoding = "bgr8";
  ros_image.is_bigendian = 0;
  ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * image.elemSize());
  ros_image.data.assign(
    image.data,
    image.data + static_cast<std::size_t>(ros_image.step) * ros_image.height);

  st7789_ros_wrapper::msg::ImageOverlay overlay;
  overlay.image = std::move(ros_image);
  overlay.x = cli_config.x;
  overlay.y = cli_config.y;
  overlay.scale = cli_config.scale;
  publisher->publish(overlay);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::shutdown();

  std::cout << "Published image overlay to " << cli_config.topic
            << " from " << cli_config.image_path
            << " (" << image.cols << "x" << image.rows << ")"
            << " x=" << cli_config.x
            << " y=" << cli_config.y
            << " scale=" << cli_config.scale << "\n";
  return 0;
}
