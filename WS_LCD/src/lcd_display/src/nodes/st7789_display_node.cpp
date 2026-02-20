#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "st7789_ros_wrapper/app/Screen.hpp"
#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"
#include "st7789_ros_wrapper/msg/image_overlay.hpp"
#include "st7789_ros_wrapper/msg/text_overlay.hpp"
#include "st7789_ros_wrapper/ros/DisplayNodeRuntimeConfig.hpp"
#include "st7789_ros_wrapper/ros/ImageMessageConverter.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

class St7789DisplayNode : public rclcpp::Node {
public:
  St7789DisplayNode()
  : Node("st7789_display_node") {
    const auto runtime_config = declareAndReadRuntimeConfig();

    st7789_ros_wrapper::app::ScreenConfig screen_config;
    std::string config_error;
    if (!st7789_ros_wrapper::app::buildScreenConfig(runtime_config, screen_config, config_error)) {
      throw std::runtime_error("Invalid display configuration: " + config_error);
    }

    const auto node_runtime_config = declareAndReadNodeRuntimeConfig();
    if (!st7789_ros_wrapper::ros::buildDisplayNodeConfig(
        node_runtime_config, node_config_, config_error))
    {
      throw std::runtime_error("Invalid node render configuration: " + config_error);
    }

    screen_config.use_dirty_rects = node_config_.use_dirty_rects;
    screen_config.dirty_rect_full_frame_threshold = node_config_.dirty_rect_full_frame_threshold;
    latest_text_x_ = node_config_.text_x;
    latest_text_y_ = node_config_.text_y;
    latest_text_scale_ = node_config_.text_scale;

    screen_ = std::make_unique<st7789_ros_wrapper::app::Screen>(screen_config);
    if (!screen_->begin()) {
      throw std::runtime_error("Failed to initialize ST7789 display hardware");
    }

    image_conversion_config_.target_width = screen_->canvas().width();
    image_conversion_config_.target_height = screen_->canvas().height();
    image_conversion_config_.resize_mode = node_config_.fit_image_to_screen ?
      st7789_ros_wrapper::ros::ImageResizeMode::kFitCenterCrop :
      st7789_ros_wrapper::ros::ImageResizeMode::kExactSize;

    screen_->clear(node_config_.clear_color);
    if (!screen_->present()) {
      throw std::runtime_error("Failed to present initial frame");
    }

    using std::placeholders::_1;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "~/image", rclcpp::SensorDataQoS(),
      std::bind(&St7789DisplayNode::onImageMessage, this, _1));
    image_overlay_sub_ = create_subscription<st7789_ros_wrapper::msg::ImageOverlay>(
      "~/image_overlay", rclcpp::SensorDataQoS(),
      std::bind(&St7789DisplayNode::onImageOverlayMessage, this, _1));
    text_sub_ = create_subscription<std_msgs::msg::String>(
      "~/text", rclcpp::QoS(10),
      std::bind(&St7789DisplayNode::onTextMessage, this, _1));
    text_overlay_sub_ = create_subscription<st7789_ros_wrapper::msg::TextOverlay>(
      "~/text_overlay", rclcpp::QoS(10),
      std::bind(&St7789DisplayNode::onTextOverlayMessage, this, _1));
    backlight_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/backlight", rclcpp::QoS(10),
      std::bind(&St7789DisplayNode::onBacklightMessage, this, _1));

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / node_config_.render_hz));
    render_timer_ = create_wall_timer(timer_period, std::bind(&St7789DisplayNode::renderIfDirty, this));

    RCLCPP_INFO(
      get_logger(),
      "Display initialized: spi_device=%s spi_speed_hz=%u gpiochip=%s dc=%u rst=%u bl=%u rotation_degrees=%lld color_order_bgr=%s display_inversion=%s x_offset=%u y_offset=%u use_threads=%s frame_queue_capacity=%u render_hz=%.2f fit_image_to_screen=%s use_dirty_rects=%s dirty_rect_full_frame_threshold=%.2f",
      screen_config.spi.device_path.c_str(),
      screen_config.spi.speed_hz,
      screen_config.gpio_chip_path.c_str(),
      screen_config.dc_gpio,
      screen_config.rst_gpio,
      screen_config.bl_gpio,
      static_cast<long long>(runtime_config.rotation_degrees),
      screen_config.panel.color_order_bgr ? "true" : "false",
      screen_config.panel.display_inversion ? "true" : "false",
      screen_config.panel.x_offset,
      screen_config.panel.y_offset,
      screen_config.use_threads ? "true" : "false",
      screen_config.frame_queue_capacity,
      node_config_.render_hz,
      node_config_.fit_image_to_screen ? "true" : "false",
      node_config_.use_dirty_rects ? "true" : "false",
      node_config_.dirty_rect_full_frame_threshold);
  }

private:
  struct PositionedOverlayImage {
    std::shared_ptr<st7789_ros_wrapper::assets::Image565> image;
    std::uint16_t x{0U};
    std::uint16_t y{0U};
  };

  static bool parseTextScale(
    const float input_scale, std::uint8_t & output_scale, std::string & error)
  {
    if (!std::isfinite(input_scale) || input_scale <= 0.0F) {
      error = "scale must be a positive finite value";
      return false;
    }

    const auto clamped = std::clamp<double>(
      static_cast<double>(input_scale), 1.0,
      static_cast<double>(std::numeric_limits<std::uint8_t>::max()));
    output_scale = static_cast<std::uint8_t>(std::lround(clamped));
    return true;
  }

  static bool buildOverlayResizeConfig(
    const sensor_msgs::msg::Image & message, const float scale,
    st7789_ros_wrapper::ros::ImageMessageConversionConfig & output, std::string & error)
  {
    if (!std::isfinite(scale) || scale <= 0.0F) {
      error = "scale must be a positive finite value";
      return false;
    }
    if (message.width == 0U || message.height == 0U) {
      error = "image dimensions must be non-zero";
      return false;
    }

    const auto scaled_width_raw = static_cast<double>(message.width) * static_cast<double>(scale);
    const auto scaled_height_raw = static_cast<double>(message.height) * static_cast<double>(scale);
    if (!std::isfinite(scaled_width_raw) || !std::isfinite(scaled_height_raw)) {
      error = "scaled image dimensions are not finite";
      return false;
    }

    const auto scaled_width = std::max<double>(1.0, std::lround(scaled_width_raw));
    const auto scaled_height = std::max<double>(1.0, std::lround(scaled_height_raw));
    if (scaled_width > static_cast<double>(std::numeric_limits<std::uint16_t>::max()) ||
      scaled_height > static_cast<double>(std::numeric_limits<std::uint16_t>::max()))
    {
      error = "scaled image dimensions exceed uint16 range";
      return false;
    }

    output = st7789_ros_wrapper::ros::ImageMessageConversionConfig{};
    output.target_width = static_cast<std::uint16_t>(scaled_width);
    output.target_height = static_cast<std::uint16_t>(scaled_height);
    output.resize_mode = st7789_ros_wrapper::ros::ImageResizeMode::kFitCenterCrop;
    return true;
  }

  void onImageMessage(const sensor_msgs::msg::Image::SharedPtr message) {
    st7789_ros_wrapper::assets::Image565 converted;
    std::string error;
    if (!st7789_ros_wrapper::ros::convertImageMessageToRgb565(
        *message, image_conversion_config_, converted, error))
    {
      RCLCPP_WARN(get_logger(), "Failed to convert image topic message: %s", error.c_str());
      return;
    }

    auto frame = std::make_shared<st7789_ros_wrapper::assets::Image565>(std::move(converted));
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_image_ = std::move(frame);
      frame_dirty_ = true;
    }
  }

  void onImageOverlayMessage(const st7789_ros_wrapper::msg::ImageOverlay::SharedPtr message) {
    st7789_ros_wrapper::ros::ImageMessageConversionConfig config;
    std::string error;
    if (!buildOverlayResizeConfig(message->image, message->scale, config, error)) {
      RCLCPP_WARN(get_logger(), "Failed to validate image_overlay message: %s", error.c_str());
      return;
    }

    st7789_ros_wrapper::assets::Image565 converted;
    if (!st7789_ros_wrapper::ros::convertImageMessageToRgb565(
        message->image, config, converted, error))
    {
      RCLCPP_WARN(get_logger(), "Failed to convert image_overlay message: %s", error.c_str());
      return;
    }

    auto frame = std::make_shared<st7789_ros_wrapper::assets::Image565>(std::move(converted));
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_image_overlay_ = PositionedOverlayImage{std::move(frame), message->x, message->y};
      frame_dirty_ = true;
    }
  }

  void onTextMessage(const std_msgs::msg::String::SharedPtr message) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_text_ = message->data;
      latest_text_x_ = node_config_.text_x;
      latest_text_y_ = node_config_.text_y;
      latest_text_scale_ = node_config_.text_scale;
      has_text_ = !latest_text_.empty();
      frame_dirty_ = true;
    }
  }

  void onTextOverlayMessage(const st7789_ros_wrapper::msg::TextOverlay::SharedPtr message) {
    std::uint8_t parsed_scale = 1U;
    std::string error;
    if (!parseTextScale(message->scale, parsed_scale, error)) {
      RCLCPP_WARN(get_logger(), "Ignoring text_overlay message: %s", error.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_text_ = message->text;
      latest_text_x_ = message->x;
      latest_text_y_ = message->y;
      latest_text_scale_ = parsed_scale;
      has_text_ = !latest_text_.empty();
      frame_dirty_ = true;
    }
  }

  void onBacklightMessage(const std_msgs::msg::Bool::SharedPtr message) {
    std::lock_guard<std::mutex> lock(screen_mutex_);
    if (!screen_->setBacklight(message->data)) {
      RCLCPP_WARN(get_logger(), "Failed to set backlight state to %s", message->data ? "on" : "off");
    }
  }

  void markFrameDirty() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    frame_dirty_ = true;
  }

  void renderIfDirty() {
    std::shared_ptr<st7789_ros_wrapper::assets::Image565> image;
    std::optional<PositionedOverlayImage> image_overlay;
    std::string text;
    bool has_text = false;
    std::uint16_t text_x = node_config_.text_x;
    std::uint16_t text_y = node_config_.text_y;
    std::uint8_t text_scale = node_config_.text_scale;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!frame_dirty_) {
        return;
      }

      image = latest_image_;
      image_overlay = latest_image_overlay_;
      text = latest_text_;
      has_text = has_text_;
      text_x = latest_text_x_;
      text_y = latest_text_y_;
      text_scale = latest_text_scale_;
      frame_dirty_ = false;
    }

    std::lock_guard<std::mutex> lock(screen_mutex_);
    if (image || image_overlay.has_value() || node_config_.clear_before_text || !has_text) {
      screen_->clear(node_config_.clear_color);
    }

    if (image && !screen_->drawImage(0U, 0U, *image)) {
      RCLCPP_WARN(get_logger(), "Failed to draw image frame to canvas");
      markFrameDirty();
      return;
    }

    if (image_overlay && image_overlay->image &&
      !screen_->drawImage(image_overlay->x, image_overlay->y, *image_overlay->image))
    {
      RCLCPP_WARN(get_logger(), "Failed to draw image_overlay frame to canvas");
    }

    if (has_text) {
      (void)screen_->drawText(
        text_x, text_y, text, node_config_.text_color, text_scale);
    }

    if (!screen_->present()) {
      RCLCPP_WARN(get_logger(), "Failed to present frame to display");
      markFrameDirty();
    }
  }

  st7789_ros_wrapper::app::ScreenRuntimeConfig declareAndReadRuntimeConfig() {
    st7789_ros_wrapper::app::ScreenRuntimeConfig config;
    config.spi_device = declare_parameter<std::string>("spi_device", config.spi_device);
    config.spi_speed_hz = declare_parameter<std::int64_t>("spi_speed_hz", config.spi_speed_hz);
    config.spi_mode = declare_parameter<std::int64_t>("spi_mode", config.spi_mode);
    config.spi_bits_per_word =
      declare_parameter<std::int64_t>("spi_bits_per_word", config.spi_bits_per_word);
    config.gpiochip = declare_parameter<std::string>("gpiochip", config.gpiochip);
    config.dc_gpio = declare_parameter<std::int64_t>("dc_gpio", config.dc_gpio);
    config.rst_gpio = declare_parameter<std::int64_t>("rst_gpio", config.rst_gpio);
    config.bl_gpio = declare_parameter<std::int64_t>("bl_gpio", config.bl_gpio);
    config.use_threads = declare_parameter<bool>("use_threads", config.use_threads);
    config.frame_queue_capacity =
      declare_parameter<std::int64_t>("frame_queue_capacity", config.frame_queue_capacity);
    config.rotation_degrees =
      declare_parameter<std::int64_t>("rotation_degrees", config.rotation_degrees);
    config.color_order_bgr =
      declare_parameter<bool>("color_order_bgr", config.color_order_bgr);
    config.display_inversion =
      declare_parameter<bool>("display_inversion", config.display_inversion);
    config.x_offset = declare_parameter<std::int64_t>("x_offset", config.x_offset);
    config.y_offset = declare_parameter<std::int64_t>("y_offset", config.y_offset);
    return config;
  }

  st7789_ros_wrapper::ros::DisplayNodeRuntimeConfig declareAndReadNodeRuntimeConfig() {
    st7789_ros_wrapper::ros::DisplayNodeRuntimeConfig config;
    config.render_hz = declare_parameter<double>("render_hz", config.render_hz);
    config.fit_image_to_screen =
      declare_parameter<bool>("fit_image_to_screen", config.fit_image_to_screen);
    config.use_dirty_rects =
      declare_parameter<bool>("use_dirty_rects", config.use_dirty_rects);
    config.dirty_rect_full_frame_threshold = declare_parameter<double>(
      "dirty_rect_full_frame_threshold", config.dirty_rect_full_frame_threshold);
    config.clear_before_text =
      declare_parameter<bool>("clear_before_text", config.clear_before_text);
    config.text_x = declare_parameter<std::int64_t>("text_x", config.text_x);
    config.text_y = declare_parameter<std::int64_t>("text_y", config.text_y);
    config.text_scale = declare_parameter<std::int64_t>("text_scale", config.text_scale);
    config.text_color_rgb565 =
      declare_parameter<std::int64_t>("text_color_rgb565", config.text_color_rgb565);
    config.clear_color_rgb565 =
      declare_parameter<std::int64_t>("clear_color_rgb565", config.clear_color_rgb565);
    return config;
  }

  std::unique_ptr<st7789_ros_wrapper::app::Screen> screen_;
  st7789_ros_wrapper::ros::DisplayNodeConfig node_config_;
  st7789_ros_wrapper::ros::ImageMessageConversionConfig image_conversion_config_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<st7789_ros_wrapper::msg::ImageOverlay>::SharedPtr image_overlay_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
  rclcpp::Subscription<st7789_ros_wrapper::msg::TextOverlay>::SharedPtr text_overlay_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr backlight_sub_;
  rclcpp::TimerBase::SharedPtr render_timer_;

  std::mutex state_mutex_;
  std::mutex screen_mutex_;
  std::shared_ptr<st7789_ros_wrapper::assets::Image565> latest_image_;
  std::optional<PositionedOverlayImage> latest_image_overlay_;
  std::string latest_text_;
  std::uint16_t latest_text_x_{0U};
  std::uint16_t latest_text_y_{0U};
  std::uint8_t latest_text_scale_{1U};
  bool has_text_{false};
  bool frame_dirty_{false};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<St7789DisplayNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("st7789_display_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
