#pragma once

#include <cstdint>
#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "st7789_ros_wrapper/assets/ImageLoader.hpp"

namespace st7789_ros_wrapper::ros {

enum class ImageResizeMode {
  kExactSize = 0,
  kFitCenterCrop = 1,
};

struct ImageMessageConversionConfig {
  std::uint16_t target_width{240};
  std::uint16_t target_height{280};
  ImageResizeMode resize_mode{ImageResizeMode::kFitCenterCrop};
};

bool convertImageMessageToRgb565(
  const sensor_msgs::msg::Image & message, const ImageMessageConversionConfig & config,
  assets::Image565 & output, std::string & error);

}  // namespace st7789_ros_wrapper::ros
