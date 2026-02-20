#include "st7789_ros_wrapper/ros/ImageMessageConverter.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>

#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace st7789_ros_wrapper::ros {
namespace {

enum class SourceEncoding {
  kRgb8,
  kBgr8,
  kRgba8,
  kBgra8,
  kMono8,
};

std::string normalizeEncoding(std::string encoding) {
  std::transform(
    encoding.begin(), encoding.end(), encoding.begin(),
    [](const unsigned char value) {return static_cast<char>(std::tolower(value));});
  return encoding;
}

bool parseEncoding(const std::string & encoding, SourceEncoding & parsed) {
  const auto normalized = normalizeEncoding(encoding);
  if (normalized == "rgb8") {
    parsed = SourceEncoding::kRgb8;
    return true;
  }
  if (normalized == "bgr8") {
    parsed = SourceEncoding::kBgr8;
    return true;
  }
  if (normalized == "rgba8") {
    parsed = SourceEncoding::kRgba8;
    return true;
  }
  if (normalized == "bgra8") {
    parsed = SourceEncoding::kBgra8;
    return true;
  }
  if (normalized == "mono8") {
    parsed = SourceEncoding::kMono8;
    return true;
  }
  return false;
}

std::size_t channelsForEncoding(const SourceEncoding encoding) {
  switch (encoding) {
    case SourceEncoding::kRgb8:
    case SourceEncoding::kBgr8:
      return 3U;
    case SourceEncoding::kRgba8:
    case SourceEncoding::kBgra8:
      return 4U;
    case SourceEncoding::kMono8:
      return 1U;
  }
  return 0U;
}

void decodePixel(
  const SourceEncoding encoding, const std::uint8_t * const pixel_ptr, std::uint8_t & r,
  std::uint8_t & g, std::uint8_t & b)
{
  switch (encoding) {
    case SourceEncoding::kRgb8:
      r = pixel_ptr[0];
      g = pixel_ptr[1];
      b = pixel_ptr[2];
      break;
    case SourceEncoding::kBgr8:
      b = pixel_ptr[0];
      g = pixel_ptr[1];
      r = pixel_ptr[2];
      break;
    case SourceEncoding::kRgba8:
      r = pixel_ptr[0];
      g = pixel_ptr[1];
      b = pixel_ptr[2];
      break;
    case SourceEncoding::kBgra8:
      b = pixel_ptr[0];
      g = pixel_ptr[1];
      r = pixel_ptr[2];
      break;
    case SourceEncoding::kMono8:
      r = pixel_ptr[0];
      g = pixel_ptr[0];
      b = pixel_ptr[0];
      break;
  }
}

std::size_t sampleCoordinate(const double coordinate, const std::size_t max_index) {
  if (coordinate <= 0.0) {
    return 0U;
  }
  const auto floored = static_cast<std::size_t>(coordinate);
  return std::min(floored, max_index);
}

}  // namespace

bool convertImageMessageToRgb565(
  const sensor_msgs::msg::Image & message, const ImageMessageConversionConfig & config,
  assets::Image565 & output, std::string & error)
{
  error.clear();
  output = assets::Image565{};

  if (config.target_width == 0U || config.target_height == 0U) {
    error = "target dimensions must be non-zero";
    return false;
  }

  if (message.width == 0U || message.height == 0U) {
    error = "message dimensions must be non-zero";
    return false;
  }

  SourceEncoding encoding;
  if (!parseEncoding(message.encoding, encoding)) {
    error = "unsupported image encoding: " + message.encoding;
    return false;
  }

  const auto source_width = static_cast<std::size_t>(message.width);
  const auto source_height = static_cast<std::size_t>(message.height);
  const auto channels = channelsForEncoding(encoding);

  const auto minimum_step = source_width * channels;
  if (message.step < minimum_step) {
    std::ostringstream stream;
    stream << "image step is too small for encoding " << message.encoding;
    error = stream.str();
    return false;
  }

  const auto required_bytes = static_cast<std::size_t>(message.step) * source_height;
  if (message.data.size() < required_bytes) {
    error = "image data buffer is smaller than step * height";
    return false;
  }

  if (config.resize_mode == ImageResizeMode::kExactSize &&
    (source_width != config.target_width || source_height != config.target_height))
  {
    std::ostringstream stream;
    stream << "image dimensions (" << source_width << "x" << source_height
           << ") do not match target (" << config.target_width << "x" << config.target_height
           << ") when resize mode is exact";
    error = stream.str();
    return false;
  }

  output.width = config.target_width;
  output.height = config.target_height;
  output.pixels.resize(static_cast<std::size_t>(output.width) * output.height);

  double crop_x = 0.0;
  double crop_y = 0.0;
  double crop_width = static_cast<double>(source_width);
  double crop_height = static_cast<double>(source_height);

  if (config.resize_mode == ImageResizeMode::kFitCenterCrop) {
    const auto source_aspect = static_cast<double>(source_width) / source_height;
    const auto target_aspect = static_cast<double>(config.target_width) / config.target_height;

    if (source_aspect > target_aspect) {
      crop_width = source_height * target_aspect;
      crop_x = (source_width - crop_width) * 0.5;
    } else if (source_aspect < target_aspect) {
      crop_height = source_width / target_aspect;
      crop_y = (source_height - crop_height) * 0.5;
    }
  }

  const auto max_x = source_width - 1U;
  const auto max_y = source_height - 1U;

  for (std::size_t target_y = 0; target_y < output.height; ++target_y) {
    double source_y = static_cast<double>(target_y);
    if (config.resize_mode == ImageResizeMode::kFitCenterCrop) {
      source_y = crop_y + ((static_cast<double>(target_y) + 0.5) * crop_height / output.height);
    }
    const auto sampled_y = sampleCoordinate(source_y, max_y);
    const auto * const row_ptr = message.data.data() + sampled_y * message.step;

    for (std::size_t target_x = 0; target_x < output.width; ++target_x) {
      double source_x = static_cast<double>(target_x);
      if (config.resize_mode == ImageResizeMode::kFitCenterCrop) {
        source_x = crop_x + ((static_cast<double>(target_x) + 0.5) * crop_width / output.width);
      }
      const auto sampled_x = sampleCoordinate(source_x, max_x);
      const auto * const pixel_ptr = row_ptr + sampled_x * channels;

      std::uint8_t r = 0;
      std::uint8_t g = 0;
      std::uint8_t b = 0;
      decodePixel(encoding, pixel_ptr, r, g, b);
      output.pixels[target_y * output.width + target_x] = gfx::rgb888To565(r, g, b);
    }
  }

  return true;
}

}  // namespace st7789_ros_wrapper::ros
