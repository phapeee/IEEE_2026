#include "st7789_ros_wrapper/ros/DisplayNodeRuntimeConfig.hpp"

#include <cmath>
#include <limits>
#include <sstream>

namespace st7789_ros_wrapper::ros {
namespace {

template<typename TargetType>
bool fitsIn(const std::int64_t value) {
  return value >= static_cast<std::int64_t>(std::numeric_limits<TargetType>::min()) &&
         value <= static_cast<std::int64_t>(std::numeric_limits<TargetType>::max());
}

template<typename TargetType>
bool assignInteger(
  const std::int64_t input, const char * const name, TargetType & output, std::string & error)
{
  if (!fitsIn<TargetType>(input)) {
    std::ostringstream stream;
    stream << name << " value " << input << " is out of range";
    error = stream.str();
    return false;
  }

  output = static_cast<TargetType>(input);
  return true;
}

}  // namespace

bool buildDisplayNodeConfig(
  const DisplayNodeRuntimeConfig & runtime, DisplayNodeConfig & output, std::string & error)
{
  error.clear();
  output = DisplayNodeConfig{};
  output.fit_image_to_screen = runtime.fit_image_to_screen;
  output.use_dirty_rects = runtime.use_dirty_rects;
  output.clear_before_text = runtime.clear_before_text;

  if (!std::isfinite(runtime.render_hz) || runtime.render_hz <= 0.0) {
    error = "render_hz must be a positive finite value";
    return false;
  }
  if (runtime.render_hz > 120.0) {
    std::ostringstream stream;
    stream << "render_hz must be <= 120.0; got " << runtime.render_hz;
    error = stream.str();
    return false;
  }
  output.render_hz = runtime.render_hz;

  if (!std::isfinite(runtime.dirty_rect_full_frame_threshold) ||
    runtime.dirty_rect_full_frame_threshold < 0.0 ||
    runtime.dirty_rect_full_frame_threshold > 1.0)
  {
    std::ostringstream stream;
    stream << "dirty_rect_full_frame_threshold must be in [0.0, 1.0]; got " <<
      runtime.dirty_rect_full_frame_threshold;
    error = stream.str();
    return false;
  }
  output.dirty_rect_full_frame_threshold = runtime.dirty_rect_full_frame_threshold;

  if (!assignInteger<std::uint16_t>(runtime.text_x, "text_x", output.text_x, error)) {
    return false;
  }
  if (!assignInteger<std::uint16_t>(runtime.text_y, "text_y", output.text_y, error)) {
    return false;
  }
  if (runtime.text_scale <= 0) {
    error = "text_scale must be greater than zero";
    return false;
  }
  if (!assignInteger<std::uint8_t>(runtime.text_scale, "text_scale", output.text_scale, error)) {
    return false;
  }

  if (!assignInteger<gfx::Color565>(
      runtime.text_color_rgb565, "text_color_rgb565", output.text_color, error))
  {
    return false;
  }
  if (!assignInteger<gfx::Color565>(
      runtime.clear_color_rgb565, "clear_color_rgb565", output.clear_color, error))
  {
    return false;
  }

  return true;
}

}  // namespace st7789_ros_wrapper::ros
