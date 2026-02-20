#pragma once

#include <cstdint>
#include <string>

#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace st7789_ros_wrapper::ros {

struct DisplayNodeRuntimeConfig {
  double render_hz{15.0};
  bool fit_image_to_screen{true};
  bool use_dirty_rects{true};
  double dirty_rect_full_frame_threshold{0.35};
  bool clear_before_text{true};
  std::int64_t text_x{8};
  std::int64_t text_y{8};
  std::int64_t text_scale{2};
  std::int64_t text_color_rgb565{static_cast<std::int64_t>(gfx::color::kWhite)};
  std::int64_t clear_color_rgb565{static_cast<std::int64_t>(gfx::color::kBlack)};
};

struct DisplayNodeConfig {
  double render_hz{15.0};
  bool fit_image_to_screen{true};
  bool use_dirty_rects{true};
  double dirty_rect_full_frame_threshold{0.35};
  bool clear_before_text{true};
  std::uint16_t text_x{8};
  std::uint16_t text_y{8};
  std::uint8_t text_scale{2};
  gfx::Color565 text_color{gfx::color::kWhite};
  gfx::Color565 clear_color{gfx::color::kBlack};
};

bool buildDisplayNodeConfig(
  const DisplayNodeRuntimeConfig & runtime, DisplayNodeConfig & output, std::string & error);

}  // namespace st7789_ros_wrapper::ros
