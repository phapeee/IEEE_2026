#pragma once

#include <cstdint>
#include <string>

#include "st7789_ros_wrapper/app/Screen.hpp"

namespace st7789_ros_wrapper::app {

struct ScreenRuntimeConfig {
  std::string spi_device{"/dev/spidev0.0"};
  std::int64_t spi_speed_hz{32000000};
  std::int64_t spi_mode{0};
  std::int64_t spi_bits_per_word{8};
  std::string gpiochip{"gpiochip4"};
  std::int64_t dc_gpio{13};
  std::int64_t rst_gpio{16};
  std::int64_t bl_gpio{26};
  bool use_threads{false};
  std::int64_t frame_queue_capacity{3};
  std::int64_t rotation_degrees{0};
  bool color_order_bgr{true};
  bool display_inversion{true};
  std::int64_t x_offset{0};
  std::int64_t y_offset{0};
};

bool buildScreenConfig(
  const ScreenRuntimeConfig & runtime, ScreenConfig & output, std::string & error);

}  // namespace st7789_ros_wrapper::app
