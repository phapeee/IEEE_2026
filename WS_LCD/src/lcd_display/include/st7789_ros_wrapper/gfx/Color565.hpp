#pragma once

#include <cstdint>

namespace st7789_ros_wrapper::gfx {

using Color565 = std::uint16_t;

constexpr Color565 rgb888To565(const std::uint8_t r, const std::uint8_t g, const std::uint8_t b) {
  return static_cast<Color565>(((r & 0xF8U) << 8U) | ((g & 0xFCU) << 3U) | (b >> 3U));
}

namespace color {
constexpr Color565 kBlack = 0x0000;
constexpr Color565 kWhite = 0xFFFF;
constexpr Color565 kRed = 0xF800;
constexpr Color565 kGreen = 0x07E0;
constexpr Color565 kBlue = 0x001F;
}  // namespace color

}  // namespace st7789_ros_wrapper::gfx
