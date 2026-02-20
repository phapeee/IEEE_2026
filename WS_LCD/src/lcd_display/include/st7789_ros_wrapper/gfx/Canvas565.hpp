#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace st7789_ros_wrapper::gfx {

class Canvas565 {
public:
  Canvas565(std::uint16_t width = 240, std::uint16_t height = 280);

  std::uint16_t width() const noexcept;
  std::uint16_t height() const noexcept;

  void clear(Color565 color);
  bool setPixel(std::uint16_t x, std::uint16_t y, Color565 color);
  bool fillRect(std::uint16_t x, std::uint16_t y, std::uint16_t w, std::uint16_t h, Color565 color);
  bool blit(
    std::uint16_t x, std::uint16_t y, const Color565 * pixels, std::uint16_t width,
    std::uint16_t height);
  bool drawChar(std::uint16_t x, std::uint16_t y, char character, Color565 color, std::uint8_t scale = 1);
  bool drawText(
    std::uint16_t x, std::uint16_t y, const std::string & text, Color565 color,
    std::uint8_t scale = 1);

  const std::vector<Color565> & pixels() const noexcept;
  std::vector<Color565> & pixels() noexcept;
  std::size_t pixelCount() const noexcept;

private:
  std::uint16_t width_;
  std::uint16_t height_;
  std::vector<Color565> pixels_;
};

}  // namespace st7789_ros_wrapper::gfx
