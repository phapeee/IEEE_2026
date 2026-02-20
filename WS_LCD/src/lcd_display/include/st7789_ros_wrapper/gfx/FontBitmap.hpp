#pragma once

#include <array>
#include <cstdint>

namespace st7789_ros_wrapper::gfx {

class FontBitmap {
public:
  using GlyphRows = std::array<std::uint8_t, 8>;

  static constexpr std::uint8_t kGlyphWidth = 8;
  static constexpr std::uint8_t kGlyphHeight = 8;
  static constexpr std::uint8_t kAdvanceX = 8;
  static constexpr std::uint8_t kLineHeight = 8;

  static GlyphRows glyphRows(unsigned char ascii) noexcept;
  static bool bitIsSet(const GlyphRows & rows, std::uint8_t x, std::uint8_t y) noexcept;
  static bool isPrintable(unsigned char ascii) noexcept;
};

}  // namespace st7789_ros_wrapper::gfx
