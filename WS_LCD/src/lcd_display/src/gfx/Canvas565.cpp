#include "st7789_ros_wrapper/gfx/Canvas565.hpp"

#include <algorithm>
#include <limits>

#include "st7789_ros_wrapper/gfx/FontBitmap.hpp"

namespace st7789_ros_wrapper::gfx {

Canvas565::Canvas565(const std::uint16_t width, const std::uint16_t height)
: width_(width), height_(height), pixels_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), color::kBlack) {}

std::uint16_t Canvas565::width() const noexcept {
  return width_;
}

std::uint16_t Canvas565::height() const noexcept {
  return height_;
}

void Canvas565::clear(const Color565 color) {
  std::fill(pixels_.begin(), pixels_.end(), color);
}

bool Canvas565::setPixel(const std::uint16_t x, const std::uint16_t y, const Color565 color) {
  if (x >= width_ || y >= height_) {
    return false;
  }

  const auto index = static_cast<std::size_t>(y) * width_ + x;
  pixels_[index] = color;
  return true;
}

bool Canvas565::fillRect(
  const std::uint16_t x, const std::uint16_t y, const std::uint16_t w, const std::uint16_t h,
  const Color565 color)
{
  if (w == 0 || h == 0 || x >= width_ || y >= height_) {
    return false;
  }

  const auto x_end = static_cast<std::uint16_t>(std::min<std::uint32_t>(width_, static_cast<std::uint32_t>(x) + w));
  const auto y_end = static_cast<std::uint16_t>(std::min<std::uint32_t>(height_, static_cast<std::uint32_t>(y) + h));

  for (std::uint16_t row = y; row < y_end; ++row) {
    for (std::uint16_t col = x; col < x_end; ++col) {
      setPixel(col, row, color);
    }
  }

  return true;
}

bool Canvas565::blit(
  const std::uint16_t x, const std::uint16_t y, const Color565 * const pixels,
  const std::uint16_t width, const std::uint16_t height)
{
  if (pixels == nullptr || width == 0 || height == 0 || x >= width_ || y >= height_) {
    return false;
  }

  const auto x_end =
    static_cast<std::uint16_t>(std::min<std::uint32_t>(width_, static_cast<std::uint32_t>(x) + width));
  const auto y_end =
    static_cast<std::uint16_t>(std::min<std::uint32_t>(height_, static_cast<std::uint32_t>(y) + height));

  if (x_end <= x || y_end <= y) {
    return false;
  }

  for (std::uint16_t row = y; row < y_end; ++row) {
    const auto source_row_offset = static_cast<std::size_t>(row - y) * width;
    const auto destination_row_offset = static_cast<std::size_t>(row) * width_;
    for (std::uint16_t col = x; col < x_end; ++col) {
      const auto source_index = source_row_offset + static_cast<std::size_t>(col - x);
      const auto destination_index = destination_row_offset + col;
      pixels_[destination_index] = pixels[source_index];
    }
  }

  return true;
}

bool Canvas565::drawChar(
  const std::uint16_t x, const std::uint16_t y, const char character, const Color565 color,
  const std::uint8_t scale)
{
  if (scale == 0U) {
    return false;
  }

  const auto glyph = FontBitmap::glyphRows(static_cast<unsigned char>(character));
  bool drew_pixel = false;

  for (std::uint8_t row = 0; row < FontBitmap::kGlyphHeight; ++row) {
    for (std::uint8_t col = 0; col < FontBitmap::kGlyphWidth; ++col) {
      if (!FontBitmap::bitIsSet(glyph, col, row)) {
        continue;
      }

      const auto base_x = static_cast<std::uint32_t>(x) + static_cast<std::uint32_t>(col) * scale;
      const auto base_y = static_cast<std::uint32_t>(y) + static_cast<std::uint32_t>(row) * scale;

      for (std::uint8_t scale_y = 0; scale_y < scale; ++scale_y) {
        for (std::uint8_t scale_x = 0; scale_x < scale; ++scale_x) {
          const auto pixel_x = base_x + scale_x;
          const auto pixel_y = base_y + scale_y;
          if (pixel_x > std::numeric_limits<std::uint16_t>::max() ||
            pixel_y > std::numeric_limits<std::uint16_t>::max())
          {
            continue;
          }

          drew_pixel = setPixel(
            static_cast<std::uint16_t>(pixel_x), static_cast<std::uint16_t>(pixel_y),
            color) || drew_pixel;
        }
      }
    }
  }

  return drew_pixel;
}

bool Canvas565::drawText(
  const std::uint16_t x, const std::uint16_t y, const std::string & text, const Color565 color,
  const std::uint8_t scale)
{
  if (scale == 0U || text.empty()) {
    return false;
  }

  const auto origin_x = static_cast<std::uint32_t>(x);
  auto cursor_x = origin_x;
  auto cursor_y = static_cast<std::uint32_t>(y);
  const auto glyph_advance = static_cast<std::uint32_t>(FontBitmap::kAdvanceX) * scale;
  const auto line_advance = static_cast<std::uint32_t>(FontBitmap::kLineHeight) * scale;
  bool drew_pixel = false;

  for (const char character : text) {
    if (character == '\r') {
      continue;
    }

    if (character == '\n') {
      cursor_x = origin_x;
      cursor_y += line_advance;
      continue;
    }

    if (cursor_x <= std::numeric_limits<std::uint16_t>::max() &&
      cursor_y <= std::numeric_limits<std::uint16_t>::max())
    {
      drew_pixel = drawChar(
        static_cast<std::uint16_t>(cursor_x), static_cast<std::uint16_t>(cursor_y), character,
        color, scale) || drew_pixel;
    }

    cursor_x += glyph_advance;
  }

  return drew_pixel;
}

const std::vector<Color565> & Canvas565::pixels() const noexcept {
  return pixels_;
}

std::vector<Color565> & Canvas565::pixels() noexcept {
  return pixels_;
}

std::size_t Canvas565::pixelCount() const noexcept {
  return pixels_.size();
}

}  // namespace st7789_ros_wrapper::gfx
