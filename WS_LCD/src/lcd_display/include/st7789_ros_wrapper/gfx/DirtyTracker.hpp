#pragma once

#include <cstddef>
#include <cstdint>

namespace st7789_ros_wrapper::gfx {

struct RectU16 {
  std::uint16_t x{0};
  std::uint16_t y{0};
  std::uint16_t width{0};
  std::uint16_t height{0};
};

class DirtyTracker {
public:
  DirtyTracker(std::uint16_t canvas_width = 240, std::uint16_t canvas_height = 280);

  void setCanvasSize(std::uint16_t canvas_width, std::uint16_t canvas_height);
  bool markRect(std::uint16_t x, std::uint16_t y, std::uint16_t width, std::uint16_t height);
  void markAll();
  void clear();

  bool hasDirty() const noexcept;
  RectU16 dirtyRect() const noexcept;
  std::size_t dirtyPixelCount() const noexcept;
  std::size_t totalPixelCount() const noexcept;
  double dirtyCoverage() const noexcept;

private:
  std::uint16_t canvas_width_{0};
  std::uint16_t canvas_height_{0};
  bool has_dirty_{false};
  RectU16 dirty_{};
};

}  // namespace st7789_ros_wrapper::gfx
