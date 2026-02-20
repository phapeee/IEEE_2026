#include "st7789_ros_wrapper/gfx/DirtyTracker.hpp"

#include <algorithm>

namespace st7789_ros_wrapper::gfx {

DirtyTracker::DirtyTracker(const std::uint16_t canvas_width, const std::uint16_t canvas_height)
: canvas_width_(canvas_width), canvas_height_(canvas_height) {}

void DirtyTracker::setCanvasSize(
  const std::uint16_t canvas_width, const std::uint16_t canvas_height)
{
  canvas_width_ = canvas_width;
  canvas_height_ = canvas_height;
  clear();
}

bool DirtyTracker::markRect(
  const std::uint16_t x, const std::uint16_t y, const std::uint16_t width,
  const std::uint16_t height)
{
  if (width == 0U || height == 0U || x >= canvas_width_ || y >= canvas_height_) {
    return false;
  }

  const auto x_end =
    std::min<std::uint32_t>(canvas_width_, static_cast<std::uint32_t>(x) + width);
  const auto y_end =
    std::min<std::uint32_t>(canvas_height_, static_cast<std::uint32_t>(y) + height);

  if (x_end <= x || y_end <= y) {
    return false;
  }

  RectU16 clipped{
    x,
    y,
    static_cast<std::uint16_t>(x_end - x),
    static_cast<std::uint16_t>(y_end - y)};

  if (!has_dirty_) {
    dirty_ = clipped;
    has_dirty_ = true;
    return true;
  }

  const auto merged_x = std::min<std::uint16_t>(dirty_.x, clipped.x);
  const auto merged_y = std::min<std::uint16_t>(dirty_.y, clipped.y);
  const auto dirty_end_x = static_cast<std::uint32_t>(dirty_.x) + dirty_.width;
  const auto clipped_end_x = static_cast<std::uint32_t>(clipped.x) + clipped.width;
  const auto dirty_end_y = static_cast<std::uint32_t>(dirty_.y) + dirty_.height;
  const auto clipped_end_y = static_cast<std::uint32_t>(clipped.y) + clipped.height;
  const auto merged_end_x = std::max<std::uint32_t>(dirty_end_x, clipped_end_x);
  const auto merged_end_y = std::max<std::uint32_t>(dirty_end_y, clipped_end_y);

  dirty_ = {
    merged_x,
    merged_y,
    static_cast<std::uint16_t>(merged_end_x - merged_x),
    static_cast<std::uint16_t>(merged_end_y - merged_y)};

  return true;
}

void DirtyTracker::markAll() {
  if (canvas_width_ == 0U || canvas_height_ == 0U) {
    clear();
    return;
  }

  dirty_ = {0U, 0U, canvas_width_, canvas_height_};
  has_dirty_ = true;
}

void DirtyTracker::clear() {
  has_dirty_ = false;
  dirty_ = {};
}

bool DirtyTracker::hasDirty() const noexcept {
  return has_dirty_;
}

RectU16 DirtyTracker::dirtyRect() const noexcept {
  return dirty_;
}

std::size_t DirtyTracker::dirtyPixelCount() const noexcept {
  if (!has_dirty_) {
    return 0U;
  }

  return static_cast<std::size_t>(dirty_.width) * dirty_.height;
}

std::size_t DirtyTracker::totalPixelCount() const noexcept {
  return static_cast<std::size_t>(canvas_width_) * canvas_height_;
}

double DirtyTracker::dirtyCoverage() const noexcept {
  const auto total_pixels = totalPixelCount();
  if (total_pixels == 0U) {
    return 0.0;
  }

  return static_cast<double>(dirtyPixelCount()) / static_cast<double>(total_pixels);
}

}  // namespace st7789_ros_wrapper::gfx
