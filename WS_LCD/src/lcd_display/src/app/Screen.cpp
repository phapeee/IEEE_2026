#include "st7789_ros_wrapper/app/Screen.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "st7789_ros_wrapper/gfx/FontBitmap.hpp"

namespace st7789_ros_wrapper::app {
namespace {

hw::GpioLineConfig makeGpioConfig(const std::string & chip_path, const std::uint32_t offset) {
  hw::GpioLineConfig config;
  config.chip_path = chip_path;
  config.offset = offset;
  return config;
}

std::uint16_t canvasWidthForRotation(const driver::St7789PanelConfig & panel) {
  return (panel.rotation % 2U == 0U) ? panel.width : panel.height;
}

std::uint16_t canvasHeightForRotation(const driver::St7789PanelConfig & panel) {
  return (panel.rotation % 2U == 0U) ? panel.height : panel.width;
}

}  // namespace

Screen::Screen(ScreenConfig config)
: Screen(std::move(config), nullptr, nullptr) {}

Screen::Screen(
  ScreenConfig config, const std::shared_ptr<hw::SpiBusBackend> & spi_backend,
  const std::shared_ptr<hw::GpioLineBackend> & gpio_backend)
: config_(std::move(config)),
  spi_(config_.spi, spi_backend),
  dc_(makeGpioConfig(config_.gpio_chip_path, config_.dc_gpio), gpio_backend),
  rst_(makeGpioConfig(config_.gpio_chip_path, config_.rst_gpio), gpio_backend),
  bl_(makeGpioConfig(config_.gpio_chip_path, config_.bl_gpio), gpio_backend),
  driver_(spi_, dc_, rst_, bl_, config_.panel),
  canvas_(canvasWidthForRotation(config_.panel), canvasHeightForRotation(config_.panel)),
  dirty_tracker_(canvas_.width(), canvas_.height())
{
  if (!std::isfinite(config_.dirty_rect_full_frame_threshold)) {
    config_.dirty_rect_full_frame_threshold = 0.35;
  }
  config_.dirty_rect_full_frame_threshold =
    std::clamp(config_.dirty_rect_full_frame_threshold, 0.0, 1.0);
  if (config_.frame_queue_capacity == 0U) {
    config_.frame_queue_capacity = 1U;
  }
  if (config_.use_threads) {
    frame_queue_ = std::make_unique<FrameQueue>(config_.frame_queue_capacity);
  }
  dirty_tracker_.markAll();
}

Screen::~Screen() {
  stopIoThread();
}

bool Screen::begin() {
  bool initialized = false;
  {
    const std::lock_guard<std::mutex> lock(driver_mutex_);
    initialized = driver_.initialize();
  }
  if (initialized) {
    dirty_tracker_.markAll();
  }

  if (!initialized || !config_.use_threads || io_thread_.joinable()) {
    return initialized;
  }

  async_present_failed_.store(false);
  if (!frame_queue_) {
    frame_queue_ = std::make_unique<FrameQueue>(config_.frame_queue_capacity);
  }
  try {
    io_thread_ = std::thread(&Screen::ioThreadLoop, this);
  } catch (...) {
    return false;
  }

  return initialized;
}

bool Screen::setBacklight(const bool on) {
  const std::lock_guard<std::mutex> lock(driver_mutex_);
  return driver_.setBacklight(on);
}

void Screen::clear(const gfx::Color565 color) {
  canvas_.clear(color);
  dirty_tracker_.markAll();
}

bool Screen::drawImage(
  const std::uint16_t x, const std::uint16_t y, const assets::Image565 & image)
{
  if (image.empty()) {
    return false;
  }

  const auto drew = canvas_.blit(x, y, image.pixels.data(), image.width, image.height);
  if (drew) {
    (void)dirty_tracker_.markRect(x, y, image.width, image.height);
  }
  return drew;
}

bool Screen::drawText(
  const std::uint16_t x, const std::uint16_t y, const std::string & text,
  const gfx::Color565 color, const std::uint8_t scale)
{
  const auto drew = canvas_.drawText(x, y, text, color, scale);
  if (drew) {
    (void)trackTextDirtyRegion(x, y, text, scale);
  }
  return drew;
}

bool Screen::present() {
  if (!config_.use_threads) {
    return presentSync();
  }

  if (async_present_failed_.load()) {
    return false;
  }

  if (!frame_queue_) {
    return false;
  }

  if (!config_.use_dirty_rects) {
    QueuedFrame frame;
    frame.kind = QueuedFrameKind::kFullFrame;
    frame.pixels = canvas_.pixels();
    return enqueueFrame(std::move(frame));
  }

  if (!dirty_tracker_.hasDirty()) {
    return true;
  }

  QueuedFrame frame;
  if (shouldUseFullPresent()) {
    frame.kind = QueuedFrameKind::kFullFrame;
    frame.pixels = canvas_.pixels();
  } else {
    const auto region = dirty_tracker_.dirtyRect();
    if (!buildRegionPixels(region, frame.pixels)) {
      return false;
    }
    frame.kind = QueuedFrameKind::kRegion;
    frame.region.x = region.x;
    frame.region.y = region.y;
    frame.region.width = region.width;
    frame.region.height = region.height;
  }

  if (!enqueueFrame(std::move(frame))) {
    return false;
  }
  dirty_tracker_.clear();
  return true;
}

gfx::Canvas565 & Screen::canvas() noexcept {
  return canvas_;
}

const gfx::Canvas565 & Screen::canvas() const noexcept {
  return canvas_;
}

bool Screen::presentDirtyRegion(const gfx::RectU16 & region) {
  if (region.width == 0U || region.height == 0U) {
    dirty_tracker_.clear();
    return true;
  }

  if (!buildRegionPixels(region, region_pixels_)) {
    return false;
  }

  {
    const std::lock_guard<std::mutex> lock(driver_mutex_);
    if (!driver_.presentRegion(
        region_pixels_.data(), region_pixels_.size(), region.x, region.y, region.width,
        region.height))
    {
      return false;
    }
  }

  dirty_tracker_.clear();
  return true;
}

bool Screen::presentSync() {
  if (!config_.use_dirty_rects) {
    const std::lock_guard<std::mutex> lock(driver_mutex_);
    return driver_.present(canvas_.pixels().data(), canvas_.pixelCount());
  }

  if (!dirty_tracker_.hasDirty()) {
    return true;
  }

  if (shouldUseFullPresent()) {
    {
      const std::lock_guard<std::mutex> lock(driver_mutex_);
      if (!driver_.present(canvas_.pixels().data(), canvas_.pixelCount())) {
        return false;
      }
    }
    dirty_tracker_.clear();
    return true;
  }

  return presentDirtyRegion(dirty_tracker_.dirtyRect());
}

bool Screen::buildRegionPixels(
  const gfx::RectU16 & region, std::vector<gfx::Color565> & pixels) const
{
  if (region.width == 0U || region.height == 0U) {
    pixels.clear();
    return true;
  }

  if (region.x >= canvas_.width() || region.y >= canvas_.height()) {
    return false;
  }

  const auto x_end = static_cast<std::uint32_t>(region.x) + region.width;
  const auto y_end = static_cast<std::uint32_t>(region.y) + region.height;
  if (x_end > canvas_.width() || y_end > canvas_.height()) {
    return false;
  }

  const auto region_pixel_count =
    static_cast<std::size_t>(region.width) * static_cast<std::size_t>(region.height);
  pixels.resize(region_pixel_count);

  const auto & source_pixels = canvas_.pixels();
  for (std::uint16_t row = 0; row < region.height; ++row) {
    const auto source_row_start =
      static_cast<std::size_t>(region.y + row) * canvas_.width() + region.x;
    const auto destination_row_start = static_cast<std::size_t>(row) * region.width;
    std::copy_n(
      source_pixels.begin() + static_cast<std::ptrdiff_t>(source_row_start), region.width,
      pixels.begin() + static_cast<std::ptrdiff_t>(destination_row_start));
  }

  return true;
}

bool Screen::enqueueFrame(QueuedFrame frame) {
  if (async_present_failed_.load()) {
    return false;
  }
  if (!frame_queue_) {
    return false;
  }
  return frame_queue_->push(std::move(frame));
}

void Screen::stopIoThread() {
  if (frame_queue_) {
    frame_queue_->stop();
  }
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void Screen::ioThreadLoop() {
  if (!frame_queue_) {
    async_present_failed_.store(true);
    return;
  }

  QueuedFrame frame;
  while (frame_queue_->waitPop(frame)) {
    bool presented = false;
    {
      const std::lock_guard<std::mutex> lock(driver_mutex_);
      if (frame.kind == QueuedFrameKind::kFullFrame) {
        presented = driver_.present(frame.pixels.data(), frame.pixels.size());
      } else {
        presented = driver_.presentRegion(
          frame.pixels.data(), frame.pixels.size(), frame.region.x, frame.region.y,
          frame.region.width, frame.region.height);
      }
    }

    if (!presented) {
      async_present_failed_.store(true);
      frame_queue_->stop();
      return;
    }
  }
}

bool Screen::shouldUseFullPresent() const {
  const auto region = dirty_tracker_.dirtyRect();
  if (region.x == 0U && region.y == 0U &&
    region.width == canvas_.width() && region.height == canvas_.height())
  {
    return true;
  }

  return dirty_tracker_.dirtyCoverage() >= config_.dirty_rect_full_frame_threshold;
}

bool Screen::trackTextDirtyRegion(
  const std::uint16_t x, const std::uint16_t y, const std::string & text,
  const std::uint8_t scale)
{
  if (text.empty() || scale == 0U) {
    return false;
  }

  const auto origin_x = static_cast<std::uint32_t>(x);
  auto cursor_x = origin_x;
  auto cursor_y = static_cast<std::uint32_t>(y);
  const auto glyph_width = static_cast<std::uint16_t>(
    static_cast<std::uint32_t>(gfx::FontBitmap::kGlyphWidth) * scale);
  const auto glyph_height = static_cast<std::uint16_t>(
    static_cast<std::uint32_t>(gfx::FontBitmap::kGlyphHeight) * scale);
  const auto glyph_advance = static_cast<std::uint32_t>(gfx::FontBitmap::kAdvanceX) * scale;
  const auto line_advance = static_cast<std::uint32_t>(gfx::FontBitmap::kLineHeight) * scale;
  bool marked = false;

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
      marked = dirty_tracker_.markRect(
        static_cast<std::uint16_t>(cursor_x), static_cast<std::uint16_t>(cursor_y), glyph_width,
        glyph_height) || marked;
    }

    cursor_x += glyph_advance;
  }

  return marked;
}

}  // namespace st7789_ros_wrapper::app
