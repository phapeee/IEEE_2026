#include "st7789_ros_wrapper/driver/St7789.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <thread>
#include <vector>

namespace st7789_ros_wrapper::driver {
namespace {

constexpr std::uint8_t kCommandSwReset = 0x01;
constexpr std::uint8_t kCommandSleepOut = 0x11;
constexpr std::uint8_t kCommandNormalDisplayOn = 0x13;
constexpr std::uint8_t kCommandInversionOff = 0x20;
constexpr std::uint8_t kCommandInversionOn = 0x21;
constexpr std::uint8_t kCommandDisplayOn = 0x29;
constexpr std::uint8_t kCommandAddressColumn = 0x2A;
constexpr std::uint8_t kCommandAddressRow = 0x2B;
constexpr std::uint8_t kCommandRamWrite = 0x2C;
constexpr std::uint8_t kCommandMadctl = 0x36;
constexpr std::uint8_t kCommandPixelFormat = 0x3A;

constexpr std::uint8_t kMadctlMy = 0x80;
constexpr std::uint8_t kMadctlMx = 0x40;
constexpr std::uint8_t kMadctlMv = 0x20;
constexpr std::uint8_t kMadctlBgr = 0x08;

constexpr std::uint8_t kPixelFormatRgb565 = 0x55;
constexpr std::size_t kPixelChunkSize = 1024;

}  // namespace

St7789::St7789(
  hw::SpiBus & spi, hw::GpioLine & dc, hw::GpioLine & rst, hw::GpioLine & bl,
  St7789PanelConfig config)
: spi_(spi), dc_(dc), rst_(rst), bl_(bl), config_(config), rotation_(config.rotation) {}

bool St7789::initialize() {
  last_error_.clear();

  if (initialized_) {
    return true;
  }

  if (config_.width == 0 || config_.height == 0) {
    setLastError("Panel width and height must both be greater than zero");
    return false;
  }

  if (!isValidRotation(config_.rotation)) {
    std::ostringstream stream;
    stream << "Invalid rotation value " << static_cast<int>(config_.rotation) << "; valid range is 0-3";
    setLastError(stream.str());
    return false;
  }

  if (!spi_.open()) {
    setLastErrorFrom("Failed to initialize SPI bus", spi_.lastError());
    cleanupHardware();
    return false;
  }

  if (!dc_.requestOutput(false)) {
    setLastErrorFrom("Failed to request DC GPIO", dc_.lastError());
    cleanupHardware();
    return false;
  }

  if (!rst_.requestOutput(true)) {
    setLastErrorFrom("Failed to request RST GPIO", rst_.lastError());
    cleanupHardware();
    return false;
  }

  if (!bl_.requestOutput(false)) {
    setLastErrorFrom("Failed to request BL GPIO", bl_.lastError());
    cleanupHardware();
    return false;
  }

  if (!resetPanel()) {
    cleanupHardware();
    return false;
  }

  if (!writeCommand(kCommandSwReset)) {
    cleanupHardware();
    return false;
  }
  sleepMs(config_.post_swreset_delay_ms);

  if (!writeCommand(kCommandSleepOut)) {
    cleanupHardware();
    return false;
  }
  sleepMs(config_.post_sleep_out_delay_ms);

  if (!writeCommand(kCommandPixelFormat)) {
    cleanupHardware();
    return false;
  }

  const std::uint8_t pixel_format = kPixelFormatRgb565;
  if (!writeData(&pixel_format, 1)) {
    cleanupHardware();
    return false;
  }

  if (!applyRotation(config_.rotation)) {
    cleanupHardware();
    return false;
  }

  const auto inversion_command = config_.display_inversion ? kCommandInversionOn : kCommandInversionOff;
  if (!writeCommand(inversion_command)) {
    cleanupHardware();
    return false;
  }

  if (!writeCommand(kCommandNormalDisplayOn)) {
    cleanupHardware();
    return false;
  }

  if (!writeCommand(kCommandDisplayOn)) {
    cleanupHardware();
    return false;
  }
  sleepMs(config_.post_display_on_delay_ms);

  initialized_ = true;
  return true;
}

bool St7789::isInitialized() const noexcept {
  return initialized_;
}

bool St7789::setRotation(const std::uint8_t rotation) {
  last_error_.clear();

  if (!isValidRotation(rotation)) {
    std::ostringstream stream;
    stream << "Invalid rotation value " << static_cast<int>(rotation) << "; valid range is 0-3";
    setLastError(stream.str());
    return false;
  }

  if (!initialized_) {
    config_.rotation = rotation;
    rotation_ = rotation;
    return true;
  }

  return applyRotation(rotation);
}

bool St7789::setBacklight(const bool on) {
  last_error_.clear();

  if (!initialized_) {
    setLastError("Display is not initialized");
    return false;
  }

  if (!bl_.setValue(on)) {
    setLastErrorFrom("Failed to set BL GPIO", bl_.lastError());
    return false;
  }

  return true;
}

bool St7789::present(const std::uint16_t * pixels, const std::size_t pixel_count) {
  if (!initialized_) {
    setLastError("Display is not initialized");
    return false;
  }

  const auto expected_pixel_count = static_cast<std::size_t>(width()) * static_cast<std::size_t>(height());
  if (pixel_count != expected_pixel_count) {
    std::ostringstream stream;
    stream << "Pixel count mismatch: expected " << expected_pixel_count << " but got " << pixel_count;
    setLastError(stream.str());
    return false;
  }

  return presentRegion(pixels, pixel_count, 0, 0, width(), height());
}

bool St7789::presentRegion(
  const std::uint16_t * pixels, const std::size_t pixel_count, const std::uint16_t x,
  const std::uint16_t y, const std::uint16_t region_width, const std::uint16_t region_height)
{
  last_error_.clear();

  if (!initialized_) {
    setLastError("Display is not initialized");
    return false;
  }

  if (pixels == nullptr) {
    setLastError("Pixel buffer pointer is null");
    return false;
  }

  if (region_width == 0 || region_height == 0) {
    setLastError("Region width and height must both be greater than zero");
    return false;
  }

  if (x >= width() || y >= height()) {
    setLastError("Region origin is out of bounds");
    return false;
  }

  const auto x_end = static_cast<std::uint32_t>(x) + static_cast<std::uint32_t>(region_width);
  const auto y_end = static_cast<std::uint32_t>(y) + static_cast<std::uint32_t>(region_height);
  if (x_end > width() || y_end > height()) {
    setLastError("Region extends beyond display bounds");
    return false;
  }

  const auto required_pixels = static_cast<std::size_t>(region_width) * static_cast<std::size_t>(region_height);
  if (pixel_count != required_pixels) {
    std::ostringstream stream;
    stream << "Pixel count mismatch: expected " << required_pixels << " but got " << pixel_count;
    setLastError(stream.str());
    return false;
  }

  if (!setAddressWindow(
      x, y, static_cast<std::uint16_t>(x_end - 1U),
      static_cast<std::uint16_t>(y_end - 1U)))
  {
    return false;
  }

  if (!writeCommand(kCommandRamWrite)) {
    return false;
  }

  return writePixels(pixels, pixel_count);
}

std::uint16_t St7789::width() const noexcept {
  return (rotation_ % 2U == 0U) ? config_.width : config_.height;
}

std::uint16_t St7789::height() const noexcept {
  return (rotation_ % 2U == 0U) ? config_.height : config_.width;
}

const std::string & St7789::lastError() const noexcept {
  return last_error_;
}

bool St7789::applyRotation(const std::uint8_t rotation) {
  if (!writeCommand(kCommandMadctl)) {
    return false;
  }

  const std::uint8_t madctl = madctlValue(rotation, config_.color_order_bgr);
  if (!writeData(&madctl, 1)) {
    return false;
  }

  config_.rotation = rotation;
  rotation_ = rotation;
  return true;
}

bool St7789::resetPanel() {
  if (!rst_.setValue(false)) {
    setLastErrorFrom("Failed to set RST low", rst_.lastError());
    return false;
  }
  sleepMs(config_.reset_pulse_ms);

  if (!rst_.setValue(true)) {
    setLastErrorFrom("Failed to set RST high", rst_.lastError());
    return false;
  }
  sleepMs(config_.reset_recovery_ms);

  return true;
}

bool St7789::setAddressWindow(
  const std::uint16_t x0, const std::uint16_t y0, const std::uint16_t x1, const std::uint16_t y1)
{
  if (x0 > x1 || y0 > y1) {
    setLastError("Invalid address window coordinates");
    return false;
  }

  const auto panel_x0_u32 = static_cast<std::uint32_t>(x0) + static_cast<std::uint32_t>(config_.x_offset);
  const auto panel_x1_u32 = static_cast<std::uint32_t>(x1) + static_cast<std::uint32_t>(config_.x_offset);
  const auto panel_y0_u32 = static_cast<std::uint32_t>(y0) + static_cast<std::uint32_t>(config_.y_offset);
  const auto panel_y1_u32 = static_cast<std::uint32_t>(y1) + static_cast<std::uint32_t>(config_.y_offset);

  if (panel_x0_u32 > 0xFFFFU || panel_x1_u32 > 0xFFFFU || panel_y0_u32 > 0xFFFFU ||
      panel_y1_u32 > 0xFFFFU)
  {
    setLastError("Address window overflow after applying panel offsets");
    return false;
  }

  const auto panel_x0 = static_cast<std::uint16_t>(panel_x0_u32);
  const auto panel_x1 = static_cast<std::uint16_t>(panel_x1_u32);
  const auto panel_y0 = static_cast<std::uint16_t>(panel_y0_u32);
  const auto panel_y1 = static_cast<std::uint16_t>(panel_y1_u32);

  const std::uint8_t column_data[4] = {
    static_cast<std::uint8_t>((panel_x0 >> 8) & 0xFFU),
    static_cast<std::uint8_t>(panel_x0 & 0xFFU),
    static_cast<std::uint8_t>((panel_x1 >> 8) & 0xFFU),
    static_cast<std::uint8_t>(panel_x1 & 0xFFU),
  };

  const std::uint8_t row_data[4] = {
    static_cast<std::uint8_t>((panel_y0 >> 8) & 0xFFU),
    static_cast<std::uint8_t>(panel_y0 & 0xFFU),
    static_cast<std::uint8_t>((panel_y1 >> 8) & 0xFFU),
    static_cast<std::uint8_t>(panel_y1 & 0xFFU),
  };

  if (!writeCommand(kCommandAddressColumn) || !writeData(column_data, sizeof(column_data))) {
    return false;
  }

  if (!writeCommand(kCommandAddressRow) || !writeData(row_data, sizeof(row_data))) {
    return false;
  }

  return true;
}

bool St7789::writeCommand(const std::uint8_t command) {
  if (!dc_.setValue(false)) {
    setLastErrorFrom("Failed to set DC low for command", dc_.lastError());
    return false;
  }

  if (!spi_.write(&command, 1)) {
    setLastErrorFrom("Failed to write display command", spi_.lastError());
    return false;
  }

  return true;
}

bool St7789::writeData(const std::uint8_t * data, const std::size_t size) {
  if (data == nullptr || size == 0) {
    setLastError("Display data write requires a non-empty buffer");
    return false;
  }

  if (!dc_.setValue(true)) {
    setLastErrorFrom("Failed to set DC high for data", dc_.lastError());
    return false;
  }

  if (!spi_.write(data, size)) {
    setLastErrorFrom("Failed to write display data", spi_.lastError());
    return false;
  }

  return true;
}

bool St7789::writePixels(const std::uint16_t * pixels, const std::size_t pixel_count) {
  if (pixels == nullptr || pixel_count == 0) {
    setLastError("Pixel write requires a non-empty buffer");
    return false;
  }

  if (!dc_.setValue(true)) {
    setLastErrorFrom("Failed to set DC high for pixel write", dc_.lastError());
    return false;
  }

  std::vector<std::uint8_t> chunk(kPixelChunkSize * 2U, 0U);
  std::size_t written_pixels = 0;

  while (written_pixels < pixel_count) {
    const auto chunk_pixels = std::min(kPixelChunkSize, pixel_count - written_pixels);
    for (std::size_t i = 0; i < chunk_pixels; ++i) {
      const auto pixel = pixels[written_pixels + i];
      chunk[2U * i] = static_cast<std::uint8_t>((pixel >> 8) & 0xFFU);
      chunk[2U * i + 1U] = static_cast<std::uint8_t>(pixel & 0xFFU);
    }

    if (!spi_.write(chunk.data(), chunk_pixels * 2U)) {
      setLastErrorFrom("Failed to write pixel payload", spi_.lastError());
      return false;
    }

    written_pixels += chunk_pixels;
  }

  return true;
}

void St7789::cleanupHardware() {
  bl_.release();
  rst_.release();
  dc_.release();
  spi_.close();
  initialized_ = false;
}

void St7789::setLastError(const std::string & message) {
  last_error_ = message;
}

void St7789::setLastErrorFrom(const std::string & prefix, const std::string & child_error) {
  if (child_error.empty()) {
    setLastError(prefix);
    return;
  }

  setLastError(prefix + ": " + child_error);
}

bool St7789::isValidRotation(const std::uint8_t rotation) {
  return rotation <= 3U;
}

std::uint8_t St7789::madctlValue(const std::uint8_t rotation, const bool color_order_bgr) {
  const auto color_order = color_order_bgr ? kMadctlBgr : static_cast<std::uint8_t>(0x00);
  switch (rotation) {
    case 0:
      return static_cast<std::uint8_t>(kMadctlMx | kMadctlMy | color_order);
    case 1:
      return static_cast<std::uint8_t>(kMadctlMy | kMadctlMv | color_order);
    case 2:
      return color_order;
    case 3:
    default:
      return static_cast<std::uint8_t>(kMadctlMx | kMadctlMv | color_order);
  }
}

void St7789::sleepMs(const std::uint32_t milliseconds) {
  if (milliseconds == 0U) {
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

}  // namespace st7789_ros_wrapper::driver
