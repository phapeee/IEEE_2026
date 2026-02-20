#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"

#include <limits>
#include <sstream>

namespace st7789_ros_wrapper::app {
namespace {

template<typename TargetType>
bool fitsIn(const std::int64_t value) {
  return value >= static_cast<std::int64_t>(std::numeric_limits<TargetType>::min()) &&
         value <= static_cast<std::int64_t>(std::numeric_limits<TargetType>::max());
}

bool convertRotationDegrees(const std::int64_t degrees, std::uint8_t & rotation, std::string & error) {
  switch (degrees) {
    case 0:
      rotation = 0U;
      return true;
    case 90:
      rotation = 1U;
      return true;
    case 180:
      rotation = 2U;
      return true;
    case 270:
      rotation = 3U;
      return true;
    default:
      std::ostringstream stream;
      stream << "rotation_degrees must be one of 0, 90, 180, 270; got " << degrees;
      error = stream.str();
      return false;
  }
}

template<typename TargetType>
bool assignInteger(
  const std::int64_t input, const char * const name, TargetType & output, std::string & error)
{
  if (!fitsIn<TargetType>(input)) {
    std::ostringstream stream;
    stream << name << " value " << input << " is out of range";
    error = stream.str();
    return false;
  }

  output = static_cast<TargetType>(input);
  return true;
}

std::string normalizeGpioChipPath(const std::string & gpiochip) {
  constexpr const char * kDevPrefix = "/dev/";
  if (gpiochip.rfind(kDevPrefix, 0) == 0U) {
    return gpiochip;
  }
  return std::string{kDevPrefix} + gpiochip;
}

}  // namespace

bool buildScreenConfig(
  const ScreenRuntimeConfig & runtime, ScreenConfig & output, std::string & error)
{
  error.clear();
  output = ScreenConfig{};

  if (runtime.spi_device.empty()) {
    error = "spi_device cannot be empty";
    return false;
  }
  output.spi.device_path = runtime.spi_device;

  if (runtime.gpiochip.empty()) {
    error = "gpiochip cannot be empty";
    return false;
  }
  output.gpio_chip_path = normalizeGpioChipPath(runtime.gpiochip);

  if (runtime.spi_speed_hz <= 0) {
    error = "spi_speed_hz must be greater than zero";
    return false;
  }
  if (!assignInteger<std::uint32_t>(
      runtime.spi_speed_hz, "spi_speed_hz", output.spi.speed_hz, error))
  {
    return false;
  }

  if (!assignInteger<std::uint8_t>(runtime.spi_mode, "spi_mode", output.spi.mode, error)) {
    return false;
  }
  if (output.spi.mode > 3U) {
    std::ostringstream stream;
    stream << "spi_mode must be between 0 and 3; got " << runtime.spi_mode;
    error = stream.str();
    return false;
  }

  if (runtime.spi_bits_per_word <= 0) {
    error = "spi_bits_per_word must be greater than zero";
    return false;
  }
  if (!assignInteger<std::uint8_t>(
      runtime.spi_bits_per_word, "spi_bits_per_word", output.spi.bits_per_word, error))
  {
    return false;
  }

  if (!assignInteger<std::uint32_t>(runtime.dc_gpio, "dc_gpio", output.dc_gpio, error)) {
    return false;
  }
  if (!assignInteger<std::uint32_t>(runtime.rst_gpio, "rst_gpio", output.rst_gpio, error)) {
    return false;
  }
  if (!assignInteger<std::uint32_t>(runtime.bl_gpio, "bl_gpio", output.bl_gpio, error)) {
    return false;
  }

  output.use_threads = runtime.use_threads;
  if (runtime.frame_queue_capacity <= 0) {
    error = "frame_queue_capacity must be greater than zero";
    return false;
  }
  if (!assignInteger<std::uint32_t>(
      runtime.frame_queue_capacity, "frame_queue_capacity", output.frame_queue_capacity, error))
  {
    return false;
  }

  if (!convertRotationDegrees(runtime.rotation_degrees, output.panel.rotation, error)) {
    return false;
  }
  output.panel.color_order_bgr = runtime.color_order_bgr;
  output.panel.display_inversion = runtime.display_inversion;

  if (!assignInteger<std::uint16_t>(runtime.x_offset, "x_offset", output.panel.x_offset, error)) {
    return false;
  }
  if (!assignInteger<std::uint16_t>(runtime.y_offset, "y_offset", output.panel.y_offset, error)) {
    return false;
  }

  return true;
}

}  // namespace st7789_ros_wrapper::app
