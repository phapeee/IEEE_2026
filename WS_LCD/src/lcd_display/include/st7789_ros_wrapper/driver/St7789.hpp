#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "st7789_ros_wrapper/hw/GpioLine.hpp"
#include "st7789_ros_wrapper/hw/SpiBus.hpp"

namespace st7789_ros_wrapper::driver {

struct St7789PanelConfig {
  std::uint16_t width{240};
  std::uint16_t height{280};
  std::uint8_t rotation{0};
  bool color_order_bgr{true};
  bool display_inversion{true};
  std::uint16_t x_offset{0};
  std::uint16_t y_offset{0};
  std::uint32_t reset_pulse_ms{20};
  std::uint32_t reset_recovery_ms{120};
  std::uint32_t post_swreset_delay_ms{150};
  std::uint32_t post_sleep_out_delay_ms{120};
  std::uint32_t post_display_on_delay_ms{120};
};

class St7789 {
public:
  St7789(
    hw::SpiBus & spi, hw::GpioLine & dc, hw::GpioLine & rst, hw::GpioLine & bl,
    St7789PanelConfig config = {});

  bool initialize();
  bool isInitialized() const noexcept;
  bool setRotation(std::uint8_t rotation);
  bool setBacklight(bool on);
  bool present(const std::uint16_t * pixels, std::size_t pixel_count);
  bool presentRegion(
    const std::uint16_t * pixels, std::size_t pixel_count, std::uint16_t x, std::uint16_t y,
    std::uint16_t width, std::uint16_t height);

  std::uint16_t width() const noexcept;
  std::uint16_t height() const noexcept;
  const std::string & lastError() const noexcept;

private:
  bool applyRotation(std::uint8_t rotation);
  bool resetPanel();
  bool setAddressWindow(std::uint16_t x0, std::uint16_t y0, std::uint16_t x1, std::uint16_t y1);
  bool writeCommand(std::uint8_t command);
  bool writeData(const std::uint8_t * data, std::size_t size);
  bool writePixels(const std::uint16_t * pixels, std::size_t pixel_count);
  void cleanupHardware();
  void setLastError(const std::string & message);
  void setLastErrorFrom(const std::string & prefix, const std::string & child_error);
  static bool isValidRotation(std::uint8_t rotation);
  static std::uint8_t madctlValue(std::uint8_t rotation, bool color_order_bgr);
  static void sleepMs(std::uint32_t milliseconds);

  hw::SpiBus & spi_;
  hw::GpioLine & dc_;
  hw::GpioLine & rst_;
  hw::GpioLine & bl_;
  St7789PanelConfig config_;
  std::uint8_t rotation_{0};
  bool initialized_{false};
  std::string last_error_;
};

}  // namespace st7789_ros_wrapper::driver
