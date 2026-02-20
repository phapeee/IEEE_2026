#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "st7789_ros_wrapper/assets/ImageLoader.hpp"
#include "st7789_ros_wrapper/driver/St7789.hpp"
#include "st7789_ros_wrapper/app/FrameQueue.hpp"
#include "st7789_ros_wrapper/gfx/Canvas565.hpp"
#include "st7789_ros_wrapper/gfx/DirtyTracker.hpp"
#include "st7789_ros_wrapper/hw/GpioLine.hpp"
#include "st7789_ros_wrapper/hw/SpiBus.hpp"

namespace st7789_ros_wrapper::app {

struct ScreenConfig {
  hw::SpiBusConfig spi;
  std::string gpio_chip_path{"/dev/gpiochip4"};
  std::uint32_t dc_gpio{13};
  std::uint32_t rst_gpio{16};
  std::uint32_t bl_gpio{26};
  bool use_threads{false};
  std::uint32_t frame_queue_capacity{3};
  bool use_dirty_rects{false};
  double dirty_rect_full_frame_threshold{0.35};
  driver::St7789PanelConfig panel;
};

class Screen {
public:
  explicit Screen(ScreenConfig config = {});
  Screen(
    ScreenConfig config, const std::shared_ptr<hw::SpiBusBackend> & spi_backend,
    const std::shared_ptr<hw::GpioLineBackend> & gpio_backend);
  ~Screen();

  bool begin();
  bool setBacklight(bool on);
  void clear(gfx::Color565 color);
  bool drawImage(std::uint16_t x, std::uint16_t y, const assets::Image565 & image);
  bool drawText(
    std::uint16_t x, std::uint16_t y, const std::string & text, gfx::Color565 color,
    std::uint8_t scale = 1);
  bool present();

  gfx::Canvas565 & canvas() noexcept;
  const gfx::Canvas565 & canvas() const noexcept;

private:
  bool presentSync();
  bool buildRegionPixels(const gfx::RectU16 & region, std::vector<gfx::Color565> & pixels) const;
  bool presentDirtyRegion(const gfx::RectU16 & region);
  bool enqueueFrame(QueuedFrame frame);
  void stopIoThread();
  void ioThreadLoop();
  bool shouldUseFullPresent() const;
  bool trackTextDirtyRegion(
    std::uint16_t x, std::uint16_t y, const std::string & text, std::uint8_t scale);

  ScreenConfig config_;
  hw::SpiBus spi_;
  hw::GpioLine dc_;
  hw::GpioLine rst_;
  hw::GpioLine bl_;
  driver::St7789 driver_;
  gfx::Canvas565 canvas_;
  gfx::DirtyTracker dirty_tracker_;
  std::vector<gfx::Color565> region_pixels_;
  std::unique_ptr<FrameQueue> frame_queue_;
  std::thread io_thread_;
  std::mutex driver_mutex_;
  std::atomic<bool> async_present_failed_{false};
};

}  // namespace st7789_ros_wrapper::app
