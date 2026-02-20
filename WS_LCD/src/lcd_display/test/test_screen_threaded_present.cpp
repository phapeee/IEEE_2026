#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/Screen.hpp"

namespace st7789_ros_wrapper::app {
namespace {

class ThreadedFakeSpiBusBackend final : public hw::SpiBusBackend {
public:
  int openDevice(const std::string &, int) override {
    return 7;
  }

  int closeDevice(int) override {
    return 0;
  }

  bool setMode(int, std::uint8_t) override {
    return true;
  }

  bool setBitsPerWord(int, std::uint8_t) override {
    return true;
  }

  bool setMaxSpeedHz(int, std::uint32_t) override {
    return true;
  }

  long writeBytes(int, const std::uint8_t * data, const std::size_t size) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      writes_.emplace_back(data, data + size);
    }
    condition_.notify_all();
    return static_cast<long>(size);
  }

  int lastErrorNumber() const override {
    return 5;
  }

  std::string errorMessage(int error_number) const override {
    return "err-" + std::to_string(error_number);
  }

  void clearWrites() {
    std::lock_guard<std::mutex> lock(mutex_);
    writes_.clear();
  }

  bool waitForWrites(
    const std::size_t expected, const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [&]() {
        return writes_.size() >= expected;
      });
  }

  bool waitForSequence(
    const std::function<bool(const std::vector<std::vector<std::uint8_t>> &)> & predicate,
    const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [&]() {
        return predicate(writes_);
      });
  }

  std::vector<std::vector<std::uint8_t>> snapshotWrites() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<std::vector<std::uint8_t>> writes_;
};

class FakeGpioLineBackend final : public hw::GpioLineBackend {
public:
  gpiod_chip * openChip(const std::string &) override {
    return reinterpret_cast<gpiod_chip *>(0x1000);
  }

  void closeChip(gpiod_chip *) override {}

  gpiod_line * getLine(gpiod_chip *, unsigned int) override {
    return reinterpret_cast<gpiod_line *>(0x2000);
  }

  int requestOutput(gpiod_line *, const std::string &, int) override {
    return 0;
  }

  int setValue(gpiod_line *, int) override {
    return 0;
  }

  void releaseLine(gpiod_line *) override {}

  int lastErrorNumber() const override {
    return 5;
  }

  std::string errorMessage(int error_number) const override {
    return "err-" + std::to_string(error_number);
  }
};

ScreenConfig makeThreadedScreenConfig(const bool use_dirty_rects, const double threshold) {
  ScreenConfig config;
  config.use_threads = true;
  config.frame_queue_capacity = 3U;
  config.use_dirty_rects = use_dirty_rects;
  config.dirty_rect_full_frame_threshold = threshold;
  config.panel.width = 20;
  config.panel.height = 20;
  config.panel.reset_pulse_ms = 0;
  config.panel.reset_recovery_ms = 0;
  config.panel.post_swreset_delay_ms = 0;
  config.panel.post_sleep_out_delay_ms = 0;
  config.panel.post_display_on_delay_ms = 0;
  return config;
}

bool containsTransferSequence(
  const std::vector<std::vector<std::uint8_t>> & writes,
  const std::vector<std::uint8_t> & x_window,
  const std::vector<std::uint8_t> & y_window,
  const std::size_t payload_bytes)
{
  if (writes.size() < 6U) {
    return false;
  }

  for (std::size_t i = 0; i + 5U < writes.size(); ++i) {
    if (writes[i] == std::vector<std::uint8_t>{0x2A} &&
      writes[i + 1U] == x_window &&
      writes[i + 2U] == std::vector<std::uint8_t>{0x2B} &&
      writes[i + 3U] == y_window &&
      writes[i + 4U] == std::vector<std::uint8_t>{0x2C} &&
      writes[i + 5U].size() == payload_bytes)
    {
      return true;
    }
  }

  return false;
}

TEST(ScreenThreadedPresentTest, FlushesFullFrameAsynchronously) {
  auto spi_backend = std::make_shared<ThreadedFakeSpiBusBackend>();
  auto gpio_backend = std::make_shared<FakeGpioLineBackend>();
  Screen screen(makeThreadedScreenConfig(false, 0.35), spi_backend, gpio_backend);
  ASSERT_TRUE(screen.begin());

  spi_backend->clearWrites();
  screen.clear(gfx::color::kBlack);
  ASSERT_TRUE(screen.present());

  ASSERT_TRUE(spi_backend->waitForSequence(
      [](const auto & writes) {
        return containsTransferSequence(
          writes, {0x00, 0x00, 0x00, 0x13}, {0x00, 0x00, 0x00, 0x13}, 20U * 20U * 2U);
      },
      std::chrono::milliseconds(200)));
}

TEST(ScreenThreadedPresentTest, UsesDirtyRegionForSmallTextUpdate) {
  auto spi_backend = std::make_shared<ThreadedFakeSpiBusBackend>();
  auto gpio_backend = std::make_shared<FakeGpioLineBackend>();
  Screen screen(makeThreadedScreenConfig(true, 0.35), spi_backend, gpio_backend);
  ASSERT_TRUE(screen.begin());

  screen.clear(gfx::color::kBlack);
  ASSERT_TRUE(screen.present());
  ASSERT_TRUE(spi_backend->waitForWrites(6U, std::chrono::milliseconds(200)));
  spi_backend->clearWrites();

  ASSERT_TRUE(screen.drawText(1, 2, "A", gfx::color::kWhite, 1));
  ASSERT_TRUE(screen.present());
  ASSERT_TRUE(spi_backend->waitForSequence(
      [](const auto & writes) {
        return containsTransferSequence(
          writes, {0x00, 0x01, 0x00, 0x08}, {0x00, 0x02, 0x00, 0x09}, 8U * 8U * 2U);
      },
      std::chrono::milliseconds(200)));
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
