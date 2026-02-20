#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/Screen.hpp"

namespace st7789_ros_wrapper::app {
namespace {

class FakeSpiBusBackend final : public hw::SpiBusBackend {
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
    writes.emplace_back(data, data + size);
    return static_cast<long>(size);
  }

  int lastErrorNumber() const override {
    return 5;
  }

  std::string errorMessage(int error_number) const override {
    return "err-" + std::to_string(error_number);
  }

  std::vector<std::vector<std::uint8_t>> writes;
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

ScreenConfig makeScreenConfig(const double threshold) {
  ScreenConfig config;
  config.use_dirty_rects = true;
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

TEST(ScreenDirtyPresentTest, UsesPartialUpdateForSmallDirtyRegion) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto gpio_backend = std::make_shared<FakeGpioLineBackend>();
  Screen screen(makeScreenConfig(0.35), spi_backend, gpio_backend);
  ASSERT_TRUE(screen.begin());

  screen.clear(gfx::color::kBlack);
  ASSERT_TRUE(screen.present());
  spi_backend->writes.clear();

  ASSERT_TRUE(screen.drawText(1, 2, "A", gfx::color::kWhite, 1));
  ASSERT_TRUE(screen.present());

  ASSERT_EQ(spi_backend->writes.size(), 6U);
  EXPECT_EQ(spi_backend->writes[0], (std::vector<std::uint8_t>{0x2A}));
  EXPECT_EQ(spi_backend->writes[1], (std::vector<std::uint8_t>{0x00, 0x01, 0x00, 0x08}));
  EXPECT_EQ(spi_backend->writes[2], (std::vector<std::uint8_t>{0x2B}));
  EXPECT_EQ(spi_backend->writes[3], (std::vector<std::uint8_t>{0x00, 0x02, 0x00, 0x09}));
  EXPECT_EQ(spi_backend->writes[4], (std::vector<std::uint8_t>{0x2C}));
  EXPECT_EQ(spi_backend->writes[5].size(), static_cast<std::size_t>(8U * 8U * 2U));
}

TEST(ScreenDirtyPresentTest, FallsBackToFullFrameWhenDirtyCoverageExceedsThreshold) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto gpio_backend = std::make_shared<FakeGpioLineBackend>();
  Screen screen(makeScreenConfig(0.10), spi_backend, gpio_backend);
  ASSERT_TRUE(screen.begin());

  screen.clear(gfx::color::kBlack);
  ASSERT_TRUE(screen.present());
  spi_backend->writes.clear();

  ASSERT_TRUE(screen.drawText(1, 2, "A", gfx::color::kWhite, 1));
  ASSERT_TRUE(screen.present());

  ASSERT_EQ(spi_backend->writes.size(), 6U);
  EXPECT_EQ(spi_backend->writes[1], (std::vector<std::uint8_t>{0x00, 0x00, 0x00, 0x13}));
  EXPECT_EQ(spi_backend->writes[3], (std::vector<std::uint8_t>{0x00, 0x00, 0x00, 0x13}));
  EXPECT_EQ(spi_backend->writes[5].size(), static_cast<std::size_t>(20U * 20U * 2U));
}

TEST(ScreenDirtyPresentTest, SkipsSpiTransferWhenNoNewDrawingOccurred) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto gpio_backend = std::make_shared<FakeGpioLineBackend>();
  Screen screen(makeScreenConfig(0.35), spi_backend, gpio_backend);
  ASSERT_TRUE(screen.begin());

  screen.clear(gfx::color::kBlack);
  ASSERT_TRUE(screen.present());
  spi_backend->writes.clear();

  ASSERT_TRUE(screen.present());
  EXPECT_TRUE(spi_backend->writes.empty());
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
