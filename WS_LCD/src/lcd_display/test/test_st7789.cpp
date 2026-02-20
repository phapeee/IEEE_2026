#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/driver/St7789.hpp"

namespace st7789_ros_wrapper::driver {
namespace {

class FakeSpiBusBackend final : public hw::SpiBusBackend {
public:
  int open_return_fd{7};
  bool set_mode_result{true};
  bool set_bits_result{true};
  bool set_speed_result{true};
  int fail_write_call_index{-1};
  long fail_write_return{-1};
  int last_error_number{5};

  int open_calls{0};
  int close_calls{0};
  int set_mode_calls{0};
  int set_bits_calls{0};
  int set_speed_calls{0};
  int write_calls{0};

  std::vector<std::vector<std::uint8_t>> writes;

  int openDevice(const std::string &, int) override {
    ++open_calls;
    return open_return_fd;
  }

  int closeDevice(int) override {
    ++close_calls;
    return 0;
  }

  bool setMode(int, std::uint8_t) override {
    ++set_mode_calls;
    return set_mode_result;
  }

  bool setBitsPerWord(int, std::uint8_t) override {
    ++set_bits_calls;
    return set_bits_result;
  }

  bool setMaxSpeedHz(int, std::uint32_t) override {
    ++set_speed_calls;
    return set_speed_result;
  }

  long writeBytes(int, const std::uint8_t * data, const std::size_t size) override {
    ++write_calls;

    if (fail_write_call_index > 0 && write_calls == fail_write_call_index) {
      return fail_write_return;
    }

    writes.emplace_back(data, data + size);
    return static_cast<long>(size);
  }

  int lastErrorNumber() const override {
    return last_error_number;
  }

  std::string errorMessage(const int error_number) const override {
    return "err-" + std::to_string(error_number);
  }
};

class FakeGpioLineBackend final : public hw::GpioLineBackend {
public:
  gpiod_chip * open_chip_result{reinterpret_cast<gpiod_chip *>(0x1000)};
  gpiod_line * get_line_result{reinterpret_cast<gpiod_line *>(0x2000)};
  int request_output_result{0};
  int fail_set_call_index{-1};
  int last_error_number{5};

  int open_chip_calls{0};
  int close_chip_calls{0};
  int get_line_calls{0};
  int request_output_calls{0};
  int set_value_calls{0};
  int release_line_calls{0};

  std::vector<int> set_values;

  gpiod_chip * openChip(const std::string &) override {
    ++open_chip_calls;
    return open_chip_result;
  }

  void closeChip(gpiod_chip *) override {
    ++close_chip_calls;
  }

  gpiod_line * getLine(gpiod_chip *, unsigned int) override {
    ++get_line_calls;
    return get_line_result;
  }

  int requestOutput(gpiod_line *, const std::string &, int) override {
    ++request_output_calls;
    return request_output_result;
  }

  int setValue(gpiod_line *, int value) override {
    ++set_value_calls;

    if (fail_set_call_index > 0 && set_value_calls == fail_set_call_index) {
      return -1;
    }

    set_values.push_back(value);
    return 0;
  }

  void releaseLine(gpiod_line *) override {
    ++release_line_calls;
  }

  int lastErrorNumber() const override {
    return last_error_number;
  }

  std::string errorMessage(const int error_number) const override {
    return "err-" + std::to_string(error_number);
  }
};

hw::GpioLine makeLine(const std::uint32_t offset, const std::shared_ptr<FakeGpioLineBackend> & backend) {
  hw::GpioLineConfig config;
  config.offset = offset;
  return hw::GpioLine(config, backend);
}

St7789PanelConfig defaultPanelConfig() {
  St7789PanelConfig config;
  config.reset_pulse_ms = 0;
  config.reset_recovery_ms = 0;
  config.post_swreset_delay_ms = 0;
  config.post_sleep_out_delay_ms = 0;
  config.post_display_on_delay_ms = 0;
  return config;
}

TEST(St7789Test, InitializeSuccessProgramsController) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  auto panel_config = defaultPanelConfig();
  panel_config.rotation = 1;

  St7789 driver(spi, dc, rst, bl, panel_config);

  ASSERT_TRUE(driver.initialize());
  EXPECT_TRUE(driver.isInitialized());
  EXPECT_TRUE(driver.lastError().empty());
  EXPECT_EQ(driver.width(), 280U);
  EXPECT_EQ(driver.height(), 240U);

  ASSERT_EQ(spi_backend->writes.size(), 9U);
  EXPECT_EQ(spi_backend->writes[0], (std::vector<std::uint8_t>{0x01}));
  EXPECT_EQ(spi_backend->writes[1], (std::vector<std::uint8_t>{0x11}));
  EXPECT_EQ(spi_backend->writes[2], (std::vector<std::uint8_t>{0x3A}));
  EXPECT_EQ(spi_backend->writes[3], (std::vector<std::uint8_t>{0x55}));
  EXPECT_EQ(spi_backend->writes[4], (std::vector<std::uint8_t>{0x36}));
  EXPECT_EQ(spi_backend->writes[5], (std::vector<std::uint8_t>{0xA8}));
  EXPECT_EQ(spi_backend->writes[6], (std::vector<std::uint8_t>{0x21}));
  EXPECT_EQ(spi_backend->writes[7], (std::vector<std::uint8_t>{0x13}));
  EXPECT_EQ(spi_backend->writes[8], (std::vector<std::uint8_t>{0x29}));

  EXPECT_EQ(rst_backend->set_values, (std::vector<int>{0, 1}));
}

TEST(St7789Test, InitializeSupportsRgbOrderAndInversionOffConfig) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  auto panel_config = defaultPanelConfig();
  panel_config.rotation = 1;
  panel_config.color_order_bgr = false;
  panel_config.display_inversion = false;

  St7789 driver(spi, dc, rst, bl, panel_config);

  ASSERT_TRUE(driver.initialize());
  EXPECT_TRUE(driver.isInitialized());
  EXPECT_TRUE(driver.lastError().empty());

  ASSERT_EQ(spi_backend->writes.size(), 9U);
  EXPECT_EQ(spi_backend->writes[4], (std::vector<std::uint8_t>{0x36}));
  EXPECT_EQ(spi_backend->writes[5], (std::vector<std::uint8_t>{0xA0}));
  EXPECT_EQ(spi_backend->writes[6], (std::vector<std::uint8_t>{0x20}));
}

TEST(St7789Test, InitializeFailsForInvalidRotationWithoutTouchingHardware) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  auto panel_config = defaultPanelConfig();
  panel_config.rotation = 5;

  St7789 driver(spi, dc, rst, bl, panel_config);

  EXPECT_FALSE(driver.initialize());
  EXPECT_FALSE(driver.isInitialized());
  EXPECT_NE(driver.lastError().find("Invalid rotation value"), std::string::npos);

  EXPECT_EQ(spi_backend->open_calls, 0);
  EXPECT_EQ(dc_backend->request_output_calls, 0);
  EXPECT_EQ(rst_backend->request_output_calls, 0);
  EXPECT_EQ(bl_backend->request_output_calls, 0);
}

TEST(St7789Test, InitializeFailureCleansUpResources) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  spi_backend->fail_write_call_index = 1;
  spi_backend->last_error_number = 16;

  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

  EXPECT_FALSE(driver.initialize());
  EXPECT_FALSE(driver.isInitialized());
  EXPECT_NE(driver.lastError().find("Failed to write display command"), std::string::npos);

  EXPECT_EQ(spi_backend->close_calls, 1);
  EXPECT_EQ(dc_backend->release_line_calls, 1);
  EXPECT_EQ(rst_backend->release_line_calls, 1);
  EXPECT_EQ(bl_backend->release_line_calls, 1);
}

TEST(St7789Test, BacklightRequiresInitializationThenWorks) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

  EXPECT_FALSE(driver.setBacklight(true));
  EXPECT_NE(driver.lastError().find("Display is not initialized"), std::string::npos);

  ASSERT_TRUE(driver.initialize());

  EXPECT_TRUE(driver.setBacklight(true));
  ASSERT_FALSE(bl_backend->set_values.empty());
  EXPECT_EQ(bl_backend->set_values.back(), 1);

  EXPECT_TRUE(driver.setBacklight(false));
  EXPECT_EQ(bl_backend->set_values.back(), 0);
}

TEST(St7789Test, PresentWritesFullFrameWithOffsets) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  auto panel_config = defaultPanelConfig();
  panel_config.width = 2;
  panel_config.height = 2;
  panel_config.x_offset = 1;
  panel_config.y_offset = 2;

  St7789 driver(spi, dc, rst, bl, panel_config);

  EXPECT_FALSE(driver.present(nullptr, 0));
  EXPECT_NE(driver.lastError().find("Display is not initialized"), std::string::npos);

  ASSERT_TRUE(driver.initialize());
  const auto writes_before = spi_backend->writes.size();

  const std::vector<std::uint16_t> pixels{0x1234, 0xABCD, 0x0001, 0xFFFF};
  ASSERT_TRUE(driver.present(pixels.data(), pixels.size()));

  ASSERT_EQ(spi_backend->writes.size(), writes_before + 6U);
  EXPECT_EQ(spi_backend->writes[writes_before + 0U], (std::vector<std::uint8_t>{0x2A}));
  EXPECT_EQ(spi_backend->writes[writes_before + 1U], (std::vector<std::uint8_t>{0x00, 0x01, 0x00, 0x02}));
  EXPECT_EQ(spi_backend->writes[writes_before + 2U], (std::vector<std::uint8_t>{0x2B}));
  EXPECT_EQ(spi_backend->writes[writes_before + 3U], (std::vector<std::uint8_t>{0x00, 0x02, 0x00, 0x03}));
  EXPECT_EQ(spi_backend->writes[writes_before + 4U], (std::vector<std::uint8_t>{0x2C}));
  EXPECT_EQ(
    spi_backend->writes[writes_before + 5U],
    (std::vector<std::uint8_t>{0x12, 0x34, 0xAB, 0xCD, 0x00, 0x01, 0xFF, 0xFF}));

  EXPECT_FALSE(driver.present(pixels.data(), 3));
  EXPECT_NE(driver.lastError().find("Pixel count mismatch"), std::string::npos);
}

TEST(St7789Test, PresentRegionAndRotationAreValidated) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  auto panel_config = defaultPanelConfig();
  panel_config.width = 4;
  panel_config.height = 3;

  St7789 driver(spi, dc, rst, bl, panel_config);
  ASSERT_TRUE(driver.initialize());

  const auto writes_before_rotation = spi_backend->writes.size();
  ASSERT_TRUE(driver.setRotation(3));
  EXPECT_EQ(driver.width(), 3U);
  EXPECT_EQ(driver.height(), 4U);

  ASSERT_EQ(spi_backend->writes.size(), writes_before_rotation + 2U);
  EXPECT_EQ(spi_backend->writes[writes_before_rotation], (std::vector<std::uint8_t>{0x36}));
  EXPECT_EQ(spi_backend->writes[writes_before_rotation + 1U], (std::vector<std::uint8_t>{0x68}));

  EXPECT_FALSE(driver.setRotation(7));
  EXPECT_NE(driver.lastError().find("Invalid rotation value"), std::string::npos);

  const std::vector<std::uint16_t> pixels{0x1111, 0x2222, 0x3333, 0x4444};

  const auto writes_before_region = spi_backend->writes.size();
  ASSERT_TRUE(driver.presentRegion(pixels.data(), pixels.size(), 1, 1, 2, 2));

  ASSERT_EQ(spi_backend->writes.size(), writes_before_region + 6U);
  EXPECT_EQ(spi_backend->writes[writes_before_region + 0U], (std::vector<std::uint8_t>{0x2A}));
  EXPECT_EQ(spi_backend->writes[writes_before_region + 1U], (std::vector<std::uint8_t>{0x00, 0x01, 0x00, 0x02}));
  EXPECT_EQ(spi_backend->writes[writes_before_region + 2U], (std::vector<std::uint8_t>{0x2B}));
  EXPECT_EQ(spi_backend->writes[writes_before_region + 3U], (std::vector<std::uint8_t>{0x00, 0x01, 0x00, 0x02}));
  EXPECT_EQ(spi_backend->writes[writes_before_region + 4U], (std::vector<std::uint8_t>{0x2C}));
  EXPECT_EQ(
    spi_backend->writes[writes_before_region + 5U],
    (std::vector<std::uint8_t>{0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x44, 0x44}));

  EXPECT_FALSE(driver.presentRegion(pixels.data(), pixels.size(), 2, 3, 2, 2));
  EXPECT_NE(driver.lastError().find("Region extends beyond display bounds"), std::string::npos);

  EXPECT_FALSE(driver.presentRegion(pixels.data(), 3, 1, 1, 2, 2));
  EXPECT_NE(driver.lastError().find("Pixel count mismatch"), std::string::npos);
}

TEST(St7789Test, InitializeHandlesSpiAndGpioBringupFailures) {
  {
    auto spi_backend = std::make_shared<FakeSpiBusBackend>();
    spi_backend->open_return_fd = -1;
    auto dc_backend = std::make_shared<FakeGpioLineBackend>();
    auto rst_backend = std::make_shared<FakeGpioLineBackend>();
    auto bl_backend = std::make_shared<FakeGpioLineBackend>();

    hw::SpiBus spi({}, spi_backend);
    auto dc = makeLine(13, dc_backend);
    auto rst = makeLine(16, rst_backend);
    auto bl = makeLine(26, bl_backend);
    St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

    EXPECT_FALSE(driver.initialize());
    EXPECT_NE(driver.lastError().find("Failed to initialize SPI bus"), std::string::npos);
    EXPECT_EQ(spi_backend->open_calls, 1);
    EXPECT_EQ(dc_backend->request_output_calls, 0);
  }

  {
    auto spi_backend = std::make_shared<FakeSpiBusBackend>();
    auto dc_backend = std::make_shared<FakeGpioLineBackend>();
    dc_backend->request_output_result = -1;
    auto rst_backend = std::make_shared<FakeGpioLineBackend>();
    auto bl_backend = std::make_shared<FakeGpioLineBackend>();

    hw::SpiBus spi({}, spi_backend);
    auto dc = makeLine(13, dc_backend);
    auto rst = makeLine(16, rst_backend);
    auto bl = makeLine(26, bl_backend);
    St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

    EXPECT_FALSE(driver.initialize());
    EXPECT_NE(driver.lastError().find("Failed to request DC GPIO"), std::string::npos);
    EXPECT_EQ(spi_backend->close_calls, 1);
    EXPECT_EQ(dc_backend->release_line_calls, 1);
  }

  {
    auto spi_backend = std::make_shared<FakeSpiBusBackend>();
    auto dc_backend = std::make_shared<FakeGpioLineBackend>();
    auto rst_backend = std::make_shared<FakeGpioLineBackend>();
    rst_backend->request_output_result = -1;
    auto bl_backend = std::make_shared<FakeGpioLineBackend>();

    hw::SpiBus spi({}, spi_backend);
    auto dc = makeLine(13, dc_backend);
    auto rst = makeLine(16, rst_backend);
    auto bl = makeLine(26, bl_backend);
    St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

    EXPECT_FALSE(driver.initialize());
    EXPECT_NE(driver.lastError().find("Failed to request RST GPIO"), std::string::npos);
    EXPECT_EQ(spi_backend->close_calls, 1);
    EXPECT_EQ(dc_backend->release_line_calls, 1);
  }

  {
    auto spi_backend = std::make_shared<FakeSpiBusBackend>();
    auto dc_backend = std::make_shared<FakeGpioLineBackend>();
    auto rst_backend = std::make_shared<FakeGpioLineBackend>();
    auto bl_backend = std::make_shared<FakeGpioLineBackend>();
    bl_backend->request_output_result = -1;

    hw::SpiBus spi({}, spi_backend);
    auto dc = makeLine(13, dc_backend);
    auto rst = makeLine(16, rst_backend);
    auto bl = makeLine(26, bl_backend);
    St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

    EXPECT_FALSE(driver.initialize());
    EXPECT_NE(driver.lastError().find("Failed to request BL GPIO"), std::string::npos);
    EXPECT_EQ(spi_backend->close_calls, 1);
    EXPECT_EQ(dc_backend->release_line_calls, 1);
    EXPECT_EQ(rst_backend->release_line_calls, 1);
  }
}

TEST(St7789Test, InitializeFailsIfResetLineCannotToggle) {
  {
    auto spi_backend = std::make_shared<FakeSpiBusBackend>();
    auto dc_backend = std::make_shared<FakeGpioLineBackend>();
    auto rst_backend = std::make_shared<FakeGpioLineBackend>();
    rst_backend->fail_set_call_index = 1;
    auto bl_backend = std::make_shared<FakeGpioLineBackend>();

    hw::SpiBus spi({}, spi_backend);
    auto dc = makeLine(13, dc_backend);
    auto rst = makeLine(16, rst_backend);
    auto bl = makeLine(26, bl_backend);
    St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

    EXPECT_FALSE(driver.initialize());
    EXPECT_NE(driver.lastError().find("Failed to set RST low"), std::string::npos);
  }

  {
    auto spi_backend = std::make_shared<FakeSpiBusBackend>();
    auto dc_backend = std::make_shared<FakeGpioLineBackend>();
    auto rst_backend = std::make_shared<FakeGpioLineBackend>();
    rst_backend->fail_set_call_index = 2;
    auto bl_backend = std::make_shared<FakeGpioLineBackend>();

    hw::SpiBus spi({}, spi_backend);
    auto dc = makeLine(13, dc_backend);
    auto rst = makeLine(16, rst_backend);
    auto bl = makeLine(26, bl_backend);
    St7789 driver(spi, dc, rst, bl, defaultPanelConfig());

    EXPECT_FALSE(driver.initialize());
    EXPECT_NE(driver.lastError().find("Failed to set RST high"), std::string::npos);
  }
}

TEST(St7789Test, PresentRegionRejectsInvalidArguments) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  auto panel_config = defaultPanelConfig();
  panel_config.width = 3;
  panel_config.height = 3;
  St7789 driver(spi, dc, rst, bl, panel_config);
  ASSERT_TRUE(driver.initialize());

  const std::vector<std::uint16_t> pixels{0x1111, 0x2222, 0x3333, 0x4444};

  EXPECT_FALSE(driver.presentRegion(nullptr, 4, 0, 0, 2, 2));
  EXPECT_NE(driver.lastError().find("Pixel buffer pointer is null"), std::string::npos);

  EXPECT_FALSE(driver.presentRegion(pixels.data(), 4, 0, 0, 0, 2));
  EXPECT_NE(driver.lastError().find("Region width and height must both be greater than zero"), std::string::npos);

  EXPECT_FALSE(driver.presentRegion(pixels.data(), 4, 3, 0, 1, 1));
  EXPECT_NE(driver.lastError().find("Region origin is out of bounds"), std::string::npos);
}

TEST(St7789Test, PresentFailsWhenWindowOverflowsAfterOffset) {
  auto spi_backend = std::make_shared<FakeSpiBusBackend>();
  auto dc_backend = std::make_shared<FakeGpioLineBackend>();
  auto rst_backend = std::make_shared<FakeGpioLineBackend>();
  auto bl_backend = std::make_shared<FakeGpioLineBackend>();

  hw::SpiBus spi({}, spi_backend);
  auto dc = makeLine(13, dc_backend);
  auto rst = makeLine(16, rst_backend);
  auto bl = makeLine(26, bl_backend);

  auto panel_config = defaultPanelConfig();
  panel_config.width = 2;
  panel_config.height = 2;
  panel_config.x_offset = 0xFFFF;
  St7789 driver(spi, dc, rst, bl, panel_config);
  ASSERT_TRUE(driver.initialize());

  const std::vector<std::uint16_t> pixels{0x0001, 0x0002, 0x0003, 0x0004};
  EXPECT_FALSE(driver.present(pixels.data(), pixels.size()));
  EXPECT_NE(driver.lastError().find("Address window overflow"), std::string::npos);
}

}  // namespace
}  // namespace st7789_ros_wrapper::driver
