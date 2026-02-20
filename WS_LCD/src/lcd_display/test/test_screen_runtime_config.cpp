#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"

namespace st7789_ros_wrapper::app {
namespace {

TEST(ScreenRuntimeConfigTest, BuildsScreenConfigFromValidRuntimeConfig) {
  ScreenRuntimeConfig runtime;
  runtime.spi_device = "/dev/spidev0.1";
  runtime.spi_speed_hz = 8000000;
  runtime.spi_mode = 3;
  runtime.spi_bits_per_word = 8;
  runtime.gpiochip = "gpiochip7";
  runtime.dc_gpio = 5;
  runtime.rst_gpio = 6;
  runtime.bl_gpio = 12;
  runtime.use_threads = true;
  runtime.frame_queue_capacity = 4;
  runtime.rotation_degrees = 270;
  runtime.color_order_bgr = false;
  runtime.display_inversion = false;
  runtime.x_offset = 4;
  runtime.y_offset = 7;

  ScreenConfig output;
  std::string error;
  ASSERT_TRUE(buildScreenConfig(runtime, output, error));
  EXPECT_TRUE(error.empty());

  EXPECT_EQ(output.spi.device_path, "/dev/spidev0.1");
  EXPECT_EQ(output.spi.speed_hz, 8000000U);
  EXPECT_EQ(output.spi.mode, 3U);
  EXPECT_EQ(output.spi.bits_per_word, 8U);
  EXPECT_EQ(output.gpio_chip_path, "/dev/gpiochip7");
  EXPECT_EQ(output.dc_gpio, 5U);
  EXPECT_EQ(output.rst_gpio, 6U);
  EXPECT_EQ(output.bl_gpio, 12U);
  EXPECT_TRUE(output.use_threads);
  EXPECT_EQ(output.frame_queue_capacity, 4U);
  EXPECT_EQ(output.panel.rotation, 3U);
  EXPECT_FALSE(output.panel.color_order_bgr);
  EXPECT_FALSE(output.panel.display_inversion);
  EXPECT_EQ(output.panel.x_offset, 4U);
  EXPECT_EQ(output.panel.y_offset, 7U);
}

TEST(ScreenRuntimeConfigTest, RejectsEmptySpiDevice) {
  ScreenRuntimeConfig runtime;
  runtime.spi_device.clear();

  ScreenConfig output;
  std::string error;
  EXPECT_FALSE(buildScreenConfig(runtime, output, error));
  EXPECT_NE(error.find("spi_device"), std::string::npos);
}

TEST(ScreenRuntimeConfigTest, RejectsEmptyGpioChip) {
  ScreenRuntimeConfig runtime;
  runtime.gpiochip.clear();

  ScreenConfig output;
  std::string error;
  EXPECT_FALSE(buildScreenConfig(runtime, output, error));
  EXPECT_NE(error.find("gpiochip"), std::string::npos);
}

TEST(ScreenRuntimeConfigTest, PreservesAbsoluteGpioChipPath) {
  ScreenRuntimeConfig runtime;
  runtime.gpiochip = "/dev/gpiochip6";

  ScreenConfig output;
  std::string error;
  ASSERT_TRUE(buildScreenConfig(runtime, output, error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(output.gpio_chip_path, "/dev/gpiochip6");
}

TEST(ScreenRuntimeConfigTest, UsesGpiochip4ByDefault) {
  ScreenRuntimeConfig runtime;

  ScreenConfig output;
  std::string error;
  ASSERT_TRUE(buildScreenConfig(runtime, output, error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(output.gpio_chip_path, "/dev/gpiochip4");
}

TEST(ScreenRuntimeConfigTest, RejectsInvalidSpiSpeed) {
  {
    ScreenRuntimeConfig runtime;
    runtime.spi_speed_hz = 0;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("spi_speed_hz"), std::string::npos);
  }

  {
    ScreenRuntimeConfig runtime;
    runtime.spi_speed_hz = static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()) + 1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("spi_speed_hz"), std::string::npos);
  }
}

TEST(ScreenRuntimeConfigTest, RejectsInvalidSpiMode) {
  {
    ScreenRuntimeConfig runtime;
    runtime.spi_mode = -1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("spi_mode"), std::string::npos);
  }

  {
    ScreenRuntimeConfig runtime;
    runtime.spi_mode = 4;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("spi_mode"), std::string::npos);
  }
}

TEST(ScreenRuntimeConfigTest, RejectsInvalidSpiBitsPerWord) {
  {
    ScreenRuntimeConfig runtime;
    runtime.spi_bits_per_word = 0;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("spi_bits_per_word"), std::string::npos);
  }

  {
    ScreenRuntimeConfig runtime;
    runtime.spi_bits_per_word = 1024;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("spi_bits_per_word"), std::string::npos);
  }
}

TEST(ScreenRuntimeConfigTest, RejectsInvalidGpioOffsets) {
  {
    ScreenRuntimeConfig runtime;
    runtime.dc_gpio = -1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("dc_gpio"), std::string::npos);
  }

  {
    ScreenRuntimeConfig runtime;
    runtime.rst_gpio = -1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("rst_gpio"), std::string::npos);
  }

  {
    ScreenRuntimeConfig runtime;
    runtime.bl_gpio = -1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("bl_gpio"), std::string::npos);
  }
}

TEST(ScreenRuntimeConfigTest, RejectsUnsupportedRotationDegrees) {
  ScreenRuntimeConfig runtime;
  runtime.rotation_degrees = 45;

  ScreenConfig output;
  std::string error;
  EXPECT_FALSE(buildScreenConfig(runtime, output, error));
  EXPECT_NE(error.find("rotation_degrees"), std::string::npos);
}

TEST(ScreenRuntimeConfigTest, RejectsInvalidFrameQueueCapacity) {
  {
    ScreenRuntimeConfig runtime;
    runtime.frame_queue_capacity = 0;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("frame_queue_capacity"), std::string::npos);
  }

  {
    ScreenRuntimeConfig runtime;
    runtime.frame_queue_capacity =
      static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()) + 1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("frame_queue_capacity"), std::string::npos);
  }
}

TEST(ScreenRuntimeConfigTest, RejectsInvalidOffsets) {
  {
    ScreenRuntimeConfig runtime;
    runtime.x_offset = -1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("x_offset"), std::string::npos);
  }

  {
    ScreenRuntimeConfig runtime;
    runtime.y_offset = static_cast<std::int64_t>(std::numeric_limits<std::uint16_t>::max()) + 1;

    ScreenConfig output;
    std::string error;
    EXPECT_FALSE(buildScreenConfig(runtime, output, error));
    EXPECT_NE(error.find("y_offset"), std::string::npos);
  }
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
