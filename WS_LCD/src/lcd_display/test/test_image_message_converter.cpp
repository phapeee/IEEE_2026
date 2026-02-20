#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "sensor_msgs/msg/image.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"
#include "st7789_ros_wrapper/ros/ImageMessageConverter.hpp"

namespace st7789_ros_wrapper::ros {
namespace {

sensor_msgs::msg::Image buildImage(
  const std::uint32_t width, const std::uint32_t height, const std::string & encoding,
  const std::uint32_t step, std::vector<std::uint8_t> data)
{
  sensor_msgs::msg::Image image;
  image.width = width;
  image.height = height;
  image.encoding = encoding;
  image.step = step;
  image.is_bigendian = 0;
  image.data = std::move(data);
  return image;
}

TEST(ImageMessageConverterTest, ConvertsRgb8WithExactSize) {
  const auto message = buildImage(2, 1, "rgb8", 6, {255, 0, 0, 0, 255, 0});

  ImageMessageConversionConfig config;
  config.target_width = 2;
  config.target_height = 1;
  config.resize_mode = ImageResizeMode::kExactSize;

  assets::Image565 output;
  std::string error;
  ASSERT_TRUE(convertImageMessageToRgb565(message, config, output, error));
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(output.pixels.size(), 2U);
  EXPECT_EQ(output.pixels[0], gfx::rgb888To565(255, 0, 0));
  EXPECT_EQ(output.pixels[1], gfx::rgb888To565(0, 255, 0));
}

TEST(ImageMessageConverterTest, ConvertsBgrAndMonoEncodings) {
  {
    const auto message = buildImage(1, 1, "bgr8", 3, {0, 0, 255});
    ImageMessageConversionConfig config;
    config.target_width = 1;
    config.target_height = 1;
    config.resize_mode = ImageResizeMode::kExactSize;

    assets::Image565 output;
    std::string error;
    ASSERT_TRUE(convertImageMessageToRgb565(message, config, output, error));
    ASSERT_EQ(output.pixels.size(), 1U);
    EXPECT_EQ(output.pixels[0], gfx::rgb888To565(255, 0, 0));
  }

  {
    const auto message = buildImage(1, 1, "mono8", 1, {128});
    ImageMessageConversionConfig config;
    config.target_width = 1;
    config.target_height = 1;
    config.resize_mode = ImageResizeMode::kExactSize;

    assets::Image565 output;
    std::string error;
    ASSERT_TRUE(convertImageMessageToRgb565(message, config, output, error));
    ASSERT_EQ(output.pixels.size(), 1U);
    EXPECT_EQ(output.pixels[0], gfx::rgb888To565(128, 128, 128));
  }
}

TEST(ImageMessageConverterTest, FitModePerformsCenteredCrop) {
  const auto message = buildImage(
    4, 2, "rgb8", 12,
    {
      255, 0, 0,      0, 255, 0,      0, 0, 255,      255, 255, 255,
      0, 0, 0,        255, 255, 0,    0, 255, 255,    255, 0, 255,
    });

  ImageMessageConversionConfig config;
  config.target_width = 2;
  config.target_height = 2;
  config.resize_mode = ImageResizeMode::kFitCenterCrop;

  assets::Image565 output;
  std::string error;
  ASSERT_TRUE(convertImageMessageToRgb565(message, config, output, error));
  EXPECT_TRUE(error.empty());

  ASSERT_EQ(output.pixels.size(), 4U);
  EXPECT_EQ(output.pixels[0], gfx::rgb888To565(0, 255, 0));
  EXPECT_EQ(output.pixels[1], gfx::rgb888To565(0, 0, 255));
  EXPECT_EQ(output.pixels[2], gfx::rgb888To565(255, 255, 0));
  EXPECT_EQ(output.pixels[3], gfx::rgb888To565(0, 255, 255));
}

TEST(ImageMessageConverterTest, RejectsUnsupportedEncodingAndInvalidLayout) {
  {
    const auto message = buildImage(1, 1, "yuv422", 2, {0, 0});
    ImageMessageConversionConfig config;
    config.target_width = 1;
    config.target_height = 1;

    assets::Image565 output;
    std::string error;
    EXPECT_FALSE(convertImageMessageToRgb565(message, config, output, error));
    EXPECT_NE(error.find("unsupported image encoding"), std::string::npos);
  }

  {
    const auto message = buildImage(2, 1, "rgb8", 5, {255, 0, 0, 0, 255});
    ImageMessageConversionConfig config;
    config.target_width = 2;
    config.target_height = 1;
    config.resize_mode = ImageResizeMode::kExactSize;

    assets::Image565 output;
    std::string error;
    EXPECT_FALSE(convertImageMessageToRgb565(message, config, output, error));
    EXPECT_NE(error.find("step"), std::string::npos);
  }
}

TEST(ImageMessageConverterTest, RejectsDimensionMismatchInExactMode) {
  const auto message = buildImage(4, 2, "rgb8", 12, std::vector<std::uint8_t>(24, 0));

  ImageMessageConversionConfig config;
  config.target_width = 2;
  config.target_height = 2;
  config.resize_mode = ImageResizeMode::kExactSize;

  assets::Image565 output;
  std::string error;
  EXPECT_FALSE(convertImageMessageToRgb565(message, config, output, error));
  EXPECT_NE(error.find("do not match target"), std::string::npos);
}

}  // namespace
}  // namespace st7789_ros_wrapper::ros
