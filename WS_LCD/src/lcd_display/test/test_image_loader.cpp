#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "st7789_ros_wrapper/assets/ImageLoader.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace st7789_ros_wrapper::assets {
namespace {

class ScopedTempFile {
public:
  explicit ScopedTempFile(std::string extension)
  : path_(std::filesystem::temp_directory_path() /
      ("st7789_image_loader_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + std::move(extension)))
  {}

  ~ScopedTempFile() {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  const std::filesystem::path & path() const noexcept {
    return path_;
  }

private:
  std::filesystem::path path_;
};

cv::Mat makeTwoPixelImageBgr() {
  cv::Mat image(1, 2, CV_8UC3);
  image.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 255);  // red
  image.at<cv::Vec3b>(0, 1) = cv::Vec3b(0, 255, 0);  // green
  return image;
}

int channelDistance(const std::uint8_t lhs, const std::uint8_t rhs) {
  return std::abs(static_cast<int>(lhs) - static_cast<int>(rhs));
}

bool roughlyMatchesRgb565(
  const gfx::Color565 color, const std::uint8_t r, const std::uint8_t g, const std::uint8_t b,
  const int tolerance)
{
  const auto color_r = static_cast<std::uint8_t>(((color >> 11U) & 0x1FU) << 3U);
  const auto color_g = static_cast<std::uint8_t>(((color >> 5U) & 0x3FU) << 2U);
  const auto color_b = static_cast<std::uint8_t>((color & 0x1FU) << 3U);
  return channelDistance(color_r, r) <= tolerance &&
         channelDistance(color_g, g) <= tolerance &&
         channelDistance(color_b, b) <= tolerance;
}

TEST(ImageLoaderTest, ReturnsNulloptForEmptyPath) {
  EXPECT_FALSE(ImageLoader::loadFile("").has_value());
}

TEST(ImageLoaderTest, ReturnsNulloptForUnsupportedExtension) {
  EXPECT_FALSE(ImageLoader::loadFile("/tmp/not_supported.gif").has_value());
}

TEST(ImageLoaderTest, ReturnsNulloptForMissingFile) {
  EXPECT_FALSE(ImageLoader::loadFile("/tmp/missing_st7789_image_loader_test.png").has_value());
}

TEST(ImageLoaderTest, LoadsPngAsRgb565) {
  ScopedTempFile png_file(".png");
  ASSERT_TRUE(cv::imwrite(png_file.path().string(), makeTwoPixelImageBgr()));

  const auto image = ImageLoader::loadFile(png_file.path().string());
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->width, 2U);
  EXPECT_EQ(image->height, 1U);
  ASSERT_EQ(image->pixels.size(), 2U);
  EXPECT_EQ(image->pixels[0], gfx::rgb888To565(255, 0, 0));
  EXPECT_EQ(image->pixels[1], gfx::rgb888To565(0, 255, 0));
}

TEST(ImageLoaderTest, LoadsBmpAsRgb565) {
  ScopedTempFile bmp_file(".bmp");
  ASSERT_TRUE(cv::imwrite(bmp_file.path().string(), makeTwoPixelImageBgr()));

  const auto image = ImageLoader::loadFile(bmp_file.path().string());
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->width, 2U);
  EXPECT_EQ(image->height, 1U);
  ASSERT_EQ(image->pixels.size(), 2U);
  EXPECT_EQ(image->pixels[0], gfx::rgb888To565(255, 0, 0));
  EXPECT_EQ(image->pixels[1], gfx::rgb888To565(0, 255, 0));
}

TEST(ImageLoaderTest, LoadsJpegWithExpectedDimensions) {
  ScopedTempFile jpg_file(".jpg");
  cv::Mat image(16, 16, CV_8UC3, cv::Scalar(0, 0, 255));
  ASSERT_TRUE(cv::imwrite(jpg_file.path().string(), image, {cv::IMWRITE_JPEG_QUALITY, 100}));

  const auto decoded = ImageLoader::loadFile(jpg_file.path().string());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->width, 16U);
  EXPECT_EQ(decoded->height, 16U);
  ASSERT_EQ(decoded->pixels.size(), 256U);
  EXPECT_TRUE(roughlyMatchesRgb565(decoded->pixels.front(), 255, 0, 0, 50));
  EXPECT_TRUE(roughlyMatchesRgb565(decoded->pixels.back(), 255, 0, 0, 50));
}

TEST(ImageLoaderTest, FitToSizeRejectsZeroDimensions) {
  EXPECT_FALSE(ImageLoader::loadFileFitToSize("/tmp/any.png", 0, 280).has_value());
  EXPECT_FALSE(ImageLoader::loadFileFitToSize("/tmp/any.png", 240, 0).has_value());
}

TEST(ImageLoaderTest, FitToSizeReturnsRequestedDimensions) {
  ScopedTempFile png_file(".png");
  cv::Mat image(40, 30, CV_8UC3, cv::Scalar(255, 0, 0));
  ASSERT_TRUE(cv::imwrite(png_file.path().string(), image));

  const auto fitted = ImageLoader::loadFileFitToSize(png_file.path().string(), 24, 28);
  ASSERT_TRUE(fitted.has_value());
  EXPECT_EQ(fitted->width, 24U);
  EXPECT_EQ(fitted->height, 28U);
  EXPECT_EQ(fitted->pixels.size(), 24U * 28U);
}

TEST(ImageLoaderTest, FitToSizeCenterCropsUsingCoverStrategy) {
  ScopedTempFile png_file(".png");
  cv::Mat image(1, 3, CV_8UC3);
  image.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 255);    // red
  image.at<cv::Vec3b>(0, 1) = cv::Vec3b(0, 255, 0);    // green
  image.at<cv::Vec3b>(0, 2) = cv::Vec3b(255, 0, 0);    // blue
  ASSERT_TRUE(cv::imwrite(png_file.path().string(), image));

  const auto fitted = ImageLoader::loadFileFitToSize(png_file.path().string(), 1, 1);
  ASSERT_TRUE(fitted.has_value());
  ASSERT_EQ(fitted->pixels.size(), 1U);
  EXPECT_EQ(fitted->pixels[0], gfx::rgb888To565(0, 255, 0));
}

TEST(ImageLoaderTest, ReturnsNulloptForInvalidImagePayload) {
  ScopedTempFile png_file(".png");
  std::ofstream output(png_file.path(), std::ios::binary);
  ASSERT_TRUE(output.is_open());
  output << "not-a-valid-image";
  output.close();

  EXPECT_FALSE(ImageLoader::loadFile(png_file.path().string()).has_value());
}

}  // namespace
}  // namespace st7789_ros_wrapper::assets
