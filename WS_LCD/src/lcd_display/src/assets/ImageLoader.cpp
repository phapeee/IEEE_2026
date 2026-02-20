#include "st7789_ros_wrapper/assets/ImageLoader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <optional>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace st7789_ros_wrapper::assets {
namespace {

bool isSupportedExtension(std::string extension) {
  std::transform(
    extension.begin(), extension.end(), extension.begin(),
    [](const unsigned char value) {return static_cast<char>(std::tolower(value));});
  return extension == ".png" || extension == ".jpg" || extension == ".jpeg" || extension == ".bmp";
}

std::optional<cv::Mat> decodeFileToBgr(const std::string & path) {
  if (path.empty()) {
    return std::nullopt;
  }

  const auto extension = std::filesystem::path(path).extension().string();
  if (!extension.empty() && !isSupportedExtension(extension)) {
    return std::nullopt;
  }

  const cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
  if (bgr.empty() || bgr.cols <= 0 || bgr.rows <= 0 || bgr.cols > 0xFFFF || bgr.rows > 0xFFFF) {
    return std::nullopt;
  }

  return bgr;
}

std::optional<Image565> convertBgrToRgb565(const cv::Mat & bgr) {
  if (bgr.empty() || bgr.cols <= 0 || bgr.rows <= 0 || bgr.cols > 0xFFFF || bgr.rows > 0xFFFF) {
    return std::nullopt;
  }

  Image565 image;
  image.width = static_cast<std::uint16_t>(bgr.cols);
  image.height = static_cast<std::uint16_t>(bgr.rows);
  image.pixels.resize(static_cast<std::size_t>(image.width) * image.height);

  for (std::uint16_t y = 0; y < image.height; ++y) {
    const auto * row = bgr.ptr<cv::Vec3b>(y);
    for (std::uint16_t x = 0; x < image.width; ++x) {
      const auto & bgr_pixel = row[x];
      const auto index = static_cast<std::size_t>(y) * image.width + x;
      image.pixels[index] = gfx::rgb888To565(bgr_pixel[2], bgr_pixel[1], bgr_pixel[0]);
    }
  }

  return image;
}

cv::Mat fitBgrImageToSize(
  const cv::Mat & input, const std::uint16_t target_width, const std::uint16_t target_height)
{
  if (input.empty() || input.cols <= 0 || input.rows <= 0 || target_width == 0 || target_height == 0) {
    return {};
  }

  const double width_scale = static_cast<double>(target_width) / input.cols;
  const double height_scale = static_cast<double>(target_height) / input.rows;
  const double scale = std::max(width_scale, height_scale);
  if (!std::isfinite(scale) || scale <= 0.0) {
    return {};
  }

  const auto scaled_width = std::max(
    static_cast<int>(target_width), static_cast<int>(std::lround(input.cols * scale)));
  const auto scaled_height = std::max(
    static_cast<int>(target_height), static_cast<int>(std::lround(input.rows * scale)));
  if (scaled_width <= 0 || scaled_height <= 0) {
    return {};
  }

  cv::Mat resized;
  const auto interpolation = (scale < 1.0) ? cv::INTER_AREA : cv::INTER_LINEAR;
  cv::resize(input, resized, cv::Size(scaled_width, scaled_height), 0.0, 0.0, interpolation);
  if (resized.empty()) {
    return {};
  }

  const auto crop_x = (resized.cols - static_cast<int>(target_width)) / 2;
  const auto crop_y = (resized.rows - static_cast<int>(target_height)) / 2;
  if (crop_x < 0 || crop_y < 0) {
    return {};
  }

  const cv::Rect crop_roi(crop_x, crop_y, target_width, target_height);
  if (crop_roi.x + crop_roi.width > resized.cols || crop_roi.y + crop_roi.height > resized.rows) {
    return {};
  }

  return resized(crop_roi).clone();
}

}  // namespace

std::optional<Image565> ImageLoader::loadFile(const std::string & path) {
  const auto bgr = decodeFileToBgr(path);
  if (!bgr.has_value()) {
    return std::nullopt;
  }

  return convertBgrToRgb565(*bgr);
}

std::optional<Image565> ImageLoader::loadFileFitToSize(
  const std::string & path, const std::uint16_t target_width, const std::uint16_t target_height)
{
  if (target_width == 0 || target_height == 0) {
    return std::nullopt;
  }

  const auto bgr = decodeFileToBgr(path);
  if (!bgr.has_value()) {
    return std::nullopt;
  }

  const cv::Mat fitted = fitBgrImageToSize(*bgr, target_width, target_height);
  if (fitted.empty()) {
    return std::nullopt;
  }

  return convertBgrToRgb565(fitted);
}

}  // namespace st7789_ros_wrapper::assets
