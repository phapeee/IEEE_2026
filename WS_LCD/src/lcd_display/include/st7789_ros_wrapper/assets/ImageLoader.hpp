#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace st7789_ros_wrapper::assets {

struct Image565 {
  std::uint16_t width{0};
  std::uint16_t height{0};
  std::vector<std::uint16_t> pixels;

  bool empty() const noexcept {
    return pixels.empty() || width == 0 || height == 0;
  }
};

class ImageLoader {
public:
  static std::optional<Image565> loadFile(const std::string & path);
  static std::optional<Image565> loadFileFitToSize(
    const std::string & path, std::uint16_t target_width, std::uint16_t target_height);
};

}  // namespace st7789_ros_wrapper::assets
