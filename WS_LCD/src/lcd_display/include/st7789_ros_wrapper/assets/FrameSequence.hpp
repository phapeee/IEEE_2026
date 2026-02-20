#pragma once

#include <string>
#include <vector>

namespace st7789_ros_wrapper::assets {

bool listFramePaths(
  const std::string & frames_directory, std::vector<std::string> & output_paths,
  std::string & error);

}  // namespace st7789_ros_wrapper::assets
