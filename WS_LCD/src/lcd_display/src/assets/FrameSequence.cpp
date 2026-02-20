#include "st7789_ros_wrapper/assets/FrameSequence.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace st7789_ros_wrapper::assets {
namespace {

bool isSupportedFrameExtension(std::string extension) {
  std::transform(
    extension.begin(), extension.end(), extension.begin(),
    [](const unsigned char value) {return static_cast<char>(std::tolower(value));});
  return extension == ".png" || extension == ".jpg" || extension == ".jpeg" || extension == ".bmp";
}

}  // namespace

bool listFramePaths(
  const std::string & frames_directory, std::vector<std::string> & output_paths,
  std::string & error)
{
  error.clear();
  output_paths.clear();

  if (frames_directory.empty()) {
    error = "frames_directory cannot be empty";
    return false;
  }

  const std::filesystem::path directory_path(frames_directory);
  std::error_code filesystem_error;
  if (!std::filesystem::exists(directory_path, filesystem_error) || filesystem_error) {
    error = "frames_directory does not exist";
    return false;
  }

  if (!std::filesystem::is_directory(directory_path, filesystem_error) || filesystem_error) {
    error = "frames_directory is not a directory";
    return false;
  }

  std::filesystem::directory_iterator iterator(
    directory_path, std::filesystem::directory_options::skip_permission_denied,
    filesystem_error);
  if (filesystem_error) {
    error = "failed to iterate frames_directory";
    return false;
  }

  for (const auto & entry : iterator) {
    std::error_code entry_error;
    if (!entry.is_regular_file(entry_error) || entry_error) {
      continue;
    }

    const auto extension = entry.path().extension().string();
    if (!isSupportedFrameExtension(extension)) {
      continue;
    }

    output_paths.emplace_back(entry.path().string());
  }

  std::sort(output_paths.begin(), output_paths.end());

  if (output_paths.empty()) {
    error = "no supported frame files (.png/.jpg/.jpeg/.bmp) found in frames_directory";
    return false;
  }

  return true;
}

}  // namespace st7789_ros_wrapper::assets
