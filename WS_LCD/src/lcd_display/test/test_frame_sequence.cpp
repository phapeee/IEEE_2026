#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/assets/FrameSequence.hpp"

namespace st7789_ros_wrapper::assets {
namespace {

class ScopedTempDir {
public:
  ScopedTempDir()
  : path_(
      std::filesystem::temp_directory_path() /
      ("st7789_frame_sequence_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
  {
    std::filesystem::create_directories(path_);
  }

  ~ScopedTempDir() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const noexcept {
    return path_;
  }

private:
  std::filesystem::path path_;
};

void touchFile(const std::filesystem::path & path) {
  std::ofstream output(path);
  output << "x";
}

bool endsWith(const std::string & value, const std::string & suffix) {
  if (suffix.size() > value.size()) {
    return false;
  }
  return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

TEST(FrameSequenceTest, RejectsEmptyDirectoryPath) {
  std::vector<std::string> frames;
  std::string error;
  EXPECT_FALSE(listFramePaths("", frames, error));
  EXPECT_NE(error.find("frames_directory"), std::string::npos);
}

TEST(FrameSequenceTest, RejectsMissingDirectory) {
  std::vector<std::string> frames;
  std::string error;
  EXPECT_FALSE(listFramePaths("/tmp/does_not_exist_st7789_frames", frames, error));
  EXPECT_NE(error.find("does not exist"), std::string::npos);
}

TEST(FrameSequenceTest, RejectsDirectoryWithoutSupportedFrames) {
  ScopedTempDir temp_dir;
  touchFile(temp_dir.path() / "notes.txt");
  touchFile(temp_dir.path() / "frame.gif");

  std::vector<std::string> frames;
  std::string error;
  EXPECT_FALSE(listFramePaths(temp_dir.path().string(), frames, error));
  EXPECT_NE(error.find("no supported frame files"), std::string::npos);
}

TEST(FrameSequenceTest, ReturnsSortedSupportedFramePaths) {
  ScopedTempDir temp_dir;
  touchFile(temp_dir.path() / "b_20.PNG");
  touchFile(temp_dir.path() / "a_10.jpg");
  touchFile(temp_dir.path() / "c_30.bmp");
  touchFile(temp_dir.path() / "skip.dat");
  std::filesystem::create_directories(temp_dir.path() / "nested");
  touchFile(temp_dir.path() / "nested" / "frame.png");

  std::vector<std::string> frames;
  std::string error;
  ASSERT_TRUE(listFramePaths(temp_dir.path().string(), frames, error));
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(frames.size(), 3U);
  EXPECT_TRUE(endsWith(frames[0], "a_10.jpg"));
  EXPECT_TRUE(endsWith(frames[1], "b_20.PNG"));
  EXPECT_TRUE(endsWith(frames[2], "c_30.bmp"));
}

}  // namespace
}  // namespace st7789_ros_wrapper::assets
