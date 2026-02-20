#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/AnimationCli.hpp"

namespace st7789_ros_wrapper::app {
namespace {

TEST(AnimationCliTest, ParsesHelpFlag) {
  char arg0[] = "lcd_play_animation";
  char arg1[] = "--help";
  char * argv[] = {arg0, arg1};

  AnimationCliConfig config;
  std::string error;
  ASSERT_TRUE(parseAnimationArgs(2, argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(config.show_help);
}

TEST(AnimationCliTest, ParsesValidArguments) {
  char arg0[] = "lcd_play_animation";
  char arg1[] = "--frames-dir";
  char arg2[] = "/tmp/frames";
  char arg3[] = "--fps";
  char arg4[] = "15.5";
  char arg5[] = "--max-frames";
  char arg6[] = "120";
  char arg7[] = "--no-loop";
  char arg8[] = "--no-leave-backlight-on";
  char arg9[] = "--spi-device";
  char arg10[] = "/dev/spidev0.1";
  char arg11[] = "--rotation-degrees";
  char arg12[] = "180";
  char * argv[] = {
    arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12,
  };

  AnimationCliConfig config;
  std::string error;
  ASSERT_TRUE(parseAnimationArgs(static_cast<int>(std::size(argv)), argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_FALSE(config.show_help);
  EXPECT_EQ(config.frames_directory, "/tmp/frames");
  EXPECT_DOUBLE_EQ(config.fps, 15.5);
  EXPECT_EQ(config.max_frames, 120);
  EXPECT_FALSE(config.loop);
  EXPECT_FALSE(config.leave_backlight_on);
  EXPECT_EQ(config.display.spi_device, "/dev/spidev0.1");
  EXPECT_EQ(config.display.rotation_degrees, 180);
}

TEST(AnimationCliTest, RejectsUnknownArgument) {
  char arg0[] = "lcd_play_animation";
  char arg1[] = "--unknown";
  char * argv[] = {arg0, arg1};

  AnimationCliConfig config;
  std::string error;
  EXPECT_FALSE(parseAnimationArgs(2, argv, config, error));
  EXPECT_NE(error.find("Unknown argument"), std::string::npos);
}

TEST(AnimationCliTest, RejectsMissingValue) {
  char arg0[] = "lcd_play_animation";
  char arg1[] = "--frames-dir";
  char * argv[] = {arg0, arg1};

  AnimationCliConfig config;
  std::string error;
  EXPECT_FALSE(parseAnimationArgs(2, argv, config, error));
  EXPECT_NE(error.find("Missing value"), std::string::npos);
}

TEST(AnimationCliTest, ValidateRejectsInvalidValues) {
  {
    AnimationCliConfig config;
    config.frames_directory.clear();
    std::string error;
    EXPECT_FALSE(validateAnimationConfig(config, error));
    EXPECT_NE(error.find("frames_directory"), std::string::npos);
  }

  {
    AnimationCliConfig config;
    config.frames_directory = "/tmp/frames";
    config.fps = 0.0;
    std::string error;
    EXPECT_FALSE(validateAnimationConfig(config, error));
    EXPECT_NE(error.find("fps"), std::string::npos);
  }

  {
    AnimationCliConfig config;
    config.frames_directory = "/tmp/frames";
    config.max_frames = -1;
    std::string error;
    EXPECT_FALSE(validateAnimationConfig(config, error));
    EXPECT_NE(error.find("max_frames"), std::string::npos);
  }
}

TEST(AnimationCliTest, UsageContainsCoreOptions) {
  const auto usage = animationUsage("lcd_play_animation");
  EXPECT_NE(usage.find("--frames-dir"), std::string::npos);
  EXPECT_NE(usage.find("--fps"), std::string::npos);
  EXPECT_NE(usage.find("--max-frames"), std::string::npos);
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
