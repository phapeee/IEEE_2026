#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/AcceptanceSmokeCli.hpp"

namespace st7789_ros_wrapper::app {
namespace {

TEST(AcceptanceSmokeCliTest, ParsesHelpFlag) {
  char arg0[] = "lcd_acceptance_smoke";
  char arg1[] = "--help";
  char * argv[] = {arg0, arg1};

  AcceptanceSmokeConfig config;
  std::string error;
  ASSERT_TRUE(parseAcceptanceSmokeArgs(2, argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(config.show_help);
}

TEST(AcceptanceSmokeCliTest, ParsesValidArguments) {
  char arg0[] = "lcd_acceptance_smoke";
  char arg1[] = "--spi-device";
  char arg2[] = "/dev/spidev0.1";
  char arg3[] = "--spi-speed-hz";
  char arg4[] = "12000000";
  char arg5[] = "--rotation-degrees";
  char arg6[] = "180";
  char arg7[] = "--backlight-cycles";
  char arg8[] = "3";
  char arg9[] = "--backlight-delay-ms";
  char arg10[] = "100";
  char arg11[] = "--benchmark-frames";
  char arg12[] = "42";
  char arg13[] = "--min-fps";
  char arg14[] = "12.5";
  char arg15[] = "--no-leave-backlight-on";
  char * argv[] = {
    arg0,  arg1,  arg2,  arg3,  arg4,  arg5,  arg6,  arg7,
    arg8,  arg9,  arg10, arg11, arg12, arg13, arg14, arg15,
  };

  AcceptanceSmokeConfig config;
  std::string error;
  ASSERT_TRUE(parseAcceptanceSmokeArgs(static_cast<int>(std::size(argv)), argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_FALSE(config.show_help);
  EXPECT_EQ(config.display.spi_device, "/dev/spidev0.1");
  EXPECT_EQ(config.display.spi_speed_hz, 12000000);
  EXPECT_EQ(config.display.rotation_degrees, 180);
  EXPECT_EQ(config.backlight_cycles, 3);
  EXPECT_EQ(config.backlight_delay_ms, 100);
  EXPECT_EQ(config.benchmark_frames, 42);
  EXPECT_DOUBLE_EQ(config.min_fps, 12.5);
  EXPECT_FALSE(config.leave_backlight_on);
}

TEST(AcceptanceSmokeCliTest, RejectsUnknownArgument) {
  char arg0[] = "lcd_acceptance_smoke";
  char arg1[] = "--unknown";
  char * argv[] = {arg0, arg1};

  AcceptanceSmokeConfig config;
  std::string error;
  EXPECT_FALSE(parseAcceptanceSmokeArgs(2, argv, config, error));
  EXPECT_NE(error.find("Unknown argument"), std::string::npos);
}

TEST(AcceptanceSmokeCliTest, RejectsMissingValue) {
  char arg0[] = "lcd_acceptance_smoke";
  char arg1[] = "--spi-speed-hz";
  char * argv[] = {arg0, arg1};

  AcceptanceSmokeConfig config;
  std::string error;
  EXPECT_FALSE(parseAcceptanceSmokeArgs(2, argv, config, error));
  EXPECT_NE(error.find("Missing value"), std::string::npos);
}

TEST(AcceptanceSmokeCliTest, ValidateRejectsInvalidValues) {
  {
    AcceptanceSmokeConfig config;
    config.backlight_cycles = 0;
    std::string error;
    EXPECT_FALSE(validateAcceptanceSmokeConfig(config, error));
    EXPECT_NE(error.find("backlight_cycles"), std::string::npos);
  }

  {
    AcceptanceSmokeConfig config;
    config.backlight_delay_ms = -1;
    std::string error;
    EXPECT_FALSE(validateAcceptanceSmokeConfig(config, error));
    EXPECT_NE(error.find("backlight_delay_ms"), std::string::npos);
  }

  {
    AcceptanceSmokeConfig config;
    config.benchmark_frames = 0;
    std::string error;
    EXPECT_FALSE(validateAcceptanceSmokeConfig(config, error));
    EXPECT_NE(error.find("benchmark_frames"), std::string::npos);
  }

  {
    AcceptanceSmokeConfig config;
    config.min_fps = -0.1;
    std::string error;
    EXPECT_FALSE(validateAcceptanceSmokeConfig(config, error));
    EXPECT_NE(error.find("min_fps"), std::string::npos);
  }
}

TEST(AcceptanceSmokeCliTest, CalculatesFramesPerSecond) {
  EXPECT_DOUBLE_EQ(calculateFramesPerSecond(120, 2.0), 60.0);
  EXPECT_DOUBLE_EQ(calculateFramesPerSecond(0, 2.0), 0.0);
  EXPECT_DOUBLE_EQ(calculateFramesPerSecond(120, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(calculateFramesPerSecond(120, -1.0), 0.0);
}

TEST(AcceptanceSmokeCliTest, UsageContainsCoreOptions) {
  const auto usage = acceptanceSmokeUsage("lcd_acceptance_smoke");
  EXPECT_NE(usage.find("--spi-device"), std::string::npos);
  EXPECT_NE(usage.find("--benchmark-frames"), std::string::npos);
  EXPECT_NE(usage.find("--min-fps"), std::string::npos);
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
