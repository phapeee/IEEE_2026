#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/ShowImageCli.hpp"

namespace st7789_ros_wrapper::app {
namespace {

TEST(ShowImageCliTest, ParsesHelpFlag) {
  char arg0[] = "lcd_show_image";
  char arg1[] = "--help";
  char * argv[] = {arg0, arg1};

  ShowImageCliConfig config;
  std::string error;
  ASSERT_TRUE(parseShowImageArgs(2, argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(config.show_help);
}

TEST(ShowImageCliTest, ParsesValidArguments) {
  char arg0[] = "lcd_show_image";
  char arg1[] = "--image";
  char arg2[] = "/tmp/panel.png";
  char arg3[] = "--no-fit-to-screen";
  char arg4[] = "--no-leave-backlight-on";
  char arg5[] = "--spi-device";
  char arg6[] = "/dev/spidev0.1";
  char arg7[] = "--rotation-degrees";
  char arg8[] = "270";
  char * argv[] = {
    arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8,
  };

  ShowImageCliConfig config;
  std::string error;
  ASSERT_TRUE(parseShowImageArgs(static_cast<int>(std::size(argv)), argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_FALSE(config.show_help);
  EXPECT_EQ(config.image_path, "/tmp/panel.png");
  EXPECT_FALSE(config.fit_to_screen);
  EXPECT_FALSE(config.leave_backlight_on);
  EXPECT_EQ(config.display.spi_device, "/dev/spidev0.1");
  EXPECT_EQ(config.display.rotation_degrees, 270);
}

TEST(ShowImageCliTest, RejectsUnknownArgument) {
  char arg0[] = "lcd_show_image";
  char arg1[] = "--unknown";
  char * argv[] = {arg0, arg1};

  ShowImageCliConfig config;
  std::string error;
  EXPECT_FALSE(parseShowImageArgs(2, argv, config, error));
  EXPECT_NE(error.find("Unknown argument"), std::string::npos);
}

TEST(ShowImageCliTest, RejectsMissingValue) {
  char arg0[] = "lcd_show_image";
  char arg1[] = "--image";
  char * argv[] = {arg0, arg1};

  ShowImageCliConfig config;
  std::string error;
  EXPECT_FALSE(parseShowImageArgs(2, argv, config, error));
  EXPECT_NE(error.find("Missing value"), std::string::npos);
}

TEST(ShowImageCliTest, ValidateRejectsInvalidValues) {
  ShowImageCliConfig config;
  config.image_path.clear();

  std::string error;
  EXPECT_FALSE(validateShowImageConfig(config, error));
  EXPECT_NE(error.find("image_path"), std::string::npos);
}

TEST(ShowImageCliTest, UsageContainsCoreOptions) {
  const auto usage = showImageUsage("lcd_show_image");
  EXPECT_NE(usage.find("--image"), std::string::npos);
  EXPECT_NE(usage.find("--fit-to-screen"), std::string::npos);
  EXPECT_NE(usage.find("--no-fit-to-screen"), std::string::npos);
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
