#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/app/TextDemoCli.hpp"

namespace st7789_ros_wrapper::app {
namespace {

TEST(TextDemoCliTest, ParsesHelpFlag) {
  char arg0[] = "lcd_text_demo";
  char arg1[] = "--help";
  char * argv[] = {arg0, arg1};

  TextDemoCliConfig config;
  std::string error;
  ASSERT_TRUE(parseTextDemoArgs(2, argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(config.show_help);
}

TEST(TextDemoCliTest, ParsesValidArguments) {
  char arg0[] = "lcd_text_demo";
  char arg1[] = "--text";
  char arg2[] = "Status";
  char arg3[] = "--x";
  char arg4[] = "10";
  char arg5[] = "--y";
  char arg6[] = "20";
  char arg7[] = "--scale";
  char arg8[] = "3";
  char arg9[] = "--text-r";
  char arg10[] = "12";
  char arg11[] = "--text-g";
  char arg12[] = "34";
  char arg13[] = "--text-b";
  char arg14[] = "56";
  char arg15[] = "--bg-r";
  char arg16[] = "1";
  char arg17[] = "--bg-g";
  char arg18[] = "2";
  char arg19[] = "--bg-b";
  char arg20[] = "3";
  char arg21[] = "--no-leave-backlight-on";
  char arg22[] = "--rotation-degrees";
  char arg23[] = "180";
  char * argv[] = {
    arg0,  arg1,  arg2,  arg3,  arg4,  arg5,  arg6,  arg7,
    arg8,  arg9,  arg10, arg11, arg12, arg13, arg14, arg15,
    arg16, arg17, arg18, arg19, arg20, arg21, arg22, arg23,
  };

  TextDemoCliConfig config;
  std::string error;
  ASSERT_TRUE(parseTextDemoArgs(static_cast<int>(std::size(argv)), argv, config, error));
  EXPECT_TRUE(error.empty());
  EXPECT_FALSE(config.show_help);
  EXPECT_EQ(config.text, "Status");
  EXPECT_EQ(config.x, 10);
  EXPECT_EQ(config.y, 20);
  EXPECT_EQ(config.scale, 3);
  EXPECT_EQ(config.text_r, 12);
  EXPECT_EQ(config.text_g, 34);
  EXPECT_EQ(config.text_b, 56);
  EXPECT_EQ(config.bg_r, 1);
  EXPECT_EQ(config.bg_g, 2);
  EXPECT_EQ(config.bg_b, 3);
  EXPECT_FALSE(config.leave_backlight_on);
  EXPECT_EQ(config.display.rotation_degrees, 180);
}

TEST(TextDemoCliTest, RejectsUnknownArgument) {
  char arg0[] = "lcd_text_demo";
  char arg1[] = "--unknown";
  char * argv[] = {arg0, arg1};

  TextDemoCliConfig config;
  std::string error;
  EXPECT_FALSE(parseTextDemoArgs(2, argv, config, error));
  EXPECT_NE(error.find("Unknown argument"), std::string::npos);
}

TEST(TextDemoCliTest, RejectsMissingValue) {
  char arg0[] = "lcd_text_demo";
  char arg1[] = "--text";
  char * argv[] = {arg0, arg1};

  TextDemoCliConfig config;
  std::string error;
  EXPECT_FALSE(parseTextDemoArgs(2, argv, config, error));
  EXPECT_NE(error.find("Missing value"), std::string::npos);
}

TEST(TextDemoCliTest, ValidateRejectsInvalidValues) {
  {
    TextDemoCliConfig config;
    config.text.clear();
    std::string error;
    EXPECT_FALSE(validateTextDemoConfig(config, error));
    EXPECT_NE(error.find("text"), std::string::npos);
  }

  {
    TextDemoCliConfig config;
    config.scale = 0;
    std::string error;
    EXPECT_FALSE(validateTextDemoConfig(config, error));
    EXPECT_NE(error.find("scale"), std::string::npos);
  }

  {
    TextDemoCliConfig config;
    config.x = -1;
    std::string error;
    EXPECT_FALSE(validateTextDemoConfig(config, error));
    EXPECT_NE(error.find("x"), std::string::npos);
  }

  {
    TextDemoCliConfig config;
    config.text_b = 999;
    std::string error;
    EXPECT_FALSE(validateTextDemoConfig(config, error));
    EXPECT_NE(error.find("text_b"), std::string::npos);
  }
}

TEST(TextDemoCliTest, UsageContainsCoreOptions) {
  const auto usage = textDemoUsage("lcd_text_demo");
  EXPECT_NE(usage.find("--text"), std::string::npos);
  EXPECT_NE(usage.find("--x"), std::string::npos);
  EXPECT_NE(usage.find("--scale"), std::string::npos);
  EXPECT_NE(usage.find("--text-r"), std::string::npos);
}

}  // namespace
}  // namespace st7789_ros_wrapper::app
