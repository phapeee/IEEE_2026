#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/ros/DisplayNodeRuntimeConfig.hpp"

namespace st7789_ros_wrapper::ros {
namespace {

TEST(DisplayNodeRuntimeConfigTest, BuildsConfigFromValidInput) {
  DisplayNodeRuntimeConfig runtime;
  runtime.render_hz = 20.0;
  runtime.fit_image_to_screen = false;
  runtime.use_dirty_rects = false;
  runtime.dirty_rect_full_frame_threshold = 0.6;
  runtime.clear_before_text = false;
  runtime.text_x = 12;
  runtime.text_y = 34;
  runtime.text_scale = 3;
  runtime.text_color_rgb565 = 0x1234;
  runtime.clear_color_rgb565 = 0x4321;

  DisplayNodeConfig output;
  std::string error;
  ASSERT_TRUE(buildDisplayNodeConfig(runtime, output, error));
  EXPECT_TRUE(error.empty());

  EXPECT_DOUBLE_EQ(output.render_hz, 20.0);
  EXPECT_FALSE(output.fit_image_to_screen);
  EXPECT_FALSE(output.use_dirty_rects);
  EXPECT_DOUBLE_EQ(output.dirty_rect_full_frame_threshold, 0.6);
  EXPECT_FALSE(output.clear_before_text);
  EXPECT_EQ(output.text_x, 12U);
  EXPECT_EQ(output.text_y, 34U);
  EXPECT_EQ(output.text_scale, 3U);
  EXPECT_EQ(output.text_color, 0x1234U);
  EXPECT_EQ(output.clear_color, 0x4321U);
}

TEST(DisplayNodeRuntimeConfigTest, RejectsInvalidRenderHz) {
  {
    DisplayNodeRuntimeConfig runtime;
    runtime.render_hz = 0.0;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("render_hz"), std::string::npos);
  }

  {
    DisplayNodeRuntimeConfig runtime;
    runtime.render_hz = std::numeric_limits<double>::infinity();

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("render_hz"), std::string::npos);
  }

  {
    DisplayNodeRuntimeConfig runtime;
    runtime.render_hz = 121.0;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("render_hz"), std::string::npos);
  }
}

TEST(DisplayNodeRuntimeConfigTest, RejectsInvalidDirtyRectThreshold) {
  {
    DisplayNodeRuntimeConfig runtime;
    runtime.dirty_rect_full_frame_threshold = -0.01;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("dirty_rect_full_frame_threshold"), std::string::npos);
  }

  {
    DisplayNodeRuntimeConfig runtime;
    runtime.dirty_rect_full_frame_threshold = 1.01;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("dirty_rect_full_frame_threshold"), std::string::npos);
  }

  {
    DisplayNodeRuntimeConfig runtime;
    runtime.dirty_rect_full_frame_threshold = std::numeric_limits<double>::quiet_NaN();

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("dirty_rect_full_frame_threshold"), std::string::npos);
  }
}

TEST(DisplayNodeRuntimeConfigTest, RejectsInvalidTextPositionAndScale) {
  {
    DisplayNodeRuntimeConfig runtime;
    runtime.text_x = -1;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("text_x"), std::string::npos);
  }

  {
    DisplayNodeRuntimeConfig runtime;
    runtime.text_y = static_cast<std::int64_t>(std::numeric_limits<std::uint16_t>::max()) + 1;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("text_y"), std::string::npos);
  }

  {
    DisplayNodeRuntimeConfig runtime;
    runtime.text_scale = 0;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("text_scale"), std::string::npos);
  }
}

TEST(DisplayNodeRuntimeConfigTest, RejectsInvalidColors) {
  {
    DisplayNodeRuntimeConfig runtime;
    runtime.text_color_rgb565 = -1;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("text_color_rgb565"), std::string::npos);
  }

  {
    DisplayNodeRuntimeConfig runtime;
    runtime.clear_color_rgb565 =
      static_cast<std::int64_t>(std::numeric_limits<std::uint16_t>::max()) + 1;

    DisplayNodeConfig output;
    std::string error;
    EXPECT_FALSE(buildDisplayNodeConfig(runtime, output, error));
    EXPECT_NE(error.find("clear_color_rgb565"), std::string::npos);
  }
}

}  // namespace
}  // namespace st7789_ros_wrapper::ros
