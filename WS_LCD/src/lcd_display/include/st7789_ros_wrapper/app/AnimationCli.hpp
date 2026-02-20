#pragma once

#include <cstdint>
#include <string>

#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"

namespace st7789_ros_wrapper::app {

struct AnimationCliConfig {
  ScreenRuntimeConfig display;
  std::string frames_directory;
  double fps{12.0};
  bool loop{true};
  std::int64_t max_frames{0};
  bool leave_backlight_on{true};
  bool show_help{false};
};

bool parseAnimationArgs(int argc, char ** argv, AnimationCliConfig & output, std::string & error);

bool validateAnimationConfig(const AnimationCliConfig & config, std::string & error);

std::string animationUsage(const std::string & program_name);

}  // namespace st7789_ros_wrapper::app
