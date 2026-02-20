#pragma once

#include <string>

#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"

namespace st7789_ros_wrapper::app {

struct ShowImageCliConfig {
  ScreenRuntimeConfig display;
  std::string image_path;
  bool fit_to_screen{true};
  bool leave_backlight_on{true};
  bool show_help{false};
};

bool parseShowImageArgs(int argc, char ** argv, ShowImageCliConfig & output, std::string & error);

bool validateShowImageConfig(const ShowImageCliConfig & config, std::string & error);

std::string showImageUsage(const std::string & program_name);

}  // namespace st7789_ros_wrapper::app
