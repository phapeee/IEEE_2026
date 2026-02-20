#pragma once

#include <cstdint>
#include <string>

#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"

namespace st7789_ros_wrapper::app {

struct TextDemoCliConfig {
  ScreenRuntimeConfig display;
  std::string text{"Hello ST7789"};
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t scale{2};
  std::int64_t text_r{255};
  std::int64_t text_g{255};
  std::int64_t text_b{255};
  std::int64_t bg_r{0};
  std::int64_t bg_g{0};
  std::int64_t bg_b{0};
  bool leave_backlight_on{true};
  bool show_help{false};
};

bool parseTextDemoArgs(int argc, char ** argv, TextDemoCliConfig & output, std::string & error);

bool validateTextDemoConfig(const TextDemoCliConfig & config, std::string & error);

std::string textDemoUsage(const std::string & program_name);

}  // namespace st7789_ros_wrapper::app
