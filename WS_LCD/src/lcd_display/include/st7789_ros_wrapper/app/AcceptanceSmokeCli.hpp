#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"

namespace st7789_ros_wrapper::app {

struct AcceptanceSmokeConfig {
  ScreenRuntimeConfig display;
  std::int64_t backlight_cycles{2};
  std::int64_t backlight_delay_ms{250};
  std::int64_t benchmark_frames{120};
  double min_fps{0.0};
  bool leave_backlight_on{true};
  bool show_help{false};
};

bool parseAcceptanceSmokeArgs(
  int argc, char ** argv, AcceptanceSmokeConfig & output, std::string & error);

bool validateAcceptanceSmokeConfig(const AcceptanceSmokeConfig & config, std::string & error);

double calculateFramesPerSecond(std::size_t frame_count, double elapsed_seconds);

std::string acceptanceSmokeUsage(const std::string & program_name);

}  // namespace st7789_ros_wrapper::app
