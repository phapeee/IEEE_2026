#include <chrono>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "st7789_ros_wrapper/app/AnimationCli.hpp"
#include "st7789_ros_wrapper/app/Screen.hpp"
#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"
#include "st7789_ros_wrapper/assets/FrameSequence.hpp"
#include "st7789_ros_wrapper/assets/ImageLoader.hpp"

namespace {

using Clock = std::chrono::steady_clock;

Clock::duration frameIntervalFromFps(const double fps) {
  const auto interval = std::chrono::duration<double>(1.0 / fps);
  const auto ticks = std::chrono::duration_cast<Clock::duration>(interval);
  if (ticks <= Clock::duration::zero()) {
    return Clock::duration(1);
  }
  return ticks;
}

}  // namespace

int main(int argc, char ** argv) {
  st7789_ros_wrapper::app::AnimationCliConfig cli_config;
  std::string error;
  if (!st7789_ros_wrapper::app::parseAnimationArgs(argc, argv, cli_config, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << st7789_ros_wrapper::app::animationUsage(argv[0]);
    return 2;
  }

  if (cli_config.show_help) {
    std::cout << st7789_ros_wrapper::app::animationUsage(argv[0]);
    return 0;
  }

  if (!st7789_ros_wrapper::app::validateAnimationConfig(cli_config, error)) {
    std::cerr << "Invalid animation config: " << error << "\n";
    return 2;
  }

  st7789_ros_wrapper::app::ScreenConfig screen_config;
  if (!st7789_ros_wrapper::app::buildScreenConfig(cli_config.display, screen_config, error)) {
    std::cerr << "Invalid display config: " << error << "\n";
    return 2;
  }

  std::vector<std::string> frame_paths;
  if (!st7789_ros_wrapper::assets::listFramePaths(
      cli_config.frames_directory, frame_paths, error))
  {
    std::cerr << "Invalid animation frame directory: " << error << "\n";
    return 2;
  }

  st7789_ros_wrapper::app::Screen screen(screen_config);
  if (!screen.begin()) {
    std::cerr << "Failed to initialize display hardware.\n";
    return 3;
  }

  if (!screen.setBacklight(true)) {
    std::cerr << "Failed to enable backlight.\n";
    return 4;
  }

  const auto frame_interval = frameIntervalFromFps(cli_config.fps);
  const auto max_frames =
    (cli_config.max_frames > 0) ? static_cast<std::size_t>(cli_config.max_frames) :
    std::numeric_limits<std::size_t>::max();

  std::size_t played_frames = 0;
  auto next_frame_time = Clock::now();
  bool keep_running = true;
  while (keep_running) {
    for (const auto & frame_path : frame_paths) {
      if (played_frames >= max_frames) {
        keep_running = false;
        break;
      }

      const auto frame = st7789_ros_wrapper::assets::ImageLoader::loadFile(frame_path);
      if (!frame.has_value()) {
        std::cerr << "Failed to decode frame: " << frame_path << "\n";
        return 5;
      }

      if (frame->width != screen.canvas().width() || frame->height != screen.canvas().height()) {
        std::cerr << "Frame dimensions must match display dimensions. frame=" << frame->width << "x"
                  << frame->height << " display=" << screen.canvas().width() << "x"
                  << screen.canvas().height() << " path=" << frame_path << "\n";
        return 6;
      }

      if (!screen.drawImage(0, 0, *frame)) {
        std::cerr << "Failed to draw frame to canvas: " << frame_path << "\n";
        return 7;
      }

      if (!screen.present()) {
        std::cerr << "Failed to present frame: " << frame_path << "\n";
        return 8;
      }

      ++played_frames;
      next_frame_time += frame_interval;
      std::this_thread::sleep_until(next_frame_time);
    }

    if (!cli_config.loop) {
      break;
    }
  }

  if (!cli_config.leave_backlight_on && !screen.setBacklight(false)) {
    std::cerr << "Failed to disable backlight at exit.\n";
    return 4;
  }

  std::cout << "Animation playback complete. frames_played=" << played_frames << "\n";
  return 0;
}
