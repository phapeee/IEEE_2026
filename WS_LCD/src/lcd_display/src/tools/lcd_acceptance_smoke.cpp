#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "st7789_ros_wrapper/app/AcceptanceSmokeCli.hpp"
#include "st7789_ros_wrapper/app/Screen.hpp"
#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace {

void sleepMs(const std::int64_t delay_ms) {
  if (delay_ms <= 0) {
    return;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

}  // namespace

int main(int argc, char ** argv) {
  st7789_ros_wrapper::app::AcceptanceSmokeConfig cli_config;
  std::string error;
  if (!st7789_ros_wrapper::app::parseAcceptanceSmokeArgs(argc, argv, cli_config, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << st7789_ros_wrapper::app::acceptanceSmokeUsage(argv[0]);
    return 2;
  }

  if (cli_config.show_help) {
    std::cout << st7789_ros_wrapper::app::acceptanceSmokeUsage(argv[0]);
    return 0;
  }

  if (!st7789_ros_wrapper::app::validateAcceptanceSmokeConfig(cli_config, error)) {
    std::cerr << "Invalid acceptance config: " << error << "\n";
    return 2;
  }

  st7789_ros_wrapper::app::ScreenConfig screen_config;
  if (!st7789_ros_wrapper::app::buildScreenConfig(cli_config.display, screen_config, error)) {
    std::cerr << "Invalid display config: " << error << "\n";
    return 2;
  }

  st7789_ros_wrapper::app::Screen screen(screen_config);
  if (!screen.begin()) {
    std::cerr << "Failed to initialize display hardware.\n";
    return 3;
  }

  std::cout << "Display initialized. Running backlight toggle test...\n";
  for (std::int64_t cycle = 0; cycle < cli_config.backlight_cycles; ++cycle) {
    if (!screen.setBacklight(false)) {
      std::cerr << "Backlight OFF failed at cycle " << (cycle + 1) << "\n";
      return 4;
    }
    sleepMs(cli_config.backlight_delay_ms);

    if (!screen.setBacklight(true)) {
      std::cerr << "Backlight ON failed at cycle " << (cycle + 1) << "\n";
      return 4;
    }
    sleepMs(cli_config.backlight_delay_ms);
  }
  std::cout << "Backlight toggle test passed (" << cli_config.backlight_cycles << " cycles).\n";

  std::cout << "Running full-frame benchmark (" << cli_config.benchmark_frames << " frames)...\n";
  auto start_time = std::chrono::steady_clock::now();
  for (std::int64_t frame = 0; frame < cli_config.benchmark_frames; ++frame) {
    const auto color = (frame % 2 == 0) ? st7789_ros_wrapper::gfx::color::kRed
                                         : st7789_ros_wrapper::gfx::color::kBlue;
    screen.clear(color);
    if (!screen.present()) {
      std::cerr << "Frame present failed at frame index " << frame << "\n";
      return 5;
    }
  }
  const auto end_time = std::chrono::steady_clock::now();

  const auto elapsed_seconds =
    std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
  const auto fps = st7789_ros_wrapper::app::calculateFramesPerSecond(
    static_cast<std::size_t>(cli_config.benchmark_frames), elapsed_seconds);

  std::cout << std::fixed << std::setprecision(2)
            << "Benchmark result: " << cli_config.benchmark_frames << " frames in "
            << elapsed_seconds << " s -> " << fps << " FPS\n";

  if (cli_config.min_fps > 0.0 && fps < cli_config.min_fps) {
    std::cerr << std::fixed << std::setprecision(2)
              << "Benchmark failed: measured FPS " << fps << " is below min-fps "
              << cli_config.min_fps << "\n";
    return 6;
  }

  if (!cli_config.leave_backlight_on) {
    if (!screen.setBacklight(false)) {
      std::cerr << "Failed to turn backlight off at exit.\n";
      return 4;
    }
  }

  std::cout << "Acceptance smoke test passed.\n";
  return 0;
}
