#include <cstdint>
#include <iostream>
#include <string>

#include "st7789_ros_wrapper/app/Screen.hpp"
#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"
#include "st7789_ros_wrapper/app/TextDemoCli.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace {

st7789_ros_wrapper::gfx::Color565 makeColor(
  const std::int64_t red, const std::int64_t green, const std::int64_t blue)
{
  return st7789_ros_wrapper::gfx::rgb888To565(
    static_cast<std::uint8_t>(red), static_cast<std::uint8_t>(green),
    static_cast<std::uint8_t>(blue));
}

}  // namespace

int main(int argc, char ** argv) {
  st7789_ros_wrapper::app::TextDemoCliConfig cli_config;
  std::string error;
  if (!st7789_ros_wrapper::app::parseTextDemoArgs(argc, argv, cli_config, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << st7789_ros_wrapper::app::textDemoUsage(argv[0]);
    return 2;
  }

  if (cli_config.show_help) {
    std::cout << st7789_ros_wrapper::app::textDemoUsage(argv[0]);
    return 0;
  }

  if (!st7789_ros_wrapper::app::validateTextDemoConfig(cli_config, error)) {
    std::cerr << "Invalid text-demo config: " << error << "\n";
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

  if (!screen.setBacklight(true)) {
    std::cerr << "Failed to enable backlight.\n";
    return 4;
  }

  const auto text_color = makeColor(cli_config.text_r, cli_config.text_g, cli_config.text_b);
  const auto background_color = makeColor(cli_config.bg_r, cli_config.bg_g, cli_config.bg_b);

  screen.clear(background_color);
  if (!screen.drawText(
      static_cast<std::uint16_t>(cli_config.x), static_cast<std::uint16_t>(cli_config.y),
      cli_config.text, text_color, static_cast<std::uint8_t>(cli_config.scale)))
  {
    std::cerr << "Failed to draw text. Check coordinates, scale, and text visibility.\n";
    return 5;
  }

  if (!screen.present()) {
    std::cerr << "Failed to present text frame on display.\n";
    return 6;
  }

  if (!cli_config.leave_backlight_on && !screen.setBacklight(false)) {
    std::cerr << "Failed to disable backlight at exit.\n";
    return 4;
  }

  std::cout << "Text rendered successfully. text=\"" << cli_config.text << "\" x=" << cli_config.x
            << " y=" << cli_config.y << " scale=" << cli_config.scale << "\n";
  return 0;
}
