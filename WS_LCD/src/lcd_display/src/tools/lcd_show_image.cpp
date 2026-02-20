#include <iostream>
#include <optional>
#include <string>

#include "st7789_ros_wrapper/app/Screen.hpp"
#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"
#include "st7789_ros_wrapper/app/ShowImageCli.hpp"
#include "st7789_ros_wrapper/assets/ImageLoader.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"

int main(int argc, char ** argv) {
  st7789_ros_wrapper::app::ShowImageCliConfig cli_config;
  std::string error;
  if (!st7789_ros_wrapper::app::parseShowImageArgs(argc, argv, cli_config, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << st7789_ros_wrapper::app::showImageUsage(argv[0]);
    return 2;
  }

  if (cli_config.show_help) {
    std::cout << st7789_ros_wrapper::app::showImageUsage(argv[0]);
    return 0;
  }

  if (!st7789_ros_wrapper::app::validateShowImageConfig(cli_config, error)) {
    std::cerr << "Invalid show-image config: " << error << "\n";
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

  std::optional<st7789_ros_wrapper::assets::Image565> image;
  if (cli_config.fit_to_screen) {
    image = st7789_ros_wrapper::assets::ImageLoader::loadFileFitToSize(
      cli_config.image_path, screen.canvas().width(), screen.canvas().height());
  } else {
    image = st7789_ros_wrapper::assets::ImageLoader::loadFile(cli_config.image_path);
  }

  if (!image.has_value()) {
    std::cerr << "Failed to decode image: " << cli_config.image_path << "\n";
    return 5;
  }

  if (!cli_config.fit_to_screen &&
    (image->width != screen.canvas().width() || image->height != screen.canvas().height()))
  {
    std::cerr << "Image dimensions must match display dimensions when --no-fit-to-screen is set. "
              << "image=" << image->width << "x" << image->height
              << " display=" << screen.canvas().width() << "x" << screen.canvas().height()
              << "\n";
    return 6;
  }

  screen.clear(st7789_ros_wrapper::gfx::color::kBlack);
  if (!screen.drawImage(0, 0, *image)) {
    std::cerr << "Failed to draw image on canvas.\n";
    return 7;
  }

  if (!screen.present()) {
    std::cerr << "Failed to present image on display.\n";
    return 8;
  }

  if (!cli_config.leave_backlight_on && !screen.setBacklight(false)) {
    std::cerr << "Failed to disable backlight at exit.\n";
    return 4;
  }

  std::cout << "Image displayed successfully. rendered=" << image->width << "x" << image->height
            << " fit_to_screen=" << (cli_config.fit_to_screen ? "true" : "false") << "\n";
  return 0;
}
