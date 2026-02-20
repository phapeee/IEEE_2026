#include "st7789_ros_wrapper/app/ShowImageCli.hpp"

#include <exception>
#include <sstream>

namespace st7789_ros_wrapper::app {
namespace {

bool parseInt64(
  const std::string & input, const char * const name, std::int64_t & output, std::string & error)
{
  try {
    std::size_t consumed = 0;
    const auto value = std::stoll(input, &consumed, 0);
    if (consumed != input.size()) {
      error = std::string(name) + " must be an integer";
      return false;
    }
    output = value;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " must be an integer";
    return false;
  }
}

}  // namespace

bool parseShowImageArgs(const int argc, char ** argv, ShowImageCliConfig & output, std::string & error) {
  error.clear();
  output = ShowImageCliConfig{};

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "-h" || arg == "--help") {
      output.show_help = true;
      return true;
    }

    const auto requireValue = [&](const char * const name, std::string & value_out) -> bool {
      if (i + 1 >= argc) {
        error = std::string("Missing value for ") + name;
        return false;
      }

      value_out = argv[++i];
      return true;
    };

    std::string value;
    if (arg == "--image") {
      if (!requireValue("--image", value)) {
        return false;
      }
      output.image_path = value;
    } else if (arg == "--fit-to-screen") {
      output.fit_to_screen = true;
    } else if (arg == "--no-fit-to-screen") {
      output.fit_to_screen = false;
    } else if (arg == "--spi-device") {
      if (!requireValue("--spi-device", value)) {
        return false;
      }
      output.display.spi_device = value;
    } else if (arg == "--spi-speed-hz") {
      if (!requireValue("--spi-speed-hz", value) ||
          !parseInt64(value, "--spi-speed-hz", output.display.spi_speed_hz, error))
      {
        return false;
      }
    } else if (arg == "--spi-mode") {
      if (!requireValue("--spi-mode", value) ||
          !parseInt64(value, "--spi-mode", output.display.spi_mode, error))
      {
        return false;
      }
    } else if (arg == "--spi-bits-per-word") {
      if (!requireValue("--spi-bits-per-word", value) ||
          !parseInt64(value, "--spi-bits-per-word", output.display.spi_bits_per_word, error))
      {
        return false;
      }
    } else if (arg == "--dc-gpio") {
      if (!requireValue("--dc-gpio", value) ||
          !parseInt64(value, "--dc-gpio", output.display.dc_gpio, error))
      {
        return false;
      }
    } else if (arg == "--rst-gpio") {
      if (!requireValue("--rst-gpio", value) ||
          !parseInt64(value, "--rst-gpio", output.display.rst_gpio, error))
      {
        return false;
      }
    } else if (arg == "--bl-gpio") {
      if (!requireValue("--bl-gpio", value) ||
          !parseInt64(value, "--bl-gpio", output.display.bl_gpio, error))
      {
        return false;
      }
    } else if (arg == "--rotation-degrees") {
      if (!requireValue("--rotation-degrees", value) ||
          !parseInt64(value, "--rotation-degrees", output.display.rotation_degrees, error))
      {
        return false;
      }
    } else if (arg == "--x-offset") {
      if (!requireValue("--x-offset", value) ||
          !parseInt64(value, "--x-offset", output.display.x_offset, error))
      {
        return false;
      }
    } else if (arg == "--y-offset") {
      if (!requireValue("--y-offset", value) ||
          !parseInt64(value, "--y-offset", output.display.y_offset, error))
      {
        return false;
      }
    } else if (arg == "--leave-backlight-on") {
      output.leave_backlight_on = true;
    } else if (arg == "--no-leave-backlight-on") {
      output.leave_backlight_on = false;
    } else {
      error = "Unknown argument: " + arg;
      return false;
    }
  }

  return true;
}

bool validateShowImageConfig(const ShowImageCliConfig & config, std::string & error) {
  error.clear();

  if (config.show_help) {
    return true;
  }

  if (config.image_path.empty()) {
    error = "image_path cannot be empty";
    return false;
  }

  return true;
}

std::string showImageUsage(const std::string & program_name) {
  std::ostringstream stream;
  stream << "Usage: " << program_name << " [options]\n"
         << "Options:\n"
         << "  --image <path>               Image file path (png/jpg/jpeg/bmp)\n"
         << "  --fit-to-screen              Resize/crop image to display size (default)\n"
         << "  --no-fit-to-screen           Require image dimensions to match display size\n"
         << "  --spi-device <path>          SPI device (default: /dev/spidev0.0)\n"
         << "  --spi-speed-hz <int>         SPI speed in Hz (default: 32000000)\n"
         << "  --spi-mode <0-3>             SPI mode (default: 0)\n"
         << "  --spi-bits-per-word <int>    SPI bits per word (default: 8)\n"
         << "  --dc-gpio <int>              DC GPIO line offset (default: 13)\n"
         << "  --rst-gpio <int>             RST GPIO line offset (default: 16)\n"
         << "  --bl-gpio <int>              BL GPIO line offset (default: 26)\n"
         << "  --rotation-degrees <deg>     One of: 0, 90, 180, 270\n"
         << "  --x-offset <int>             Panel X offset (default: 0)\n"
         << "  --y-offset <int>             Panel Y offset (default: 0)\n"
         << "  --leave-backlight-on         Leave BL on at exit (default)\n"
         << "  --no-leave-backlight-on      Turn BL off before exit\n"
         << "  -h, --help                   Show this help message\n";
  return stream.str();
}

}  // namespace st7789_ros_wrapper::app
