#include "st7789_ros_wrapper/app/TextDemoCli.hpp"

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

bool validateByte(const std::int64_t value, const char * const name, std::string & error) {
  if (value < 0 || value > 255) {
    std::ostringstream stream;
    stream << name << " must be between 0 and 255";
    error = stream.str();
    return false;
  }
  return true;
}

}  // namespace

bool parseTextDemoArgs(const int argc, char ** argv, TextDemoCliConfig & output, std::string & error) {
  error.clear();
  output = TextDemoCliConfig{};

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
    if (arg == "--text") {
      if (!requireValue("--text", value)) {
        return false;
      }
      output.text = value;
    } else if (arg == "--x") {
      if (!requireValue("--x", value) || !parseInt64(value, "--x", output.x, error)) {
        return false;
      }
    } else if (arg == "--y") {
      if (!requireValue("--y", value) || !parseInt64(value, "--y", output.y, error)) {
        return false;
      }
    } else if (arg == "--scale") {
      if (!requireValue("--scale", value) || !parseInt64(value, "--scale", output.scale, error)) {
        return false;
      }
    } else if (arg == "--text-r") {
      if (!requireValue("--text-r", value) || !parseInt64(value, "--text-r", output.text_r, error)) {
        return false;
      }
    } else if (arg == "--text-g") {
      if (!requireValue("--text-g", value) || !parseInt64(value, "--text-g", output.text_g, error)) {
        return false;
      }
    } else if (arg == "--text-b") {
      if (!requireValue("--text-b", value) || !parseInt64(value, "--text-b", output.text_b, error)) {
        return false;
      }
    } else if (arg == "--bg-r") {
      if (!requireValue("--bg-r", value) || !parseInt64(value, "--bg-r", output.bg_r, error)) {
        return false;
      }
    } else if (arg == "--bg-g") {
      if (!requireValue("--bg-g", value) || !parseInt64(value, "--bg-g", output.bg_g, error)) {
        return false;
      }
    } else if (arg == "--bg-b") {
      if (!requireValue("--bg-b", value) || !parseInt64(value, "--bg-b", output.bg_b, error)) {
        return false;
      }
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

bool validateTextDemoConfig(const TextDemoCliConfig & config, std::string & error) {
  error.clear();

  if (config.show_help) {
    return true;
  }

  if (config.text.empty()) {
    error = "text cannot be empty";
    return false;
  }

  if (config.x < 0 || config.x > 0xFFFF) {
    error = "x must be between 0 and 65535";
    return false;
  }

  if (config.y < 0 || config.y > 0xFFFF) {
    error = "y must be between 0 and 65535";
    return false;
  }

  if (config.scale <= 0 || config.scale > 255) {
    error = "scale must be between 1 and 255";
    return false;
  }

  if (!validateByte(config.text_r, "text_r", error) ||
      !validateByte(config.text_g, "text_g", error) ||
      !validateByte(config.text_b, "text_b", error) ||
      !validateByte(config.bg_r, "bg_r", error) ||
      !validateByte(config.bg_g, "bg_g", error) ||
      !validateByte(config.bg_b, "bg_b", error))
  {
    return false;
  }

  return true;
}

std::string textDemoUsage(const std::string & program_name) {
  std::ostringstream stream;
  stream << "Usage: " << program_name << " [options]\n"
         << "Options:\n"
         << "  --text <value>               Text to draw (default: Hello ST7789)\n"
         << "  --x <int>                    Text origin X (default: 0)\n"
         << "  --y <int>                    Text origin Y (default: 0)\n"
         << "  --scale <int>                Glyph scale factor 1..255 (default: 2)\n"
         << "  --text-r <0-255>             Text red channel (default: 255)\n"
         << "  --text-g <0-255>             Text green channel (default: 255)\n"
         << "  --text-b <0-255>             Text blue channel (default: 255)\n"
         << "  --bg-r <0-255>               Background red channel (default: 0)\n"
         << "  --bg-g <0-255>               Background green channel (default: 0)\n"
         << "  --bg-b <0-255>               Background blue channel (default: 0)\n"
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
