#include "st7789_ros_wrapper/app/AnimationCli.hpp"

#include <cmath>
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

bool parseDouble(
  const std::string & input, const char * const name, double & output, std::string & error)
{
  try {
    std::size_t consumed = 0;
    const auto value = std::stod(input, &consumed);
    if (consumed != input.size() || !std::isfinite(value)) {
      error = std::string(name) + " must be a finite number";
      return false;
    }
    output = value;
    return true;
  } catch (const std::exception &) {
    error = std::string(name) + " must be a finite number";
    return false;
  }
}

}  // namespace

bool parseAnimationArgs(
  const int argc, char ** argv, AnimationCliConfig & output, std::string & error)
{
  error.clear();
  output = AnimationCliConfig{};

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
    if (arg == "--frames-dir") {
      if (!requireValue("--frames-dir", value)) {
        return false;
      }
      output.frames_directory = value;
    } else if (arg == "--fps") {
      if (!requireValue("--fps", value) || !parseDouble(value, "--fps", output.fps, error)) {
        return false;
      }
    } else if (arg == "--max-frames") {
      if (!requireValue("--max-frames", value) ||
          !parseInt64(value, "--max-frames", output.max_frames, error))
      {
        return false;
      }
    } else if (arg == "--loop") {
      output.loop = true;
    } else if (arg == "--no-loop") {
      output.loop = false;
    } else if (arg == "--leave-backlight-on") {
      output.leave_backlight_on = true;
    } else if (arg == "--no-leave-backlight-on") {
      output.leave_backlight_on = false;
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
    } else {
      error = "Unknown argument: " + arg;
      return false;
    }
  }

  return true;
}

bool validateAnimationConfig(const AnimationCliConfig & config, std::string & error) {
  error.clear();

  if (config.show_help) {
    return true;
  }

  if (config.frames_directory.empty()) {
    error = "frames_directory cannot be empty";
    return false;
  }

  if (!std::isfinite(config.fps) || config.fps <= 0.0) {
    error = "fps must be greater than zero";
    return false;
  }

  if (config.max_frames < 0) {
    error = "max_frames must be zero or greater";
    return false;
  }

  return true;
}

std::string animationUsage(const std::string & program_name) {
  std::ostringstream stream;
  stream << "Usage: " << program_name << " [options]\n"
         << "Options:\n"
         << "  --frames-dir <path>          Frame directory (png/jpg/jpeg/bmp)\n"
         << "  --fps <float>                Target playback FPS (default: 12.0)\n"
         << "  --max-frames <int>           Stop after N frames (0 means unlimited, default: 0)\n"
         << "  --loop                       Loop playback continuously (default)\n"
         << "  --no-loop                    Play each frame once and stop\n"
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
