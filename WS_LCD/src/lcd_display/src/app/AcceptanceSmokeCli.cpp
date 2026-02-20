#include "st7789_ros_wrapper/app/AcceptanceSmokeCli.hpp"

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

bool parseAcceptanceSmokeArgs(
  const int argc, char ** argv, AcceptanceSmokeConfig & output, std::string & error)
{
  error.clear();
  output = AcceptanceSmokeConfig{};

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
    if (arg == "--spi-device") {
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
    } else if (arg == "--backlight-cycles") {
      if (!requireValue("--backlight-cycles", value) ||
          !parseInt64(value, "--backlight-cycles", output.backlight_cycles, error))
      {
        return false;
      }
    } else if (arg == "--backlight-delay-ms") {
      if (!requireValue("--backlight-delay-ms", value) ||
          !parseInt64(value, "--backlight-delay-ms", output.backlight_delay_ms, error))
      {
        return false;
      }
    } else if (arg == "--benchmark-frames") {
      if (!requireValue("--benchmark-frames", value) ||
          !parseInt64(value, "--benchmark-frames", output.benchmark_frames, error))
      {
        return false;
      }
    } else if (arg == "--min-fps") {
      if (!requireValue("--min-fps", value) || !parseDouble(value, "--min-fps", output.min_fps, error)) {
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

bool validateAcceptanceSmokeConfig(const AcceptanceSmokeConfig & config, std::string & error) {
  error.clear();

  if (config.show_help) {
    return true;
  }

  if (config.backlight_cycles <= 0) {
    error = "backlight_cycles must be greater than zero";
    return false;
  }

  if (config.backlight_delay_ms < 0) {
    error = "backlight_delay_ms must be zero or greater";
    return false;
  }

  if (config.benchmark_frames <= 0) {
    error = "benchmark_frames must be greater than zero";
    return false;
  }

  if (config.min_fps < 0.0) {
    error = "min_fps must be zero or greater";
    return false;
  }

  return true;
}

double calculateFramesPerSecond(const std::size_t frame_count, const double elapsed_seconds) {
  if (frame_count == 0 || elapsed_seconds <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(frame_count) / elapsed_seconds;
}

std::string acceptanceSmokeUsage(const std::string & program_name) {
  std::ostringstream stream;
  stream << "Usage: " << program_name << " [options]\n"
         << "Options:\n"
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
         << "  --backlight-cycles <int>     Toggle BL off/on cycles (default: 2)\n"
         << "  --backlight-delay-ms <int>   Delay between BL toggles (default: 250)\n"
         << "  --benchmark-frames <int>     Number of full-frame presents (default: 120)\n"
         << "  --min-fps <float>            Optional FPS pass threshold (default: 0)\n"
         << "  --leave-backlight-on         Leave BL on at exit (default)\n"
         << "  --no-leave-backlight-on      Turn BL off before exit\n"
         << "  -h, --help                   Show this help message\n";
  return stream.str();
}

}  // namespace st7789_ros_wrapper::app
