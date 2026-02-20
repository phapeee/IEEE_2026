#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cerrno>

#include <signal.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/parameter_map.hpp"
#include "st7789_ros_wrapper/app/AnimationCli.hpp"
#include "st7789_ros_wrapper/app/Screen.hpp"
#include "st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp"
#include "st7789_ros_wrapper/app/ShowImageCli.hpp"
#include "st7789_ros_wrapper/assets/FrameSequence.hpp"
#include "st7789_ros_wrapper/assets/ImageLoader.hpp"
#include "st7789_ros_wrapper/gfx/Color565.hpp"

namespace {

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_stop_requested{false};

void stopSignalHandler(int) {
  g_stop_requested.store(true);
}

Clock::duration frameIntervalFromFps(const double fps) {
  const auto interval = std::chrono::duration<double>(1.0 / fps);
  const auto ticks = std::chrono::duration_cast<Clock::duration>(interval);
  if (ticks <= Clock::duration::zero()) {
    return Clock::duration(1);
  }
  return ticks;
}

std::filesystem::path runtimeDirectory() {
  std::error_code error;
  const auto temp_dir = std::filesystem::temp_directory_path(error);
  if (error) {
    return "/tmp";
  }
  return temp_dir;
}

std::filesystem::path animationPidFilePath() {
  return runtimeDirectory() / "st7789_lcd_animation.pid";
}

std::filesystem::path animationStopFilePath() {
  return runtimeDirectory() / "st7789_lcd_animation.stop";
}

void removeFileIfExists(const std::filesystem::path & path) {
  std::error_code error;
  std::filesystem::remove(path, error);
}

bool writeAnimationPidFile(const std::filesystem::path & path, std::string & error) {
  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) {
    error = "Failed to create PID file: " + path.string();
    return false;
  }

  output << static_cast<long long>(::getpid()) << "\n";
  if (!output.good()) {
    error = "Failed to write PID file: " + path.string();
    return false;
  }

  return true;
}

bool readAnimationPidFile(
  const std::filesystem::path & path, pid_t & pid, std::string & error)
{
  error.clear();
  std::ifstream input(path);
  if (!input.is_open()) {
    error = "Animation PID file not found";
    return false;
  }

  long long raw_pid = 0;
  input >> raw_pid;
  if (!input.good() && !input.eof()) {
    error = "Failed to read animation PID file";
    return false;
  }

  if (raw_pid <= 0 || raw_pid > std::numeric_limits<pid_t>::max()) {
    error = "Animation PID file contains an invalid PID";
    return false;
  }

  pid = static_cast<pid_t>(raw_pid);
  return true;
}

bool writeStopRequestFile(const std::filesystem::path & path, std::string & error) {
  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) {
    error = "Failed to create stop request file: " + path.string();
    return false;
  }

  output << "stop\n";
  if (!output.good()) {
    error = "Failed to write stop request file: " + path.string();
    return false;
  }

  return true;
}

bool isStopRequested(const std::filesystem::path & stop_file) {
  if (g_stop_requested.load()) {
    return true;
  }

  std::error_code error;
  return std::filesystem::exists(stop_file, error) && !error;
}

void installStopSignalHandlers() {
  std::signal(SIGINT, stopSignalHandler);
  std::signal(SIGTERM, stopSignalHandler);
}

struct RuntimeFilesGuard {
  std::filesystem::path pid_file;
  std::filesystem::path stop_file;

  ~RuntimeFilesGuard() {
    removeFileIfExists(pid_file);
    removeFileIfExists(stop_file);
  }
};

struct DisplayConfigSource {
  bool loaded{false};
  std::filesystem::path path;
  st7789_ros_wrapper::app::ScreenRuntimeConfig runtime;
};

struct GlobalCliOptions {
  bool use_display_config{true};
  std::optional<std::filesystem::path> config_path_override;
};

bool parseGlobalCliOptions(
  const int argc, char ** argv, GlobalCliOptions & options, int & command_index, std::string & error)
{
  error.clear();
  options = GlobalCliOptions{};
  command_index = -1;

  int index = 1;
  while (index < argc) {
    if (argv[index] == nullptr) {
      error = "Invalid null argument";
      return false;
    }

    const std::string arg = argv[index];
    if (arg == "--config") {
      if (index + 1 >= argc || argv[index + 1] == nullptr) {
        error = "Missing value for --config";
        return false;
      }
      options.config_path_override = std::filesystem::path(argv[index + 1]);
      options.use_display_config = true;
      index += 2;
      continue;
    }
    if (arg == "--no-config") {
      options.use_display_config = false;
      options.config_path_override.reset();
      ++index;
      continue;
    }
    command_index = index;
    return true;
  }

  return true;
}

std::optional<std::filesystem::path> tryResolveDefaultDisplayYamlPath() {
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("st7789_ros_wrapper");
    const std::filesystem::path candidate = std::filesystem::path(share_dir) / "config" / "display.yaml";
    std::error_code error;
    if (std::filesystem::exists(candidate, error) && !error) {
      return candidate;
    }
  } catch (const ament_index_cpp::PackageNotFoundError &) {
  } catch (const std::exception &) {
  }

  const auto current_path = std::filesystem::current_path();
  const std::vector<std::filesystem::path> fallback_candidates{
    current_path / "config" / "display.yaml",
    current_path / "WS_LCD" / "config" / "display.yaml",
  };

  for (const auto & candidate : fallback_candidates) {
    std::error_code error;
    if (std::filesystem::exists(candidate, error) && !error) {
      return candidate;
    }
  }

  return std::nullopt;
}

bool assignDisplayRuntimeParameter(
  const rclcpp::Parameter & parameter, st7789_ros_wrapper::app::ScreenRuntimeConfig & output,
  std::string & error)
{
  error.clear();

  try {
    const auto & name = parameter.get_name();
    if (name == "spi_device") {
      output.spi_device = parameter.as_string();
    } else if (name == "spi_speed_hz") {
      output.spi_speed_hz = parameter.as_int();
    } else if (name == "spi_mode") {
      output.spi_mode = parameter.as_int();
    } else if (name == "spi_bits_per_word") {
      output.spi_bits_per_word = parameter.as_int();
    } else if (name == "gpiochip") {
      output.gpiochip = parameter.as_string();
    } else if (name == "dc_gpio") {
      output.dc_gpio = parameter.as_int();
    } else if (name == "rst_gpio") {
      output.rst_gpio = parameter.as_int();
    } else if (name == "bl_gpio") {
      output.bl_gpio = parameter.as_int();
    } else if (name == "use_threads") {
      output.use_threads = parameter.as_bool();
    } else if (name == "frame_queue_capacity") {
      output.frame_queue_capacity = parameter.as_int();
    } else if (name == "rotation_degrees") {
      output.rotation_degrees = parameter.as_int();
    } else if (name == "color_order_bgr") {
      output.color_order_bgr = parameter.as_bool();
    } else if (name == "display_inversion") {
      output.display_inversion = parameter.as_bool();
    } else if (name == "x_offset") {
      output.x_offset = parameter.as_int();
    } else if (name == "y_offset") {
      output.y_offset = parameter.as_int();
    }
  } catch (const std::exception &) {
    error = "Invalid type for parameter '" + parameter.get_name() + "' in display YAML";
    return false;
  }

  return true;
}

bool loadDisplayRuntimeConfigFromYaml(
  const std::filesystem::path & yaml_path, st7789_ros_wrapper::app::ScreenRuntimeConfig & output,
  std::string & error)
{
  error.clear();
  output = st7789_ros_wrapper::app::ScreenRuntimeConfig{};

  std::error_code filesystem_error;
  if (!std::filesystem::exists(yaml_path, filesystem_error) || filesystem_error) {
    error = "display YAML path does not exist: " + yaml_path.string();
    return false;
  }

  rclcpp::ParameterMap parameter_map;
  try {
    parameter_map = rclcpp::parameter_map_from_yaml_file(yaml_path.string());
  } catch (const std::exception & exception) {
    error = "Failed to parse display YAML '" + yaml_path.string() + "': " + exception.what();
    return false;
  }

  const std::vector<rclcpp::Parameter> * node_parameters = nullptr;
  if (const auto exact = parameter_map.find("/st7789_display_node"); exact != parameter_map.end()) {
    node_parameters = &exact->second;
  } else if (
    const auto relative = parameter_map.find("st7789_display_node");
    relative != parameter_map.end())
  {
    node_parameters = &relative->second;
  } else if (parameter_map.size() == 1U) {
    node_parameters = &parameter_map.begin()->second;
  }

  if (node_parameters == nullptr) {
    error = "display YAML does not contain 'st7789_display_node/ros__parameters'";
    return false;
  }

  for (const auto & parameter : *node_parameters) {
    if (!assignDisplayRuntimeParameter(parameter, output, error)) {
      return false;
    }
  }

  return true;
}

std::vector<std::string> displayRuntimeToCliArgs(
  const st7789_ros_wrapper::app::ScreenRuntimeConfig & runtime)
{
  return {
    "--spi-device",
    runtime.spi_device,
    "--spi-speed-hz",
    std::to_string(runtime.spi_speed_hz),
    "--spi-mode",
    std::to_string(runtime.spi_mode),
    "--spi-bits-per-word",
    std::to_string(runtime.spi_bits_per_word),
    "--dc-gpio",
    std::to_string(runtime.dc_gpio),
    "--rst-gpio",
    std::to_string(runtime.rst_gpio),
    "--bl-gpio",
    std::to_string(runtime.bl_gpio),
    "--rotation-degrees",
    std::to_string(runtime.rotation_degrees),
    "--x-offset",
    std::to_string(runtime.x_offset),
    "--y-offset",
    std::to_string(runtime.y_offset),
  };
}

std::vector<std::string> mergeDisplayDefaultsIntoArgs(
  const std::vector<std::string> & user_args, const DisplayConfigSource & display_source)
{
  std::vector<std::string> merged;
  if (user_args.empty()) {
    return merged;
  }

  const auto defaults = displayRuntimeToCliArgs(display_source.runtime);
  merged.reserve(user_args.size() + defaults.size());
  merged.push_back(user_args.front());
  merged.insert(merged.end(), defaults.begin(), defaults.end());
  merged.insert(merged.end(), user_args.begin() + 1, user_args.end());
  return merged;
}

std::string rootUsage(const std::string & program_name) {
  return
    "Usage: " + program_name + " [global options] <command> [options]\n"
    "Global options:\n"
    "  --config <path>        Use this display YAML file (ROS parameter format)\n"
    "  --no-config            Disable YAML config loading\n"
    "  (default: try package/workspace config/display.yaml)\n"
    "Commands:\n"
    "  image        Show one image on screen\n"
    "  animation    Play animation frames from a folder\n"
    "  stop         Stop a running animation started by this tool\n"
    "  help         Show this help message\n\n"
    "Examples:\n"
    "  " + program_name + " image --image /path/to/pic.png\n"
    "  " + program_name + " animation --frames-dir /path/to/frames --fps 12 --loop\n"
    "  " + program_name + " stop\n";
}

bool stripCenterFlags(
  const int argc, char ** argv, bool & center, std::vector<std::string> & forwarded,
  std::string & error)
{
  error.clear();
  forwarded.clear();
  center = false;
  if (argc <= 0 || argv == nullptr || argv[0] == nullptr) {
    error = "Invalid argument vector";
    return false;
  }

  forwarded.emplace_back(argv[0]);
  for (int index = 1; index < argc; ++index) {
    if (argv[index] == nullptr) {
      error = "Invalid null argument";
      return false;
    }
    const std::string arg = argv[index];
    if (arg == "--center") {
      center = true;
      continue;
    }
    if (arg == "--no-center") {
      center = false;
      continue;
    }
    forwarded.emplace_back(arg);
  }

  return true;
}

std::vector<char *> makeArgv(std::vector<std::string> & args) {
  std::vector<char *> output;
  output.reserve(args.size());
  for (auto & value : args) {
    output.push_back(value.data());
  }
  return output;
}

bool computeCenteredOffset(
  const std::uint16_t canvas_width, const std::uint16_t canvas_height, const std::uint16_t image_width,
  const std::uint16_t image_height, std::uint16_t & x, std::uint16_t & y)
{
  if (image_width > canvas_width || image_height > canvas_height) {
    return false;
  }

  x = static_cast<std::uint16_t>((static_cast<std::uint32_t>(canvas_width) - image_width) / 2U);
  y = static_cast<std::uint16_t>((static_cast<std::uint32_t>(canvas_height) - image_height) / 2U);
  return true;
}

std::string imageCommandUsage(const std::string & program_name) {
  std::string usage = st7789_ros_wrapper::app::showImageUsage(program_name + " image");
  usage += "Additional lcd_cli options:\n"
           "  --center                    Center image on screen without scaling\n"
           "  --no-center                 Disable centering (default)\n";
  return usage;
}

std::string animationCommandUsage(const std::string & program_name) {
  std::string usage = st7789_ros_wrapper::app::animationUsage(program_name + " animation");
  usage += "Additional lcd_cli options:\n"
           "  --center                    Center each frame on screen\n"
           "  --no-center                 Disable centering (default)\n";
  return usage;
}

int runImageCommand(
  int argc, char ** argv, const std::string & program_name,
  const DisplayConfigSource & display_source)
{
  bool center_image = false;
  std::vector<std::string> user_args;
  std::string error;
  if (!stripCenterFlags(argc, argv, center_image, user_args, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << imageCommandUsage(program_name);
    return 2;
  }

  std::vector<std::string> merged_args_storage;
  if (display_source.loaded) {
    merged_args_storage = mergeDisplayDefaultsIntoArgs(user_args, display_source);
  } else {
    merged_args_storage = std::move(user_args);
  }
  auto merged_args = makeArgv(merged_args_storage);

  st7789_ros_wrapper::app::ShowImageCliConfig cli_config;
  if (!st7789_ros_wrapper::app::parseShowImageArgs(
      static_cast<int>(merged_args.size()), merged_args.data(), cli_config, error))
  {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << imageCommandUsage(program_name);
    return 2;
  }

  if (cli_config.show_help) {
    std::cout << imageCommandUsage(program_name);
    return 0;
  }

  if (!st7789_ros_wrapper::app::validateShowImageConfig(cli_config, error)) {
    std::cerr << "Invalid image config: " << error << "\n";
    return 2;
  }

  if (display_source.loaded) {
    // These runtime fields are not exposed by ShowImage CLI flags.
    cli_config.display.gpiochip = display_source.runtime.gpiochip;
    cli_config.display.use_threads = display_source.runtime.use_threads;
    cli_config.display.frame_queue_capacity = display_source.runtime.frame_queue_capacity;
    cli_config.display.color_order_bgr = display_source.runtime.color_order_bgr;
    cli_config.display.display_inversion = display_source.runtime.display_inversion;
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
  if (center_image) {
    image = st7789_ros_wrapper::assets::ImageLoader::loadFile(cli_config.image_path);
  } else if (cli_config.fit_to_screen) {
    image = st7789_ros_wrapper::assets::ImageLoader::loadFileFitToSize(
      cli_config.image_path, screen.canvas().width(), screen.canvas().height());
  } else {
    image = st7789_ros_wrapper::assets::ImageLoader::loadFile(cli_config.image_path);
  }

  if (!image.has_value()) {
    std::cerr << "Failed to decode image: " << cli_config.image_path << "\n";
    return 5;
  }

  std::uint16_t draw_x = 0U;
  std::uint16_t draw_y = 0U;
  if (center_image) {
    if (!computeCenteredOffset(
        screen.canvas().width(), screen.canvas().height(), image->width, image->height, draw_x,
        draw_y))
    {
      std::cerr << "Image dimensions must be less than or equal to display dimensions when "
                   "--center is used. "
                << "image=" << image->width << "x" << image->height
                << " display=" << screen.canvas().width() << "x" << screen.canvas().height()
                << "\n";
      return 6;
    }
  } else if (!cli_config.fit_to_screen &&
    (image->width != screen.canvas().width() || image->height != screen.canvas().height()))
  {
    std::cerr << "Image dimensions must match display dimensions when --no-fit-to-screen is set. "
              << "image=" << image->width << "x" << image->height
              << " display=" << screen.canvas().width() << "x" << screen.canvas().height()
              << "\n";
    return 6;
  }

  screen.clear(st7789_ros_wrapper::gfx::color::kBlack);
  if (!screen.drawImage(draw_x, draw_y, *image)) {
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
            << " fit_to_screen=" << (cli_config.fit_to_screen ? "true" : "false")
            << " centered=" << (center_image ? "true" : "false") << "\n";
  return 0;
}

int runAnimationCommand(
  int argc, char ** argv, const std::string & program_name,
  const DisplayConfigSource & display_source)
{
  bool center_animation = false;
  std::vector<std::string> user_args;
  st7789_ros_wrapper::app::AnimationCliConfig cli_config;
  std::string error;
  if (!stripCenterFlags(argc, argv, center_animation, user_args, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << animationCommandUsage(program_name);
    return 2;
  }

  std::vector<std::string> merged_args_storage;
  if (display_source.loaded) {
    merged_args_storage = mergeDisplayDefaultsIntoArgs(user_args, display_source);
  } else {
    merged_args_storage = std::move(user_args);
  }
  auto merged_args = makeArgv(merged_args_storage);

  if (!st7789_ros_wrapper::app::parseAnimationArgs(
      static_cast<int>(merged_args.size()), merged_args.data(), cli_config, error))
  {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << animationCommandUsage(program_name);
    return 2;
  }

  if (cli_config.show_help) {
    std::cout << animationCommandUsage(program_name);
    return 0;
  }

  if (!st7789_ros_wrapper::app::validateAnimationConfig(cli_config, error)) {
    std::cerr << "Invalid animation config: " << error << "\n";
    return 2;
  }

  if (display_source.loaded) {
    // These runtime fields are not exposed by Animation CLI flags.
    cli_config.display.gpiochip = display_source.runtime.gpiochip;
    cli_config.display.use_threads = display_source.runtime.use_threads;
    cli_config.display.frame_queue_capacity = display_source.runtime.frame_queue_capacity;
    cli_config.display.color_order_bgr = display_source.runtime.color_order_bgr;
    cli_config.display.display_inversion = display_source.runtime.display_inversion;
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

  const auto pid_file = animationPidFilePath();
  const auto stop_file = animationStopFilePath();
  removeFileIfExists(stop_file);
  if (!writeAnimationPidFile(pid_file, error)) {
    std::cerr << error << "\n";
    return 9;
  }
  RuntimeFilesGuard runtime_files{pid_file, stop_file};

  g_stop_requested.store(false);
  installStopSignalHandlers();

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
  bool stopped = false;
  while (keep_running) {
    for (const auto & frame_path : frame_paths) {
      if (played_frames >= max_frames) {
        keep_running = false;
        break;
      }
      if (isStopRequested(stop_file)) {
        stopped = true;
        keep_running = false;
        break;
      }

      const auto frame = st7789_ros_wrapper::assets::ImageLoader::loadFile(frame_path);
      if (!frame.has_value()) {
        std::cerr << "Failed to decode frame: " << frame_path << "\n";
        return 5;
      }

      std::uint16_t draw_x = 0U;
      std::uint16_t draw_y = 0U;
      if (center_animation) {
        if (!computeCenteredOffset(
            screen.canvas().width(), screen.canvas().height(), frame->width, frame->height, draw_x,
            draw_y))
        {
          std::cerr << "Frame dimensions must be less than or equal to display dimensions when "
                       "--center is used. frame="
                    << frame->width << "x" << frame->height << " display="
                    << screen.canvas().width() << "x" << screen.canvas().height()
                    << " path=" << frame_path << "\n";
          return 6;
        }
        screen.clear(st7789_ros_wrapper::gfx::color::kBlack);
      } else if (
        frame->width != screen.canvas().width() || frame->height != screen.canvas().height())
      {
        std::cerr << "Frame dimensions must match display dimensions. frame=" << frame->width << "x"
                  << frame->height << " display=" << screen.canvas().width() << "x"
                  << screen.canvas().height() << " path=" << frame_path << "\n";
        return 6;
      }

      if (!screen.drawImage(draw_x, draw_y, *frame)) {
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

  if (stopped) {
    std::cout << "Animation stopped. frames_played=" << played_frames << "\n";
  } else {
    std::cout << "Animation playback complete. frames_played=" << played_frames << "\n";
  }
  return 0;
}

int runStopCommand() {
  const auto pid_file = animationPidFilePath();
  const auto stop_file = animationStopFilePath();

  std::string error;
  if (!writeStopRequestFile(stop_file, error)) {
    std::cerr << error << "\n";
    return 2;
  }

  pid_t pid = 0;
  if (!readAnimationPidFile(pid_file, pid, error)) {
    std::cout << "Stop request recorded. " << error << ".\n";
    return 0;
  }

  if (::kill(pid, SIGTERM) != 0) {
    if (errno == ESRCH) {
      removeFileIfExists(pid_file);
      std::cout << "Stop request recorded. No running animation process for PID " << pid << ".\n";
      return 0;
    }
    std::cerr << "Failed to signal animation PID " << pid << ". errno=" << errno << "\n";
    return 3;
  }

  std::cout << "Stop signal sent to animation process PID " << pid << ".\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv) {
  const std::string program_name = (argc > 0 && argv[0] != nullptr) ? argv[0] : "lcd_cli";
  if (argc < 2) {
    std::cerr << rootUsage(program_name);
    return 2;
  }

  GlobalCliOptions global_options;
  int command_index = -1;
  std::string error;
  if (!parseGlobalCliOptions(argc, argv, global_options, command_index, error)) {
    std::cerr << "Argument error: " << error << "\n\n";
    std::cerr << rootUsage(program_name);
    return 2;
  }

  if (command_index < 0 || command_index >= argc || argv[command_index] == nullptr) {
    std::cerr << rootUsage(program_name);
    return 2;
  }

  const std::string command = argv[command_index];
  if (command == "-h" || command == "--help" || command == "help") {
    std::cout << rootUsage(program_name);
    return 0;
  }

  DisplayConfigSource display_source;
  const bool command_uses_display = (command == "image" || command == "animation");
  if (command_uses_display && global_options.use_display_config) {
    std::optional<std::filesystem::path> yaml_path = global_options.config_path_override;
    if (!yaml_path.has_value()) {
      yaml_path = tryResolveDefaultDisplayYamlPath();
    }

    if (yaml_path.has_value()) {
      display_source.path = *yaml_path;
      if (!loadDisplayRuntimeConfigFromYaml(display_source.path, display_source.runtime, error)) {
        std::cerr << error << "\n";
        return 2;
      }
      display_source.loaded = true;
    } else {
      std::cerr << "Warning: no display YAML found; using built-in display defaults.\n";
    }
  }

  const int subcommand_argc = argc - command_index;
  char ** subcommand_argv = argv + command_index;

  if (command == "image") {
    return runImageCommand(subcommand_argc, subcommand_argv, program_name, display_source);
  }
  if (command == "animation") {
    return runAnimationCommand(subcommand_argc, subcommand_argv, program_name, display_source);
  }
  if (command == "stop") {
    if (subcommand_argc > 1) {
      std::cerr << "The stop command does not accept extra arguments.\n";
      return 2;
    }
    return runStopCommand();
  }

  std::cerr << "Unknown command: " << command << "\n\n";
  std::cerr << rootUsage(program_name);
  return 2;
}
