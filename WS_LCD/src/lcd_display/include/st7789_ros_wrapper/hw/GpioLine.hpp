#pragma once

#include <cstdint>
#include <memory>
#include <string>

struct gpiod_chip;
struct gpiod_line;

namespace st7789_ros_wrapper::hw {

struct GpioLineConfig {
  std::string chip_path{"/dev/gpiochip0"};
  std::uint32_t offset{0};
  std::string consumer{"st7789_ros_wrapper"};
};

class GpioLineBackend {
public:
  virtual ~GpioLineBackend() = default;

  virtual gpiod_chip * openChip(const std::string & chip_path) = 0;
  virtual void closeChip(gpiod_chip * chip) = 0;

  virtual gpiod_line * getLine(gpiod_chip * chip, unsigned int line_offset) = 0;
  virtual int requestOutput(gpiod_line * line, const std::string & consumer, int default_value) = 0;
  virtual int setValue(gpiod_line * line, int value) = 0;
  virtual void releaseLine(gpiod_line * line) = 0;

  virtual int lastErrorNumber() const = 0;
  virtual std::string errorMessage(int error_number) const = 0;
};

std::shared_ptr<GpioLineBackend> makeDefaultGpioLineBackend();

class GpioLine {
public:
  explicit GpioLine(GpioLineConfig config = {}, std::shared_ptr<GpioLineBackend> backend = nullptr);
  ~GpioLine();

  bool requestOutput(bool initial_value = false);
  void release();
  bool isRequested() const noexcept;

  bool setValue(bool value);
  bool value() const noexcept;

  const GpioLineConfig & config() const noexcept;
  const std::string & lastError() const noexcept;

private:
  void setLastError(const std::string & message);
  void setLastErrorFromErrno(const std::string & prefix);

  GpioLineConfig config_;
  std::shared_ptr<GpioLineBackend> backend_;
  gpiod_chip * chip_{nullptr};
  gpiod_line * line_{nullptr};
  bool requested_{false};
  bool value_{false};
  std::string last_error_;
};

}  // namespace st7789_ros_wrapper::hw
