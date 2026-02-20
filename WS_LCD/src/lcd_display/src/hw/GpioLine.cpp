#include "st7789_ros_wrapper/hw/GpioLine.hpp"

#include <cerrno>
#include <cstring>
#include <gpiod.h>

#include <memory>
#include <sstream>
#include <utility>

namespace st7789_ros_wrapper::hw {
namespace {

class LinuxGpioLineBackend final : public GpioLineBackend {
public:
  gpiod_chip * openChip(const std::string & chip_path) override {
    return gpiod_chip_open(chip_path.c_str());
  }

  void closeChip(gpiod_chip * chip) override {
    gpiod_chip_close(chip);
  }

  gpiod_line * getLine(gpiod_chip * chip, const unsigned int line_offset) override {
    return gpiod_chip_get_line(chip, line_offset);
  }

  int requestOutput(gpiod_line * line, const std::string & consumer, const int default_value) override {
    return gpiod_line_request_output(line, consumer.c_str(), default_value);
  }

  int setValue(gpiod_line * line, const int value) override {
    return gpiod_line_set_value(line, value);
  }

  void releaseLine(gpiod_line * line) override {
    gpiod_line_release(line);
  }

  int lastErrorNumber() const override {
    return errno;
  }

  std::string errorMessage(const int error_number) const override {
    return std::strerror(error_number);
  }
};

}  // namespace

std::shared_ptr<GpioLineBackend> makeDefaultGpioLineBackend() {
  return std::make_shared<LinuxGpioLineBackend>();
}

GpioLine::GpioLine(GpioLineConfig config, std::shared_ptr<GpioLineBackend> backend)
: config_(std::move(config)), backend_(std::move(backend)) {
  if (backend_ == nullptr) {
    backend_ = makeDefaultGpioLineBackend();
  }
}

GpioLine::~GpioLine() {
  release();
}

bool GpioLine::requestOutput(const bool initial_value) {
  last_error_.clear();

  if (config_.chip_path.empty()) {
    setLastError("GPIO chip path is empty");
    return false;
  }

  if (requested_) {
    return setValue(initial_value);
  }

  chip_ = backend_->openChip(config_.chip_path);
  if (chip_ == nullptr) {
    setLastErrorFromErrno("Failed to open GPIO chip " + config_.chip_path);
    return false;
  }

  line_ = backend_->getLine(chip_, config_.offset);
  if (line_ == nullptr) {
    setLastErrorFromErrno("Failed to get GPIO line " + std::to_string(config_.offset));
    backend_->closeChip(chip_);
    chip_ = nullptr;
    return false;
  }

  const std::string consumer = config_.consumer.empty() ? "st7789_ros_wrapper" : config_.consumer;
  if (backend_->requestOutput(line_, consumer, initial_value ? 1 : 0) < 0) {
    setLastErrorFromErrno("Failed to request GPIO line as output");
    backend_->releaseLine(line_);
    line_ = nullptr;
    backend_->closeChip(chip_);
    chip_ = nullptr;
    return false;
  }

  requested_ = true;
  value_ = initial_value;
  return true;
}

void GpioLine::release() {
  if (line_ != nullptr) {
    backend_->releaseLine(line_);
    line_ = nullptr;
  }

  if (chip_ != nullptr) {
    backend_->closeChip(chip_);
    chip_ = nullptr;
  }

  requested_ = false;
}

bool GpioLine::isRequested() const noexcept {
  return requested_;
}

bool GpioLine::setValue(const bool value) {
  last_error_.clear();

  if (!requested_ || line_ == nullptr) {
    setLastError("GPIO line is not requested");
    return false;
  }

  if (backend_->setValue(line_, value ? 1 : 0) < 0) {
    setLastErrorFromErrno("Failed to set GPIO line value");
    return false;
  }

  value_ = value;
  return true;
}

bool GpioLine::value() const noexcept {
  return value_;
}

const GpioLineConfig & GpioLine::config() const noexcept {
  return config_;
}

const std::string & GpioLine::lastError() const noexcept {
  return last_error_;
}

void GpioLine::setLastError(const std::string & message) {
  last_error_ = message;
}

void GpioLine::setLastErrorFromErrno(const std::string & prefix) {
  const auto error_number = backend_->lastErrorNumber();
  std::ostringstream stream;
  stream << prefix << ": " << backend_->errorMessage(error_number) << " (errno=" << error_number << ")";
  setLastError(stream.str());
}

}  // namespace st7789_ros_wrapper::hw
