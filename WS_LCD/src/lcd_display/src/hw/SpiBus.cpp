#include "st7789_ros_wrapper/hw/SpiBus.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <memory>
#include <sstream>
#include <utility>

namespace st7789_ros_wrapper::hw {
namespace {

class LinuxSpiBusBackend final : public SpiBusBackend {
public:
  int openDevice(const std::string & path, const int flags) override {
    return ::open(path.c_str(), flags);
  }

  int closeDevice(const int fd) override {
    return ::close(fd);
  }

  bool setMode(const int fd, const std::uint8_t mode) override {
    auto mode_copy = mode;
    return ::ioctl(fd, SPI_IOC_WR_MODE, &mode_copy) >= 0;
  }

  bool setBitsPerWord(const int fd, const std::uint8_t bits_per_word) override {
    auto bits_copy = bits_per_word;
    return ::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits_copy) >= 0;
  }

  bool setMaxSpeedHz(const int fd, const std::uint32_t speed_hz) override {
    auto speed_copy = speed_hz;
    return ::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_copy) >= 0;
  }

  long writeBytes(const int fd, const std::uint8_t * data, const std::size_t size) override {
    return static_cast<long>(::write(fd, data, size));
  }

  int lastErrorNumber() const override {
    return errno;
  }

  std::string errorMessage(const int error_number) const override {
    return std::strerror(error_number);
  }
};

}  // namespace

std::shared_ptr<SpiBusBackend> makeDefaultSpiBusBackend() {
  return std::make_shared<LinuxSpiBusBackend>();
}

SpiBus::SpiBus(SpiBusConfig config, std::shared_ptr<SpiBusBackend> backend)
: config_(std::move(config)), backend_(std::move(backend)) {
  if (backend_ == nullptr) {
    backend_ = makeDefaultSpiBusBackend();
  }
}

SpiBus::~SpiBus() {
  close();
}

bool SpiBus::open() {
  last_error_.clear();

  if (opened_) {
    return true;
  }

  if (config_.device_path.empty()) {
    setLastError("SPI device path is empty");
    return false;
  }

  if (config_.speed_hz == 0) {
    setLastError("SPI speed_hz must be greater than zero");
    return false;
  }

  if (config_.bits_per_word == 0) {
    setLastError("SPI bits_per_word must be greater than zero");
    return false;
  }

  const int opened_fd = backend_->openDevice(config_.device_path, O_RDWR | O_CLOEXEC);
  if (opened_fd < 0) {
    setLastErrorFromErrno("Failed to open SPI device " + config_.device_path);
    return false;
  }

  if (!backend_->setMode(opened_fd, config_.mode)) {
    setLastErrorFromErrno("Failed to configure SPI mode");
    backend_->closeDevice(opened_fd);
    return false;
  }

  if (!backend_->setBitsPerWord(opened_fd, config_.bits_per_word)) {
    setLastErrorFromErrno("Failed to configure SPI bits_per_word");
    backend_->closeDevice(opened_fd);
    return false;
  }

  if (!backend_->setMaxSpeedHz(opened_fd, config_.speed_hz)) {
    setLastErrorFromErrno("Failed to configure SPI speed_hz");
    backend_->closeDevice(opened_fd);
    return false;
  }

  fd_ = opened_fd;
  opened_ = true;
  return true;
}

void SpiBus::close() {
  if (!opened_) {
    return;
  }

  backend_->closeDevice(fd_);
  fd_ = -1;
  opened_ = false;
}

bool SpiBus::isOpen() const noexcept {
  return opened_;
}

bool SpiBus::write(const std::uint8_t * data, const std::size_t size) {
  last_error_.clear();

  if (!opened_) {
    setLastError("SPI bus is not open");
    return false;
  }

  if (data == nullptr || size == 0) {
    setLastError("Cannot write empty SPI payload");
    return false;
  }

  std::size_t written_total = 0;
  while (written_total < size) {
    const auto wrote = backend_->writeBytes(fd_, data + written_total, size - written_total);
    if (wrote <= 0) {
      setLastErrorFromErrno("Failed to write SPI payload");
      return false;
    }
    written_total += static_cast<std::size_t>(wrote);
  }

  return true;
}

bool SpiBus::write(const std::vector<std::uint8_t> & data) {
  return write(data.data(), data.size());
}

const SpiBusConfig & SpiBus::config() const noexcept {
  return config_;
}

const std::string & SpiBus::lastError() const noexcept {
  return last_error_;
}

void SpiBus::setLastError(const std::string & message) {
  last_error_ = message;
}

void SpiBus::setLastErrorFromErrno(const std::string & prefix) {
  const auto error_number = backend_->lastErrorNumber();
  std::ostringstream stream;
  stream << prefix << ": " << backend_->errorMessage(error_number) << " (errno=" << error_number << ")";
  setLastError(stream.str());
}

}  // namespace st7789_ros_wrapper::hw
