#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace st7789_ros_wrapper::hw {

struct SpiBusConfig {
  std::string device_path{"/dev/spidev0.0"};
  std::uint32_t speed_hz{32000000};
  std::uint8_t mode{0};
  std::uint8_t bits_per_word{8};
};

class SpiBusBackend {
public:
  virtual ~SpiBusBackend() = default;

  virtual int openDevice(const std::string & path, int flags) = 0;
  virtual int closeDevice(int fd) = 0;

  virtual bool setMode(int fd, std::uint8_t mode) = 0;
  virtual bool setBitsPerWord(int fd, std::uint8_t bits_per_word) = 0;
  virtual bool setMaxSpeedHz(int fd, std::uint32_t speed_hz) = 0;

  virtual long writeBytes(int fd, const std::uint8_t * data, std::size_t size) = 0;

  virtual int lastErrorNumber() const = 0;
  virtual std::string errorMessage(int error_number) const = 0;
};

std::shared_ptr<SpiBusBackend> makeDefaultSpiBusBackend();

class SpiBus {
public:
  explicit SpiBus(SpiBusConfig config = {}, std::shared_ptr<SpiBusBackend> backend = nullptr);
  ~SpiBus();

  bool open();
  void close();
  bool isOpen() const noexcept;

  bool write(const std::uint8_t * data, std::size_t size);
  bool write(const std::vector<std::uint8_t> & data);

  const SpiBusConfig & config() const noexcept;
  const std::string & lastError() const noexcept;

private:
  void setLastError(const std::string & message);
  void setLastErrorFromErrno(const std::string & prefix);

  SpiBusConfig config_;
  std::shared_ptr<SpiBusBackend> backend_;
  int fd_{-1};
  bool opened_{false};
  std::string last_error_;
};

}  // namespace st7789_ros_wrapper::hw
