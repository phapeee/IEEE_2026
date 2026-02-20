#include <fcntl.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/hw/SpiBus.hpp"

namespace st7789_ros_wrapper::hw {
namespace {

class FakeSpiBusBackend final : public SpiBusBackend {
public:
  int open_return_fd{7};
  bool set_mode_result{true};
  bool set_bits_result{true};
  bool set_speed_result{true};
  std::vector<long> write_results;
  int last_error_number{5};

  int open_calls{0};
  int close_calls{0};
  int set_mode_calls{0};
  int set_bits_calls{0};
  int set_speed_calls{0};
  int write_calls{0};

  std::string last_open_path;
  int last_open_flags{0};
  int last_close_fd{-1};
  std::uint8_t last_mode{0};
  std::uint8_t last_bits{0};
  std::uint32_t last_speed{0};
  std::vector<std::size_t> write_sizes;

  int openDevice(const std::string & path, const int flags) override {
    ++open_calls;
    last_open_path = path;
    last_open_flags = flags;
    return open_return_fd;
  }

  int closeDevice(const int fd) override {
    ++close_calls;
    last_close_fd = fd;
    return 0;
  }

  bool setMode(const int, const std::uint8_t mode) override {
    ++set_mode_calls;
    last_mode = mode;
    return set_mode_result;
  }

  bool setBitsPerWord(const int, const std::uint8_t bits_per_word) override {
    ++set_bits_calls;
    last_bits = bits_per_word;
    return set_bits_result;
  }

  bool setMaxSpeedHz(const int, const std::uint32_t speed_hz) override {
    ++set_speed_calls;
    last_speed = speed_hz;
    return set_speed_result;
  }

  long writeBytes(const int, const std::uint8_t *, const std::size_t size) override {
    ++write_calls;
    write_sizes.push_back(size);

    if (!write_results.empty()) {
      const auto result = write_results.front();
      write_results.erase(write_results.begin());
      return result;
    }

    return static_cast<long>(size);
  }

  int lastErrorNumber() const override {
    return last_error_number;
  }

  std::string errorMessage(const int error_number) const override {
    return "err-" + std::to_string(error_number);
  }
};

TEST(SpiBusTest, OpenSuccessConfiguresDeviceAndIsIdempotent) {
  auto backend = std::make_shared<FakeSpiBusBackend>();
  SpiBusConfig config;
  config.device_path = "/dev/spidev0.0";
  config.mode = 3;
  config.bits_per_word = 8;
  config.speed_hz = 16000000;

  SpiBus bus(config, backend);

  EXPECT_TRUE(bus.open());
  EXPECT_TRUE(bus.isOpen());
  EXPECT_TRUE(bus.open());
  EXPECT_TRUE(bus.lastError().empty());

  EXPECT_EQ(backend->open_calls, 1);
  EXPECT_EQ(backend->set_mode_calls, 1);
  EXPECT_EQ(backend->set_bits_calls, 1);
  EXPECT_EQ(backend->set_speed_calls, 1);
  EXPECT_EQ(backend->last_open_path, "/dev/spidev0.0");
  EXPECT_NE(backend->last_open_flags & O_RDWR, 0);
  EXPECT_NE(backend->last_open_flags & O_CLOEXEC, 0);
  EXPECT_EQ(backend->last_mode, 3);
  EXPECT_EQ(backend->last_bits, 8);
  EXPECT_EQ(backend->last_speed, 16000000U);
}

TEST(SpiBusTest, OpenFailsForInvalidConfiguration) {
  auto backend = std::make_shared<FakeSpiBusBackend>();

  SpiBusConfig empty_path;
  empty_path.device_path = "";
  SpiBus bus_empty_path(empty_path, backend);
  EXPECT_FALSE(bus_empty_path.open());
  EXPECT_EQ(bus_empty_path.lastError(), "SPI device path is empty");
  EXPECT_EQ(backend->open_calls, 0);

  SpiBusConfig zero_speed;
  zero_speed.speed_hz = 0;
  SpiBus bus_zero_speed(zero_speed, backend);
  EXPECT_FALSE(bus_zero_speed.open());
  EXPECT_EQ(bus_zero_speed.lastError(), "SPI speed_hz must be greater than zero");

  SpiBusConfig zero_bits;
  zero_bits.bits_per_word = 0;
  SpiBus bus_zero_bits(zero_bits, backend);
  EXPECT_FALSE(bus_zero_bits.open());
  EXPECT_EQ(bus_zero_bits.lastError(), "SPI bits_per_word must be greater than zero");
}

TEST(SpiBusTest, OpenFailsWhenSyscallsFailAndClosesOnPartialFailure) {
  {
    auto backend = std::make_shared<FakeSpiBusBackend>();
    backend->open_return_fd = -1;
    backend->last_error_number = 13;

    SpiBus bus({}, backend);
    EXPECT_FALSE(bus.open());
    EXPECT_NE(bus.lastError().find("Failed to open SPI device"), std::string::npos);
    EXPECT_NE(bus.lastError().find("errno=13"), std::string::npos);
    EXPECT_EQ(backend->close_calls, 0);
  }

  {
    auto backend = std::make_shared<FakeSpiBusBackend>();
    backend->set_mode_result = false;
    backend->last_error_number = 22;

    SpiBus bus({}, backend);
    EXPECT_FALSE(bus.open());
    EXPECT_NE(bus.lastError().find("Failed to configure SPI mode"), std::string::npos);
    EXPECT_EQ(backend->close_calls, 1);
  }

  {
    auto backend = std::make_shared<FakeSpiBusBackend>();
    backend->set_bits_result = false;

    SpiBus bus({}, backend);
    EXPECT_FALSE(bus.open());
    EXPECT_NE(bus.lastError().find("Failed to configure SPI bits_per_word"), std::string::npos);
    EXPECT_EQ(backend->close_calls, 1);
  }

  {
    auto backend = std::make_shared<FakeSpiBusBackend>();
    backend->set_speed_result = false;

    SpiBus bus({}, backend);
    EXPECT_FALSE(bus.open());
    EXPECT_NE(bus.lastError().find("Failed to configure SPI speed_hz"), std::string::npos);
    EXPECT_EQ(backend->close_calls, 1);
  }
}

TEST(SpiBusTest, WriteRejectsInvalidUsageAndHandlesChunkedWrites) {
  auto backend = std::make_shared<FakeSpiBusBackend>();
  SpiBus bus({}, backend);

  const std::vector<std::uint8_t> payload{1, 2, 3, 4, 5};

  EXPECT_FALSE(bus.write(payload));
  EXPECT_EQ(bus.lastError(), "SPI bus is not open");

  ASSERT_TRUE(bus.open());

  EXPECT_FALSE(bus.write(nullptr, payload.size()));
  EXPECT_EQ(bus.lastError(), "Cannot write empty SPI payload");

  EXPECT_FALSE(bus.write(payload.data(), 0));
  EXPECT_EQ(bus.lastError(), "Cannot write empty SPI payload");

  backend->write_results = {2, 3};
  EXPECT_TRUE(bus.write(payload.data(), payload.size()));
  EXPECT_EQ(backend->write_calls, 2);
  ASSERT_EQ(backend->write_sizes.size(), 2U);
  EXPECT_EQ(backend->write_sizes[0], 5U);
  EXPECT_EQ(backend->write_sizes[1], 3U);
}

TEST(SpiBusTest, WriteFailsOnBackendFailure) {
  auto backend = std::make_shared<FakeSpiBusBackend>();
  SpiBus bus({}, backend);

  ASSERT_TRUE(bus.open());

  const std::vector<std::uint8_t> payload{9, 8, 7};

  backend->write_results = {-1};
  backend->last_error_number = 5;
  EXPECT_FALSE(bus.write(payload));
  EXPECT_NE(bus.lastError().find("Failed to write SPI payload"), std::string::npos);

  backend->write_results = {0};
  backend->last_error_number = 11;
  EXPECT_FALSE(bus.write(payload));
  EXPECT_NE(bus.lastError().find("errno=11"), std::string::npos);
}

TEST(SpiBusTest, CloseAndDestructorReleaseFileDescriptor) {
  auto backend = std::make_shared<FakeSpiBusBackend>();

  {
    SpiBus bus({}, backend);
    ASSERT_TRUE(bus.open());
    EXPECT_TRUE(bus.isOpen());
    bus.close();
    EXPECT_FALSE(bus.isOpen());
    EXPECT_EQ(backend->close_calls, 1);

    bus.close();
    EXPECT_EQ(backend->close_calls, 1);

    EXPECT_TRUE(bus.open());
    EXPECT_TRUE(bus.isOpen());
  }

  EXPECT_EQ(backend->close_calls, 2);
}

}  // namespace
}  // namespace st7789_ros_wrapper::hw
