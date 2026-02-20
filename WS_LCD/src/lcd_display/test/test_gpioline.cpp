#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "st7789_ros_wrapper/hw/GpioLine.hpp"

namespace st7789_ros_wrapper::hw {
namespace {

class FakeGpioLineBackend final : public GpioLineBackend {
public:
  gpiod_chip * open_chip_result{reinterpret_cast<gpiod_chip *>(0x1000)};
  gpiod_line * get_line_result{reinterpret_cast<gpiod_line *>(0x2000)};
  int request_output_result{0};
  int set_value_result{0};
  int last_error_number{5};

  int open_chip_calls{0};
  int close_chip_calls{0};
  int get_line_calls{0};
  int request_output_calls{0};
  int set_value_calls{0};
  int release_line_calls{0};

  std::string last_chip_path;
  unsigned int last_line_offset{0};
  std::string last_consumer;
  int last_default_value{0};
  int last_set_value{0};

  gpiod_chip * openChip(const std::string & chip_path) override {
    ++open_chip_calls;
    last_chip_path = chip_path;
    return open_chip_result;
  }

  void closeChip(gpiod_chip *) override {
    ++close_chip_calls;
  }

  gpiod_line * getLine(gpiod_chip *, const unsigned int line_offset) override {
    ++get_line_calls;
    last_line_offset = line_offset;
    return get_line_result;
  }

  int requestOutput(gpiod_line *, const std::string & consumer, const int default_value) override {
    ++request_output_calls;
    last_consumer = consumer;
    last_default_value = default_value;
    return request_output_result;
  }

  int setValue(gpiod_line *, const int value) override {
    ++set_value_calls;
    last_set_value = value;
    return set_value_result;
  }

  void releaseLine(gpiod_line *) override {
    ++release_line_calls;
  }

  int lastErrorNumber() const override {
    return last_error_number;
  }

  std::string errorMessage(const int error_number) const override {
    return "err-" + std::to_string(error_number);
  }
};

TEST(GpioLineTest, RequestOutputSuccessInitializesLine) {
  auto backend = std::make_shared<FakeGpioLineBackend>();
  GpioLineConfig config;
  config.chip_path = "/dev/gpiochip0";
  config.offset = 26;
  config.consumer = "lcd-test";

  GpioLine line(config, backend);

  EXPECT_TRUE(line.requestOutput(true));
  EXPECT_TRUE(line.isRequested());
  EXPECT_TRUE(line.value());
  EXPECT_TRUE(line.lastError().empty());

  EXPECT_EQ(backend->open_chip_calls, 1);
  EXPECT_EQ(backend->get_line_calls, 1);
  EXPECT_EQ(backend->request_output_calls, 1);
  EXPECT_EQ(backend->last_chip_path, "/dev/gpiochip0");
  EXPECT_EQ(backend->last_line_offset, 26U);
  EXPECT_EQ(backend->last_consumer, "lcd-test");
  EXPECT_EQ(backend->last_default_value, 1);
}

TEST(GpioLineTest, RequestOutputUsesDefaultConsumerWhenEmpty) {
  auto backend = std::make_shared<FakeGpioLineBackend>();
  GpioLineConfig config;
  config.consumer = "";

  GpioLine line(config, backend);

  EXPECT_TRUE(line.requestOutput(false));
  EXPECT_EQ(backend->last_consumer, "st7789_ros_wrapper");
  EXPECT_EQ(backend->last_default_value, 0);
}

TEST(GpioLineTest, RequestOutputFailsOnInvalidConfigOrBackendFailure) {
  {
    auto backend = std::make_shared<FakeGpioLineBackend>();
    GpioLineConfig config;
    config.chip_path = "";

    GpioLine line(config, backend);
    EXPECT_FALSE(line.requestOutput(true));
    EXPECT_EQ(line.lastError(), "GPIO chip path is empty");
    EXPECT_EQ(backend->open_chip_calls, 0);
  }

  {
    auto backend = std::make_shared<FakeGpioLineBackend>();
    backend->open_chip_result = nullptr;
    backend->last_error_number = 2;

    GpioLine line({}, backend);
    EXPECT_FALSE(line.requestOutput(true));
    EXPECT_NE(line.lastError().find("Failed to open GPIO chip"), std::string::npos);
    EXPECT_NE(line.lastError().find("errno=2"), std::string::npos);
    EXPECT_EQ(backend->close_chip_calls, 0);
  }

  {
    auto backend = std::make_shared<FakeGpioLineBackend>();
    backend->get_line_result = nullptr;

    GpioLine line({}, backend);
    EXPECT_FALSE(line.requestOutput(true));
    EXPECT_NE(line.lastError().find("Failed to get GPIO line"), std::string::npos);
    EXPECT_EQ(backend->close_chip_calls, 1);
  }

  {
    auto backend = std::make_shared<FakeGpioLineBackend>();
    backend->request_output_result = -1;

    GpioLine line({}, backend);
    EXPECT_FALSE(line.requestOutput(true));
    EXPECT_NE(line.lastError().find("Failed to request GPIO line as output"), std::string::npos);
    EXPECT_EQ(backend->release_line_calls, 1);
    EXPECT_EQ(backend->close_chip_calls, 1);
  }
}

TEST(GpioLineTest, SetValueRequiresRequestAndTracksCachedState) {
  auto backend = std::make_shared<FakeGpioLineBackend>();
  GpioLine line({}, backend);

  EXPECT_FALSE(line.setValue(true));
  EXPECT_EQ(line.lastError(), "GPIO line is not requested");

  ASSERT_TRUE(line.requestOutput(false));
  EXPECT_FALSE(line.value());

  backend->set_value_result = -1;
  backend->last_error_number = 16;
  EXPECT_FALSE(line.setValue(true));
  EXPECT_NE(line.lastError().find("Failed to set GPIO line value"), std::string::npos);
  EXPECT_FALSE(line.value());

  backend->set_value_result = 0;
  EXPECT_TRUE(line.setValue(true));
  EXPECT_TRUE(line.value());
  EXPECT_EQ(backend->last_set_value, 1);
}

TEST(GpioLineTest, ReRequestUsesSetValueAndReleaseIsIdempotent) {
  auto backend = std::make_shared<FakeGpioLineBackend>();
  GpioLine line({}, backend);

  ASSERT_TRUE(line.requestOutput(false));
  EXPECT_EQ(backend->open_chip_calls, 1);
  EXPECT_EQ(backend->request_output_calls, 1);

  EXPECT_TRUE(line.requestOutput(true));
  EXPECT_EQ(backend->open_chip_calls, 1);
  EXPECT_EQ(backend->request_output_calls, 1);
  EXPECT_EQ(backend->set_value_calls, 1);
  EXPECT_TRUE(line.value());

  line.release();
  EXPECT_FALSE(line.isRequested());
  EXPECT_EQ(backend->release_line_calls, 1);
  EXPECT_EQ(backend->close_chip_calls, 1);

  line.release();
  EXPECT_EQ(backend->release_line_calls, 1);
  EXPECT_EQ(backend->close_chip_calls, 1);
}

TEST(GpioLineTest, DestructorReleasesResources) {
  auto backend = std::make_shared<FakeGpioLineBackend>();

  {
    GpioLine line({}, backend);
    ASSERT_TRUE(line.requestOutput(true));
  }

  EXPECT_EQ(backend->release_line_calls, 1);
  EXPECT_EQ(backend->close_chip_calls, 1);
}

}  // namespace
}  // namespace st7789_ros_wrapper::hw
