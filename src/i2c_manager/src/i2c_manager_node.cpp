#include <cerrno>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "i2c_manager/srv/i2c_transfer.hpp"

namespace
{
constexpr char kDefaultServiceName[] = "i2c_manager/transfer";
}

class I2cManagerNode : public rclcpp::Node
{
public:
  I2cManagerNode()
  : rclcpp::Node("i2c_manager")
  {
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    service_name_ = declare_parameter<std::string>("service_name", kDefaultServiceName);
    recovery_scl_gpio_ = declare_parameter<int>("recovery_scl_gpio", -1);
    recovery_pulses_ = declare_parameter<int>("recovery_pulses", 9);
    recovery_delay_us_ = declare_parameter<int>("recovery_delay_us", 5);
    log_i2c_commands_ = declare_parameter<bool>("log_i2c_commands", false);

    service_ = create_service<i2c_manager::srv::I2cTransfer>(
      service_name_,
      std::bind(&I2cManagerNode::handle_transfer, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "I2C manager ready on %s (device %s)",
      service_name_.c_str(), i2c_device_.c_str());
  }

  ~I2cManagerNode() override
  {
    close_i2c();
  }

private:
  void handle_transfer(
    const std::shared_ptr<i2c_manager::srv::I2cTransfer::Request> request,
    std::shared_ptr<i2c_manager::srv::I2cTransfer::Response> response)
  {
    std::lock_guard<std::mutex> lock(i2c_mutex_);

    response->success = false;
    response->read_data.clear();
    response->error_code = 0;
    response->error.clear();

    if (log_i2c_commands_ && !request->write_data.empty() && request->write_data[0] == 0x01)
    {
      RCLCPP_INFO(
        get_logger(), "I2C command write to 0x%02x: %s (read_len=%u)", request->address,
        bytes_to_hex(request->write_data).c_str(), request->read_length);
    }

    if (!ensure_open())
    {
      response->error_code = errno;
      response->error = strerror(errno);
      attempt_recovery("open");
      return;
    }

    if (request->read_length == 0 && request->write_data.empty())
    {
      response->error_code = EINVAL;
      response->error = "Empty I2C transfer";
      return;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, request->address) < 0)
    {
      response->error_code = errno;
      response->error = strerror(errno);
      attempt_recovery("I2C_SLAVE ioctl");
      return;
    }

    if (!request->write_data.empty() && request->read_length > 0)
    {
      std::vector<uint8_t> read_buf(request->read_length, 0);

      struct i2c_msg messages[2];
      messages[0].addr = static_cast<uint16_t>(request->address);
      messages[0].flags = 0;
      messages[0].len = static_cast<__u16>(request->write_data.size());
      messages[0].buf = const_cast<uint8_t *>(request->write_data.data());

      messages[1].addr = static_cast<uint16_t>(request->address);
      messages[1].flags = I2C_M_RD;
      messages[1].len = static_cast<__u16>(read_buf.size());
      messages[1].buf = read_buf.data();

      struct i2c_rdwr_ioctl_data packets;
      packets.msgs = messages;
      packets.nmsgs = 2;

      if (ioctl(i2c_fd_, I2C_RDWR, &packets) < 0)
      {
        response->error_code = errno;
        response->error = strerror(errno);
        attempt_recovery("I2C_RDWR");
        return;
      }

      response->read_data = std::move(read_buf);
    }
    else if (!request->write_data.empty())
    {
      const ssize_t written = ::write(i2c_fd_, request->write_data.data(), request->write_data.size());
      if (written != static_cast<ssize_t>(request->write_data.size()))
      {
        response->error_code = errno;
        response->error = strerror(errno);
        attempt_recovery("write");
        return;
      }
    }
    else
    {
      std::vector<uint8_t> read_buf(request->read_length, 0);
      const ssize_t read_bytes = ::read(i2c_fd_, read_buf.data(), read_buf.size());
      if (read_bytes != static_cast<ssize_t>(read_buf.size()))
      {
        response->error_code = errno;
        response->error = strerror(errno);
        attempt_recovery("read");
        return;
      }
      response->read_data = std::move(read_buf);
    }

    response->success = true;
  }

  bool ensure_open()
  {
    if (i2c_fd_ >= 0)
    {
      return true;
    }

    i2c_fd_ = ::open(i2c_device_.c_str(), O_RDWR | O_CLOEXEC);
    return i2c_fd_ >= 0;
  }

  static std::string bytes_to_hex(const std::vector<uint8_t> & data)
  {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < data.size(); ++i)
    {
      if (i)
      {
        oss << ' ';
      }
      oss << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
  }

  bool attempt_recovery(const char * reason)
  {
    if (recovery_scl_gpio_ < 0)
    {
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_recovery_time_ < std::chrono::milliseconds(200))
    {
      return false;
    }
    last_recovery_time_ = now;

    RCLCPP_WARN(get_logger(), "Attempting I2C bus recovery via SCL pulses after %s error.", reason);
    close_i2c();
    if (!pulse_scl())
    {
      RCLCPP_WARN(get_logger(), "I2C bus recovery failed to toggle SCL.");
      return false;
    }

    if (!ensure_open())
    {
      RCLCPP_WARN(get_logger(), "I2C bus recovery toggled SCL but failed to reopen device.");
      return false;
    }
    return true;
  }

  static bool write_sysfs_value(const std::string & path, const std::string & value)
  {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0)
    {
      return false;
    }
    const ssize_t written = ::write(fd, value.c_str(), value.size());
    ::close(fd);
    return written == static_cast<ssize_t>(value.size());
  }

  bool ensure_gpio_exported(int gpio)
  {
    const std::string base = "/sys/class/gpio/gpio" + std::to_string(gpio);
    if (::access(base.c_str(), F_OK) == 0)
    {
      return true;
    }

    if (!write_sysfs_value("/sys/class/gpio/export", std::to_string(gpio)))
    {
      return false;
    }

    for (int i = 0; i < 50; ++i)
    {
      if (::access(base.c_str(), F_OK) == 0)
      {
        return true;
      }
      usleep(2000);
    }
    return false;
  }

  bool pulse_scl()
  {
    if (recovery_scl_gpio_ < 0)
    {
      return false;
    }

    const std::string base = "/sys/class/gpio/gpio" + std::to_string(recovery_scl_gpio_);
    if (!ensure_gpio_exported(recovery_scl_gpio_))
    {
      return false;
    }

    const std::string direction = base + "/direction";
    const std::string value = base + "/value";
    if (!write_sysfs_value(direction, "out"))
    {
      return false;
    }

    const int delay_us = recovery_delay_us_ > 0 ? recovery_delay_us_ : 5;
    if (!write_sysfs_value(value, "1"))
    {
      return false;
    }
    usleep(delay_us);

    const int pulses = recovery_pulses_ > 0 ? recovery_pulses_ : 9;
    for (int i = 0; i < pulses; ++i)
    {
      if (!write_sysfs_value(value, "0"))
      {
        return false;
      }
      usleep(delay_us);
      if (!write_sysfs_value(value, "1"))
      {
        return false;
      }
      usleep(delay_us);
    }

    (void)write_sysfs_value(direction, "in");
    return true;
  }

  void close_i2c()
  {
    if (i2c_fd_ >= 0)
    {
      ::close(i2c_fd_);
      i2c_fd_ = -1;
    }
  }

  std::string i2c_device_;
  std::string service_name_;
  rclcpp::Service<i2c_manager::srv::I2cTransfer>::SharedPtr service_;
  int recovery_scl_gpio_{-1};
  int recovery_pulses_{9};
  int recovery_delay_us_{5};
  std::chrono::steady_clock::time_point last_recovery_time_{};
  bool log_i2c_commands_{false};

  int i2c_fd_{-1};
  std::mutex i2c_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<I2cManagerNode>());
  rclcpp::shutdown();
  return 0;
}
