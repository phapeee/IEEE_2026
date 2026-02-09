#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <uwb_msgs/msg/int_float_array_stamped.hpp>

#include "i2c_manager/srv/i2c_transfer.hpp"

namespace
{
constexpr size_t kPayloadSize = 14;
constexpr uint8_t kVersionExpected = 0x02;
constexpr uint8_t kStatusAnchorAValid = 1u << 0;
constexpr uint8_t kStatusAnchorBValid = 1u << 1;
constexpr uint8_t kStatusBothFresh = 1u << 2;
constexpr uint8_t kStatusStale = 1u << 5;

constexpr size_t kGenOffset = 2;
constexpr size_t kLastServedGenOffset = 4;
constexpr size_t kAnchorAIdOffset = 6;
constexpr size_t kAnchorADistOffset = 8;
constexpr size_t kAnchorBIdOffset = 10;
constexpr size_t kAnchorBDistOffset = 12;

uint16_t read_u16_le(const std::vector<uint8_t> & payload, size_t offset)
{
  return static_cast<uint16_t>(payload[offset]) |
    (static_cast<uint16_t>(payload[offset + 1]) << 8);
}
}  // namespace

class UwbI2cReaderNode : public rclcpp::Node
{
public:
  UwbI2cReaderNode()
  : rclcpp::Node("uwb_i2c_reader")
  {
    i2c_address_ = declare_parameter<int>("i2c_address", 0x42);
    i2c_service_ = declare_parameter<std::string>("i2c_service", "i2c_manager/transfer");
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 20.0);
    topic_name_ = declare_parameter<std::string>("topic", "uwb_data");
    frame_id_ = declare_parameter<std::string>("frame_id", "uwb");

    if (poll_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "poll_rate_hz must be > 0. Using 20 Hz.");
      poll_rate_hz_ = 20.0;
    }

    publisher_ = create_publisher<uwb_msgs::msg::IntFloatArrayStamped>(topic_name_, 10);

    client_ = create_client<i2c_manager::srv::I2cTransfer>(i2c_service_);
    if (!client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_WARN(get_logger(), "I2C manager service not available after startup wait.");
    }

    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UwbI2cReaderNode::poll_i2c, this));
  }

private:
  void poll_i2c()
  {
    if (request_in_flight_)
    {
      return;
    }
    if (!client_->service_is_ready())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "I2C manager service not available.");
      return;
    }

    auto request = std::make_shared<i2c_manager::srv::I2cTransfer::Request>();
    request->address = static_cast<uint8_t>(i2c_address_);
    request->read_length = kPayloadSize;

    request_in_flight_ = true;
    client_->async_send_request(
      request,
      [this](rclcpp::Client<i2c_manager::srv::I2cTransfer>::SharedFuture future)
      {
        request_in_flight_ = false;
        const auto response = future.get();
        if (!response->success)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "I2C transfer failed: %s",
            response->error.c_str());
          return;
        }
        if (response->read_data.size() != kPayloadSize)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "I2C read returned %zu bytes.",
            response->read_data.size());
          return;
        }
        const auto & payload = response->read_data;
        if (payload[0] != kVersionExpected)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Unexpected payload version 0x%02X", payload[0]);
          return;
        }

        const uint8_t status = payload[1];
        const uint16_t gen = read_u16_le(payload, kGenOffset);
        const uint16_t last_served_gen = read_u16_le(payload, kLastServedGenOffset);
        (void)gen;
        (void)last_served_gen;

        const bool anchor_a_valid = (status & kStatusAnchorAValid) != 0u;
        const bool anchor_b_valid = (status & kStatusAnchorBValid) != 0u;
        const bool both_fresh = (status & kStatusBothFresh) != 0u;
        const bool stale = (status & kStatusStale) != 0u;

        const uint16_t anchor_a_id = read_u16_le(payload, kAnchorAIdOffset);
        const uint16_t anchor_a_distance_mm = read_u16_le(payload, kAnchorADistOffset);
        const uint16_t anchor_b_id = read_u16_le(payload, kAnchorBIdOffset);
        const uint16_t anchor_b_distance_mm = read_u16_le(payload, kAnchorBDistOffset);

        if (stale)
        {
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 5000, "UWB data is stale (status=0x%02X).",
            status);
          return;
        }
        if (!both_fresh)
        {
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "UWB data not fresh for both anchors (status=0x%02X).", status);
          return;
        }
        if (!anchor_a_valid || !anchor_b_valid)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Both-fresh flag set but one or more anchors are invalid (status=0x%02X).",
            status);
          return;
        }

        uwb_msgs::msg::IntFloatArrayStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;

        msg.ids = {static_cast<int32_t>(anchor_a_id), static_cast<int32_t>(anchor_b_id)};
        msg.values = {
          static_cast<float>(anchor_a_distance_mm) * 0.001f,
          static_cast<float>(anchor_b_distance_mm) * 0.001f};

        publisher_->publish(msg);
      });
  }

  rclcpp::Publisher<uwb_msgs::msg::IntFloatArrayStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int i2c_address_{0x42};
  std::string i2c_service_;
  double poll_rate_hz_{20.0};
  std::string topic_name_;
  std::string frame_id_;

  rclcpp::Client<i2c_manager::srv::I2cTransfer>::SharedPtr client_;
  bool request_in_flight_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbI2cReaderNode>());
  rclcpp::shutdown();
  return 0;
}
