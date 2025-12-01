#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace laser_scan_merger
{

class LaserScanMergerNode : public rclcpp::Node
{
public:
  LaserScanMergerNode()
  : Node("laser_scan_merger"),
    max_topics_(4)
  {
    namespace_param_ = normalize_namespace(declare_parameter<std::string>("namespace", ""));
    auto input_topics = declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>{});
    output_topic_ = declare_parameter<std::string>("output_topic", "scan");
    max_age_sec_ = declare_parameter<double>("max_age_sec", 0.5);
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");

    if (input_topics.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'input_topics' must contain between 1 and 4 topics");
      throw std::runtime_error("input_topics parameter is required");
    }
    if (input_topics.size() > max_topics_) {
      RCLCPP_FATAL(get_logger(), "Received %zu topics but maximum supported is %zu", input_topics.size(), max_topics_);
      throw std::runtime_error("Too many input topics");
    }

    std::vector<std::string> resolved_topics;
    resolved_topics.reserve(input_topics.size());
    for (const auto & topic : input_topics) {
      resolved_topics.push_back(resolve_topic(topic));
    }
    output_topic_ = resolve_topic(output_topic_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);

    latest_scans_.assign(resolved_topics.size(), nullptr);
    subscribed_topic_count_ = resolved_topics.size();
    subscriptions_.reserve(resolved_topics.size());

    for (size_t index = 0; index < resolved_topics.size(); ++index) {
      const auto & topic = resolved_topics[index];
      auto callback = [this, index](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        latest_scans_[index] = msg;
        publish_if_ready();
      };
      subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(topic, qos, callback));
      RCLCPP_INFO(get_logger(), "Subscribed to LaserScan topic %s", topic.c_str());
    }

    RCLCPP_INFO(
      get_logger(),
      "Publishing merged LaserScan on %s with namespace '%s'",
      output_topic_.c_str(),
      namespace_param_.empty() ? "/" : namespace_param_.c_str());
  }

private:
  void publish_if_ready()
  {
    const rclcpp::Time now = get_clock()->now();
    std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> valid_scans;
    valid_scans.reserve(latest_scans_.size());
    size_t missing_scans = 0;
    size_t stale_scans = 0;
    for (const auto & scan : latest_scans_) {
      if (!scan) {
        ++missing_scans;
        continue;
      }
      if (!is_fresh(scan, now)) {
        ++stale_scans;
        continue;
      }
      valid_scans.push_back(scan);
    }

    if (valid_scans.empty()) {
      if (stale_scans > 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "All %zu received scans are older than %.2f s; waiting for fresh data",
          stale_scans, max_age_sec_);
      } else {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for first LaserScan message (%zu/%zu subscriptions have not received data)",
          missing_scans, subscribed_topic_count_);
      }
      return;
    }

    sensor_msgs::msg::LaserScan merged;
    if (merge_scans(valid_scans, now, merged)) {
      publisher_->publish(merged);
    }
  }

  bool is_fresh(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan, const rclcpp::Time & now) const
  {
    if (max_age_sec_ <= 0.0) {
      return true;
    }
    const rclcpp::Time scan_time(scan->header.stamp);
    return (now - scan_time).seconds() <= max_age_sec_;
  }

  bool merge_scans(
    const std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> & scans,
    const rclcpp::Time & now,
    sensor_msgs::msg::LaserScan & out_msg)
  {
    std::vector<double> increments;
    increments.reserve(scans.size());
    for (const auto & scan : scans) {
      if (scan->angle_increment > 0.0) {
        increments.push_back(scan->angle_increment);
      }
    }
    if (increments.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "All scans have invalid angle_increment; skipping publish");
      return false;
    }
    const double global_increment = *std::min_element(increments.begin(), increments.end());

    double angle_min = std::numeric_limits<double>::infinity();
    double angle_max = -std::numeric_limits<double>::infinity();
    for (const auto & scan : scans) {
      angle_min = std::min(angle_min, static_cast<double>(scan->angle_min));
      angle_max = std::max(angle_max, static_cast<double>(scan->angle_max));
    }
    if (!std::isfinite(angle_min) || !std::isfinite(angle_max) || angle_max <= angle_min) {
      RCLCPP_WARN(get_logger(), "Invalid angle bounds; skipping publish");
      return false;
    }

    const int count = static_cast<int>(std::floor((angle_max - angle_min) / global_increment)) + 1;
    if (count <= 0) {
      RCLCPP_WARN(get_logger(), "Computed invalid beam count; skipping publish");
      return false;
    }

    std::vector<float> merged_ranges(count, std::numeric_limits<float>::infinity());
    std::vector<float> merged_intensities(count, 0.0f);

    float range_min = std::numeric_limits<float>::infinity();
    float range_max = 0.0f;
    rclcpp::Time latest_stamp(0, 0, get_clock()->get_clock_type());
    std::string frame_id = output_frame_id_;
    bool had_valid_measurement = false;

    for (const auto & scan : scans) {
      range_min = std::min(range_min, scan->range_min);
      range_max = std::max(range_max, scan->range_max);
      const rclcpp::Time stamp(scan->header.stamp);
      if (stamp > latest_stamp) {
        latest_stamp = stamp;
      }
      if (frame_id.empty()) {
        frame_id = scan->header.frame_id;
      }

      const double start_angle = scan->angle_min;
      const double increment = scan->angle_increment;
      const auto & ranges = scan->ranges;
      const auto & intensities = scan->intensities;
      for (size_t i = 0; i < ranges.size(); ++i) {
        const float value = ranges[i];
        if (!std::isfinite(value)) {
          continue;
        }
        const double angle = start_angle + increment * static_cast<double>(i);
        const int target_index = static_cast<int>(std::round((angle - angle_min) / global_increment));
        if (target_index < 0 || target_index >= count) {
          continue;
        }
        merged_ranges[target_index] = std::min(merged_ranges[target_index], value);
        had_valid_measurement = true;
        if (i < intensities.size() && std::isfinite(intensities[i])) {
          merged_intensities[target_index] = std::max(merged_intensities[target_index], intensities[i]);
        }
      }
    }

    std::vector<double> time_increments;
    std::vector<double> scan_times;
    time_increments.reserve(scans.size());
    scan_times.reserve(scans.size());
    for (const auto & scan : scans) {
      if (scan->time_increment > 0.0) {
        time_increments.push_back(scan->time_increment);
      }
      if (scan->scan_time > 0.0) {
        scan_times.push_back(scan->scan_time);
      }
    }

    const rclcpp::Time chosen_stamp = (latest_stamp.nanoseconds() == 0 ? now : latest_stamp);
    out_msg.header.stamp = time_to_msg(chosen_stamp);
    out_msg.header.frame_id = frame_id;
    out_msg.angle_min = static_cast<float>(angle_min);
    out_msg.angle_max = static_cast<float>(angle_min + global_increment * (count - 1));
    out_msg.angle_increment = static_cast<float>(global_increment);
    out_msg.time_increment = time_increments.empty() ? 0.0f : static_cast<float>(*std::min_element(time_increments.begin(), time_increments.end()));
    out_msg.scan_time = scan_times.empty() ? 0.0f : static_cast<float>(*std::max_element(scan_times.begin(), scan_times.end()));
    out_msg.range_min = std::isfinite(range_min) ? range_min : 0.0f;
    out_msg.range_max = range_max;
    out_msg.ranges = merged_ranges;
    out_msg.intensities = merged_intensities;

    if (!had_valid_measurement) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Merged scan contains no finite measurements (inputs may be out of range)");
    }

    return true;
  }

  std::string resolve_topic(const std::string & topic) const
  {
    auto trimmed = trim(topic);
    if (trimmed.empty()) {
      throw std::runtime_error("Empty topic provided in parameters");
    }
    if (trimmed.front() == '/') {
      return trimmed;
    }
    if (namespace_param_.empty()) {
      return trimmed;
    }
    if (namespace_param_.back() == '/') {
      return namespace_param_ + trimmed;
    }
    return namespace_param_ + "/" + trimmed;
  }

  static std::string normalize_namespace(const std::string & value)
  {
    auto trimmed = trim(value);
    if (trimmed.empty()) {
      return "";
    }
    std::string normalized = trimmed;
    if (normalized.front() != '/') {
      normalized.insert(normalized.begin(), '/');
    }
    while (normalized.size() > 1 && normalized.back() == '/') {
      normalized.pop_back();
    }
    return normalized;
  }

  static std::string trim(const std::string & input)
  {
    auto begin = std::find_if_not(input.begin(), input.end(), [](unsigned char c) { return std::isspace(c); });
    auto end = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    if (begin >= end) {
      return "";
    }
    return std::string(begin, end);
  }

  static builtin_interfaces::msg::Time time_to_msg(const rclcpp::Time & time)
  {
    builtin_interfaces::msg::Time msg;
    int64_t nanoseconds = time.nanoseconds();
    if (nanoseconds < 0) {
      nanoseconds = 0;
    }
    msg.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return msg;
  }

  const size_t max_topics_;
  std::string namespace_param_;
  std::string output_topic_;
  double max_age_sec_;
  std::string output_frame_id_;
  size_t subscribed_topic_count_ {0};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> latest_scans_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
};

}  // namespace laser_scan_merger

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<laser_scan_merger::LaserScanMergerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
