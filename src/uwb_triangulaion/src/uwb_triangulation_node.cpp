#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uwb_msgs/msg/int_float_array_stamped.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
constexpr double kEpsilon = 1e-6;

struct AnchorInfo
{
  double x{0.0};
  double y{0.0};
};

struct AnchorMeasurement
{
  AnchorInfo info;
  double distance{0.0};
};

std::string deriveOutputTopic(const std::string & input_topic)
{
  if (input_topic.empty()) {
    return "uwb_pose";
  }

  std::string topic = input_topic;
  while (topic.size() > 1 && topic.back() == '/') {
    topic.pop_back();
  }

  const bool is_absolute = !topic.empty() && topic[0] == '/';
  const auto slash_pos = topic.find_last_of('/');
  if (slash_pos == std::string::npos) {
    return is_absolute ? "/uwb_pose" : "uwb_pose";
  }

  if (slash_pos == 0) {
    return "/uwb_pose";
  }

  return topic.substr(0, slash_pos) + "/uwb_pose";
}

template <typename T>
T get_or(const YAML::Node & node, const T & default_value)
{
  return node ? node.as<T>() : default_value;
}

}  // namespace

class UwbTriangulationNode : public rclcpp::Node
{
public:
  explicit UwbTriangulationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("uwb_triangulaion_node", options)
  {
    config_file_path_ = declare_parameter<std::string>(
      "config_file", "/ws/config/uwb_config.yaml");
    loadConfiguration(config_file_path_);

    RCLCPP_INFO(get_logger(), "Subscribed to %s, publishing to %s", input_topic_.c_str(),
      output_topic_.c_str());

    subscription_ = create_subscription<uwb_msgs::msg::IntFloatArrayStamped>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UwbTriangulationNode::handleMessage, this, std::placeholders::_1));
    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, 10);
  }

private:
  void loadConfiguration(const std::string & config_file)
  {
    YAML::Node config = YAML::LoadFile(config_file);
    input_topic_ = config["input_topic"].as<std::string>();
    if (input_topic_.empty()) {
      throw std::runtime_error("Input topic is empty in configuration file.");
    }
    output_topic_ = deriveOutputTopic(input_topic_);

    const YAML::Node anchors = config["anchors"];
    if (!anchors || anchors.size() < 2) {
      throw std::runtime_error("Need at least two anchors defined in the configuration.");
    }

    anchors_.clear();
    for (const YAML::Node & anchor : anchors) {
      if (!anchor["id"] || !anchor["position"]) {
        throw std::runtime_error("Each anchor must have 'id' and 'position'.");
      }
      const auto id = anchor["id"].as<int32_t>();
      const YAML::Node position = anchor["position"];
      if (position.size() < 2) {
        throw std::runtime_error("Anchor position must contain x and y coordinates.");
      }
      AnchorInfo info;
      info.x = position[0].as<double>();
      info.y = position[1].as<double>();
      anchors_[id] = info;
    }

    const YAML::Node sign_constraints = config["sign_constraints"];
    sign_x_ = get_or<int>(sign_constraints["sign_x"], 0);
    sign_y_ = get_or<int>(sign_constraints["sign_y"], 0);

    const YAML::Node pose_covariance = config["pose_covariance"];
    pose_covariance_.fill(0.0);
    if (pose_covariance && pose_covariance.size() == 36) {
      for (size_t i = 0; i < 36; ++i) {
        pose_covariance_[i] = pose_covariance[i].as<double>();
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Expected 36 entries for pose_covariance, defaulting to zero covariance.");
    }
  }

  void handleMessage(const uwb_msgs::msg::IntFloatArrayStamped::SharedPtr msg)
  {
    if (msg->ids.size() != msg->values.size()) {
      RCLCPP_ERROR(
        get_logger(), "IDs and values do not have the same length (%zu vs %zu).",
        msg->ids.size(), msg->values.size());
      return;
    }

    std::vector<AnchorMeasurement> measurements;
    measurements.reserve(msg->ids.size());
    for (size_t i = 0; i < msg->ids.size(); ++i) {
      const auto id = msg->ids[i];
      const auto search = anchors_.find(id);
      if (search == anchors_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring measurement from unknown anchor %d.", id);
        continue;
      }
      AnchorMeasurement measurement;
      measurement.info = search->second;
      measurement.distance = static_cast<double>(msg->values[i]);
      measurements.push_back(measurement);
    }

    if (measurements.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No known anchors present in measurement.");
      return;
    }

    try {
      const auto [x, y] = estimatePosition(measurements);
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
      pose_msg.header = msg->header;
      pose_msg.pose.pose.position.x = x;
      pose_msg.pose.pose.position.y = y;
      pose_msg.pose.pose.position.z = 0.0;
      pose_msg.pose.pose.orientation.x = 0.0;
      pose_msg.pose.pose.orientation.y = 0.0;
      pose_msg.pose.pose.orientation.z = 0.0;
      pose_msg.pose.pose.orientation.w = 1.0;
      for (size_t i = 0; i < pose_covariance_.size(); ++i) {
        pose_msg.pose.covariance[i] = pose_covariance_[i];
      }
      publisher_->publish(pose_msg);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to estimate pose: %s", ex.what());
    }
  }

  std::pair<double, double> estimatePosition(const std::vector<AnchorMeasurement> & measurements)
  {
    if (measurements.size() == 1u) {
      throw std::runtime_error("Cannot estimate pose from a single anchor measurement.");
    }

    if (measurements.size() == 2u) {
      return solveTwoAnchorCase(measurements[0], measurements[1]);
    }

    return solveLeastSquares(measurements);
  }

  std::pair<double, double> solveTwoAnchorCase(
    const AnchorMeasurement & first, const AnchorMeasurement & second) const
  {
    const double dx = second.info.x - first.info.x;
    const double dy = second.info.y - first.info.y;
    const double d = std::hypot(dx, dy);
    if (d < kEpsilon) {
      throw std::runtime_error("Anchor positions are identical; cannot triangulate.");
    }

    const double r0 = first.distance;
    const double r1 = second.distance;
    if (d > (r0 + r1) || d < std::fabs(r0 - r1)) {
      throw std::runtime_error("Circles do not intersect; invalid distance combination.");
    }

    const double a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
    double h_sq = r0 * r0 - a * a;
    if (h_sq < 0.0) {
      h_sq = 0.0;
    }
    const double h = std::sqrt(h_sq);

    const double xm = first.info.x + a * dx / d;
    const double ym = first.info.y + a * dy / d;

    std::vector<std::pair<double, double>> candidates;
    candidates.emplace_back(
      xm + h * (dy / d),
      ym - h * (dx / d));
    candidates.emplace_back(
      xm - h * (dy / d),
      ym + h * (dx / d));

    const auto matches_signs = [&](const std::pair<double, double> & point) {
      const bool x_ok = (sign_x_ == 0) ||
        (sign_x_ > 0 ? point.first >= 0.0 : point.first <= 0.0);
      const bool y_ok = (sign_y_ == 0) ||
        (sign_y_ > 0 ? point.second >= 0.0 : point.second <= 0.0);
      return x_ok && y_ok;
    };

    for (const auto & candidate : candidates) {
      if (matches_signs(candidate)) {
        return candidate;
      }
    }

    throw std::runtime_error("No circle intersection matches the configured sign constraints.");
  }

  std::pair<double, double> solveLeastSquares(
    const std::vector<AnchorMeasurement> & measurements) const
  {
    const AnchorMeasurement & reference = measurements.front();
    double ata00 = 0.0;
    double ata01 = 0.0;
    double ata11 = 0.0;
    double atb0 = 0.0;
    double atb1 = 0.0;

    for (size_t i = 1; i < measurements.size(); ++i) {
      const auto & current = measurements[i];
      const double ax = 2.0 * (current.info.x - reference.info.x);
      const double ay = 2.0 * (current.info.y - reference.info.y);
      const double b = (current.info.x * current.info.x - reference.info.x * reference.info.x) +
        (current.info.y * current.info.y - reference.info.y * reference.info.y) +
        (reference.distance * reference.distance - current.distance * current.distance);

      ata00 += ax * ax;
      ata01 += ax * ay;
      ata11 += ay * ay;
      atb0 += ax * b;
      atb1 += ay * b;
    }

    const double det = ata00 * ata11 - ata01 * ata01;
    if (std::fabs(det) < kEpsilon) {
      throw std::runtime_error("Anchors configuration leads to a singular system.");
    }

    const double x = (atb0 * ata11 - ata01 * atb1) / det;
    const double y = (ata00 * atb1 - ata01 * atb0) / det;
    return {x, y};
  }

  std::string config_file_path_;
  std::string input_topic_;
  std::string output_topic_;
  int sign_x_{0};
  int sign_y_{0};
  std::unordered_map<int32_t, AnchorInfo> anchors_;
  std::array<double, 36> pose_covariance_{};

  rclcpp::Subscription<uwb_msgs::msg::IntFloatArrayStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbTriangulationNode>());
  rclcpp::shutdown();
  return 0;
}
