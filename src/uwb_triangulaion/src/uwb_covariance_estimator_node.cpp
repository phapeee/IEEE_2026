#include <array>
#include <cstddef>
#include <fstream>
#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace
{
std::array<double, 36> zeroCovariance()
{
  std::array<double, 36> result{};
  result.fill(0.0);
  return result;
}
}  // namespace

class UwbCovarianceEstimatorNode : public rclcpp::Node
{
public:
  explicit UwbCovarianceEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("uwb_covariance_estimator_node", options)
  {
    config_file_path_ = declare_parameter<std::string>(
      "config_file", "/ws/config/uwb_config.yaml");
    loadConfiguration();

    RCLCPP_INFO(
      get_logger(), "Sampling %zu poses from %s to update pose covariance in %s",
      sample_count_, input_topic_.c_str(), config_file_path_.c_str());

    subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&UwbCovarianceEstimatorNode::handlePose, this, std::placeholders::_1));
  }

private:
  void loadConfiguration()
  {
    config_root_ = YAML::LoadFile(config_file_path_);
    if (!config_root_) {
      throw std::runtime_error("Failed to load configuration file: " + config_file_path_);
    }
    const YAML::Node estimator = config_root_["covariance_estimator"];
    if (!estimator) {
      throw std::runtime_error("Missing 'covariance_estimator' section in configuration.");
    }
    input_topic_ = estimator["input_topic"].as<std::string>();
    if (input_topic_.empty()) {
      throw std::runtime_error("Covariance estimator input topic is empty.");
    }
    sample_count_ = estimator["sample_count"].as<std::size_t>();
    if (sample_count_ < 2) {
      throw std::runtime_error("Sample count must be at least 2.");
    }
    export_data_ = estimator["export_data"] ? estimator["export_data"].as<bool>() : false;

    if (export_data_) {
      prepareCsvFile();
    }
  }

  void handlePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (result_written_) {
      return;
    }

    x_samples_.push_back(msg->pose.pose.position.x);
    y_samples_.push_back(msg->pose.pose.position.y);
    writeSampleToCsv(*msg);
    ++sample_index_;

    const auto collected = x_samples_.size();
    if (collected < sample_count_) {
      if ((collected % 10) == 0) {
        RCLCPP_INFO(
          get_logger(), "Collected %zu/%zu samples...", collected, sample_count_);
      }
      return;
    }

    try {
      const auto covariance = computeCovariance();
      writeCovarianceToConfig(covariance);
      if (export_data_ && !csv_file_path_.empty()) {
        RCLCPP_INFO(
          get_logger(), "Exported %zu samples to %s",
          sample_index_, csv_file_path_.c_str());
        closeCsvFile();
      }
      result_written_ = true;
      RCLCPP_INFO(get_logger(), "Pose covariance updated. Shutting down node.");
      rclcpp::shutdown();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to compute/write covariance: %s", ex.what());
    }
  }

  void prepareCsvFile()
  {
    namespace fs = std::filesystem;
    const fs::path config_path(config_file_path_);
    const fs::path directory = config_path.parent_path();
    std::string stem = config_path.stem().string();
    if (stem.empty()) {
      stem = "uwb_covariance";
    }
    fs::path csv_path = directory / (stem + "_samples.csv");
    csv_file_path_ = csv_path.string();
    csv_stream_.open(csv_file_path_, std::ios::trunc);
    if (!csv_stream_.is_open()) {
      throw std::runtime_error("Unable to open CSV file: " + csv_file_path_);
    }
    csv_stream_ << "sample_index,stamp_sec,stamp_nanosec,x,y\n";
  }

  void writeSampleToCsv(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    if (!export_data_ || !csv_stream_.is_open()) {
      return;
    }

    csv_stream_
      << sample_index_ << ','
      << msg.header.stamp.sec << ','
      << msg.header.stamp.nanosec << ','
      << msg.pose.pose.position.x << ','
      << msg.pose.pose.position.y << '\n';
  }

  void closeCsvFile()
  {
    if (csv_stream_.is_open()) {
      csv_stream_.close();
    }
  }

  std::array<double, 36> computeCovariance() const
  {
    const std::size_t n = x_samples_.size();
    if (n < 2) {
      throw std::runtime_error("Not enough samples to compute covariance.");
    }

    const double mean_x = std::accumulate(x_samples_.begin(), x_samples_.end(), 0.0) /
      static_cast<double>(n);
    const double mean_y = std::accumulate(y_samples_.begin(), y_samples_.end(), 0.0) /
      static_cast<double>(n);

    double sum_xx = 0.0;
    double sum_yy = 0.0;
    double sum_xy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double dx = x_samples_[i] - mean_x;
      const double dy = y_samples_[i] - mean_y;
      sum_xx += dx * dx;
      sum_yy += dy * dy;
      sum_xy += dx * dy;
    }

    const double denom = static_cast<double>(n - 1);
    std::array<double, 36> covariance = zeroCovariance();
    covariance[0] = sum_xx / denom;
    covariance[1] = sum_xy / denom;
    covariance[6] = sum_xy / denom;
    covariance[7] = sum_yy / denom;
    return covariance;
  }

  void writeCovarianceToConfig(const std::array<double, 36> & covariance)
  {
    YAML::Node pose_covariance_node{YAML::NodeType::Sequence};
    for (const double value : covariance) {
      pose_covariance_node.push_back(value);
    }
    config_root_["pose_covariance"] = pose_covariance_node;

    YAML::Emitter emitter;
    emitter << config_root_;
    if (!emitter.good()) {
      throw std::runtime_error("Failed to serialize updated configuration.");
    }

    std::ofstream output(config_file_path_);
    if (!output.is_open()) {
      throw std::runtime_error("Unable to open config file for writing: " + config_file_path_);
    }
    output << emitter.c_str();
    output.close();
  }

  std::string config_file_path_;
  std::string input_topic_;
  std::size_t sample_count_{0};
  bool result_written_{false};
  bool export_data_{false};
  std::string csv_file_path_;
  std::ofstream csv_stream_;
  std::size_t sample_index_{0};
  YAML::Node config_root_;
  std::vector<double> x_samples_;
  std::vector<double> y_samples_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbCovarianceEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
