#pragma once

#include <mutex>
#include <vector>

#include <optional>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pointcloud_filter
{

class PointcloudFilterNode : public rclcpp::Node
{
public:
  explicit PointcloudFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  void cloudCallback1(const PointCloud2::SharedPtr msg);
  void cloudCallback2(const PointCloud2::SharedPtr msg);
  void processClouds();

  bool transformCloud(const PointCloud2 & input, PointCloud2 & output, rclcpp::Time & latest_stamp);
  void appendFilteredPoints(
    const PointCloud2 & cloud, std::vector<std::array<double, 3>> & points, rclcpp::Time & latest_stamp);
  void publishResult(const std::vector<std::array<double, 3>> & points, const rclcpp::Time & stamp);
  void publishOutputPointCloud(const std::optional<std::array<double, 3>> & center, const rclcpp::Time & stamp);

  std::vector<std::size_t> extractLargestCluster(const std::vector<std::array<double, 3>> & points);
  std::vector<std::array<double, 3>> removeNoise(const std::vector<std::array<double, 3>> & points);
  sensor_msgs::msg::PointCloud2 buildPointCloud(
    const std::vector<std::array<double, 3>> & points, const rclcpp::Time & stamp) const;
  void publishDebugCloud(
    const rclcpp::Publisher<PointCloud2>::SharedPtr & pub,
    const std::vector<std::array<double, 3>> & points,
    const rclcpp::Time & stamp);
  std::optional<std::array<double, 3>> computeCenterPoint(const std::vector<std::array<double, 3>> & points) const;

  // Parameters
  std::string target_frame_;
  std::string cloud_in1_topic_;
  std::string cloud_in2_topic_;
  std::string output_topic_;
  std::string output_cloud_topic_;
  std::string debug_topic_prefix_;
  std::string center_method_;
  double min_height_;
  double max_height_;
  double min_range_;
  double max_range_;
  double cluster_tolerance_;
  std::size_t min_cluster_size_;
  double noise_radius_;
  std::size_t noise_min_neighbors_;
  double sync_timeout_;
  bool denoise_enabled_;
  double transform_timeout_;
  bool debug_output_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub1_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub2_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr output_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<PointCloud2>::SharedPtr debug_cropped_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr debug_denoised_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr debug_cluster_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  PointCloud2::SharedPtr last_cloud1_;
  PointCloud2::SharedPtr last_cloud2_;
  rclcpp::Time last_cloud1_time_;
  rclcpp::Time last_cloud2_time_;
  std::mutex cloud_mutex_;
};

}  // namespace pointcloud_filter
