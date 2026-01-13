#include "pointcloud_filter/pointcloud_filter_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <string>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace pointcloud_filter
{

PointcloudFilterNode::PointcloudFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_filter", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  cloud_in1_topic_ = this->declare_parameter("cloud_in1_topic", "/cloud_in1");
  cloud_in2_topic_ = this->declare_parameter("cloud_in2_topic", "/cloud_in2");
  output_topic_ = this->declare_parameter("output_topic", "filtered_point");
  output_cloud_topic_ = this->declare_parameter("output_pointcloud_topic", "");
  center_method_ = this->declare_parameter("center_method", "average");
  std::string method_lower = center_method_;
  std::transform(method_lower.begin(), method_lower.end(), method_lower.begin(), ::tolower);
  if (method_lower != "average" && method_lower != "midpoint") {
    RCLCPP_WARN(
      this->get_logger(),
      "Unsupported center_method '%s'. Falling back to 'average'.",
      center_method_.c_str());
    method_lower = "average";
  }
  center_method_ = method_lower;
  debug_output_ = this->declare_parameter("debug_output", false);
  debug_topic_prefix_ = this->declare_parameter("debug_topic_prefix", "debug/pointcloud_filter");
  min_height_ = this->declare_parameter("min_height", -std::numeric_limits<double>::max());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  min_range_ = this->declare_parameter("min_range", 0.0);
  max_range_ = this->declare_parameter("max_range", std::numeric_limits<double>::max());
  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.2);
  min_cluster_size_ = static_cast<std::size_t>(this->declare_parameter("min_cluster_size", 5));
  denoise_enabled_ = this->declare_parameter("denoise_enabled", true);
  noise_radius_ = this->declare_parameter("noise_radius", 0.1);
  noise_min_neighbors_ = static_cast<std::size_t>(this->declare_parameter("noise_min_neighbors", 0));
  double process_rate = this->declare_parameter("process_rate", 10.0);
  sync_timeout_ = this->declare_parameter("sync_timeout", 0.25);
  transform_timeout_ = this->declare_parameter("transform_timeout", 0.05);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto qos = rclcpp::SensorDataQoS();

  sub1_ = this->create_subscription<PointCloud2>(
    cloud_in1_topic_, qos,
    std::bind(&PointcloudFilterNode::cloudCallback1, this, std::placeholders::_1));
  sub2_ = this->create_subscription<PointCloud2>(
    cloud_in2_topic_, qos,
    std::bind(&PointcloudFilterNode::cloudCallback2, this, std::placeholders::_1));
  pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(output_topic_, 10);
  if (!output_cloud_topic_.empty()) {
    output_cloud_pub_ = this->create_publisher<PointCloud2>(output_cloud_topic_, rclcpp::SensorDataQoS());
  }

  if (debug_output_) {
    auto make_topic = [&](const std::string & suffix) {
      std::string base = debug_topic_prefix_;
      if (base.empty()) {
        base = "debug/pointcloud_filter";
      }
      while (!base.empty() && base.front() == '/') {
        base.erase(base.begin());
      }
      if (!base.empty() && base.back() == '/') {
        base.pop_back();
      }
      return base + suffix;
    };
    debug_cropped_pub_ = this->create_publisher<PointCloud2>(make_topic("/cropped"), 10);
    debug_denoised_pub_ = this->create_publisher<PointCloud2>(make_topic("/denoised"), 10);
    debug_cluster_pub_ = this->create_publisher<PointCloud2>(make_topic("/cluster"), 10);
  }

  if (process_rate <= 0.0) {
    process_rate = 10.0;
  }
  auto period = std::chrono::duration<double>(1.0 / process_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PointcloudFilterNode::processClouds, this));
}

void PointcloudFilterNode::cloudCallback1(const PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  last_cloud1_ = msg;
  last_cloud1_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  if (last_cloud1_time_.nanoseconds() == 0) {
    last_cloud1_time_ = this->now();
  }
}

void PointcloudFilterNode::cloudCallback2(const PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  last_cloud2_ = msg;
  last_cloud2_time_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  if (last_cloud2_time_.nanoseconds() == 0) {
    last_cloud2_time_ = this->now();
  }
}

bool PointcloudFilterNode::transformCloud(const PointCloud2 & input, PointCloud2 & output, rclcpp::Time & latest_stamp)
{
  if (!target_frame_.empty() && input.header.frame_id != target_frame_) {
    try {
      tf_buffer_->transform(input, output, target_frame_, tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Transform failed: %s", ex.what());
      return false;
    }
  } else {
    output = input;
  }
  rclcpp::Time stamp = rclcpp::Time(output.header.stamp);
  if (stamp > latest_stamp) {
    latest_stamp = stamp;
  }
  return true;
}

void PointcloudFilterNode::appendFilteredPoints(
  const PointCloud2 & cloud, std::vector<std::array<double, 3>> & points, rclcpp::Time & latest_stamp)
{
  if (cloud.height == 0 || cloud.width == 0) {
    return;
  }
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x"),
    iter_y(cloud, "y"), iter_z(cloud, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const double x = *iter_x;
    const double y = *iter_y;
    const double z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (z < min_height_ || z > max_height_) {
      continue;
    }
    const double range = std::hypot(x, y);
    if (range < min_range_ || range > max_range_) {
      continue;
    }
    points.push_back({x, y, z});
  }

  const rclcpp::Time stamp(cloud.header.stamp, this->get_clock()->get_clock_type());
  if (stamp > latest_stamp) {
    latest_stamp = stamp;
  }
}

std::vector<std::size_t> PointcloudFilterNode::extractLargestCluster(const std::vector<std::array<double, 3>> & points)
{
  const std::size_t n = points.size();
  std::vector<bool> processed(n, false);
  std::vector<std::size_t> best_cluster;

  for (std::size_t i = 0; i < n; ++i) {
    if (processed[i]) {
      continue;
    }
    std::vector<std::size_t> cluster;
    std::queue<std::size_t> q;
    q.push(i);
    processed[i] = true;
    while (!q.empty()) {
      auto idx = q.front();
      q.pop();
      cluster.push_back(idx);
      for (std::size_t j = 0; j < n; ++j) {
        if (processed[j]) {
          continue;
        }
        const double dx = points[idx][0] - points[j][0];
        const double dy = points[idx][1] - points[j][1];
        const double dz = points[idx][2] - points[j][2];
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist <= cluster_tolerance_) {
          processed[j] = true;
          q.push(j);
        }
      }
    }
    if (cluster.size() >= min_cluster_size_ && cluster.size() > best_cluster.size()) {
      best_cluster = cluster;
    }
  }
  return best_cluster;
}

std::vector<std::array<double, 3>> PointcloudFilterNode::removeNoise(
  const std::vector<std::array<double, 3>> & points)
{
  if (!denoise_enabled_ || points.empty() || noise_min_neighbors_ == 0 || noise_radius_ <= 0.0) {
    return points;
  }
  std::vector<std::array<double, 3>> filtered;
  filtered.reserve(points.size());
  const double radius_sq = noise_radius_ * noise_radius_;
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::size_t neighbor_count = 0;
    for (std::size_t j = 0; j < points.size(); ++j) {
      if (i == j) {
        continue;
      }
      const double dx = points[i][0] - points[j][0];
      const double dy = points[i][1] - points[j][1];
      const double dz = points[i][2] - points[j][2];
      const double dist_sq = dx * dx + dy * dy + dz * dz;
      if (dist_sq <= radius_sq) {
        ++neighbor_count;
        if (neighbor_count >= noise_min_neighbors_) {
          break;
        }
      }
    }
    if (neighbor_count >= noise_min_neighbors_) {
      filtered.push_back(points[i]);
    }
  }
  return filtered;
}

sensor_msgs::msg::PointCloud2 PointcloudFilterNode::buildPointCloud(
  const std::vector<std::array<double, 3>> & points, const rclcpp::Time & stamp) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = target_frame_.empty() ? "" : target_frame_;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = sizeof(float);
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 2 * sizeof(float);
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;
  cloud.data.resize(cloud.row_step * cloud.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto & p : points) {
    *iter_x = static_cast<float>(p[0]);
    *iter_y = static_cast<float>(p[1]);
    *iter_z = static_cast<float>(p[2]);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  return cloud;
}

void PointcloudFilterNode::publishDebugCloud(
  const rclcpp::Publisher<PointCloud2>::SharedPtr & pub,
  const std::vector<std::array<double, 3>> & points,
  const rclcpp::Time & stamp)
{
  if (!debug_output_ || !pub) {
    return;
  }
  pub->publish(buildPointCloud(points, stamp));
}

void PointcloudFilterNode::publishResult(const std::vector<std::array<double, 3>> & points, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = target_frame_.empty() ? "" : target_frame_;
  msg.header.stamp = stamp;

  auto center = computeCenterPoint(points);
  if (!center) {
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    msg.point.x = invalid;
    msg.point.y = invalid;
    msg.point.z = invalid;
  } else {
    msg.point.x = (*center)[0];
    msg.point.y = (*center)[1];
    msg.point.z = (*center)[2];
  }

  pub_->publish(msg);
  publishOutputPointCloud(center, stamp);
}

void PointcloudFilterNode::publishOutputPointCloud(
  const std::optional<std::array<double, 3>> & center, const rclcpp::Time & stamp)
{
  if (!output_cloud_pub_) {
    return;
  }
  std::vector<std::array<double, 3>> data;
  if (center) {
    data.push_back(*center);
  }
  output_cloud_pub_->publish(buildPointCloud(data, stamp));
}

std::optional<std::array<double, 3>> PointcloudFilterNode::computeCenterPoint(
  const std::vector<std::array<double, 3>> & points) const
{
  if (points.empty()) {
    return std::nullopt;
  }
  if (center_method_ == "midpoint") {
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto & p : points) {
      min_x = std::min(min_x, p[0]);
      min_y = std::min(min_y, p[1]);
      min_z = std::min(min_z, p[2]);
      max_x = std::max(max_x, p[0]);
      max_y = std::max(max_y, p[1]);
      max_z = std::max(max_z, p[2]);
    }
    return std::array<double, 3>{
      0.5 * (min_x + max_x),
      0.5 * (min_y + max_y),
      0.5 * (min_z + max_z)};
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  for (const auto & p : points) {
    sum_x += p[0];
    sum_y += p[1];
    sum_z += p[2];
  }
  const double inv = 1.0 / static_cast<double>(points.size());
  return std::array<double, 3>{sum_x * inv, sum_y * inv, sum_z * inv};
}

void PointcloudFilterNode::processClouds()
{
  PointCloud2::SharedPtr cloud1;
  PointCloud2::SharedPtr cloud2;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud1 = last_cloud1_;
    cloud2 = last_cloud2_;
  }

  if (!cloud1 && !cloud2) {
    return;
  }

  const auto now = this->now();
  auto should_wait = [&](const PointCloud2::SharedPtr & missing_cloud, const rclcpp::Time & last_time) {
    if (missing_cloud) {
      return false;
    }
    if (sync_timeout_ <= 0.0) {
      return false;
    }
    if (last_time.nanoseconds() == 0) {
      return true;
    }
    const auto elapsed = now - last_time;
    return elapsed < rclcpp::Duration::from_seconds(sync_timeout_);
  };

  if (should_wait(cloud1, last_cloud1_time_) || should_wait(cloud2, last_cloud2_time_)) {
    return;
  }

  std::vector<std::array<double, 3>> cropped_points;
  rclcpp::Time latest_stamp(0, 0, this->get_clock()->get_clock_type());
  if (cloud1) {
    PointCloud2 cloud_target;
    if (transformCloud(*cloud1, cloud_target, latest_stamp)) {
      appendFilteredPoints(cloud_target, cropped_points, latest_stamp);
    }
  }
  if (cloud2) {
    PointCloud2 cloud_target;
    if (transformCloud(*cloud2, cloud_target, latest_stamp)) {
      appendFilteredPoints(cloud_target, cropped_points, latest_stamp);
    }
  }

  auto stamp = latest_stamp == rclcpp::Time(0, 0, this->get_clock()->get_clock_type()) ? this->now() : latest_stamp;
  publishDebugCloud(debug_cropped_pub_, cropped_points, stamp);

  auto denoised_points = removeNoise(cropped_points);
  publishDebugCloud(debug_denoised_pub_, denoised_points, stamp);

  auto cluster_indices = extractLargestCluster(denoised_points);
  std::vector<std::array<double, 3>> cluster_points;
  cluster_points.reserve(cluster_indices.size());
  for (auto idx : cluster_indices) {
    cluster_points.push_back(denoised_points[idx]);
  }
  publishDebugCloud(debug_cluster_pub_, cluster_points, stamp);
  publishResult(cluster_points, stamp);
}

}  // namespace pointcloud_filter

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_filter::PointcloudFilterNode)
