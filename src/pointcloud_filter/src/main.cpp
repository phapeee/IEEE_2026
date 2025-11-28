#include "pointcloud_filter/pointcloud_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pointcloud_filter::PointcloudFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
