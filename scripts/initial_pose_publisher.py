#!/usr/bin/env python3

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


def _build_message(frame_id: str, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
  msg = PoseWithCovarianceStamped()
  msg.header.frame_id = frame_id
  msg.pose.pose.position.x = x
  msg.pose.pose.position.y = y
  msg.pose.pose.position.z = 0.0
  msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
  msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
  # Leave covariance zeros; caller can extend if needed later.
  return msg


def main():
  parser = argparse.ArgumentParser(description="Publish an initial pose once a subscriber is present.")
  parser.add_argument("--topic", required=True, help="Topic to publish the initial pose message on.")
  parser.add_argument("--frame-id", required=True, help="Frame id used in the pose header.")
  parser.add_argument("--x", type=float, required=True, help="Initial pose X position.")
  parser.add_argument("--y", type=float, required=True, help="Initial pose Y position.")
  parser.add_argument("--yaw", type=float, required=True, help="Initial pose yaw (radians).")
  parser.add_argument("--delay", type=float, default=0.0, help="Delay (seconds) before publishing.")
  parser.add_argument("--node-name", default="initial_pose_publisher_cli")
  args = parser.parse_args()

  rclpy.init()
  node = rclpy.create_node(args.node_name)
  publisher = node.create_publisher(PoseWithCovarianceStamped, args.topic, 10)
  msg = _build_message(args.frame_id, args.x, args.y, args.yaw)

  wait_logged = False
  published = False
  start_time = node.get_clock().now()
  delay_duration = rclpy.duration.Duration(seconds=max(args.delay, 0.0))

  def timer_callback():
    nonlocal wait_logged, published
    if published:
      return
    if (node.get_clock().now() - start_time) < delay_duration:
      return
    if publisher.get_subscription_count() == 0:
      if not wait_logged:
        node.get_logger().info(
            f"[initial_pose_publisher] Waiting for a subscriber on '{args.topic}' before publishing.")
        wait_logged = True
      return
    publisher.publish(msg)
    node.get_logger().info(
        f"[initial_pose_publisher] Published initial pose at ({args.x:.3f}, "
        f"{args.y:.3f}, {args.yaw:.3f} rad) to '{args.topic}'.")
    published = True
    rclpy.shutdown()

  node.create_timer(0.1, timer_callback)
  try:
    rclpy.spin(node)
  finally:
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == "__main__":
  main()
