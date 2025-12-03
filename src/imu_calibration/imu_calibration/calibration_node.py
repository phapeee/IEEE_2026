"""Node that calibrates IMU orientation offsets to publish flattened data."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class ImuCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_calibration_node")
        self.declare_parameter("input_topic", "imu_in")
        self.declare_parameter("output_topic", "imu_out")
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("calibration_samples", 100)
        self.declare_parameter("calibration_timeout_sec", 5.0)

        self._input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self._output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._queue_size = self.get_parameter("queue_size").get_parameter_value().integer_value
        self._required_samples = self.get_parameter("calibration_samples").get_parameter_value().integer_value
        self._timeout_sec = self.get_parameter("calibration_timeout_sec").get_parameter_value().double_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=self._queue_size,
        )

        self._publisher = self.create_publisher(Imu, self._output_topic, qos)
        self._subscription = self.create_subscription(Imu, self._input_topic, self._imu_callback, qos)

        self._samples_collected = 0
        self._calibrated = False
        self._roll_offset = 0.0
        self._pitch_offset = 0.0
        self._yaw_offset = 0.0
        self._calibration_start_time = self.get_clock().now()

        self.get_logger().info(
            f"Waiting for {self._required_samples} IMU samples to calibrate offsets"
        )

    def _imu_callback(self, msg: Imu) -> None:
        if not self._calibrated:
            now = self.get_clock().now()
            elapsed = (now - self._calibration_start_time).nanoseconds / 1e9
            roll, pitch, yaw = self._quaternion_to_rpy(msg.orientation)
            self._roll_offset += roll
            self._pitch_offset += pitch
            self._yaw_offset += yaw
            self._samples_collected += 1

            if self._samples_collected >= self._required_samples or elapsed >= self._timeout_sec:
                if self._samples_collected:
                    self._roll_offset /= self._samples_collected
                    self._pitch_offset /= self._samples_collected
                    self._yaw_offset /= self._samples_collected
                self._calibrated = True
                self.get_logger().info(
                    f"IMU calibration complete after {self._samples_collected} samples "
                    f"(roll={self._roll_offset:.3f}, pitch={self._pitch_offset:.3f}, yaw={self._yaw_offset:.3f})"
                )

        if not self._calibrated:
            return

        adjusted = self._apply_offsets(msg)
        self._publisher.publish(adjusted)

    def _apply_offsets(self, msg: Imu) -> Imu:
        calibrated = Imu()
        calibrated.header = msg.header
        calibrated.linear_acceleration = msg.linear_acceleration
        calibrated.angular_velocity = msg.angular_velocity
        roll, pitch, yaw = self._quaternion_to_rpy(msg.orientation)
        roll -= self._roll_offset
        pitch -= self._pitch_offset
        yaw -= self._yaw_offset
        calibrated.orientation = self._rpy_to_quaternion(roll, pitch, yaw)
        calibrated.orientation_covariance = msg.orientation_covariance
        return calibrated

    @staticmethod
    def _quaternion_to_rpy(quat: Quaternion) -> tuple[float, float, float]:
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quat = Quaternion()
        quat.w = cr * cp * cy + sr * sp * sy
        quat.x = sr * cp * cy - cr * sp * sy
        quat.y = cr * sp * cy + sr * cp * sy
        quat.z = cr * cp * sy - sr * sp * cy
        return quat


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
