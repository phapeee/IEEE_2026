import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    """Relay Twist commands to TwistStamped with configurable topics."""

    def __init__(self):
        super().__init__("cmd_vel_relay")

        self.input_topic = self.declare_parameter("input_topic", "/cmd_vel").get_parameter_value().string_value
        self.output_topic = self.declare_parameter(
            "output_topic", "/controller_manager/mecanum_controller/reference").get_parameter_value().string_value
        self.frame_id = self.declare_parameter("frame_id", "odom").get_parameter_value().string_value

        qos = 10
        self.publisher = self.create_publisher(TwistStamped, self.output_topic, qos)
        self.subscription = self.create_subscription(Twist, self.input_topic, self._callback, qos)

        self.get_logger().info(
            f"Relaying Twist from '{self.input_topic}' to '{self.output_topic}' with frame_id '{self.frame_id}'"
        )

    def _callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
