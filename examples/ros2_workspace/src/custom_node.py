#!/usr/bin/env python3
# Custom ROS Node Example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor


class CustomNode(Node):
    def __init__(self):
        super().__init__('custom_node')

        # Declare parameters with default values
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('robot_name', 'my_robot')

        # Get parameter values
        self.frequency = self.get_parameter('publish_frequency').value
        self.robot_name = self.get_parameter('robot_name').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'custom_topic', 10)

        # Create subscriber
        self.subscriber = self.create_subscription(
            String,
            'input_topic',
            self.input_callback,
            10
        )

        # Create timer based on frequency parameter
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)

        self.get_logger().info(f'Custom node initialized for {self.robot_name} at {self.frequency}Hz')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from {self.robot_name}!'
        self.publisher.publish(msg)

    def input_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CustomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()