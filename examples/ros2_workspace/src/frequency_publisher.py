#!/usr/bin/env python3
# Publisher with adjustable frequency
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


class FrequencyPublisher(Node):
    def __init__(self):
        super().__init__('frequency_publisher')

        # Declare parameter with default value
        self.declare_parameter('publish_frequency', 1.0)

        # Get parameter value
        self.frequency = self.get_parameter('publish_frequency').value

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'frequency_topic', 10)

        # Create timer - initially with default frequency
        self.update_timer()

        # Parameter callback to update timer when frequency changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Frequency publisher started at {self.frequency}Hz')
        self.counter = 0

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            if param.name == 'publish_frequency' and param.type_ == Parameter.Type.DOUBLE:
                self.frequency = param.value
                self.update_timer()
                self.get_logger().info(f'Frequency updated to {self.frequency}Hz')
        return SetParametersResult(successful=True)

    def update_timer(self):
        """Update the timer with the current frequency"""
        # Cancel existing timer if it exists
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()

        # Create new timer with current frequency
        if self.frequency > 0:  # Only create timer if frequency is positive
            self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
        else:
            self.get_logger().warn('Frequency must be positive, timer not created')

    def timer_callback(self):
        msg = String()
        msg.data = f'Frequency message #{self.counter} at {self.frequency}Hz'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = FrequencyPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()