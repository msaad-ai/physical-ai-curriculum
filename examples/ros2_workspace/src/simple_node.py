#!/usr/bin/env python3
# Simple ROS 2 Node Example
# This is a basic node that demonstrates the fundamental structure of a ROS 2 node

import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    """
    A simple ROS 2 node that demonstrates the basic structure.
    This node doesn't do much, but it shows the essential components
    of a ROS 2 node that you'll find in all humanoid robot applications.
    """

    def __init__(self):
        # Initialize the node with the name 'simple_node'
        super().__init__('simple_node')

        # Log a message to indicate the node has started
        self.get_logger().info('Simple node has started')

        # This node could be extended with publishers, subscribers,
        # services, or other ROS 2 functionality as needed for
        # humanoid robot applications


def main(args=None):
    """
    Main function that initializes ROS 2, creates the node,
    runs it, and handles cleanup.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the SimpleNode
    simple_node = SimpleNode()

    try:
        # Keep the node running until interrupted
        rclpy.spin(simple_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up resources
        simple_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()