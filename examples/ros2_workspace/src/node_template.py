#!/usr/bin/env python3
# Basic ROS 2 Node Template

import rclpy
from rclpy.node import Node


class TemplateNode(Node):
    def __init__(self):
        super().__init__('template_node')
        self.get_logger().info('Template node started')


def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()