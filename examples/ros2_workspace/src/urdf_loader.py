#!/usr/bin/env python3
# URDF Loader Example
# This node demonstrates how to work with URDF in ROS 2
# It simulates loading a URDF model and publishing related information

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import math


class URDFLoaderNode(Node):
    """
    Example node that demonstrates working with URDF concepts
    This simulates loading a humanoid robot URDF and publishing joint states
    """

    def __init__(self):
        super().__init__('urdf_loader_node')

        # Create a publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to periodically publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

        # Initialize joint positions (simulating a humanoid robot)
        self.time = 0.0

        # Define joint names based on our simple humanoid URDF
        self.joint_names = [
            'head_joint',
            'left_arm_joint',
            'right_arm_joint',
            'left_leg_joint',
            'right_leg_joint'
        ]

        self.get_logger().info('URDF Loader Node started - simulating humanoid robot joint states')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Set joint names
        msg.name = self.joint_names

        # Simulate oscillating joint positions
        self.time += 0.1  # Increment time

        # Calculate joint positions (oscillating for demonstration)
        msg.position = [
            0.1 * math.sin(self.time),           # head joint
            0.2 * math.sin(self.time * 0.5),     # left arm
            -0.2 * math.sin(self.time * 0.5),    # right arm
            0.05 * math.sin(self.time * 2),      # left leg
            -0.05 * math.sin(self.time * 2)      # right leg
        ]

        # Velocities (derivatives of positions)
        msg.velocity = [
            0.1 * math.cos(self.time),           # head joint
            0.1 * math.cos(self.time * 0.5),     # left arm
            0.1 * math.cos(self.time * 0.5),     # right arm (in opposite direction)
            0.1 * math.cos(self.time * 2),       # left leg
            0.1 * math.cos(self.time * 2)        # right leg (in opposite direction)
        ]

        # Effort values (simulated)
        msg.effort = [0.0] * len(msg.name)

        # Publish the joint state
        self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = URDFLoaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()