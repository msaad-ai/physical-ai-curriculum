#!/usr/bin/env python3
# URDF Example Node
# This node demonstrates how to work with URDF in ROS 2
# It doesn't directly parse URDF but shows how URDF-related topics are used

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


class URDFExampleNode(Node):
    """
    Example node that demonstrates working with robot state information
    that would typically come from URDF-defined joints.
    This simulates publishing joint states for a simple humanoid model.
    """

    def __init__(self):
        super().__init__('urdf_example_node')

        # Create a publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to periodically publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

        # Initialize joint positions (simulating a simple humanoid)
        self.time = 0.0

        self.get_logger().info('URDF Example Node started')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Define joint names (for a simple humanoid)
        msg.name = [
            'head_joint',
            'left_arm_joint',
            'right_arm_joint',
            'left_leg_joint',
            'right_leg_joint'
        ]

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
    node = URDFExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()