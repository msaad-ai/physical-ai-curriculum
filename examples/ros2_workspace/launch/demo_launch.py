# Demo launch file
# This launch file demonstrates how to start multiple nodes together

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch the URDF example node
        Node(
            package='demo_nodes_py',  # This would be your package name in a real scenario
            executable='urdf_example',  # This would match your node name
            name='urdf_example_node',
            output='screen'
        ),

        # Example of launching multiple nodes
        # In a real scenario, you might launch:
        # - Joint state publisher
        # - Robot state publisher
        # - Controller nodes
        # - Perception nodes
    ])