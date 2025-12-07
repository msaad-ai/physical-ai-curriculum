---
title: Chapter 4 - Exercises + Mini Project
description: Practical exercises and mini project combining all learned concepts
---

# Chapter 4 - Exercises + Mini Project

## Concept Introduction

This chapter brings together all the concepts you've learned in the previous chapters to create practical exercises and a mini project. You'll apply your knowledge of nodes, topics, services, and URDF to build a simple humanoid robot controller.

The exercises in this chapter are designed to reinforce your understanding of ROS 2 fundamentals while introducing you to practical challenges you'll face when working with humanoid robots.

## Why It Matters for Humanoids

Practical application is essential for mastering ROS 2 concepts in the context of humanoid robotics:

- **Integration**: Humanoid robots require multiple systems to work together
- **Coordination**: Different components must communicate effectively
- **Real-world challenges**: Practical exercises expose you to issues you'll encounter in actual robot development
- **Problem-solving**: Building complete systems develops the skills needed for complex humanoid projects

## Implementation Breakdown

### Exercise 1: Custom ROS Node

Create a custom ROS node that extends the simple node concept with additional functionality. This node should:

1. Have a configurable parameter (like publishing frequency)
2. Include both a publisher and a subscriber
3. Implement proper logging and error handling

Here's a template to get you started:

```python
#!/usr/bin/env python3
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
```

### Exercise 2: Modify Publisher Frequency

Building on the publisher example from Chapter 2, create a version that allows the publishing frequency to be modified at runtime using parameters.

### Exercise 3: URDF Integration

Load and visualize the simple humanoid URDF model created earlier, and publish joint states for it.

## Real-World Use Cases

The skills developed in these exercises are directly applicable to:

- **Robot development**: Creating custom nodes for specific robot functions
- **Simulation**: Integrating with simulation environments
- **Testing**: Developing test nodes for robot systems
- **Prototyping**: Rapidly building and testing robot behaviors

## Student Tasks/Exercises

### Exercise 1: Create a Custom ROS Node
1. Implement the CustomNode template above
2. Launch it with different parameter values
3. Verify it works correctly with both publisher and subscriber

### Exercise 2: Modify Publisher Frequency
1. Create a new publisher node that uses parameters to control frequency
2. Change the frequency while the node is running using `ros2 param` commands
3. Observe how the publishing rate changes

### Exercise 3: URDF Integration
1. Load the simple humanoid URDF in RViz
2. Use the URDF example node to publish joint states
3. Visualize the moving robot in RViz

### Mini Project: Simple Humanoid Controller
Combine all concepts to create a simple humanoid controller:
1. Create a node that publishes joint commands to make the robot move
2. Use a service to start/stop the movement
3. Include parameter configuration for movement patterns
4. Visualize the robot in RViz as it moves

## Glossary

**Parameter**: A configuration value that can be set at runtime and modified during execution.

**Robot State Publisher**: A ROS 2 node that publishes transforms for robot links based on joint states.

**RViz**: ROS visualization tool for viewing robot models, sensor data, and other information.

**Joint State**: Message type that contains information about joint positions, velocities, and efforts.

**Transform**: Mathematical representation of the position and orientation of one coordinate frame relative to another.

**TF2**: ROS 2 transform library for managing coordinate frame relationships.

**Launch file**: File that allows multiple ROS 2 nodes to be started together with configuration.

## Summary

This chapter provided practical exercises that combine all the concepts learned in previous chapters. You've worked with nodes, topics, services, parameters, and URDF in practical applications that mirror real humanoid robotics development. These skills form the foundation for more advanced ROS 2 development in future modules.

## Review Questions

1. How do parameters allow for runtime configuration of ROS 2 nodes?
2. What role does URDF play in humanoid robotics development?
3. How do the different communication patterns (topics, services) serve different purposes in robot systems?
4. Why is visualization important in robot development?