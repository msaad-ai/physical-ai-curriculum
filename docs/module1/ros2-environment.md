---
title: Chapter 2 - ROS 2 Environment + First Node
description: Setting up your ROS 2 development environment and creating your first node
---

# Chapter 2 - ROS 2 Environment + First Node

## Concept Introduction

Now that you understand the theoretical foundations of ROS 2, it's time to set up your development environment and create your first ROS 2 node. This chapter will guide you through installing ROS 2 Humble Hawksbill, setting up a workspace, and creating your first publisher and subscriber nodes.

Setting up your ROS 2 environment is a critical first step in working with humanoid robots. A properly configured environment ensures that all the different components of your robot can communicate effectively.

## Why It Matters for Humanoids

For humanoid robotics development, having a consistent and properly configured ROS 2 environment is crucial because:

- Multiple developers often work on different aspects of the robot
- Different hardware components may run on separate computers
- Consistent environments ensure that code developed on one system works on the robot
- Simulation and real robot deployment require compatible environments

## Implementation Breakdown

### Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is an LTS (Long Term Support) version that provides stability for humanoid robotics projects. Follow these steps to install it on Ubuntu 22.04:

<!-- DIAGRAM_PROMPT: ROS 2 installation workflow diagram showing the key steps from repository setup to installation completion -->

1. **Set up locale**:
   ```bash
   locale  # Check for UTF-8
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US.UTF-8
   ```

2. **Set up sources**:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Install ROS 2**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

4. **Install additional tools**:
   ```bash
   sudo apt install python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

### Setting up a ROS 2 Workspace

A workspace is a directory where you modify and build ROS 2 code. For humanoid robotics projects, you'll typically have multiple workspaces for different subsystems.

1. **Create the workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Build the workspace**:
   ```bash
   colcon build
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

### Creating Your First Publisher Node

Let's create a simple publisher node that publishes "Hello World" messages:

```python
#!/usr/bin/env python3
# Publisher node example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating Your First Subscriber Node

Now let's create a subscriber node that receives the messages:

```python
#!/usr/bin/env python3
# Subscriber node example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Real-World Use Cases

The publisher-subscriber pattern is fundamental to humanoid robotics:

- **Sensor data distribution**: Multiple nodes might need access to camera images or IMU data
- **State information**: Robot state (position, joint angles) can be published for use by controllers and planners
- **Command distribution**: Motion commands can be published for different subsystems to respond to

## Student Tasks/Exercises

1. Set up your own ROS 2 environment following the steps above
2. Create and run the publisher and subscriber nodes
3. Modify the publisher to publish a different message
4. Change the topic name and verify that the subscriber still works
5. Adjust the timer period in the publisher and observe the effect

## Glossary

**Workspace**: A directory where you develop and build ROS 2 packages; contains source, build, and install directories.

**colcon**: The build tool used in ROS 2 for building multiple packages in a workspace.

**Source**: The process of loading environment variables from a setup file to make ROS 2 commands available.

**Package**: A container for ROS 2 code that includes nodes, libraries, and other resources.

**Publisher**: A ROS 2 component that sends messages to a topic.

**Subscriber**: A ROS 2 component that receives messages from a topic.

**Topic**: The named channel over which messages are sent between nodes.

**LTS**: Long Term Support; a version of ROS 2 that receives updates and support for an extended period.

## Summary

This chapter guided you through setting up your ROS 2 development environment and creating your first publisher-subscriber pair. You learned how to install ROS 2 Humble, create a workspace, and write basic nodes that communicate via topics. These foundational skills are essential for humanoid robotics development.