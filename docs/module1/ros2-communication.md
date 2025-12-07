---
title: Chapter 3 - Node Communication + URDF Basics
description: Understanding ROS 2 communication patterns and URDF for humanoid robots
---

# Chapter 3 - Node Communication + URDF Basics

## Concept Introduction

In this chapter, we'll explore the different communication patterns available in ROS 2 and introduce URDF (Unified Robot Description Format). Understanding these concepts is crucial for creating complex humanoid robots where multiple systems need to coordinate effectively.

ROS 2 provides several communication mechanisms:
- **Topics** for asynchronous, one-way communication
- **Services** for synchronous request/response communication
- **Actions** for long-running tasks with feedback (covered in advanced modules)

URDF is essential for humanoid robotics as it describes the robot's physical structure, including links (rigid bodies) and joints (constraints).

## Why It Matters for Humanoids

Humanoid robots have complex kinematic structures with many degrees of freedom. URDF allows us to:

- Model the robot's physical structure accurately
- Simulate the robot's movement and interactions
- Generate controllers for the robot's joints
- Visualize the robot in simulation and real-time

Communication patterns in ROS 2 are especially important for humanoid robots because:

- Multiple sensors need to share data (topics)
- Control commands need to be sent to actuators (services/actions)
- Feedback about robot state needs to be distributed (topics)
- Coordination between different control systems is essential

## Implementation Breakdown

### Topic Communication

Topics use a publish/subscribe model where publishers send messages to a named topic and subscribers receive messages from that topic. This is ideal for:

- Sensor data distribution (camera images, IMU data)
- Robot state broadcasting
- Control command distribution

Example of publisher-subscriber communication:

```python
# Publisher creates a publisher for a specific message type and topic
publisher = self.create_publisher(String, 'topic_name', 10)

# Publisher sends messages to the topic
msg = String()
msg.data = 'Hello'
publisher.publish(msg)

# Subscriber creates a subscription to receive messages
subscription = self.create_subscription(
    String,
    'topic_name',
    self.callback_function,
    10)
```

### Service Communication

Services provide a request/response model where a client sends a request and receives a response. This is ideal for:

- Configuration changes
- Planning requests
- Action triggering
- Data queries

### URDF Basics

URDF (Unified Robot Description Format) is an XML format that describes a robot's physical structure:

```xml
<!-- A simple link -->
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.2 0.3"/>
    </geometry>
  </visual>
</link>

<!-- A joint connecting two links -->
<joint name="joint_name" type="revolute">
  <parent link="base_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### URDF for Humanoid Robots

For humanoid robots, URDF typically includes:

- **Links**: Represent rigid bodies (head, torso, arms, legs)
- **Joints**: Define how links connect and move relative to each other
- **Materials**: Define visual appearance
- **Inertial properties**: Define mass, center of mass, and inertia

<!-- DIAGRAM_PROMPT: Simple humanoid robot URDF structure diagram showing links and joints -->

## Real-World Use Cases

Communication patterns in humanoid robots:

- **Topics**: Joint states, sensor data, robot pose
- **Services**: Walking gait activation, grasping control, navigation planning
- **Actions**: Walking to a location, moving arms to a position

URDF applications:

- **Simulation**: Creating accurate robot models for Gazebo
- **Visualization**: Displaying robots in RViz
- **Control**: Generating inverse kinematics solvers
- **Collision detection**: Defining robot-environment interactions

## Student Tasks/Exercises

1. Create a service client and server that exchanges a simple request/response
2. Modify the simple humanoid URDF to add an extra joint
3. Create a launch file that starts multiple nodes together
4. Use rqt_graph to visualize the communication between your nodes

## Glossary

**URDF**: Unified Robot Description Format; XML format for describing robot structure.

**Link**: A rigid body in a URDF model; represents a physical part of the robot.

**Joint**: Defines the connection between two links in URDF; specifies degrees of freedom.

**Visual**: Element in URDF that defines how a link appears visually.

**Collision**: Element in URDF that defines collision properties of a link.

**Launch file**: XML or Python file that starts multiple ROS 2 nodes together.

**rqt_graph**: Tool for visualizing the communication graph between ROS 2 nodes.

**Inverse kinematics**: Mathematical process for determining joint angles needed to achieve a desired end-effector position.

## Summary

This chapter covered ROS 2 communication patterns (topics and services) and introduced URDF for describing robot structure. You learned how these concepts are essential for humanoid robotics applications where complex coordination and accurate physical modeling are required. The next chapter will combine these concepts in practical exercises and a mini-project.