---
title: Chapter 1 - ROS 2 Foundations
description: Introduction to ROS 2 concepts as the nervous system of humanoid robots
---

# Chapter 1 - ROS 2 Foundations

## Concept Introduction

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's not an operating system but rather a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Think of ROS 2 as the "nervous system" of a humanoid robot. Just as the nervous system allows different parts of the human body to communicate and coordinate, ROS 2 enables different components of a robot to work together seamlessly.

ROS 2 is designed to be:
- **Distributed**: Different parts of your robot application can run on different computers
- **Modular**: Functionality is broken down into reusable packages
- **Language-agnostic**: Supports multiple programming languages (Python, C++, etc.)
- **Middleware-based**: Uses DDS (Data Distribution Service) for communication

## Why It Matters for Humanoids

Humanoid robots are particularly complex because they have many degrees of freedom and need to coordinate multiple systems simultaneously. A humanoid robot might have:
- Multiple sensors (cameras, IMUs, force/torque sensors)
- Actuators for each joint
- Complex control systems for balance and movement
- Perception systems for understanding the environment
- Planning systems for decision making

ROS 2 provides the infrastructure that allows all these systems to communicate effectively. The distributed nature of ROS 2 means that different computers can handle different aspects of the robot's functionality, which is essential for humanoid robots that have high computational requirements.

## Implementation Breakdown

ROS 2 is built around several core concepts:

### Nodes
Nodes are processes that perform computation. In a humanoid robot, you might have nodes for:
- Joint controllers
- Perception systems
- Path planners
- Behavior managers

Nodes are the fundamental building blocks of a ROS-based system. They allow you to break down complex robot behavior into manageable, modular components.

### Topics and Messages
Topics allow nodes to communicate with each other through a publish/subscribe model. A node publishes data to a topic, and other nodes subscribe to that topic to receive the data.

For example, a camera node might publish images to a topic called `/camera/image_raw`, and a perception node might subscribe to that topic to process the images.

Messages are the data structures that are passed between nodes. Each topic has a specific message type that defines the structure of the data.

### Services
Services provide a request/reply communication pattern. A node can make a request to a service and receive a response.

For example, you might have a service that plans a path from a start location to a goal location. A navigation node could request a path from this service and receive the planned path as a response.

### Parameters
Parameters are configuration values that can be set at runtime. They allow you to configure your nodes without recompiling.

For example, you might have a parameter that controls the speed of a humanoid robot's walking gait.

### Actions
Actions are used for long-running tasks that have feedback. They combine the features of topics and services.

For example, moving a humanoid robot's arm to a specific position might be implemented as an action, with feedback about the progress of the movement.

## Real-World Use Cases

ROS 2 is used in many humanoid robotics applications:

- **Boston Dynamics robots** use ROS-like architectures for coordination
- **NAO and Pepper robots** have been used with ROS for research
- **NASA's Robonaut** has used ROS for human-robot interaction
- **Research humanoid platforms** like ATLAS, SCHAFT, and WALK-MAN use ROS for development

ROS 2's architecture is particularly well-suited for humanoid robots because it allows for:
- Real-time performance requirements
- Distributed computing across multiple processors
- Integration of diverse sensors and actuators
- Safe and reliable communication between components

## Student Tasks/Exercises

1. Research and list 3 other humanoid robots that use ROS or ROS 2
2. Explain in your own words the difference between a topic and a service
3. Draw a simple diagram showing how 3 nodes might communicate in a humanoid robot system

## Glossary

**Node**: A process that performs computation in ROS 2; the fundamental building block of a ROS-based system.

**Topic**: A named bus over which nodes exchange messages; uses publish/subscribe communication model.

**Message**: The data structure passed between nodes in ROS 2; defines the format of data exchanged.

**Service**: A communication pattern in ROS 2 that provides request/reply interaction between nodes.

**Parameter**: A configuration value that can be set at runtime in ROS 2 nodes.

**Action**: A communication pattern for long-running tasks that provides feedback and goal management.

**DDS**: Data Distribution Service; the middleware that ROS 2 uses for communication.

**Package**: A modular unit of organization in ROS 2 that contains libraries, nodes, and other resources.

## Summary

This chapter introduced the fundamental concepts of ROS 2 and positioned them as the "nervous system" of humanoid robots. We covered the core concepts of nodes, topics, services, parameters, and actions, and explained why these are important for humanoid robotics applications. The next chapter will dive into setting up your ROS 2 environment and creating your first node.