# Digital Twin Glossary

## Core Concepts

### Digital Twin
A virtual replica of a physical system that mirrors real-world behavior for simulation, testing, and analysis. In robotics, it allows testing robot behaviors and algorithms in a safe, controlled virtual environment.

### Simulation Environment
A virtual space for testing and validating robot behaviors, typically incorporating physics simulation, visual rendering, and sensor modeling.

### Physics Simulation
Computational modeling of real-world physics to create realistic robot-environment interactions, including gravity, collisions, and friction.

### Sensor Simulation
Virtual implementation of physical sensors (LiDAR, cameras, IMUs) that produce realistic data for testing perception algorithms.

## Robotics-Specific Terms

### PR2 Robot
A standard robot model commonly used in ROS/Gazebo tutorials. Provides enough complexity to demonstrate concepts while remaining accessible to beginners.

### URDF (Unified Robot Description Format)
XML format for representing robot models, including links, joints, and physical properties.

### SDF (Simulation Description Format)
XML format used by Gazebo to describe simulation worlds, models, and environments.

### LiDAR (Light Detection and Ranging)
Sensor that measures distances by illuminating targets with laser light and measuring the reflection.

### IMU (Inertial Measurement Unit)
Sensor that measures orientation, velocity, and gravitational forces using accelerometers and gyroscopes.

### Depth Camera
Camera that captures distance information for each pixel, providing 3D perception capabilities.

### Sensor Fusion
Combining data from multiple sensors to improve accuracy and robustness of robot perception.

## Software Tools

### Gazebo
Physics-based simulation environment that provides realistic robot simulation with accurate physics, sensor simulation, and rendering capabilities.

### Unity
3D development platform that excels at creating realistic visual environments and rendering complex scenes.

### ROS (Robot Operating System)
Flexible framework for writing robot software, providing services like hardware abstraction and message passing.

## Simulation Concepts

### Physics Engine
Software component that simulates real-world physics for realistic interactions in simulation environments.

### Real-time Simulation
Simulation that runs at the same rate as real-world time, allowing for interactive testing and control.

### Forward Kinematics
Calculating the position of robot end-effectors based on joint angles.

### Inverse Kinematics
Calculating required joint angles to achieve a desired end-effector position.

### Collision Detection
Algorithm for detecting when two objects in a simulation intersect or come into contact.

### Ray Tracing
Rendering technique for generating realistic lighting and shadows in visual simulations.