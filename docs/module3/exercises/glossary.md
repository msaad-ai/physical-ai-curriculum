# Glossary: AI-Robot Brain Concepts (NVIDIA Isaac)

## A

**Action Space**: The set of possible actions that a robot can perform in a given environment, used in reinforcement learning and decision-making systems.

**Artificial Intelligence (AI)**: The simulation of human intelligence processes by machines, especially computer systems, used for perception, decision-making, and control in robotics.

**Autonomous Navigation**: The capability of a robot to move through an environment without human intervention, using sensors and algorithms to plan and execute paths.

## B

**Behavior Tree**: A hierarchical structure used to organize and execute robot behaviors, providing a flexible way to design complex autonomous behaviors.

**Bounding Box**: A rectangular box that encloses an object in an image, used in object detection to specify the location and size of detected objects.

## C

**Camera Calibration**: The process of determining the internal parameters of a camera, such as focal length and optical center, to accurately map 3D points to 2D image coordinates.

**Cognitive Robotics**: A field of robotics that focuses on endowing robots with cognitive capabilities, including perception, reasoning, and learning.

**Coordinate Frame**: A system for assigning coordinates to points in space, used in robotics to define positions and orientations relative to different reference points.

**Convolutional Neural Network (CNN)**: A class of deep neural networks commonly used in image recognition and processing, particularly effective for object detection and classification.

## D

**Data Augmentation**: The process of artificially increasing the size of a training dataset by creating modified versions of existing data, commonly used in machine learning.

**Deep Learning**: A subset of machine learning that uses neural networks with multiple layers to model complex patterns in data, particularly effective for perception tasks.

**Depth Image**: An image where each pixel value represents the distance from the camera to the object in the scene, providing 3D spatial information.

**Domain Randomization**: A technique in synthetic data generation that involves randomizing various aspects of a simulation (textures, lighting, object positions) to improve model generalization.

## E

**Embodied AI**: Artificial intelligence that is integrated with a physical body (robot), allowing it to interact with the real world through perception and action.

**End-to-End Learning**: A machine learning approach where the system learns to map directly from raw sensor inputs to control outputs without intermediate hand-crafted features.

**Environment Map**: A representation of the robot's surroundings, typically created through SLAM or other mapping techniques, used for navigation and planning.

## F

**Field of View (FoV)**: The extent of the observable world that is seen at any given moment by a camera or sensor, typically measured in degrees.

**Focal Length**: The distance over which parallel rays of light converge to a single point in a camera, affecting the magnification and field of view.

**Fused Perception**: The integration of data from multiple sensors to create a more accurate and comprehensive understanding of the environment than any single sensor could provide.

## G

**Ground Truth**: Accurate reference data used to evaluate the performance of algorithms, typically obtained through precise measurements or simulation.

**Graph-based SLAM**: A formulation of the SLAM problem as an optimization over a graph of poses and constraints, commonly used in VSLAM systems.

## H

**Humanoid Robot**: A robot with a physical body structure similar to that of a human, typically featuring a head, torso, two arms, and two legs.

## I

**Inertial Measurement Unit (IMU)**: A device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, used for navigation and stabilization.

**Isaac ROS**: NVIDIA's collection of ROS 2 packages that accelerate perception, navigation, and manipulation tasks for robotics applications using GPU acceleration.

**Isaac Sim**: NVIDIA's robotics simulation environment built on the Omniverse platform, providing high-fidelity simulation for robotics development and testing.

## K

**Kalman Filter**: A mathematical method used to estimate the state of a system from a series of noisy measurements, commonly used in sensor fusion and tracking.

## L

**LiDAR (Light Detection and Ranging)**: A remote sensing method that uses light in the form of a pulsed laser to measure distances, commonly used for 3D mapping and obstacle detection.

**Loop Closure**: A process in SLAM where the system recognizes that it has returned to a previously visited location, allowing for map correction and drift reduction.

## M

**Machine Learning**: A method of training computer systems to improve their performance on a task through experience, using data rather than explicit programming.

**Mapping**: The process of creating a representation of the environment, typically as a 2D grid or 3D point cloud, used for navigation and localization.

**Monte Carlo Localization**: A probabilistic algorithm for robot localization that uses particle filters to estimate the robot's position in a known map.

## N

**Navigation Stack (Nav2)**: The standard navigation system for ROS 2, providing path planning, obstacle avoidance, and localization capabilities.

**Neural Network**: A computing system inspired by the biological neural networks that constitute animal brains, used for pattern recognition and decision making.

## O

**Object Detection**: The computer vision task of identifying and locating objects within an image or video, typically outputting bounding boxes and class labels.

**Obstacle Avoidance**: The capability of a robot to detect and navigate around obstacles in its path while maintaining its overall navigation goal.

**Occupancy Grid**: A probabilistic representation of space where each cell in a grid contains the probability of being occupied by an obstacle.

## P

**Path Planning**: The computational problem of finding a valid sequence of configurations to move from a start state to a goal state while avoiding obstacles.

**Perception Pipeline**: A sequence of processing steps that transform raw sensor data into meaningful information about the environment, such as object locations and classifications.

**Point Cloud**: A set of data points in space, typically representing the external surface of an object, commonly generated by 3D scanners or LiDAR.

**Pose Estimation**: The determination of an object's position and orientation in 3D space, crucial for robot navigation and manipulation.

## R

**Reinforcement Learning**: A type of machine learning where an agent learns to make decisions by taking actions in an environment to maximize cumulative reward.

**RGB-D Camera**: A camera that captures both color (RGB) and depth (D) information simultaneously, providing both visual appearance and 3D structure.

**Robot Operating System (ROS)**: A flexible framework for writing robot software, providing services such as hardware abstraction, device drivers, and message passing.

**Robotics Middleware**: Software infrastructure that enables communication and coordination between different components of a robotic system.

## S

**Sensor Fusion**: The process of combining data from multiple sensors to achieve improved accuracy and reliability compared to using a single sensor.

**Simultaneous Localization and Mapping (SLAM)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

**Simulation-to-Reality Gap**: The difference in performance between models trained on simulated data versus those trained on real-world data, a key challenge in robotics.

**State Estimation**: The process of determining the state of a system (position, velocity, etc.) from noisy sensor measurements, typically using filtering techniques.

**Synthetic Data**: Artificially generated data that mimics real-world data, used for training machine learning models without requiring real-world data collection.

## T

**TF (Transforms)**: The system in ROS used to keep track of coordinate frame relationships over time, essential for multi-sensor integration and navigation.

**Trajectory Planning**: The process of determining the optimal path and timing for a robot to follow to reach a goal while satisfying various constraints.

## V

**Visual-Inertial SLAM (VSLAM)**: A SLAM approach that combines visual information from cameras with inertial measurements from IMUs to improve localization and mapping accuracy.

**Visual Odometry**: The process of incrementally estimating the position and orientation of a robot using visual information from cameras.

## W

**Waypoint Navigation**: A navigation approach where a robot moves to a series of predefined points in space, commonly used in autonomous mobile robots.

## X

**X, Y, Z Axes**: The three-dimensional coordinate system used to define positions and orientations in space, with X typically representing forward/backward, Y representing left/right, and Z representing up/down.

## Y

**Yaw**: The rotation around the vertical (Z) axis of a robot, describing the left/right turning motion.

## Z

**Zero-Shot Learning**: A machine learning paradigm where a model can recognize objects or perform tasks it has never seen before, based on prior knowledge and reasoning.