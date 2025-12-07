# Common Glossary: Isaac Sim, ROS, and AI Concepts

## A

- **API (Application Programming Interface)**: A set of rules and protocols for building and interacting with software applications
- **Ackermann Steering**: A steering mechanism common in cars where the inner wheel turns at a different angle than the outer wheel
- **Action**: A goal-oriented communication pattern in ROS 2 for long-running tasks with feedback
- **Apriltag**: A visual fiducial marker system used for pose estimation and localization
- **Autonomous Navigation**: The ability of a robot to navigate through an environment without human intervention

## B

- **Bounding Box**: A rectangular box that encloses an object in an image, used for object detection
- **Bipedal Robot**: A robot with two legs, designed to walk like humans
- **Buffer**: A temporary storage area used to hold data while it's being transferred between processes

## C

- **Camera Calibration**: The process of determining the internal parameters of a camera for accurate measurements
- **Cartographer**: Google's 2D and 3D simultaneous localization and mapping (SLAM) library
- **COCO Format**: Common Objects in Context, a standard format for object detection and segmentation datasets
- **Collision Mesh**: A simplified mesh used for collision detection to improve performance
- **Control Theory**: The branch of engineering that deals with the behavior of dynamical systems with inputs
- **Controller**: A component that generates commands to drive a system toward a desired state
- **Costmap**: A representation of an environment where each cell contains the cost of traversing that area

## D

- **Deep Learning**: A subset of machine learning that uses neural networks with multiple layers
- **Depth Camera**: A camera that captures depth information in addition to color information
- **Differential Drive**: A common wheel configuration for mobile robots using two independently driven wheels
- **Docker**: A platform for developing, shipping, and running applications in containers
- **Domain Randomization**: A technique for training models in simulation with randomized environments to improve real-world performance
- **Dynamic Reconfigure**: A ROS tool for changing parameters of running nodes

## E

- **Encoder**: A sensor that measures the rotation of a wheel or joint
- **Environment Map**: A representation of the environment around a robot
- **Epoch**: One complete pass through the training dataset in machine learning

## F

- **Fiducial Marker**: A visual marker with a known pattern used for pose estimation
- **Field of View (FOV)**: The extent of the observable world that is seen at any given moment
- **Filter**: An algorithm that processes sensor data to estimate the state of a system
- **Footstep Planning**: The process of planning where and when to place feet for bipedal locomotion
- **Forward Kinematics**: Calculating the position of the end effector given joint angles

## G

- **Gazebo**: A robot simulation environment used in ROS
- **Gimbal**: A pivoted support that allows an object to rotate about an axis
- **Git**: A distributed version control system
- **Global Planner**: A path planning algorithm that creates a path from start to goal in a global map
- **GPU (Graphics Processing Unit)**: A specialized processor designed to accelerate graphics and compute operations
- **Ground Truth**: Accurate information about the state of the world, often used for evaluation

## H

- **Hardware Acceleration**: Using specialized hardware to speed up computations
- **Heuristic**: A practical approach not guaranteed to be optimal but sufficient for solving a problem
- **Humanoid Robot**: A robot with human-like characteristics and movements
- **Hyperparameter**: A parameter whose value is set before the learning process begins

## I

- **IMU (Inertial Measurement Unit)**: A device that measures specific force, angular rate, and sometimes magnetic field
- **Isaac ROS**: NVIDIA's accelerated ROS 2 framework with hardware acceleration
- **Isaac Sim**: NVIDIA's robotics simulator built on the Omniverse platform
- **Inference**: The process of using a trained model to make predictions
- **Inflation Layer**: A costmap layer that adds cost to areas near obstacles
- **Instance Segmentation**: Identifying and delineating each object instance in an image

## J

- **Joint**: A connection between two or more robot links that allows relative motion
- **Joint State**: The position, velocity, and effort of a robot joint

## K

- **Kalman Filter**: An algorithm that uses a series of measurements to estimate the state of a system
- **Kinematics**: The study of motion without considering the forces that cause it
- **KITTI Format**: A standard format for autonomous driving datasets

## L

- **LIDAR (Light Detection and Ranging)**: A remote sensing method that uses light in the form of a pulsed laser
- **Label**: An annotation that assigns a class or value to data
- **Local Planner**: A path planning algorithm that creates short-term trajectories avoiding immediate obstacles
- **Localization**: Determining the position and orientation of a robot in a known map
- **Loop Closure**: A technique in SLAM to recognize when a robot returns to a previously visited location

## M

- **Machine Learning**: A type of artificial intelligence that enables computers to learn and make decisions from data
- **Mapping**: Creating a representation of the environment
- **Marker**: A visual object used for tracking or as a reference point
- **Middleware**: Software that provides common services and capabilities to applications
- **Model**: A mathematical or computational representation of a system
- **Monte Carlo**: A method using random sampling to obtain numerical results
- **Motion Planning**: Finding a sequence of movements to achieve a goal while avoiding obstacles

## N

- **Navigation2 (Nav2)**: The standard navigation framework for ROS 2
- **Neural Network**: A computational model inspired by biological neural networks
- **Node**: A process that performs computation in ROS
- **Noise Model**: A mathematical representation of random variations in sensor data

## O

- **Object Detection**: Identifying and localizing objects within sensor data
- **Omniverse**: NVIDIA's simulation and collaboration platform
- **Odometry**: Estimating position based on velocity measurements over time
- **Omnidirectional**: Capable of movement in any direction

## P

- **Parameter**: A value that influences the behavior of a system
- **Path Planning**: Finding a route from a start location to a goal location
- **Perception**: The process by which robots interpret sensory data to understand their environment
- **PID Controller**: A control loop feedback mechanism using Proportional, Integral, and Derivative terms
- **Point Cloud**: A set of data points in space, typically representing the external surface of an object
- **Pose**: The position and orientation of an object in space
- **Pure Pursuit**: A path following algorithm that steers toward a look-ahead point on the path

## Q

- **Quaternion**: A mathematical representation of rotation in 3D space
- **Q-Learning**: A reinforcement learning algorithm that learns a policy telling an agent what action to take

## R

- **ROS (Robot Operating System)**: A flexible framework for writing robot software
- **ROS 2**: The second generation of the Robot Operating System
- **ROS Bridge**: A component that connects ROS systems with other systems
- **RViz**: The 3D visualization tool for ROS
- **Range**: The maximum distance a sensor can detect objects
- **Rate**: The frequency at which a process executes, typically measured in Hz
- **Reinforcement Learning**: A type of machine learning where an agent learns to behave in an environment
- **Robotics**: The branch of technology that deals with the design, construction, operation, and application of robots

## S

- **SLAM (Simultaneous Localization and Mapping)**: The computational problem of constructing or updating a map of an unknown environment
- **Service**: A synchronous communication pattern in ROS for request-response interactions
- **Sensor**: A device that detects or measures a physical property
- **Semantic Segmentation**: Classifying each pixel in an image into a category
- **Simulation**: The imitation of the operation of a real-world process or system
- **State Estimation**: Determining the state of a system from noisy measurements
- **Stereo Vision**: Depth perception using two cameras to mimic human binocular vision
- **System**: A set of connected parts that form a complex whole

## T

- **Tensor**: A multi-dimensional array used in machine learning
- **TensorRT**: NVIDIA's inference optimizer for deep learning
- **TF (Transform)**: The system for tracking coordinate frame relationships in ROS
- **Topic**: An asynchronous communication mechanism in ROS for publishing and subscribing to data
- **Trajectory**: A time-parameterized path with velocity and acceleration information
- **Training**: The process of teaching a model using data
- **Transform**: A mathematical operation that changes the position, orientation, or scale of an object

## V

- **VSLAM (Visual-Inertial SLAM)**: SLAM using both visual and inertial measurements
- **Velocity**: The rate of change of displacement
- **Velocity Smoother**: A component that limits acceleration and deceleration commands
- **Visualization**: The representation of data in a visual format

## Z

- **ZMP (Zero Moment Point)**: A concept used in bipedal locomotion to determine balance
- **Zero-shot Learning**: A machine learning approach that recognizes objects not seen during training