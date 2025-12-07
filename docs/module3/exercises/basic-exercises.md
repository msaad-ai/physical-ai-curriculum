# Basic Exercises for Module 3

## Exercise 1: Setup Isaac Sim Environment

### Objective
Successfully install and configure the NVIDIA Isaac Sim environment with Isaac ROS.

### Prerequisites
- Completed Module 1 (ROS 2) and Module 2 (Digital Twin)
- Compatible system with NVIDIA GPU
- Internet access for downloading required packages

### Steps
1. Verify system compatibility with Isaac Sim requirements
   ```bash
   # Check GPU capabilities
   nvidia-smi
   # Check CUDA version
   nvcc --version
   # Verify system meets minimum requirements
   free -h  # Check available RAM
   df -h    # Check available disk space
   ```

2. Install Isaac Sim following the official documentation
   ```bash
   # Download Isaac Sim
   wget [download-url]
   # Extract and install
   tar -xf isaac_sim-*.tar.gz
   cd isaac_sim-*
   python3 -m pip install -e .
   ```

3. Configure Isaac ROS packages
   ```bash
   # Install Isaac ROS packages
   sudo apt install ros-humble-isaac-ros-*
   # Source ROS 2 and Isaac ROS
   source /opt/ros/humble/setup.bash
   source /opt/isaac/isaac_ros-dev/setup.bash
   ```

4. Run basic verification tests
   ```bash
   # Launch Isaac Sim in headless mode
   isaac-sim --no-window --headless
   # Test Isaac ROS Python API
   python3 -c "import rclpy; print('Isaac ROS accessible')"
   # Check Isaac ROS packages
   ros2 pkg list | grep isaac
   ```

5. Document any issues encountered and their solutions

### Expected Outcome
A fully functional Isaac Sim environment with Isaac ROS integration that can load basic robot models and execute simple commands.

---

## Exercise 2: Implement VSLAM Pipeline

### Objective
Set up and run a Visual-Inertial SLAM pipeline using Isaac ROS.

### Prerequisites
- Working Isaac Sim environment
- Basic understanding of ROS 2 concepts (nodes, topics, services)

### Steps
1. Launch Isaac Sim with a humanoid robot model
   ```bash
   # Launch Isaac Sim
   isaac-sim
   ```

2. Configure camera and IMU sensors
   ```bash
   # Launch robot with sensors using Isaac Sim
   # In Isaac Sim, add camera and IMU sensors to the robot model
   # Verify sensors are publishing data
   ros2 topic list | grep -E "(camera|imu)"
   ```

3. Initialize the VSLAM pipeline
   ```bash
   # Launch Isaac ROS VSLAM pipeline
   ros2 launch isaac_ros_visual_slam visual_slam.launch.py
   ```

4. Move the robot through the environment
   ```bash
   # Send movement commands to the robot
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
   ```

5. Observe the generated map and localization data
   ```bash
   # Monitor VSLAM topics
   ros2 topic echo /visual_slam/visual_odometry
   ros2 topic echo /visual_slam/tracking/feature_tracks
   ```

6. Validate the accuracy of the SLAM output
   ```bash
   # Check pose graph optimization results
   ros2 topic echo /visual_slam/pose_graph/optimized_poses
   # Visualize results in RViz2
   rviz2
   ```

### Expected Outcome
A working VSLAM system that creates a map of the environment and localizes the robot within that map in real-time.

---

## Exercise 3: Navigate Robot in Simulation

### Objective
Configure and execute basic navigation tasks using Nav2 in Isaac Sim.

### Prerequisites
- Working Isaac Sim environment
- Basic VSLAM knowledge from Exercise 2

### Steps
1. Set up Nav2 configuration for a humanoid robot
   ```bash
   # Create a launch file for Nav2 with bipedal parameters
   # Copy and modify the default Nav2 configuration
   ros2 launch nav2_bringup isaac_sim.launch.py
   ```

2. Define a navigation goal in the simulation environment
   ```bash
   # Use RViz2 to set a goal pose
   # Or send a goal programmatically
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   ```

3. Execute path planning and navigation
   ```bash
   # Monitor navigation progress
   ros2 topic echo /plan
   ros2 topic echo /cmd_vel
   ```

4. Monitor the robot's progress and obstacle avoidance
   ```bash
   # Monitor local costmap for obstacle detection
   ros2 topic echo /local_costmap/costmap
   # Monitor global costmap
   ros2 topic echo /global_costmap/costmap
   ```

5. Evaluate navigation performance metrics
   ```bash
   # Check navigation logs
   ros2 topic echo /behavior_tree_log
   # Monitor execution time and success rate
   ros2 topic echo /navigation_result
   ```

### Expected Outcome
A robot that successfully plans and executes paths to reach specified goals while avoiding obstacles in the simulation environment.

---

## Exercise 4: Complete Integration Challenge

### Objective
Integrate perception and navigation to complete a complex task using both VSLAM and Nav2.

### Prerequisites
- All previous exercises completed successfully
- Understanding of both perception and navigation systems

### Steps
1. Launch both perception and navigation systems
   ```bash
   # Terminal 1: Launch Isaac Sim with robot
   isaac-sim

   # Terminal 2: Launch VSLAM pipeline
   ros2 launch isaac_ros_visual_slam visual_slam.launch.py

   # Terminal 3: Launch Nav2 navigation
   ros2 launch nav2_bringup isaac_sim.launch.py
   ```

2. Navigate to a location while maintaining a map
   ```bash
   # Send navigation goal
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 4.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   ```

3. Monitor both perception and navigation performance
   ```bash
   # Monitor SLAM pose graph optimization
   ros2 topic echo /visual_slam/pose_graph/optimized_poses
   # Monitor navigation progress
   ros2 topic echo /behavior_tree_log
   ```

4. Analyze the interplay between perception and navigation
   - How does navigation affect SLAM performance?
   - How does SLAM accuracy impact navigation success?

### Expected Outcome
A fully integrated system that demonstrates the interaction between perception and navigation systems, with the robot successfully navigating while maintaining an accurate map of the environment.

### Next Steps
After completing these basic exercises, continue with:
- [Integrated Exercises](./integrated-exercises.md) - More complex scenarios combining multiple concepts
- [Review Questions](./review-questions.md) - Test your understanding of the concepts
- [Solution Hints](./solution-hints.md) - Get help if you're stuck on any exercise

### Related Chapters
- [Chapter 1: NVIDIA Isaac Sim Introduction](../intro.md) - Concepts covered in these exercises
- [Chapter 2: AI Perception Pipelines](../perception-pipelines.md) - For VSLAM exercise
- [Chapter 3: Navigation & Path Planning](../navigation-planning.md) - For navigation exercise