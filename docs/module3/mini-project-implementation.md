# Mini-Project Implementation Guide: AI-Robot Brain System

## Overview

This guide provides detailed implementation instructions for the Mini AI-Robot Brain Project that integrates perception and navigation systems. The project creates a complete autonomous system that combines VSLAM, object detection, and Nav2 navigation for humanoid robots.

## Prerequisites

Before starting the implementation, ensure you have:

- Completed all previous chapters and exercises in Module 3
- Working Isaac Sim environment with Isaac ROS packages
- Functional VSLAM and object detection pipelines
- Configured Nav2 navigation system
- Basic understanding of ROS 2 concepts and Python/C++ programming

## Project Structure

The complete project will be organized as follows:

```
isaac_robot_brain/
├── src/
│   ├── perception/
│   │   ├── vslam_node.py
│   │   ├── object_detection_node.py
│   │   └── sensor_fusion_node.py
│   ├── navigation/
│   │   ├── path_planner_node.py
│   │   ├── local_planner_node.py
│   │   └── controller_node.py
│   ├── coordination/
│   │   ├── behavior_coordinator.py
│   │   ├── state_machine.py
│   │   └── task_manager.py
│   └── utils/
│       ├── tf_utils.py
│       ├── config_loader.py
│       └── performance_monitor.py
├── launch/
│   ├── perception_launch.py
│   ├── navigation_launch.py
│   └── complete_system_launch.py
├── config/
│   ├── vslam_params.yaml
│   ├── detection_params.yaml
│   ├── nav2_params.yaml
│   └── coordination_params.yaml
├── test/
│   ├── test_perception.py
│   ├── test_navigation.py
│   └── test_integration.py
└── CMakeLists.txt
```

## Phase 1: Environment Setup and Planning

### 1.1 Workspace Setup

Create a new ROS 2 workspace for the project:

```bash
# Create workspace directory
mkdir -p ~/isaac_robot_brain_ws/src
cd ~/isaac_robot_brain_ws

# Create package for the project
cd src
ros2 pkg create --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs \
  builtin_interfaces tf2_ros tf2_geometry_msgs launch launch_ros \
  isaac_ros_visual_slam_interfaces isaac_ros_detectnet_interfaces \
  nav2_msgs --destination-directory . isaac_robot_brain

cd ~/isaac_robot_brain_ws
colcon build --packages-select isaac_robot_brain
source install/setup.bash
```

### 1.2 Configuration Files

Create the main configuration file (`config/coordination_params.yaml`):

```yaml
isaac_robot_brain:
  ros__parameters:
    # General parameters
    robot_name: "humanoid_robot"
    update_rate: 30.0

    # Perception parameters
    perception_timeout: 5.0
    min_detection_confidence: 0.7
    detection_topic: "/object_detections"
    vslam_topic: "/visual_slam/visual_odometry"

    # Navigation parameters
    navigation_timeout: 60.0
    goal_tolerance: 0.5
    navigation_topic: "/navigate_to_pose"

    # Coordination parameters
    coordination_mode: "perception_guided"  # perception_guided, navigation_guided, or hybrid
    max_retries: 3
    retry_delay: 2.0
```

### 1.3 Launch File Setup

Create the main launch file (`launch/complete_system_launch.py`):

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    isaac_package = get_package_share_directory('isaac_robot_brain')
    nav2_package = get_package_share_directory('nav2_bringup')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Include Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_package, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Perception coordinator node
    perception_coordinator = Node(
        package='isaac_robot_brain',
        executable='perception_coordinator',
        name='perception_coordinator',
        parameters=[params_file],
        remappings=[
            ('/input/detections', '/isaac_ros_detectnet/detections'),
            ('/input/odometry', '/visual_slam/visual_odometry'),
            ('/output/goal', '/navigate_to_pose')
        ],
        output='screen'
    )

    # Performance monitor node
    performance_monitor = Node(
        package='isaac_robot_brain',
        executable='performance_monitor',
        name='performance_monitor',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(isaac_package, 'config', 'coordination_params.yaml'),
            description='Full path to the ROS2 parameters file to use for the nodes'
        ),
        nav2_launch,
        perception_coordinator,
        performance_monitor
    ])
```

## Phase 2: Perception System Implementation

### 2.1 Perception Coordinator Node

Create the main perception coordinator (`src/perception/perception_coordinator.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Duration
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class PerceptionCoordinator(Node):
    def __init__(self):
        super().__init__('perception_coordinator')

        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('min_detection_confidence', 0.7)
        self.declare_parameter('goal_tolerance', 0.5)

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.min_confidence = self.get_parameter('min_detection_confidence').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Initialize perception subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.detection_sub = self.create_subscription(
            # Use appropriate Isaac ROS detection message type
            'isaac_ros_detectnet_interfaces/msg/Detections2D',
            '/isaac_ros_detectnet/detections',
            self.detection_callback,
            qos_profile
        )

        # Navigation result subscription
        self.nav_result_sub = self.create_subscription(
            'nav2_msgs/msg/NavigationResult',
            '/navigation_result',
            self.navigation_result_callback,
            10
        )

        # Timer for periodic processing
        self.timer = self.create_timer(1.0/self.update_rate, self.process_perception_data)

        # State variables
        self.current_detections = []
        self.navigation_goal_active = False
        self.navigation_start_time = None
        self.navigation_timeout = 60.0  # seconds

        self.get_logger().info('Perception Coordinator initialized')

    def detection_callback(self, msg):
        """Process incoming detection messages"""
        # Filter detections by confidence
        high_conf_detections = [
            detection for detection in msg.detections
            if detection.confidence > self.min_confidence
        ]

        self.current_detections = high_conf_detections
        self.get_logger().debug(f'Received {len(high_conf_detections)} high-confidence detections')

    def process_perception_data(self):
        """Main processing loop for perception and navigation coordination"""
        if not self.current_detections:
            return

        # Select target object (e.g., first red object)
        target_object = self.select_target_object(self.current_detections)

        if target_object and not self.navigation_goal_active:
            # Convert detection to navigation goal
            goal_pose = self.detection_to_pose(target_object)

            if goal_pose:
                self.send_navigation_goal(goal_pose)

    def select_target_object(self, detections):
        """Select the most suitable target object based on criteria"""
        for detection in detections:
            # Example: Look for objects of a specific class
            if detection.class_id == 1:  # Example: class ID for 'target'
                return detection
        return None

    def detection_to_pose(self, detection):
        """Convert detection to navigation pose"""
        try:
            # Get transform from camera frame to robot base frame
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                detection.header.frame_id,
                detection.header.stamp,
                timeout=Duration(sec=1)
            )

            # Calculate object position in robot frame
            # This is simplified - actual implementation would use camera intrinsics
            # and detection bounding box to estimate 3D position
            object_pose = PoseStamped()
            object_pose.header.frame_id = 'map'
            object_pose.header.stamp = self.get_clock().now().to_msg()

            # Placeholder calculation - implement proper 3D position estimation
            object_pose.pose.position.x = detection.bbox.center.x
            object_pose.pose.position.y = detection.bbox.center.y
            object_pose.pose.position.z = 0.0

            # Simple orientation (face the object)
            object_pose.pose.orientation.w = 1.0

            return object_pose

        except TransformException as ex:
            self.get_logger().error(f'Could not transform detection: {ex}')
            return None

    def send_navigation_goal(self, pose):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)

        self.navigation_goal_active = True
        self.navigation_start_time = time.time()

        self.get_logger().info(f'Navigation goal sent to position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        return True

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.navigation_goal_active = False
            return

        self.get_logger().info('Navigation goal accepted')

        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed with status: {result}')

        self.navigation_goal_active = False
        self.navigation_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.2 State Machine Implementation

Create a state machine for coordinating behaviors (`src/coordination/state_machine.py`):

```python
from enum import Enum
import time

class RobotState(Enum):
    IDLE = 1
    PERCEIVING = 2
    NAVIGATING = 3
    AVOIDING_OBSTACLES = 4
    REPORTING = 5
    ERROR = 6

class StateMachine:
    def __init__(self, perception_coordinator):
        self.perception_coordinator = perception_coordinator
        self.current_state = RobotState.IDLE
        self.previous_state = None
        self.state_start_time = time.time()

        # State transition conditions
        self.state_conditions = {
            RobotState.IDLE: self.check_idle_conditions,
            RobotState.PERCEIVING: self.check_perceiving_conditions,
            RobotState.NAVIGATING: self.check_navigating_conditions,
            RobotState.AVOIDING_OBSTACLES: self.check_avoiding_conditions,
            RobotState.REPORTING: self.check_reporting_conditions,
            RobotState.ERROR: self.check_error_conditions
        }

    def update_state(self):
        """Update the robot's state based on current conditions"""
        self.previous_state = self.current_state
        new_state = self.determine_next_state()

        if new_state != self.current_state:
            self.transition_to_state(new_state)

    def determine_next_state(self):
        """Determine the next state based on current conditions"""
        for state, condition_func in self.state_conditions.items():
            if condition_func():
                return state
        return self.current_state  # Stay in current state if no conditions match

    def check_idle_conditions(self):
        """Check if robot should transition from IDLE state"""
        # Transition to PERCEIVING if detections are available
        return len(self.perception_coordinator.current_detections) > 0

    def check_perceiving_conditions(self):
        """Check if robot should transition from PERCEIVING state"""
        # Transition to NAVIGATING if target is selected
        target = self.perception_coordinator.select_target_object(
            self.perception_coordinator.current_detections
        )
        return target is not None and not self.perception_coordinator.navigation_goal_active

    def check_navigating_conditions(self):
        """Check if robot should transition from NAVIGATING state"""
        # Check for timeout
        if (time.time() - self.perception_coordinator.navigation_start_time) > \
           self.perception_coordinator.navigation_timeout:
            return True  # Timeout - need to handle this

        # Check if navigation is complete
        return not self.perception_coordinator.navigation_goal_active

    def check_avoiding_conditions(self):
        """Check if robot should transition from AVOIDING_OBSTACLES state"""
        # Implementation depends on obstacle detection system
        return False  # Placeholder

    def check_reporting_conditions(self):
        """Check if robot should transition from REPORTING state"""
        # Report complete after some time
        return (time.time() - self.state_start_time) > 5.0

    def check_error_conditions(self):
        """Check if robot should transition from ERROR state"""
        # Return to IDLE after recovery attempt
        return (time.time() - self.state_start_time) > 10.0

    def transition_to_state(self, new_state):
        """Handle state transition"""
        self.get_logger().info(f'Transitioning from {self.current_state.name} to {new_state.name}')

        # Execute exit behavior for current state
        self.exit_state(self.current_state)

        # Update state
        self.current_state = new_state
        self.state_start_time = time.time()

        # Execute entry behavior for new state
        self.enter_state(new_state)

    def exit_state(self, state):
        """Execute exit behavior for a state"""
        if state == RobotState.NAVIGATING:
            # Cancel navigation if needed
            pass
        elif state == RobotState.PERCEIVING:
            # Stop perception processing
            pass

    def enter_state(self, state):
        """Execute entry behavior for a state"""
        if state == RobotState.NAVIGATING:
            # Start navigation processing
            pass
        elif state == RobotState.PERCEIVING:
            # Start perception processing
            pass

    def get_current_state(self):
        """Get the current state"""
        return self.current_state
```

## Phase 3: Navigation System Implementation

### 3.1 Enhanced Navigation Node

Create an enhanced navigation node that works with perception data (`src/navigation/navigation_controller.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import numpy as np

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('obstacle_threshold', 0.5)

        # Get parameters
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Laser scan subscription for obstacle detection
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10)
        )

        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.obstacle_detected = False
        self.navigation_active = False
        self.last_scan = None

        self.get_logger().info('Navigation Controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        if len(msg.ranges) == 0:
            return

        # Check for obstacles in front of robot
        front_ranges = msg.ranges[:len(msg.ranges)//8] + msg.ranges[-len(msg.ranges)//8:]
        front_ranges = [r for r in front_ranges if r > msg.range_min and r < msg.range_max]

        if front_ranges:
            min_distance = min(front_ranges)
            self.obstacle_detected = min_distance < self.obstacle_threshold
            self.last_scan = msg

    def execute_navigation_with_obstacle_avoidance(self, goal_pose):
        """Execute navigation with real-time obstacle avoidance"""
        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)

        self.navigation_active = True
        return True

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            self.navigation_active = False
            return

        self.get_logger().info('Navigation goal accepted')

        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation was canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info('Navigation failed')

        self.navigation_active = False

def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 4: Integration and Coordination

### 4.1 Behavior Coordinator

Create the main behavior coordinator (`src/coordination/behavior_coordinator.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import time
from .state_machine import StateMachine, RobotState

class BehaviorCoordinator(Node):
    def __init__(self):
        super().__init__('behavior_coordinator')

        # Declare parameters
        self.declare_parameter('coordination_mode', 'perception_guided')
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_delay', 2.0)

        # Get parameters
        self.coordination_mode = self.get_parameter('coordination_mode').value
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_delay = self.get_parameter('retry_delay').value

        # Publishers and subscribers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/perception_navigation/goal',
            self.goal_callback,
            QoSProfile(depth=10)
        )

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Initialize state machine
        self.state_machine = StateMachine(self)

        # Timer for state machine updates
        self.timer = self.create_timer(0.1, self.update_coordination)

        # State variables
        self.current_goal = None
        self.goal_retry_count = 0
        self.last_goal_time = 0

        self.get_logger().info('Behavior Coordinator initialized')

    def goal_callback(self, msg):
        """Handle incoming navigation goals from perception system"""
        self.current_goal = msg
        self.goal_retry_count = 0
        self.last_goal_time = time.time()

        self.get_logger().info(f'Received navigation goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def update_coordination(self):
        """Main coordination update loop"""
        # Update state machine
        self.state_machine.update_state()

        # Execute behavior based on current state
        current_state = self.state_machine.get_current_state()

        if current_state == RobotState.IDLE:
            self.execute_idle_behavior()
        elif current_state == RobotState.PERCEIVING:
            self.execute_perceiving_behavior()
        elif current_state == RobotState.NAVIGATING:
            self.execute_navigating_behavior()
        elif current_state == RobotState.REPORTING:
            self.execute_reporting_behavior()
        elif current_state == RobotState.ERROR:
            self.execute_error_behavior()

    def execute_idle_behavior(self):
        """Execute behavior when robot is idle"""
        status_msg = String()
        status_msg.data = "IDLE"
        self.status_pub.publish(status_msg)

        # Wait for perception system to provide a goal
        pass

    def execute_perceiving_behavior(self):
        """Execute behavior when robot is perceiving"""
        status_msg = String()
        status_msg.data = "PERCEIVING"
        self.status_pub.publish(status_msg)

        # Process current detections and prepare for navigation
        if self.current_goal:
            self.initiate_navigation()

    def execute_navigating_behavior(self):
        """Execute behavior when robot is navigating"""
        status_msg = String()
        status_msg.data = "NAVIGATING"
        self.status_pub.publish(status_msg)

        # Monitor navigation progress and handle obstacles
        pass

    def execute_reporting_behavior(self):
        """Execute behavior when robot is reporting"""
        status_msg = String()
        status_msg.data = "REPORTING"
        self.status_pub.publish(status_msg)

        # Report findings to central system
        pass

    def execute_error_behavior(self):
        """Execute behavior when robot encounters an error"""
        status_msg = String()
        status_msg.data = "ERROR"
        self.status_pub.publish(status_msg)

        # Attempt recovery or request assistance
        pass

    def initiate_navigation(self):
        """Initiate navigation to current goal"""
        if not self.current_goal:
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_goal

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return False

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)

        self.get_logger().info(f'Initiating navigation to goal: ({self.current_goal.pose.position.x:.2f}, {self.current_goal.pose.position.y:.2f})')
        return True

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')

            # Handle navigation failure
            self.goal_retry_count += 1
            if self.goal_retry_count < self.max_retries:
                self.get_logger().info(f'Navigation failed, retrying ({self.goal_retry_count}/{self.max_retries})')
                time.sleep(self.retry_delay)
                if self.current_goal:
                    self.initiate_navigation()
            else:
                self.get_logger().info('Navigation failed after maximum retries')
                self.current_goal = None
            return

        self.get_logger().info('Navigation goal accepted')

        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            # Report success and clear current goal
            self.current_goal = None
            self.goal_retry_count = 0
        else:
            self.get_logger().info('Navigation failed')
            # Handle failure (already handled in goal response)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 5: Testing and Validation

### 5.1 Test Scripts

Create a comprehensive test script (`test/test_integration.py`):

```python
#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class TestPerceptionNavigationIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node('integration_tester')

        # Publishers for testing
        self.goal_publisher = self.node.create_publisher(
            PoseStamped,
            '/perception_navigation/goal',
            10
        )

        # Subscribers for validation
        self.status_subscriber = self.node.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            10
        )

        self.current_status = None
        self.status_received = False

    def status_callback(self, msg):
        self.current_status = msg.data
        self.status_received = True

    def test_basic_navigation(self):
        """Test basic navigation functionality"""
        # Create a simple navigation goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 1.0
        goal.pose.position.y = 1.0
        goal.pose.orientation.w = 1.0

        # Publish the goal
        self.goal_publisher.publish(goal)

        # Wait for status update
        timeout = time.time() + 10.0  # 10 second timeout
        while not self.status_received and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Validate that robot entered navigating state
        self.assertIsNotNone(self.current_status)
        self.assertIn(self.current_status, ['NAVIGATING', 'PERCEIVING'])

    def test_perception_guided_navigation(self):
        """Test perception-guided navigation"""
        # This would involve more complex testing with simulated perception data
        # For now, just verify the basic functionality
        self.test_basic_navigation()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    unittest.main()

if __name__ == '__main__':
    main()
```

### 5.2 Performance Monitoring

Create a performance monitoring utility (`src/utils/performance_monitor.py`):

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
import time
import psutil
from collections import deque

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Declare parameters
        self.declare_parameter('monitoring_rate', 1.0)

        # Get parameters
        self.monitoring_rate = self.get_parameter('monitoring_rate').value

        # Publishers for performance metrics
        self.cpu_pub = self.create_publisher(Float32, '/performance/cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, '/performance/memory_usage', 10)
        self.processing_time_pub = self.create_publisher(Float32, '/performance/processing_time', 10)

        # Timer for monitoring
        self.timer = self.create_timer(1.0/self.monitoring_rate, self.monitor_performance)

        # Performance tracking
        self.processing_times = deque(maxlen=100)  # Keep last 100 measurements
        self.last_monitor_time = time.time()

        self.get_logger().info('Performance Monitor initialized')

    def monitor_performance(self):
        """Monitor system performance metrics"""
        # CPU usage
        cpu_percent = psutil.cpu_percent()
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_pub.publish(cpu_msg)

        # Memory usage
        memory_percent = psutil.virtual_memory().percent
        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_pub.publish(memory_msg)

        # Processing time (if available)
        if self.processing_times:
            avg_processing_time = sum(self.processing_times) / len(self.processing_times)
            time_msg = Float32()
            time_msg.data = avg_processing_time
            self.processing_time_pub.publish(time_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Phase 6: Documentation and Evaluation

### 6.1 Project Evaluation Criteria

Create evaluation criteria documentation (`docs/project_evaluation.md`):

```markdown
# Project Evaluation Criteria

## Technical Implementation (40 points)

### Perception System (10 points)
- [ ] VSLAM system properly configured and functional
- [ ] Object detection working with acceptable accuracy
- [ ] Sensor fusion implemented correctly
- [ ] Real-time performance maintained

### Navigation System (10 points)
- [ ] Nav2 properly configured for humanoid robot
- [ ] Path planning working in various scenarios
- [ ] Obstacle avoidance functional
- [ ] Navigation accuracy within tolerance

### Integration (10 points)
- [ ] Perception and navigation properly coordinated
- [ ] Data flow between components working
- [ ] Error handling implemented
- [ ] System stability maintained

### Coordination (10 points)
- [ ] State machine implemented correctly
- [ ] Behavior coordination working
- [ ] Task management functional
- [ ] Performance monitoring in place

## Performance and Robustness (30 points)

### Performance Metrics (15 points)
- [ ] System operates in real-time (30+ FPS)
- [ ] Memory usage within acceptable limits
- [ ] CPU usage optimized
- [ ] Computational resources managed efficiently

### Robustness (15 points)
- [ ] System handles edge cases gracefully
- [ ] Error recovery mechanisms functional
- [ ] Fallback behaviors implemented
- [ ] System operates for extended periods without failure

## Project Completion (20 points)

### Requirements Fulfillment (10 points)
- [ ] All functional requirements met
- [ ] Performance requirements satisfied
- [ ] Technical requirements implemented
- [ ] Project objectives achieved

### Documentation (10 points)
- [ ] Implementation guide complete
- [ ] Design decisions documented
- [ ] Code properly commented
- [ ] User manual provided

## Innovation and Complexity (10 points)

### Innovation (5 points)
- [ ] Creative solutions implemented
- [ ] Novel approaches to challenges
- [ ] Advanced features beyond basic requirements
- [ ] Efficient implementations

### Complexity (5 points)
- [ ] Handles complex scenarios
- [ ] Manages multiple objectives
- [ ] Deals with uncertainty
- [ ] Demonstrates advanced concepts
```

## Building and Running the Project

### 1. Build the Project

```bash
cd ~/isaac_robot_brain_ws
colcon build --packages-select isaac_robot_brain
source install/setup.bash
```

### 2. Launch the Complete System

```bash
# Launch Isaac Sim (in separate terminal)
isaac-sim

# Launch the complete system
ros2 launch isaac_robot_brain complete_system_launch.py use_sim_time:=true
```

### 3. Run Tests

```bash
# Run unit tests
cd ~/isaac_robot_brain_ws
source install/setup.bash
python3 test/test_integration.py
```

## Troubleshooting

### Common Issues

1. **TF Transform Issues**: Ensure all coordinate frames are properly connected
   ```bash
   ros2 run tf2_tools view_frames
   ```

2. **Performance Problems**: Monitor resource usage
   ```bash
   ros2 topic echo /performance/cpu_usage
   ```

3. **Navigation Failures**: Check costmap configuration
   ```bash
   ros2 param list | grep costmap
   ```

### Debugging Tips

- Use RViz2 to visualize perception results and navigation plans
- Monitor ROS 2 topics with `ros2 topic echo`
- Check logs with `ros2 run rclpy_logging_demo logging_demo`
- Use Isaac Sim's debugging tools for sensor and physics issues

## Conclusion

This implementation guide provides a complete framework for the Mini AI-Robot Brain Project. The system integrates perception and navigation capabilities to create an autonomous robot that can detect objects and navigate to them while avoiding obstacles. The modular design allows for easy extension and modification based on specific requirements.

Remember to validate each component individually before integrating them, and use the performance monitoring tools to ensure the system meets real-time requirements.

### Related Chapters
- [Chapter 1: NVIDIA Isaac Sim Introduction](./intro.md) - Environment setup foundation
- [Chapter 2: AI Perception Pipelines](./perception-pipelines.md) - Perception concepts implemented here
- [Chapter 3: Navigation & Path Planning](./navigation-planning.md) - Navigation concepts implemented here
- [Chapter 4: Mini AI-Robot Brain Project](./mini-project.md) - Project requirements and evaluation
- [Basic Exercises](./exercises/basic-exercises.md) - Practice problems to reinforce implementation concepts