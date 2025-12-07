#!/usr/bin/env python3
"""
Isaac ROS Obstacle Avoidance Example
This script demonstrates obstacle avoidance for bipedal robots in Isaac Sim
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import numpy as np
from typing import List, Tuple, Optional
import math


class BipedalObstacleAvoidanceNode(Node):
    """
    A node that demonstrates obstacle avoidance for bipedal robots
    """
    def __init__(self):
        super().__init__('bipedal_obstacle_avoidance')

        # Initialize TF listener for pose tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Robot state
        self.current_scan: Optional[LaserScan] = None
        self.current_odom: Optional[Odometry] = None
        self.current_pose = None
        self.target_pose = (5.0, 5.0)  # Example target

        # Obstacle avoidance parameters (bipedal-specific)
        self.safety_distance = 0.8  # meters - larger for bipedal stability
        self.max_linear_speed = 0.4  # slower for bipedal stability
        self.max_angular_speed = 1.0
        self.min_obstacle_distance = 0.5
        self.robot_radius = 0.4  # Effective robot radius

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Bipedal Obstacle Avoidance initialized')

    def scan_callback(self, msg: LaserScan):
        """
        Handle incoming laser scan data
        """
        self.current_scan = msg

    def odom_callback(self, msg: Odometry):
        """
        Handle incoming odometry data
        """
        self.current_odom = msg

    def get_current_pose(self):
        """
        Get current robot pose from TF or odometry
        """
        if self.current_odom:
            pose = self.current_odom.pose.pose
            return (pose.position.x, pose.position.y)
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time()
                )
                return (
                    transform.transform.translation.x,
                    transform.transform.translation.y
                )
            except Exception:
                return None

    def control_loop(self):
        """
        Main control loop for obstacle avoidance
        """
        if not self.current_scan:
            return

        # Get current pose
        current_pose = self.get_current_pose()
        if not current_pose:
            return

        # Calculate desired direction to target
        target_direction = self.calculate_target_direction(current_pose)

        # Check for obstacles in the path
        safe_direction = self.avoid_obstacles(target_direction)

        # Generate velocity command
        cmd_vel = self.generate_velocity_command(safe_direction, current_pose)

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

    def calculate_target_direction(self, current_pose: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculate the desired direction to the target
        """
        dx = self.target_pose[0] - current_pose[0]
        dy = self.target_pose[1] - current_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 0.1:  # Close to target
            return (0.0, 0.0)

        return (dx/distance, dy/distance)  # Normalized direction

    def avoid_obstacles(self, desired_direction: Tuple[float, float]) -> Tuple[float, float]:
        """
        Modify desired direction to avoid obstacles
        """
        if not self.current_scan:
            return desired_direction

        # Convert scan to obstacle points in robot frame
        obstacle_points = self.scan_to_obstacles(self.current_scan)

        # Check if desired direction is blocked
        if self.is_direction_blocked(desired_direction, obstacle_points):
            # Find alternative direction
            alternative_dir = self.find_alternative_direction(
                desired_direction, obstacle_points
            )
            if alternative_dir:
                return alternative_dir

        # If not blocked or no alternative found, use desired direction
        return desired_direction

    def scan_to_obstacles(self, scan: LaserScan) -> List[Tuple[float, float]]:
        """
        Convert laser scan to obstacle points in robot frame
        """
        obstacles = []

        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        for i, range_val in enumerate(scan.ranges):
            if not (float('inf') > range_val > scan.range_min):
                continue  # Invalid range

            angle = angle_min + i * angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)

            # Only consider obstacles within safety distance
            if math.sqrt(x*x + y*y) < self.safety_distance:
                obstacles.append((x, y))

        return obstacles

    def is_direction_blocked(self, direction: Tuple[float, float],
                           obstacles: List[Tuple[float, float]]) -> bool:
        """
        Check if the desired direction is blocked by obstacles
        """
        # Calculate a point ahead in the desired direction
        look_ahead = 0.8  # meters ahead
        look_point = (
            direction[0] * look_ahead,
            direction[1] * look_ahead
        )

        # Check if any obstacle is near the look point
        for obs_x, obs_y in obstacles:
            dist_to_look = math.sqrt(
                (obs_x - look_point[0])**2 + (obs_y - look_point[1])**2
            )
            # Add robot radius to ensure safe clearance
            if dist_to_look < self.robot_radius + self.min_obstacle_distance:
                return True

        return False

    def find_alternative_direction(self, desired_direction: Tuple[float, float],
                                 obstacles: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """
        Find an alternative direction to avoid obstacles
        """
        # Try different angles around the desired direction
        angles_to_try = [0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]

        for angle_offset in angles_to_try:
            # Calculate new direction with angle offset (in degrees)
            angle_rad = math.atan2(desired_direction[1], desired_direction[0])
            new_angle = angle_rad + math.radians(angle_offset)

            new_direction = (
                math.cos(new_angle),
                math.sin(new_angle)
            )

            # Check if this direction is clear of obstacles
            if not self.is_direction_blocked(new_direction, obstacles):
                # Verify this direction is safe by checking a path
                if self.is_path_clear(new_direction, obstacles):
                    return new_direction

        # If no clear direction found, try to find the safest option
        return self.find_safest_direction(desired_direction, obstacles)

    def is_path_clear(self, direction: Tuple[float, float],
                     obstacles: List[Tuple[float, float]]) -> bool:
        """
        Check if a path in the given direction is clear of obstacles
        """
        # Check multiple points along the path
        path_length = 0.8  # meters
        num_checks = 5
        step_size = path_length / num_checks

        for i in range(1, num_checks + 1):
            check_point = (
                direction[0] * step_size * i,
                direction[1] * step_size * i
            )

            for obs_x, obs_y in obstacles:
                dist = math.sqrt(
                    (obs_x - check_point[0])**2 + (obs_y - check_point[1])**2
                )
                if dist < self.robot_radius + self.min_obstacle_distance:
                    return False

        return True

    def find_safest_direction(self, desired_direction: Tuple[float, float],
                            obstacles: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """
        Find the safest direction when no clear path exists
        """
        best_direction = None
        max_clear_distance = 0

        # Sample directions in 30-degree increments
        for angle_offset in range(0, 360, 30):
            angle_rad = math.atan2(desired_direction[1], desired_direction[0])
            new_angle = angle_rad + math.radians(angle_offset)

            new_direction = (
                math.cos(new_angle),
                math.sin(new_angle)
            )

            # Find distance to nearest obstacle in this direction
            min_obstacle_dist = float('inf')
            for obs_x, obs_y in obstacles:
                # Project obstacle onto direction ray
                obs_dist = math.sqrt(obs_x**2 + obs_y**2)
                obs_angle = math.atan2(obs_y, obs_x)
                angle_diff = abs(new_angle - obs_angle)
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)

                # Consider obstacle if it's roughly in this direction
                if angle_diff < math.radians(45):  # 90-degree cone
                    if obs_dist < min_obstacle_dist:
                        min_obstacle_dist = obs_dist

            if min_obstacle_dist > max_clear_distance:
                max_clear_distance = min_obstacle_dist
                best_direction = new_direction

        return best_direction

    def generate_velocity_command(self, direction: Tuple[float, float],
                                current_pose: Tuple[float, float]) -> Twist:
        """
        Generate velocity command based on desired direction
        """
        cmd_vel = Twist()

        # Calculate distance to target
        dist_to_target = math.sqrt(
            (self.target_pose[0] - current_pose[0])**2 +
            (self.target_pose[1] - current_pose[1])**2
        )

        # If very close to target, stop
        if dist_to_target < 0.3:
            return cmd_vel  # Zero velocity

        # Calculate desired heading
        desired_yaw = math.atan2(direction[1], direction[0])

        # Get current robot orientation (simplified - in practice, get from odom or tf)
        # For this example, we'll assume the robot is aligned with the direction it should move
        current_yaw = desired_yaw  # Simplified assumption

        # Calculate angular error
        angle_error = desired_yaw - current_yaw
        # Normalize angle to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Set velocities
        if abs(angle_error) > 0.2:  # Need to turn
            cmd_vel.angular.z = max(-self.max_angular_speed,
                                   min(self.max_angular_speed, angle_error * 1.0))
            cmd_vel.linear.x = 0.0  # Don't move forward while turning significantly
        else:
            # Move forward with speed proportional to clearance
            cmd_vel.linear.x = min(self.max_linear_speed * 0.7,  # Reduced for safety
                                  max(0.1, self.max_linear_speed * 0.5))
            cmd_vel.angular.z = angle_error * 0.5  # Small correction

        # Limit velocities
        cmd_vel.linear.x = max(-self.max_linear_speed,
                              min(self.max_linear_speed, cmd_vel.linear.x))
        cmd_vel.angular.z = max(-self.max_angular_speed,
                               min(self.max_angular_speed, cmd_vel.angular.z))

        return cmd_vel


def main(args=None):
    """
    Main function to run the obstacle avoidance node
    """
    rclpy.init(args=args)

    obstacle_avoider = BipedalObstacleAvoidanceNode()

    try:
        rclpy.spin(obstacle_avoider)
    except KeyboardInterrupt:
        obstacle_avoider.get_logger().info('Shutting down Bipedal Obstacle Avoidance')
    finally:
        obstacle_avoider.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()