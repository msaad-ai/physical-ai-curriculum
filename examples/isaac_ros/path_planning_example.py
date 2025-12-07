#!/usr/bin/env python3
"""
Isaac ROS Path Planning Example
This script demonstrates path planning for bipedal robots in Isaac Sim
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
from scipy.spatial import distance
from typing import List, Tuple
import math


class BipedalPathPlannerNode(Node):
    """
    A node that demonstrates path planning for bipedal robots
    """
    def __init__(self):
        super().__init__('bipedal_path_planner')

        # Publisher for planned paths
        self.path_pub = self.create_publisher(
            Path,
            '/plan',
            10
        )

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/path_planning_markers',
            10
        )

        # Timer to plan paths periodically
        self.timer = self.create_timer(5.0, self.plan_new_path)

        # Example start and goal positions
        self.start_pos = (0.0, 0.0)
        self.goal_pos = (5.0, 5.0)
        self.obstacles = [(2.0, 2.0, 0.5), (3.5, 1.5, 0.4), (1.0, 4.0, 0.3)]

        self.get_logger().info('Bipedal Path Planner initialized')

    def plan_new_path(self):
        """
        Plan a new path and publish it
        """
        self.get_logger().info(f'Planning path from {self.start_pos} to {self.goal_pos}')

        # Plan path using A* algorithm with bipedal constraints
        path = self.a_star_path_planning(self.start_pos, self.goal_pos)

        if path:
            # Convert path to ROS Path message
            ros_path = self.create_ros_path(path)
            self.path_pub.publish(ros_path)

            # Publish visualization markers
            markers = self.create_path_markers(path)
            self.marker_pub.publish(markers)

            self.get_logger().info(f'Published path with {len(path)} waypoints')
        else:
            self.get_logger().warn('Failed to find a valid path')

    def a_star_path_planning(self, start: Tuple[float, float],
                           goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        A* path planning algorithm with bipedal constraints
        """
        # Define grid parameters
        grid_resolution = 0.5  # meters
        grid_size = 20  # 10x10 meters grid

        # Convert start and goal to grid coordinates
        start_grid = (int(start[0] / grid_resolution), int(start[1] / grid_resolution))
        goal_grid = (int(goal[0] / grid_resolution), int(goal[1] / grid_resolution))

        # Create grid with obstacles
        grid = self.create_obstacle_grid(grid_size, grid_resolution)

        # Run A* algorithm
        path_grid = self.a_star_search(grid, start_grid, goal_grid)

        if path_grid:
            # Convert grid path back to world coordinates
            world_path = []
            for grid_pos in path_grid:
                world_x = grid_pos[0] * grid_resolution + grid_resolution / 2
                world_y = grid_pos[1] * grid_resolution + grid_resolution / 2
                world_path.append((world_x, world_y))
            return world_path
        else:
            return []

    def create_obstacle_grid(self, grid_size: int, resolution: float) -> np.ndarray:
        """
        Create a grid with obstacles marked as 1, free space as 0
        """
        grid = np.zeros((grid_size, grid_size), dtype=int)

        # Add obstacles to grid
        for obs_x, obs_y, obs_radius in self.obstacles:
            grid_x = int(obs_x / resolution)
            grid_y = int(obs_y / resolution)
            grid_radius = int(obs_radius / resolution) + 1

            # Mark cells around obstacle as occupied
            for dx in range(-grid_radius, grid_radius + 1):
                for dy in range(-grid_radius, grid_radius + 1):
                    if dx*dx + dy*dy <= grid_radius*grid_radius:
                        check_x = grid_x + dx
                        check_y = grid_y + dy
                        if 0 <= check_x < grid_size and 0 <= check_y < grid_size:
                            grid[check_x, check_y] = 1

        return grid

    def a_star_search(self, grid: np.ndarray, start: Tuple[int, int],
                     goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        A* search algorithm implementation
        """
        import heapq

        def heuristic(pos1, pos2):
            return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

        def get_neighbors(pos):
            neighbors = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    new_x = pos[0] + dx
                    new_y = pos[1] + dy
                    if (0 <= new_x < grid.shape[0] and
                        0 <= new_y < grid.shape[1] and
                        grid[new_x, new_y] == 0):
                        neighbors.append((new_x, new_y))
            return neighbors

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor in get_neighbors(current):
                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def create_ros_path(self, waypoints: List[Tuple[float, float]]) -> Path:
        """
        Convert list of waypoints to ROS Path message
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            # Simple orientation pointing to next waypoint
            if i < len(waypoints) - 1:
                next_x, next_y = waypoints[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                # Convert yaw to quaternion
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

            path_msg.poses.append(pose)

        return path_msg

    def create_path_markers(self, waypoints: List[Tuple[float, float]]) -> MarkerArray:
        """
        Create visualization markers for the path
        """
        marker_array = MarkerArray()

        # Path line marker
        line_marker = Marker()
        line_marker.header = Header()
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.header.frame_id = 'map'
        line_marker.ns = 'path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0

        for x, y in waypoints:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.05  # Slightly above ground for visibility
            line_marker.points.append(point)

        marker_array.markers.append(line_marker)

        # Waypoint markers
        for i, (x, y) in enumerate(waypoints):
            point_marker = Marker()
            point_marker.header = Header()
            point_marker.header.stamp = line_marker.header.stamp
            point_marker.header.frame_id = 'map'
            point_marker.ns = 'waypoints'
            point_marker.id = i + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = x
            point_marker.pose.position.y = y
            point_marker.pose.position.z = 0.1
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.1
            point_marker.scale.y = 0.1
            point_marker.scale.z = 0.1
            point_marker.color.r = 1.0
            point_marker.color.g = 0.0
            point_marker.color.b = 0.0
            point_marker.color.a = 0.8

            marker_array.markers.append(point_marker)

        # Start marker
        start_marker = Marker()
        start_marker.header = Header()
        start_marker.header.stamp = line_marker.header.stamp
        start_marker.header.frame_id = 'map'
        start_marker.ns = 'start_goal'
        start_marker.id = 1000
        start_marker.type = Marker.CYLINDER
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.start_pos[0]
        start_marker.pose.position.y = self.start_pos[1]
        start_marker.pose.position.z = 0.1
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.2
        start_marker.color.g = 1.0
        start_marker.color.a = 0.8

        marker_array.markers.append(start_marker)

        # Goal marker
        goal_marker = Marker()
        goal_marker.header = Header()
        goal_marker.header.stamp = line_marker.header.stamp
        goal_marker.header.frame_id = 'map'
        goal_marker.ns = 'start_goal'
        goal_marker.id = 1001
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.goal_pos[0]
        goal_marker.pose.position.y = self.goal_pos[1]
        goal_marker.pose.position.z = 0.1
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.2
        goal_marker.color.r = 1.0
        goal_marker.color.a = 0.8

        marker_array.markers.append(goal_marker)

        # Obstacle markers
        for i, (obs_x, obs_y, obs_radius) in enumerate(self.obstacles):
            obs_marker = Marker()
            obs_marker.header = Header()
            obs_marker.header.stamp = line_marker.header.stamp
            obs_marker.header.frame_id = 'map'
            obs_marker.ns = 'obstacles'
            obs_marker.id = 2000 + i
            obs_marker.type = Marker.CYLINDER
            obs_marker.action = Marker.ADD
            obs_marker.pose.position.x = obs_x
            obs_marker.pose.position.y = obs_y
            obs_marker.pose.position.z = 0.2
            obs_marker.pose.orientation.w = 1.0
            obs_marker.scale.x = obs_radius * 2
            obs_marker.scale.y = obs_radius * 2
            obs_marker.scale.z = 0.4
            obs_marker.color.r = 0.5
            obs_marker.color.b = 0.5
            obs_marker.color.a = 0.6

            marker_array.markers.append(obs_marker)

        return marker_array


def main(args=None):
    """
    Main function to run the path planning node
    """
    rclpy.init(args=args)

    path_planner = BipedalPathPlannerNode()

    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        path_planner.get_logger().info('Shutting down Bipedal Path Planner')
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()