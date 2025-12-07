"""
Base ROS 2 action definitions for the VLA module.

This module defines the standard actions that can be executed by the humanoid robot
in the VLA system. These actions serve as the building blocks for more complex
behavior sequences.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any, Optional
from geometry_msgs.msg import Point


class ActionType(Enum):
    """Enumeration of available action types in the VLA system."""
    MOVE_TO = "move_to"
    PICK_UP = "pick_up"
    SPEAK = "speak"
    GREET = "greet"
    MOVE_ARM = "move_arm"
    MOVE_GRIPPER = "move_gripper"
    SCAN_ENVIRONMENT = "scan_environment"
    WAIT = "wait"


@dataclass
class ROS2Action:
    """
    Base class representing a ROS 2 action in the VLA system.

    Attributes:
        action_type: The type of action to execute
        parameters: Dictionary of action-specific parameters
        target_position: 3D coordinates for movement actions
        priority: Execution priority level (0-10, where 10 is highest)
        timeout: Maximum time allowed for action completion in seconds
    """
    action_type: ActionType
    parameters: Dict[str, Any]
    target_position: Optional[Point] = None
    priority: int = 5  # Default medium priority
    timeout: float = 10.0  # Default 10 seconds timeout


class ActionFactory:
    """Factory class for creating standardized ROS 2 actions."""

    @staticmethod
    def create_move_to_action(x: float, y: float, z: float, speed: float = 1.0) -> ROS2Action:
        """
        Create a move_to action.

        Args:
            x: Target X coordinate
            y: Target Y coordinate
            z: Target Z coordinate
            speed: Movement speed (0.0-2.0)

        Returns:
            ROS2Action instance for move_to action
        """
        target = Point()
        target.x = x
        target.y = y
        target.z = z

        return ROS2Action(
            action_type=ActionType.MOVE_TO,
            parameters={
                "speed": speed,
                "relative": False
            },
            target_position=target
        )

    @staticmethod
    def create_pick_up_action(object_name: str, approach_height: float = 0.1) -> ROS2Action:
        """
        Create a pick_up action.

        Args:
            object_name: Name or identifier of the object to pick up
            approach_height: Height to approach the object from (meters)

        Returns:
            ROS2Action instance for pick_up action
        """
        return ROS2Action(
            action_type=ActionType.PICK_UP,
            parameters={
                "object_name": object_name,
                "approach_height": approach_height
            }
        )

    @staticmethod
    def create_speak_action(text: str, volume: float = 0.7) -> ROS2Action:
        """
        Create a speak action.

        Args:
            text: Text to speak
            volume: Volume level (0.0-1.0)

        Returns:
            ROS2Action instance for speak action
        """
        return ROS2Action(
            action_type=ActionType.SPEAK,
            parameters={
                "text": text,
                "volume": volume
            }
        )

    @staticmethod
    def create_greet_action(greeting_type: str = "wave") -> ROS2Action:
        """
        Create a greet action.

        Args:
            greeting_type: Type of greeting gesture ("wave", "nod", "handshake")

        Returns:
            ROS2Action instance for greet action
        """
        return ROS2Action(
            action_type=ActionType.GREET,
            parameters={
                "greeting_type": greeting_type
            }
        )

    @staticmethod
    def create_move_arm_action(joint_positions: list) -> ROS2Action:
        """
        Create a move_arm action.

        Args:
            joint_positions: List of joint positions for the robot arm

        Returns:
            ROS2Action instance for move_arm action
        """
        return ROS2Action(
            action_type=ActionType.MOVE_ARM,
            parameters={
                "joint_positions": joint_positions
            }
        )

    @staticmethod
    def create_move_gripper_action(width: float, force: float = 50.0) -> ROS2Action:
        """
        Create a move_gripper action.

        Args:
            width: Gripper opening width in meters
            force: Gripper force in Newtons

        Returns:
            ROS2Action instance for move_gripper action
        """
        return ROS2Action(
            action_type=ActionType.MOVE_GRIPPER,
            parameters={
                "width": width,
                "force": force
            }
        )

    @staticmethod
    def create_scan_environment_action(range_limit: float = 5.0) -> ROS2Action:
        """
        Create a scan_environment action.

        Args:
            range_limit: Maximum range to scan in meters

        Returns:
            ROS2Action instance for scan_environment action
        """
        return ROS2Action(
            action_type=ActionType.SCAN_ENVIRONMENT,
            parameters={
                "range_limit": range_limit
            }
        )

    @staticmethod
    def create_wait_action(duration: float) -> ROS2Action:
        """
        Create a wait action.

        Args:
            duration: Duration to wait in seconds

        Returns:
            ROS2Action instance for wait action
        """
        return ROS2Action(
            action_type=ActionType.WAIT,
            parameters={
                "duration": duration
            }
        )


# Predefined action templates for common use cases
COMMON_ACTIONS = {
    "move_forward_1m": ActionFactory.create_move_to_action(1.0, 0.0, 0.0),
    "move_backward_1m": ActionFactory.create_move_to_action(-1.0, 0.0, 0.0),
    "turn_left": ActionFactory.create_move_to_action(0.0, 0.5, 0.0),
    "turn_right": ActionFactory.create_move_to_action(0.0, -0.5, 0.0),
    "greet": ActionFactory.create_greet_action("wave"),
    "say_hello": ActionFactory.create_speak_action("Hello! How can I help you?"),
}