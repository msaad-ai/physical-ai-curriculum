#!/usr/bin/env python3
"""
LLM to ROS 2 Integration Example

This script demonstrates how to integrate with an LLM (using Anthropic Claude)
to translate natural language commands to ROS 2 actions for robot execution.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import anthropic
import os
import json
import time
from dotenv import load_dotenv


class LLMToROS2Translator(Node):
    def __init__(self):
        super().__init__('llm_to_ros2_translator')

        # Load environment variables
        load_dotenv()
        self.client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_status_publisher = self.create_publisher(String, '/action_status', 10)

        # Subscribers
        self.voice_command_subscriber = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        # Robot state
        self.robot_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.robot_orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self.available_objects = [
            {"id": "red_ball", "name": "red ball", "position": {"x": 2.0, "y": 1.0}},
            {"id": "blue_cube", "name": "blue cube", "position": {"x": -1.0, "y": 3.0}},
        ]
        self.known_locations = {
            "kitchen": {"x": 5.0, "y": 3.0},
            "bedroom": {"x": -3.0, "y": -2.0},
            "living_room": {"x": 1.0, "y": -1.0}
        }

        self.get_logger().info("LLM to ROS 2 Translator initialized")

    def voice_command_callback(self, msg):
        """Process voice command from voice processing module"""
        command = msg.data
        self.get_logger().info(f"Received voice command: {command}")

        # Plan action sequence using LLM
        plan_result = self.plan_action_sequence(command)

        if 'action_sequence' in plan_result:
            self.get_logger().info(f"Generated action sequence: {plan_result['action_sequence']}")
            self.execute_action_sequence(plan_result['action_sequence'])
        else:
            self.get_logger().error(f"Failed to generate action sequence: {plan_result.get('error', 'Unknown error')}")

    def plan_action_sequence(self, natural_language_command):
        """
        Plan an action sequence from a natural language command using Claude
        """
        # Prepare the prompt for Claude
        prompt = self._create_planning_prompt(natural_language_command)

        try:
            self.get_logger().info("Sending request to Claude for cognitive planning...")

            response = self.client.messages.create(
                model="claude-3-sonnet-20240229",
                max_tokens=1000,
                temperature=0.1,
                messages=[
                    {
                        "role": "user",
                        "content": prompt
                    }
                ]
            )

            # Parse the response
            action_sequence = self._parse_claude_response(response.content)

            self.get_logger().info(f"Action sequence planned: {action_sequence}")

            return {
                'original_command': natural_language_command,
                'action_sequence': action_sequence,
                'confidence': self._calculate_confidence(action_sequence),
                'reasoning': str(response.content)
            }

        except Exception as e:
            self.get_logger().error(f"Error in cognitive planning: {e}")
            return {
                'error': f'Planning failed: {str(e)}',
                'original_command': natural_language_command
            }

    def _create_planning_prompt(self, command):
        """
        Create a prompt for Claude to generate an action plan
        """
        return f"""
        You are a cognitive planning system for a humanoid robot. Your task is to translate natural language commands into sequences of executable robot actions.

        Current environment context:
        - Robot capabilities: ["move_to", "pick_up", "put_down", "greet", "speak", "turn", "stop"]
        - Robot current position: {self.robot_position}
        - Available objects: {self.available_objects}
        - Known locations: {list(self.known_locations.keys())}

        Natural language command: "{command}"

        Please provide a detailed action plan as a JSON array of action objects. Each action should have:
        - action_type: The type of action (move_to, pick_up, put_down, greet, speak, turn, stop)
        - parameters: Any required parameters for the action
        - estimated_duration: Estimated time in seconds

        Example format:
        [
          {{
            "action_type": "move_to",
            "parameters": {{"target_position": {{"x": 5, "y": 3, "z": 0}}}},
            "estimated_duration": 3.5
          }},
          {{
            "action_type": "pick_up",
            "parameters": {{"object_id": "red_ball"}},
            "estimated_duration": 2.0
          }}
        ]

        Return only the JSON array, nothing else.
        """

    def _parse_claude_response(self, response_content):
        """
        Parse Claude's response to extract the action sequence
        """
        response_str = str(response_content)

        # Find JSON array in response
        start_idx = response_str.find('[')
        end_idx = response_str.rfind(']') + 1

        if start_idx != -1 and end_idx != 0:
            json_str = response_str[start_idx:end_idx]
            try:
                parsed = json.loads(json_str)
                # Validate the structure of each action
                for action in parsed:
                    if 'action_type' not in action:
                        action['action_type'] = 'unknown'
                    if 'parameters' not in action:
                        action['parameters'] = {}
                    if 'estimated_duration' not in action:
                        action['estimated_duration'] = 1.0
                return parsed
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Error parsing Claude response as JSON: {e}")
                return []

        return []

    def _calculate_confidence(self, action_sequence):
        """
        Calculate confidence in the action plan
        """
        if not action_sequence:
            return 0.0

        # Simple confidence calculation based on plan completeness
        valid_actions = [action for action in action_sequence
                        if action.get('action_type') in ["move_to", "pick_up", "put_down", "greet", "speak", "turn", "stop"]]

        return min(len(valid_actions) / len(action_sequence), 1.0) if action_sequence else 0.0

    def execute_action_sequence(self, action_sequence):
        """
        Execute a sequence of actions as ROS 2 commands
        """
        self.get_logger().info(f"Executing action sequence with {len(action_sequence)} actions")

        for i, action in enumerate(action_sequence):
            action_type = action.get('action_type', 'unknown')
            parameters = action.get('parameters', {})
            duration = action.get('estimated_duration', 1.0)

            self.get_logger().info(f"Executing action {i+1}/{len(action_sequence)}: {action_type}")

            # Publish action status
            status_msg = String()
            status_msg.data = f"Executing: {action_type}"
            self.action_status_publisher.publish(status_msg)

            # Execute specific action
            if action_type == 'move_to':
                self._execute_move_to(parameters, duration)
            elif action_type == 'pick_up':
                self._execute_pick_up(parameters, duration)
            elif action_type == 'put_down':
                self._execute_put_down(parameters, duration)
            elif action_type == 'greet':
                self._execute_greet(duration)
            elif action_type == 'speak':
                self._execute_speak(parameters, duration)
            elif action_type == 'turn':
                self._execute_turn(parameters, duration)
            elif action_type == 'stop':
                self._execute_stop(duration)
            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")

            # Wait for action to complete (or timeout)
            time.sleep(duration)

        # Publish completion status
        status_msg = String()
        status_msg.data = "Action sequence completed"
        self.action_status_publisher.publish(status_msg)

    def _execute_move_to(self, parameters, duration):
        """Execute a move_to action"""
        target_pos = parameters.get('target_position', {})

        if 'x' in target_pos and 'y' in target_pos:
            # Calculate direction vector
            dx = target_pos['x'] - self.robot_position['x']
            dy = target_pos['y'] - self.robot_position['y']

            # Create Twist message for movement
            twist = Twist()
            twist.linear.x = dx / duration if duration > 0 else 0.0
            twist.linear.y = dy / duration if duration > 0 else 0.0

            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(f"Moving to position: {target_pos}")

            # Update robot position (in a real system, this would come from odometry)
            self.robot_position['x'] = target_pos['x']
            self.robot_position['y'] = target_pos['y']
        else:
            self.get_logger().warn("Invalid target position for move_to action")

    def _execute_pick_up(self, parameters, duration):
        """Execute a pick_up action"""
        object_id = parameters.get('object_id')

        if object_id:
            # In a real system, this would trigger the robot's gripper
            self.get_logger().info(f"Picking up object: {object_id}")
        else:
            self.get_logger().warn("No object_id specified for pick_up action")

    def _execute_put_down(self, parameters, duration):
        """Execute a put_down action"""
        # In a real system, this would release the robot's gripper
        self.get_logger().info("Putting down object")

    def _execute_greet(self, duration):
        """Execute a greet action"""
        self.get_logger().info("Greeting gesture")

    def _execute_speak(self, parameters, duration):
        """Execute a speak action"""
        text = parameters.get('text', 'Hello')
        self.get_logger().info(f"Speaking: {text}")

    def _execute_turn(self, parameters, duration):
        """Execute a turn action"""
        angle = parameters.get('angle', 90)  # Default to 90 degrees
        twist = Twist()
        twist.angular.z = angle / duration if duration > 0 else 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Turning by {angle} degrees")

    def _execute_stop(self, duration):
        """Execute a stop action"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Stopping robot")


def main(args=None):
    rclpy.init(args=args)

    llm_translator = LLMToROS2Translator()

    try:
        print("LLM to ROS 2 Translator is running...")
        print("Send voice commands to /voice_commands topic to test the system")
        rclpy.spin(llm_translator)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        llm_translator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()