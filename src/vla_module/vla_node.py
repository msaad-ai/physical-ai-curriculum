"""
Main VLA (Vision-Language-Action) Node

This module integrates all components of the VLA system into a single ROS 2 node
that orchestrates voice processing, cognitive planning, and action execution.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image  # For future vision integration
import time
import threading
from typing import Dict, Any, Optional

from .whisper_integration import WhisperIntegration, transcribe_voice_command
from .cognitive_planner import CognitivePlanner
from .api.voice_service import create_voice_service
from .api.cognitive_service import create_cognitive_service
from .api.action_service import create_action_service
from .api.feedback_service import create_feedback_service


class VLAMainNode(Node):
    """
    Main VLA node that orchestrates all components
    """

    def __init__(self):
        super().__init__('vla_main_node')

        # Initialize VLA components
        self.whisper_integration = WhisperIntegration()
        self.cognitive_planner = CognitivePlanner()

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/vla_status', 10)
        self.feedback_publisher = self.create_publisher(String, '/vla_feedback', 10)

        # Subscribers
        self.voice_command_subscriber = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        self.text_command_subscriber = self.create_subscription(
            String,
            '/text_commands',
            self.text_command_callback,
            10
        )

        # Timer for continuous processing
        self.processing_timer = self.create_timer(1.0, self.process_commands)

        # State management
        self.current_state = "idle"
        self.is_listening = False
        self.command_queue = []
        self.robot_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.available_objects = []
        self.known_locations = {
            "kitchen": {"x": 5.0, "y": 3.0},
            "bedroom": {"x": -3.0, "y": -2.0},
            "living_room": {"x": 1.0, "y": -1.0}
        }

        # API services (running in separate threads)
        self.voice_service_thread = None
        self.cognitive_service_thread = None
        self.action_service_thread = None
        self.feedback_service_thread = None

        self.get_logger().info("VLA Main Node initialized successfully")

    def voice_command_callback(self, msg: String):
        """
        Handle incoming voice commands
        """
        self.get_logger().info(f"Received voice command: {msg.data}")
        self.command_queue.append({
            'type': 'voice',
            'data': msg.data,
            'timestamp': time.time()
        })

    def text_command_callback(self, msg: String):
        """
        Handle incoming text commands
        """
        self.get_logger().info(f"Received text command: {msg.data}")
        self.command_queue.append({
            'type': 'text',
            'data': msg.data,
            'timestamp': time.time()
        })

    def process_commands(self):
        """
        Main processing loop that handles command queue
        """
        if not self.command_queue:
            return

        # Process the oldest command
        command = self.command_queue.pop(0)

        self.get_logger().info(f"Processing {command['type']} command: {command['data']}")

        try:
            if command['type'] == 'voice':
                # If it's a voice command, we need to process it through Whisper first
                self.process_voice_command(command['data'])
            elif command['type'] == 'text':
                # If it's a text command, process directly through cognitive planner
                self.process_text_command(command['data'])
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            self.publish_status(f"Error processing command: {e}")

    def process_voice_command(self, audio_data: str):
        """
        Process a voice command through the full VLA pipeline
        """
        self.publish_status("Processing voice command")

        try:
            # Step 1: Transcribe audio to text (in this case, we assume it's already transcribed)
            # In a real scenario, we'd record and transcribe audio
            if audio_data.startswith("base64:"):  # Simulated audio data
                # Simulate transcription
                transcribed_text = self.simulate_transcription(audio_data)
            else:
                transcribed_text = audio_data  # Already transcribed

            self.get_logger().info(f"Transcribed text: {transcribed_text}")

            # Step 2: Plan cognitive action
            action_plan = self.plan_cognitive_action(transcribed_text)

            if action_plan and action_plan.is_valid:
                # Step 3: Execute action sequence
                self.execute_action_sequence(action_plan.action_sequence)
            else:
                self.get_logger().error(f"Planning failed: {action_plan.validation_errors if action_plan else 'No plan generated'}")
                self.publish_status("Planning failed")

        except Exception as e:
            self.get_logger().error(f"Error in voice command processing: {e}")
            self.publish_status(f"Voice processing error: {e}")

    def process_text_command(self, text_command: str):
        """
        Process a text command through cognitive planning and execution
        """
        self.publish_status("Processing text command")

        try:
            # Plan cognitive action
            action_plan = self.plan_cognitive_action(text_command)

            if action_plan and action_plan.is_valid:
                # Execute action sequence
                self.execute_action_sequence(action_plan.action_sequence)
            else:
                self.get_logger().error(f"Planning failed: {action_plan.validation_errors if action_plan else 'No plan generated'}")
                self.publish_status("Planning failed")

        except Exception as e:
            self.get_logger().error(f"Error in text command processing: {e}")
            self.publish_status(f"Text processing error: {e}")

    def plan_cognitive_action(self, natural_language_command: str):
        """
        Plan cognitive action using the cognitive planner
        """
        self.publish_status("Planning cognitive action")

        # Prepare context
        context = {
            "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak", "turn", "stop"],
            "robot_position": self.robot_position,
            "available_objects": self.available_objects,
            "known_locations": self.known_locations
        }

        # Plan action sequence
        action_plan = self.cognitive_planner.plan_action_sequence(
            natural_language_command,
            context
        )

        self.get_logger().info(f"Action plan generated with {len(action_plan.action_sequence)} actions")

        return action_plan

    def execute_action_sequence(self, action_sequence):
        """
        Execute a sequence of actions
        """
        self.publish_status(f"Executing action sequence with {len(action_sequence)} actions")

        for i, action in enumerate(action_sequence):
            self.get_logger().info(f"Executing action {i+1}/{len(action_sequence)}: {action.action_type}")

            success = self.execute_single_action(action)

            if not success:
                self.get_logger().error(f"Action {action.action_type} failed")
                self.publish_status(f"Action execution failed at step {i+1}")
                break

        self.publish_status("Action sequence completed")

    def execute_single_action(self, action) -> bool:
        """
        Execute a single action and return success status
        """
        action_type = action.action_type
        parameters = action.parameters

        try:
            if action_type == 'move_to':
                self.execute_move_to(parameters)
            elif action_type == 'pick_up':
                self.execute_pick_up(parameters)
            elif action_type == 'put_down':
                self.execute_put_down(parameters)
            elif action_type == 'greet':
                self.execute_greet()
            elif action_type == 'speak':
                self.execute_speak(parameters)
            elif action_type == 'turn':
                self.execute_turn(parameters)
            elif action_type == 'stop':
                self.execute_stop()
            else:
                self.get_logger().warn(f"Unknown action type: {action_type}")
                return False

            return True

        except Exception as e:
            self.get_logger().error(f"Error executing action {action_type}: {e}")
            return False

    def execute_move_to(self, parameters: Dict[str, Any]):
        """
        Execute move_to action
        """
        target_pos = parameters.get('target_position', {})
        if 'x' in target_pos and 'y' in target_pos:
            # Create Twist message for movement
            twist = Twist()
            # Calculate direction toward target (simplified)
            dx = target_pos['x'] - self.robot_position['x']
            dy = target_pos['y'] - self.robot_position['y']

            # Normalize and set velocity
            dist = (dx**2 + dy**2)**0.5
            if dist > 0.1:  # If not already at target
                twist.linear.x = min(1.0, dx / dist)  # Normalize and limit speed
                twist.linear.y = min(1.0, dy / dist)
            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0

            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(f"Moving to position: {target_pos}")

            # Update robot position (in real system, this comes from odometry)
            self.robot_position['x'] = target_pos['x']
            self.robot_position['y'] = target_pos['y']

    def execute_pick_up(self, parameters: Dict[str, Any]):
        """
        Execute pick_up action
        """
        object_id = parameters.get('object_id')
        if object_id:
            self.get_logger().info(f"Attempting to pick up object: {object_id}")
            # In a real system, this would control the gripper

    def execute_put_down(self, parameters: Dict[str, Any]):
        """
        Execute put_down action
        """
        self.get_logger().info("Putting down object")
        # In a real system, this would control the gripper

    def execute_greet(self):
        """
        Execute greet action
        """
        self.get_logger().info("Executing greeting action")
        # In a real system, this might move arms or speak

    def execute_speak(self, parameters: Dict[str, Any]):
        """
        Execute speak action
        """
        text = parameters.get('text', 'Hello')
        self.get_logger().info(f"Speaking: {text}")
        # In a real system, this would use text-to-speech

    def execute_turn(self, parameters: Dict[str, Any]):
        """
        Execute turn action
        """
        angle = parameters.get('angle', 90)  # Default to 90 degrees
        self.get_logger().info(f"Turning by {angle} degrees")

        twist = Twist()
        twist.angular.z = 0.5  # Turn at 0.5 rad/s
        self.cmd_vel_publisher.publish(twist)

    def execute_stop(self):
        """
        Execute stop action
        """
        self.get_logger().info("Stopping robot")

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def simulate_transcription(self, audio_data: str) -> str:
        """
        Simulate audio transcription (in real system, this would use Whisper)
        """
        # This is a placeholder - in real system, this would call Whisper API
        return "Go to the kitchen and pick up the red ball"

    def publish_status(self, status: str):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)

    def start_api_services(self):
        """
        Start API services in separate threads (for external integration)
        """
        import threading
        from flask import Flask

        def run_voice_service():
            app = create_voice_service()
            app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)

        def run_cognitive_service():
            app = create_cognitive_service()
            app.run(host='0.0.0.0', port=5002, debug=False, use_reloader=False)

        def run_action_service():
            app = create_action_service()
            app.run(host='0.0.0.0', port=5003, debug=False, use_reloader=False)

        def run_feedback_service():
            app = create_feedback_service()
            app.run(host='0.0.0.0', port=5004, debug=False, use_reloader=False)

        # Start services in separate threads
        self.voice_service_thread = threading.Thread(target=run_voice_service, daemon=True)
        self.cognitive_service_thread = threading.Thread(target=run_cognitive_service, daemon=True)
        self.action_service_thread = threading.Thread(target=run_action_service, daemon=True)
        self.feedback_service_thread = threading.Thread(target=run_feedback_service, daemon=True)

        self.voice_service_thread.start()
        self.cognitive_service_thread.start()
        self.action_service_thread.start()
        self.feedback_service_thread.start()

        self.get_logger().info("API services started in separate threads")


def main(args=None):
    """
    Main function to run the VLA node
    """
    rclpy.init(args=args)

    vla_node = VLAMainNode()

    # Start API services
    try:
        vla_node.start_api_services()
    except Exception as e:
        vla_node.get_logger().error(f"Error starting API services: {e}")

    try:
        print("VLA Main Node is running...")
        print("Listening for voice and text commands on /voice_commands and /text_commands topics")
        rclpy.spin(vla_node)
    except KeyboardInterrupt:
        print("\nShutting down VLA Main Node...")
    finally:
        vla_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()