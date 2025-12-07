#!/usr/bin/env python3
"""
Voice to Text to ROS 2 Action Example

This script demonstrates the complete pipeline from voice command to ROS 2 action execution.
It records voice input, transcribes it using OpenAI Whisper, processes the intent,
and maps it to a ROS 2 action for robot execution.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import openai
import sounddevice as sd
import numpy as np
import wave
import os
from dotenv import load_dotenv


class VoiceToROS2Action(Node):
    def __init__(self):
        super().__init__('voice_to_ros2_action')

        # Load environment variables
        load_dotenv()
        openai.api_key = os.getenv("OPENAI_API_KEY")

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for voice commands (for logging/debugging)
        self.voice_command_publisher = self.create_publisher(String, '/voice_commands', 10)

        # Audio parameters
        self.sample_rate = 16000

        self.get_logger().info("Voice to ROS 2 Action node initialized")

    def record_audio(self, duration=5):
        """Record audio from microphone"""
        self.get_logger().info(f"Recording for {duration} seconds...")
        audio_data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()  # Wait for recording to complete
        return audio_data

    def save_audio_to_wav(self, audio_data, filename="temp_voice.wav"):
        """Save audio data to WAV file"""
        audio_int16 = (audio_data * 32767).astype(np.int16)

        with wave.open(filename, 'wb') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)  # 2 bytes for 16-bit
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio_int16.tobytes())

        return filename

    def transcribe_with_whisper(self, audio_file_path):
        """Transcribe audio using OpenAI Whisper API"""
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file,
                    response_format="text"
                )
            return transcript.strip()
        except Exception as e:
            self.get_logger().error(f"Error transcribing audio: {e}")
            return None

    def extract_intent(self, text):
        """Extract intent from transcribed text"""
        text_lower = text.lower()

        # Define action mappings
        intent_mappings = {
            'move forward': 'forward',
            'go forward': 'forward',
            'move back': 'backward',
            'move backward': 'backward',
            'go back': 'backward',
            'turn left': 'left',
            'turn right': 'right',
            'stop': 'stop',
            'halt': 'stop',
            'wave': 'wave',
            'greet': 'greet',
            'hello': 'greet'
        }

        for keyword, intent in intent_mappings.items():
            if keyword in text_lower:
                return intent

        return 'unknown'

    def map_intent_to_ros2_action(self, intent):
        """Map intent to ROS 2 action (Twist message)"""
        twist = Twist()

        if intent == 'forward':
            twist.linear.x = 1.0  # Move forward at 1.0 m/s
        elif intent == 'backward':
            twist.linear.x = -1.0  # Move backward at 1.0 m/s
        elif intent == 'left':
            twist.angular.z = 1.0  # Turn left at 1.0 rad/s
        elif intent == 'right':
            twist.angular.z = -1.0  # Turn right at 1.0 rad/s
        elif intent == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif intent == 'greet':
            # For greeting, we might want to do something special
            # This could trigger a specific robot behavior
            self.get_logger().info("Robot greeting action triggered")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().warn(f"Unknown intent: {intent}")
            return None

        return twist

    def process_voice_command(self):
        """Complete pipeline: record -> transcribe -> intent -> ROS 2 action"""
        # 1. Record audio
        audio_data = self.record_audio(duration=5)

        # 2. Save to temporary file
        temp_file = self.save_audio_to_wav(audio_data)

        # 3. Transcribe using Whisper
        transcribed_text = self.transcribe_with_whisper(temp_file)

        if transcribed_text:
            self.get_logger().info(f"Transcribed: {transcribed_text}")

            # Publish the voice command
            voice_msg = String()
            voice_msg.data = transcribed_text
            self.voice_command_publisher.publish(voice_msg)

            # 4. Extract intent
            intent = self.extract_intent(transcribed_text)
            self.get_logger().info(f"Detected intent: {intent}")

            # 5. Map intent to ROS 2 action
            ros2_action = self.map_intent_to_ros2_action(intent)

            if ros2_action:
                # 6. Execute ROS 2 action
                self.cmd_vel_publisher.publish(ros2_action)
                self.get_logger().info(f"Published ROS 2 action for intent: {intent}")
            else:
                self.get_logger().warn("Could not map intent to ROS 2 action")
        else:
            self.get_logger().error("Could not transcribe audio")

        # Clean up temporary file
        if os.path.exists(temp_file):
            os.remove(temp_file)


def main(args=None):
    rclpy.init(args=args)

    voice_to_ros2_node = VoiceToROS2Action()

    # Process a single voice command
    print("Press Enter to start recording voice command...")
    input()

    voice_to_ros2_node.process_voice_command()

    # Give some time for the command to be processed
    voice_to_ros2_node.get_logger().info("Command processed. Shutting down...")

    voice_to_ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()