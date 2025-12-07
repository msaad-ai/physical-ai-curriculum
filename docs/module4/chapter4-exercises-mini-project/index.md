# Chapter 4: VLA Exercises and Mini Project

## Introduction

This chapter provides hands-on exercises and a mini-project to integrate all components of the Vision-Language-Action (VLA) system. You'll practice voice command processing, cognitive planning, and action execution in a simulated environment.

## Exercise 1: Basic Voice Command Processing

### Objective
Process a simple voice command and convert it to text using Whisper.

### Instructions
1. Set up your microphone and audio recording environment
2. Record the voice command: "Move forward"
3. Use the Whisper API to transcribe the audio to text
4. Verify the transcription matches the expected text

### Code Template
```python
from src.vla_module.whisper_integration import transcribe_voice_command

# Record and transcribe a voice command
transcript = transcribe_voice_command(duration=3)
print(f"Transcribed text: {transcript}")
```

### Expected Output
The transcription should closely match "Move forward" or similar.

## Exercise 2: Cognitive Planning with LLM

### Objective
Use the cognitive planner to translate a natural language command into an action sequence.

### Instructions
1. Use the cognitive planner to process the command: "Go to the kitchen"
2. Provide appropriate context (robot position, known locations)
3. Examine the generated action sequence
4. Validate the action sequence for correctness

### Code Template
```python
from src.vla_module.cognitive_planner import CognitivePlanner

# Initialize the cognitive planner
planner = CognitivePlanner()

# Define context
context = {
    "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak"],
    "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "known_locations": {
        "kitchen": {"x": 5.0, "y": 3.0},
        "bedroom": {"x": -3.0, "y": -2.0}
    }
}

# Plan action sequence
result = planner.plan_action_sequence("Go to the kitchen", context)

if result.is_valid:
    print(f"Action sequence: {result.action_sequence}")
    print(f"Confidence: {result.confidence}")
else:
    print(f"Planning failed: {result.validation_errors}")
```

### Expected Output
A valid action sequence to move the robot to the kitchen location.

## Exercise 3: Voice to Action Pipeline

### Objective
Combine voice processing and cognitive planning to execute a complete command.

### Instructions
1. Record a voice command: "Go to the kitchen and pick up the red ball"
2. Transcribe the voice command to text
3. Use cognitive planning to generate an action sequence
4. Validate and execute the action sequence

### Code Template
```python
from src.vla_module.whisper_integration import transcribe_voice_command
from src.vla_module.cognitive_planner import CognitivePlanner

# Initialize components
planner = CognitivePlanner()

# Step 1: Record and transcribe voice command
print("Recording voice command...")
transcript = transcribe_voice_command(duration=5)

if transcript:
    print(f"Transcribed: {transcript}")

    # Step 2: Plan action sequence
    context = {
        "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak"],
        "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "available_objects": [
            {"id": "red_ball", "name": "red ball", "position": {"x": 5.0, "y": 3.0}}
        ],
        "known_locations": {
            "kitchen": {"x": 5.0, "y": 3.0}
        }
    }

    # Step 3: Plan and validate
    result = planner.plan_action_sequence(transcript, context)

    if result.is_valid:
        print(f"Generated {len(result.action_sequence)} actions")
        for i, action in enumerate(result.action_sequence):
            print(f"  {i+1}. {action.action_type} - {action.parameters}")
    else:
        print(f"Planning failed: {result.validation_errors}")
else:
    print("Could not transcribe voice command")
```

## Exercise 4: API Integration Practice

### Objective
Use the VLA API services to process commands through HTTP requests.

### Instructions
1. Start the voice processing API service
2. Send an audio file to the `/voice/process` endpoint
3. Take the transcribed text and send it to the cognitive planning API
4. Use the action sequence from cognitive planning

### API Request Examples
```bash
# Process voice command
curl -X POST http://localhost:5001/voice/process \
  -H "Content-Type: application/json" \
  -d '{
    "audio_data": "base64_encoded_audio_here",
    "audio_format": "wav",
    "language": "en"
  }'

# Plan cognitive action
curl -X POST http://localhost:5002/cognitive/plan \
  -H "Content-Type: application/json" \
  -d '{
    "natural_language_command": "Go to the kitchen",
    "robot_capabilities": ["move_to", "pick_up", "speak"],
    "environment_context": {
      "robot_position": {"x": 0, "y": 0, "z": 0},
      "known_locations": {"kitchen": {"x": 5, "y": 3}}
    }
  }'
```

## Mini-Project: Complete VLA System

### Objective
Build and test a complete VLA system that integrates voice processing, cognitive planning, and action execution.

### Project Requirements
1. **Voice Input**: Capture voice commands from the user
2. **Speech-to-Text**: Convert voice to text using Whisper
3. **Cognitive Planning**: Translate text to action sequence using Claude
4. **Action Validation**: Validate the action sequence for safety
5. **Execution**: Execute the actions in simulation
6. **Feedback**: Provide feedback on execution results

### Implementation Steps
1. Create a main VLA controller that orchestrates all components
2. Implement error handling for each stage of processing
3. Add safety checks before executing actions
4. Test with various voice commands
5. Document the system architecture and workflow

### Code Structure Template
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

from src.vla_module.whisper_integration import WhisperIntegration
from src.vla_module.cognitive_planner import CognitivePlanner
from src.vla_module.api.voice_service import create_voice_service
from src.vla_module.api.cognitive_service import create_cognitive_service

class VLAMainController(Node):
    def __init__(self):
        super().__init__('vla_main_controller')

        # Initialize VLA components
        self.whisper = WhisperIntegration()
        self.planner = CognitivePlanner()

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/vla_status', 10)

        # Timer for processing loop
        self.processing_timer = self.create_timer(1.0, self.process_commands)

        self.get_logger().info("VLA Main Controller initialized")

    def process_commands(self):
        """Main processing loop"""
        # Record voice command
        transcript = self.whisper.record_and_transcribe(duration=5)

        if transcript:
            self.get_logger().info(f"Processing: {transcript}")

            # Plan action sequence
            context = self.get_current_context()
            result = self.planner.plan_action_sequence(transcript, context)

            if result.is_valid:
                self.execute_action_sequence(result.action_sequence)
            else:
                self.get_logger().error(f"Planning failed: {result.validation_errors}")

    def get_current_context(self):
        """Get current environment context"""
        return {
            "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak"],
            "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "available_objects": [],
            "known_locations": {}
        }

    def execute_action_sequence(self, action_sequence):
        """Execute the planned action sequence"""
        for action in action_sequence:
            self.execute_single_action(action)

    def execute_single_action(self, action):
        """Execute a single action"""
        if action.action_type == 'move_to':
            self.move_to_position(action.parameters.get('target_position', {}))
        elif action.action_type == 'speak':
            self.speak_text(action.parameters.get('text', ''))
        # Add other action types as needed

    def move_to_position(self, position):
        """Move robot to specified position"""
        # Implementation for movement
        pass

    def speak_text(self, text):
        """Make robot speak text"""
        # Implementation for speech
        pass

def main(args=None):
    rclpy.init(args=args)

    vla_controller = VLAMainController()

    try:
        rclpy.spin(vla_controller)
    except KeyboardInterrupt:
        pass
    finally:
        vla_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing Scenarios
1. **Simple Navigation**: "Go to the kitchen"
2. **Object Interaction**: "Pick up the red ball"
3. **Complex Command**: "Go to the kitchen and pick up the red ball"
4. **Multi-step**: "Go to the bedroom, then go to the kitchen"
5. **Error Handling**: Speak unclearly to test error handling

### Evaluation Criteria
- Voice command is accurately transcribed (≥80% accuracy)
- Cognitive planning generates valid action sequences (≥90% success rate)
- Actions are executed safely in simulation
- Error handling works appropriately
- System provides clear feedback to user

## Solution Hints

For each exercise, consider:

1. **Audio Quality**: Ensure clear audio input for better Whisper transcription
2. **Context Richness**: Provide detailed environment context for better planning
3. **Validation**: Always validate action sequences before execution
4. **Error Handling**: Implement graceful error handling for API failures
5. **Safety**: Include safety checks to prevent invalid robot actions

## Summary and Review Questions

### Key Takeaways
- Voice processing converts speech to text using Whisper
- Cognitive planning translates natural language to action sequences using LLMs
- Action validation ensures safe and feasible execution
- API services provide modular, scalable VLA system components
- Integration requires careful coordination of all components

### Review Questions
1. How does Whisper API integration work in the VLA system?
2. What role does the cognitive planner play in translating commands?
3. Why is action validation important for robot safety?
4. How do the API services improve system modularity?
5. What are the main challenges in voice-to-action pipeline integration?

### Next Steps
After completing these exercises and the mini-project, you should be able to:
- Build a complete VLA system from individual components
- Integrate voice processing, planning, and execution
- Handle errors and edge cases in the pipeline
- Deploy the system in a simulated environment
- Extend the system with additional capabilities