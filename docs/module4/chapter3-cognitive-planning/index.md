# Chapter 3: Cognitive Planning - Mapping Natural Language to ROS 2 Actions

## Introduction

In this chapter, we'll explore cognitive planning, which is the process of translating natural language instructions into specific ROS 2 actions that a robot can execute. Cognitive planning bridges the gap between human communication and robotic execution by interpreting high-level commands and breaking them down into executable action sequences.

## Understanding Natural Language to Action Mapping

### The Challenge

Natural language is inherently ambiguous and context-dependent. When a human says "Pick up the red ball and take it to the kitchen," the robot must:

1. Identify the objects mentioned ("red ball")
2. Locate the objects in the environment
3. Understand spatial relationships ("to the kitchen")
4. Break down the complex command into primitive actions
5. Sequence the actions in the correct order
6. Execute the actions safely

### Cognitive Planning Pipeline

The cognitive planning process involves several key steps:

1. **Natural Language Understanding**: Parse the command and extract entities and actions
2. **Environment Context**: Understand the current state of the world
3. **Action Planning**: Generate a sequence of ROS 2 actions
4. **Validation**: Verify the plan is executable and safe
5. **Execution**: Execute the action sequence

## Natural Language Processing for Robot Commands

### Command Parsing

To process natural language commands, we need to extract key information:

```python
import re
from typing import Dict, List, Optional

class CommandParser:
    def __init__(self):
        # Define action patterns
        self.action_patterns = {
            'move_to': [r'go to (.+)', r'move to (.+)', r'go (.+)', r'walk to (.+)'],
            'pick_up': [r'pick up (.+)', r'grab (.+)', r'get (.+)', r'take (.+)'],
            'put_down': [r'put down (.+)', r'drop (.+)', r'place (.+)'],
            'greet': [r'greet', r'say hello', r'hello'],
            'speak': [r'say (.+)', r'tell (.+)']
        }

        # Location patterns
        self.location_patterns = [
            r'kitchen', r'living room', r'bedroom', r'office', r'garden', r'entrance'
        ]

        # Object patterns
        self.object_patterns = [
            r'red ball', r'blue cube', r'green cylinder', r'table', r'chair'
        ]

    def parse_command(self, command: str) -> Dict:
        """
        Parse a natural language command and extract structured information
        """
        command_lower = command.lower()

        # Extract action
        action = self._extract_action(command_lower)

        # Extract objects
        objects = self._extract_objects(command_lower)

        # Extract locations
        locations = self._extract_locations(command_lower)

        return {
            'original_command': command,
            'action': action,
            'objects': objects,
            'locations': locations,
            'entities': objects + locations
        }

    def _extract_action(self, command: str) -> Optional[str]:
        """
        Extract the primary action from the command
        """
        for action_type, patterns in self.action_patterns.items():
            for pattern in patterns:
                if re.search(pattern, command):
                    return action_type
        return None

    def _extract_objects(self, command: str) -> List[str]:
        """
        Extract object entities from the command
        """
        objects = []
        for obj_pattern in self.object_patterns:
            matches = re.findall(obj_pattern, command)
            objects.extend(matches)
        return list(set(objects))  # Remove duplicates

    def _extract_locations(self, command: str) -> List[str]:
        """
        Extract location entities from the command
        """
        locations = []
        for loc_pattern in self.location_patterns:
            if loc_pattern in command:
                locations.append(loc_pattern)
        return locations
```

### Context Integration

The cognitive planner needs to understand the current environment context:

```python
class EnvironmentContext:
    def __init__(self):
        self.robot_position = {'x': 0, 'y': 0, 'z': 0}
        self.available_objects = []
        self.known_locations = {}
        self.robot_capabilities = ['move_to', 'pick_up', 'put_down', 'greet', 'speak']

    def update_object_positions(self, objects: List[Dict]):
        """
        Update positions of known objects in the environment
        """
        self.available_objects = objects

    def get_location_coordinates(self, location_name: str) -> Optional[Dict]:
        """
        Get coordinates for a named location
        """
        return self.known_locations.get(location_name)

    def is_object_available(self, object_name: str) -> bool:
        """
        Check if an object is available in the environment
        """
        return any(obj.get('name') == object_name for obj in self.available_objects)
```

## LLM Integration for Cognitive Planning

### Using Anthropic Claude for Planning

We'll use Anthropic Claude to perform the cognitive planning task:

```python
import anthropic
import os
import json
from dotenv import load_dotenv

load_dotenv()

class CognitivePlanner:
    def __init__(self):
        self.client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))
        self.command_parser = CommandParser()
        self.context = EnvironmentContext()

    def plan_action_sequence(self, natural_language_command: str) -> Dict:
        """
        Plan an action sequence from a natural language command using Claude
        """
        # Parse the command
        parsed_command = self.command_parser.parse_command(natural_language_command)

        # Prepare the prompt for Claude
        prompt = self._create_planning_prompt(natural_language_command, parsed_command)

        try:
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

            return {
                'original_command': natural_language_command,
                'parsed_command': parsed_command,
                'action_sequence': action_sequence,
                'confidence': self._calculate_confidence(action_sequence),
                'reasoning': str(response.content)
            }

        except Exception as e:
            return {
                'error': f'Planning failed: {str(e)}',
                'original_command': natural_language_command
            }

    def _create_planning_prompt(self, command: str, parsed: Dict) -> str:
        """
        Create a prompt for Claude to generate an action plan
        """
        return f"""
        You are a cognitive planning system for a humanoid robot. Your task is to translate natural language commands into sequences of executable robot actions.

        Current environment context:
        - Robot capabilities: {self.context.robot_capabilities}
        - Robot position: {self.context.robot_position}
        - Available objects: {[obj.get('name', 'unknown') for obj in self.context.available_objects]}
        - Known locations: {list(self.context.known_locations.keys())}

        Natural language command: "{command}"

        Parsed command: {json.dumps(parsed, indent=2)}

        Please provide a detailed action plan as a JSON array of action objects. Each action should have:
        - action_type: The type of action (move_to, pick_up, put_down, greet, speak)
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

    def _parse_claude_response(self, response_content) -> List[Dict]:
        """
        Parse Claude's response to extract the action sequence
        """
        # Extract JSON from response
        response_str = str(response_content)

        # Find JSON array in response
        start_idx = response_str.find('[')
        end_idx = response_str.rfind(']') + 1

        if start_idx != -1 and end_idx != 0:
            json_str = response_str[start_idx:end_idx]
            try:
                return json.loads(json_str)
            except json.JSONDecodeError:
                pass

        return []

    def _calculate_confidence(self, action_sequence: List[Dict]) -> float:
        """
        Calculate confidence in the action plan
        """
        if not action_sequence:
            return 0.0

        # Simple confidence calculation based on plan completeness
        valid_actions = [action for action in action_sequence
                        if action.get('action_type') in self.context.robot_capabilities]

        return min(len(valid_actions) / len(action_sequence), 1.0) if action_sequence else 0.0
```

## Safety and Validation

### Action Validation

Before executing actions, we must validate them for safety:

```python
class ActionValidator:
    def __init__(self, environment_context):
        self.context = environment_context

    def validate_action_sequence(self, action_sequence: List[Dict]) -> Dict:
        """
        Validate an action sequence for safety and feasibility
        """
        validation_results = {
            'is_valid': True,
            'errors': [],
            'warnings': []
        }

        for i, action in enumerate(action_sequence):
            action_type = action.get('action_type')
            parameters = action.get('parameters', {})

            # Check if action is supported
            if action_type not in self.context.robot_capabilities:
                validation_results['is_valid'] = False
                validation_results['errors'].append(
                    f"Action {i}: Unsupported action type '{action_type}'"
                )
                continue

            # Validate specific action parameters
            if action_type == 'move_to':
                if 'target_position' not in parameters:
                    validation_results['is_valid'] = False
                    validation_results['errors'].append(
                        f"Action {i}: move_to requires target_position parameter"
                    )

            elif action_type == 'pick_up':
                if 'object_id' not in parameters:
                    validation_results['is_valid'] = False
                    validation_results['errors'].append(
                        f"Action {i}: pick_up requires object_id parameter"
                    )

        return validation_results
```

## Integration with ROS 2

The planned actions need to be converted to ROS 2 messages:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String

class ROS2ActionExecutor(Node):
    def __init__(self):
        super().__init__('cognitive_planner_executor')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_status_publisher = self.create_publisher(String, '/action_status', 10)

    def execute_action_sequence(self, action_sequence: List[Dict]):
        """
        Execute a sequence of actions as ROS 2 commands
        """
        for action in action_sequence:
            action_type = action['action_type']

            if action_type == 'move_to':
                self._execute_move_to(action['parameters'])
            elif action_type == 'pick_up':
                self._execute_pick_up(action['parameters'])
            elif action_type == 'put_down':
                self._execute_put_down(action['parameters'])
            elif action_type == 'greet':
                self._execute_greet()
            elif action_type == 'speak':
                self._execute_speak(action['parameters'])

    def _execute_move_to(self, parameters):
        """
        Execute a move_to action
        """
        target_pos = parameters.get('target_position', {})
        # Convert to ROS 2 Twist message or navigation goal
        # Implementation depends on navigation stack used

    def _execute_pick_up(self, parameters):
        """
        Execute a pick_up action
        """
        object_id = parameters.get('object_id')
        # Implementation for pick and place operations
```

## Summary of Key Takeaways

Cognitive planning is a critical component of the VLA system that translates human intentions into robot actions. The key components include:

1. **Natural Language Understanding**: Parsing commands and extracting entities
2. **Context Awareness**: Understanding the environment and available objects
3. **LLM Integration**: Using Claude for complex reasoning and planning
4. **Action Validation**: Ensuring plans are safe and executable
5. **ROS 2 Integration**: Converting plans to executable robot commands

### Key Concepts

1. **Command Parsing**: The process of extracting meaningful entities and actions from natural language
2. **Context Integration**: Using environmental information to inform planning decisions
3. **LLM Reasoning**: Leveraging large language models for complex cognitive planning
4. **Safety Validation**: Ensuring action sequences are safe and feasible before execution
5. **Action Sequencing**: Breaking down complex commands into executable steps

### Implementation Best Practices

- Always validate action sequences before execution for safety
- Include context information to improve planning accuracy
- Implement proper error handling for LLM API calls
- Design modular validation systems that can be extended
- Consider the robot's physical limitations when planning actions

This chapter has covered the fundamental concepts of cognitive planning. In the next chapter, we'll integrate all components into a complete VLA system.