"""
Cognitive Planner Module

This module implements cognitive planning functionality that translates natural language
instructions into ROS 2 action sequences, with validation and safety checks.
"""

import anthropic
import os
import json
import logging
import re
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


@dataclass
class Action:
    """
    Represents a single robot action
    """
    action_type: str
    parameters: Dict[str, Any]
    estimated_duration: float = 1.0


@dataclass
class PlanningResult:
    """
    Result of the cognitive planning process
    """
    original_command: str
    action_sequence: List[Action]
    confidence: float
    reasoning: str
    is_valid: bool
    validation_errors: List[str]


class ActionValidator:
    """
    Validates action sequences for safety and feasibility
    """

    def __init__(self):
        self.supported_actions = {
            'move_to', 'pick_up', 'put_down', 'greet', 'speak', 'turn', 'stop'
        }

    def validate_action_sequence(self, actions: List[Action], context: Dict[str, Any] = None) -> Tuple[bool, List[str]]:
        """
        Validate an action sequence for safety and feasibility

        Args:
            actions: List of actions to validate
            context: Environment context for validation

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        for i, action in enumerate(actions):
            # Check if action type is supported
            if action.action_type not in self.supported_actions:
                errors.append(f"Action {i}: Unsupported action type '{action.action_type}'")
                continue

            # Validate specific action parameters
            action_errors = self._validate_action_parameters(action, i, context)
            errors.extend(action_errors)

        return len(errors) == 0, errors

    def _validate_action_parameters(self, action: Action, index: int, context: Dict[str, Any]) -> List[str]:
        """
        Validate parameters for a specific action

        Args:
            action: Action to validate
            index: Index of action in sequence
            context: Environment context

        Returns:
            List of validation errors
        """
        errors = []

        if action.action_type == 'move_to':
            if 'target_position' not in action.parameters:
                errors.append(f"Action {index}: move_to requires target_position parameter")
            else:
                pos = action.parameters['target_position']
                if not isinstance(pos, dict) or not all(k in pos for k in ['x', 'y']):
                    errors.append(f"Action {index}: move_to target_position must have x and y coordinates")

        elif action.action_type == 'pick_up':
            if 'object_id' not in action.parameters:
                errors.append(f"Action {index}: pick_up requires object_id parameter")
            else:
                obj_id = action.parameters['object_id']
                if not isinstance(obj_id, str) or not obj_id.strip():
                    errors.append(f"Action {index}: pick_up object_id must be a non-empty string")

        elif action.action_type == 'put_down':
            # put_down typically doesn't require specific parameters
            pass

        elif action.action_type == 'speak':
            if 'text' not in action.parameters:
                errors.append(f"Action {index}: speak requires text parameter")
            else:
                text = action.parameters['text']
                if not isinstance(text, str) or not text.strip():
                    errors.append(f"Action {index}: speak text must be a non-empty string")

        elif action.action_type == 'turn':
            if 'angle' not in action.parameters:
                errors.append(f"Action {index}: turn requires angle parameter")
            else:
                angle = action.parameters['angle']
                if not isinstance(angle, (int, float)):
                    errors.append(f"Action {index}: turn angle must be a number")

        return errors

    def validate_environment_constraints(self, actions: List[Action], context: Dict[str, Any]) -> Tuple[bool, List[str]]:
        """
        Validate actions against environment constraints

        Args:
            actions: List of actions to validate
            context: Environment context

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        if context:
            # Check if objects exist for pick_up actions
            for i, action in enumerate(actions):
                if action.action_type == 'pick_up':
                    obj_id = action.parameters.get('object_id')
                    if obj_id and context.get('available_objects'):
                        available_ids = [obj.get('id') for obj in context.get('available_objects', [])]
                        if obj_id not in available_ids:
                            errors.append(f"Action {i}: Object '{obj_id}' is not available for pick_up")

                elif action.action_type == 'move_to':
                    target_pos = action.parameters.get('target_position', {})
                    if target_pos and context.get('navigation_map'):
                        # Check if target position is navigable
                        if not self._is_navigable(target_pos, context['navigation_map']):
                            errors.append(f"Action {i}: Target position {target_pos} is not navigable")

        return len(errors) == 0, errors

    def _is_navigable(self, position: Dict[str, float], navigation_map: Any) -> bool:
        """
        Check if a position is navigable (simplified implementation)
        """
        # In a real implementation, this would check against a navigation map
        return True  # Simplified for this example


class CognitivePlanner:
    """
    Main cognitive planning class that translates natural language to ROS 2 actions
    """

    def __init__(self):
        """
        Initialize the cognitive planner with Anthropic client
        """
        api_key = os.getenv("ANTHROPIC_API_KEY")
        if not api_key:
            raise ValueError("ANTHROPIC_API_KEY environment variable is required")

        self.client = anthropic.Anthropic(api_key=api_key)
        self.validator = ActionValidator()
        self.command_parser = CommandParser()

        logger.info("CognitivePlanner initialized successfully")

    def plan_action_sequence(self,
                           natural_language_command: str,
                           context: Dict[str, Any] = None) -> PlanningResult:
        """
        Plan an action sequence from a natural language command

        Args:
            natural_language_command: Natural language command to process
            context: Environment context for planning

        Returns:
            PlanningResult containing the action sequence and metadata
        """
        logger.info(f"Planning action sequence for command: {natural_language_command}")

        # Parse the command
        parsed_command = self.command_parser.parse_command(natural_language_command)

        # Prepare the prompt for Claude
        prompt = self._create_planning_prompt(natural_language_command, parsed_command, context)

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
            raw_action_sequence = self._parse_claude_response(response.content)

            # Convert to Action objects
            action_objects = [Action(**action) for action in raw_action_sequence]

            # Validate the action sequence
            is_valid, validation_errors = self.validator.validate_action_sequence(
                action_objects, context
            )

            # Additional environment validation
            env_valid, env_errors = self.validator.validate_environment_constraints(
                action_objects, context
            )

            is_valid = is_valid and env_valid
            validation_errors.extend(env_errors)

            # Calculate confidence
            confidence = self._calculate_confidence(action_objects)

            logger.info(f"Action sequence planned with {len(action_objects)} actions, "
                       f"confidence: {confidence}, valid: {is_valid}")

            return PlanningResult(
                original_command=natural_language_command,
                action_sequence=action_objects,
                confidence=confidence,
                reasoning=str(response.content),
                is_valid=is_valid,
                validation_errors=validation_errors
            )

        except Exception as e:
            logger.error(f"Error in cognitive planning: {e}")
            return PlanningResult(
                original_command=natural_language_command,
                action_sequence=[],
                confidence=0.0,
                reasoning="",
                is_valid=False,
                validation_errors=[f"Planning failed: {str(e)}"]
            )

    def _create_planning_prompt(self, command: str, parsed: Dict, context: Dict[str, Any]) -> str:
        """
        Create a prompt for Claude to generate an action plan
        """
        context_str = json.dumps(context or {}, indent=2)

        return f"""
        You are a cognitive planning system for a humanoid robot. Your task is to translate natural language commands into sequences of executable robot actions.

        Current environment context:
        {context_str}

        Natural language command: "{command}"

        Parsed command: {json.dumps(parsed, indent=2)}

        Please provide a detailed action plan as a JSON array of action objects. Each action should have:
        - action_type: The type of action (move_to, pick_up, put_down, greet, speak, turn, stop)
        - parameters: Any required parameters for the action
        - estimated_duration: Estimated time in seconds (default to 1.0 if not sure)

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
        response_str = str(response_content)

        # Find JSON array in response
        start_idx = response_str.find('[')
        end_idx = response_str.rfind(']') + 1

        if start_idx != -1 and end_idx != 0:
            json_str = response_str[start_idx:end_idx]
            try:
                parsed = json.loads(json_str)

                # Validate the structure of each action
                validated_actions = []
                for action in parsed:
                    if not isinstance(action, dict):
                        continue

                    # Ensure required fields exist
                    action_type = action.get('action_type', 'unknown')
                    parameters = action.get('parameters', {})
                    duration = action.get('estimated_duration', 1.0)

                    validated_actions.append({
                        'action_type': action_type,
                        'parameters': parameters,
                        'estimated_duration': float(duration)
                    })

                return validated_actions
            except json.JSONDecodeError as e:
                logger.error(f"Error parsing Claude response as JSON: {e}")
                return []

        return []

    def _calculate_confidence(self, action_sequence: List[Action]) -> float:
        """
        Calculate confidence in the action plan
        """
        if not action_sequence:
            return 0.0

        # Simple confidence calculation based on plan completeness and validity
        valid_actions = [action for action in action_sequence
                        if action.action_type in self.validator.supported_actions]

        base_confidence = len(valid_actions) / len(action_sequence) if action_sequence else 0.0

        # Add bonus for complete action sequences
        if len(action_sequence) > 0:
            base_confidence = base_confidence * 0.8 + 0.2  # Ensure some base confidence

        return min(base_confidence, 1.0)


class CommandParser:
    """
    Parses natural language commands to extract entities and actions
    """

    def __init__(self):
        # Define action patterns
        self.action_patterns = {
            'move_to': [r'go to (.+)', r'move to (.+)', r'go (.+)', r'walk to (.+)', r'travel to (.+)'],
            'pick_up': [r'pick up (.+)', r'grab (.+)', r'get (.+)', r'take (.+)', r'lift (.+)'],
            'put_down': [r'put down (.+)', r'drop (.+)', r'place (.+)', r'release (.+)'],
            'greet': [r'greet', r'say hello', r'hello', r'wave'],
            'speak': [r'say (.+)', r'tell (.+)', r'speak (.+)', r'announce (.+)'],
            'turn': [r'turn (.+)', r'rotate (.+)', r'pivot (.+)'],
            'stop': [r'stop', r'hold', r'pause', r'freeze']
        }

        # Location patterns
        self.location_patterns = [
            r'kitchen', r'living room', r'bedroom', r'office', r'garden',
            r'entrance', r'bathroom', r'dining room', r'garage', r'balcony'
        ]

        # Object patterns
        self.object_patterns = [
            r'red ball', r'blue cube', r'green cylinder', r'table', r'chair',
            r'cup', r'book', r'pen', r'box', r'bottle', r'phone', r'laptop'
        ]

    def parse_command(self, command: str) -> Dict[str, Any]:
        """
        Parse a natural language command and extract structured information
        """
        command_lower = command.lower().strip()

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
            'entities': objects + locations,
            'raw_parsed': f"Action: {action}, Objects: {objects}, Locations: {locations}"
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
            matches = re.findall(r'\b' + obj_pattern + r'\b', command, re.IGNORECASE)
            objects.extend(matches)
        return list(set(objects))  # Remove duplicates

    def _extract_locations(self, command: str) -> List[str]:
        """
        Extract location entities from the command
        """
        locations = []
        for loc_pattern in self.location_patterns:
            if re.search(r'\b' + loc_pattern + r'\b', command, re.IGNORECASE):
                matches = re.findall(r'\b' + loc_pattern + r'\b', command, re.IGNORECASE)
                locations.extend(matches)
        return list(set(locations))  # Remove duplicates


# Example usage and testing
if __name__ == "__main__":
    print("Testing Cognitive Planner...")

    try:
        planner = CognitivePlanner()

        # Example command
        command = "Go to the kitchen and pick up the red ball"

        # Example context
        context = {
            "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak", "turn", "stop"],
            "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "available_objects": [
                {"id": "red_ball", "name": "red ball", "position": {"x": 2.0, "y": 1.0}},
                {"id": "blue_cube", "name": "blue cube", "position": {"x": -1.0, "y": 3.0}}
            ],
            "known_locations": {
                "kitchen": {"x": 5.0, "y": 3.0},
                "bedroom": {"x": -3.0, "y": -2.0}
            }
        }

        result = planner.plan_action_sequence(command, context)

        print(f"Command: {result.original_command}")
        print(f"Confidence: {result.confidence}")
        print(f"Is Valid: {result.is_valid}")
        print(f"Validation Errors: {result.validation_errors}")
        print("Action Sequence:")
        for i, action in enumerate(result.action_sequence):
            print(f"  {i+1}. {action.action_type} - {action.parameters} (est. {action.estimated_duration}s)")

    except Exception as e:
        print(f"Error: {e}")