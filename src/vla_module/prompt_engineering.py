"""
Prompt Engineering Framework for Cognitive Planning

This module provides a framework for crafting effective prompts for LLMs
used in cognitive planning for the VLA system.
"""

import json
from typing import Dict, List, Any, Optional
from string import Template
import logging

# Configure logging
logger = logging.getLogger(__name__)


class PromptTemplate:
    """
    Base class for prompt templates
    """
    def __init__(self, template: str, required_vars: List[str] = None):
        self.template = Template(template)
        self.required_vars = required_vars or []

    def format(self, **kwargs) -> str:
        """
        Format the template with provided variables
        """
        # Check for required variables
        missing_vars = set(self.required_vars) - set(kwargs.keys())
        if missing_vars:
            raise ValueError(f"Missing required variables: {missing_vars}")

        return self.template.safe_substitute(**kwargs)


class CognitivePlanningPromptTemplate(PromptTemplate):
    """
    Prompt template for cognitive planning tasks
    """
    def __init__(self):
        template = """
You are a cognitive planning system for a humanoid robot. Your task is to translate natural language commands into sequences of executable robot actions.

Current environment context:
$context

Natural language command: "$command"

$parsing_instruction

Please provide a detailed action plan as a JSON array of action objects. Each action should have:
- action_type: The type of action (move_to, pick_up, put_down, greet, speak, turn, stop)
- parameters: Any required parameters for the action
- estimated_duration: Estimated time in seconds

$example_format

Return only the JSON array, nothing else.
        """.strip()

        super().__init__(template, [
            'context', 'command', 'parsing_instruction', 'example_format'
        ])


class VLAGuidelines:
    """
    Class containing guidelines for VLA-specific prompt engineering
    """

    @staticmethod
    def get_context_formatting_guidelines() -> str:
        """
        Get guidelines for formatting context information in prompts
        """
        return """
Context should include:
- Robot capabilities: List of supported actions
- Robot position: Current coordinates (x, y, z)
- Available objects: List of objects with their properties and positions
- Known locations: Named locations with coordinates
- Constraints: Any limitations or safety considerations
        """.strip()

    @staticmethod
    def get_command_clarification_guidelines() -> str:
        """
        Get guidelines for handling ambiguous commands
        """
        return """
For ambiguous commands:
- Ask for clarification if multiple interpretations exist
- Use context to resolve ambiguities
- Default to safest interpretation when uncertain
- Break complex commands into simpler steps
        """.strip()

    @staticmethod
    def get_safety_guidelines() -> str:
        """
        Get safety guidelines for prompt construction
        """
        return """
Safety considerations:
- Always validate action feasibility
- Consider robot physical limitations
- Avoid actions that might cause harm
- Include error handling in action sequences
- Plan for contingencies and failures
        """.strip()


class PromptEngineeringFramework:
    """
    Main framework for prompt engineering in cognitive planning
    """

    def __init__(self):
        self.planning_template = CognitivePlanningPromptTemplate()
        self.guidelines = VLAGuidelines()
        logger.info("Prompt Engineering Framework initialized")

    def create_planning_prompt(self,
                             command: str,
                             context: Dict[str, Any],
                             custom_instructions: Optional[str] = None) -> str:
        """
        Create a cognitive planning prompt

        Args:
            command: Natural language command to process
            context: Environment context
            custom_instructions: Additional instructions to include

        Returns:
            Formatted prompt string
        """
        context_str = json.dumps(context, indent=2)

        parsing_instruction = custom_instructions or (
            "Please analyze the command and extract the intent, "
            "objects, and target locations. Consider the environment context."
        )

        example_format = """
Example format:
[
  {
    "action_type": "move_to",
    "parameters": {"target_position": {"x": 5, "y": 3, "z": 0}},
    "estimated_duration": 3.5
  },
  {
    "action_type": "pick_up",
    "parameters": {"object_id": "red_ball"},
    "estimated_duration": 2.0
  }
]
        """.strip()

        return self.planning_template.format(
            context=context_str,
            command=command,
            parsing_instruction=parsing_instruction,
            example_format=example_format
        )

    def create_context_aware_prompt(self,
                                  command: str,
                                  robot_capabilities: List[str],
                                  robot_position: Dict[str, float],
                                  available_objects: List[Dict[str, Any]],
                                  known_locations: Dict[str, Dict[str, float]]) -> str:
        """
        Create a context-aware planning prompt with structured environment information
        """
        context = {
            "robot_capabilities": robot_capabilities,
            "robot_position": robot_position,
            "available_objects": available_objects,
            "known_locations": known_locations
        }

        return self.create_planning_prompt(command, context)

    def create_validation_prompt(self, action_sequence: List[Dict[str, Any]], context: Dict[str, Any]) -> str:
        """
        Create a prompt for validating an action sequence
        """
        template = Template("""
You are a safety validation system for robot action sequences. Validate the following action sequence for safety and feasibility.

Environment context:
$context

Action sequence to validate:
$action_sequence

Validation criteria:
- Actions must be supported by robot capabilities
- Object references must exist in the environment
- Navigation targets must be reachable
- Actions must follow logical sequence
- Safety constraints must be respected

Please respond with a JSON object containing:
- is_valid: boolean indicating if sequence is valid
- issues: array of issues found (empty if valid)
- suggestions: array of suggestions for improvement (if applicable)

Example response format:
{
  "is_valid": true,
  "issues": [],
  "suggestions": []
}

Return only the JSON response, nothing else.
        """.strip())

        return template.safe_substitute(
            context=json.dumps(context, indent=2),
            action_sequence=json.dumps(action_sequence, indent=2)
        )

    def create_refinement_prompt(self,
                               command: str,
                               current_plan: List[Dict[str, Any]],
                               feedback: str) -> str:
        """
        Create a prompt for refining an action plan based on feedback
        """
        template = Template("""
You are a cognitive planning refinement system. Refine the following action plan based on feedback.

Original command: "$command"

Current action plan:
$current_plan

Feedback: "$feedback"

Please provide an improved action plan as a JSON array of action objects, addressing the feedback while maintaining the original intent.

Improved plan:
        """.strip())

        return template.safe_substitute(
            command=command,
            current_plan=json.dumps(current_plan, indent=2),
            feedback=feedback
        )

    def optimize_prompt_for_claude(self, prompt: str) -> str:
        """
        Apply Claude-specific optimizations to the prompt
        """
        # Claude prefers clear, direct instructions
        optimized = prompt.replace(
            "Please provide",
            "Provide"
        ).replace(
            "Please respond with",
            "Respond with"
        ).replace(
            "Please analyze",
            "Analyze"
        )

        # Ensure clear separation of instructions and data
        lines = optimized.split('\n')
        optimized_lines = []
        for i, line in enumerate(lines):
            if line.strip().startswith('{') or line.strip().endswith('}'):
                # Add extra line breaks around JSON examples
                if i > 0 and not lines[i-1].strip() == '':
                    optimized_lines.append('')
                optimized_lines.append(line)
                if i < len(lines) - 1 and not lines[i+1].strip() == '':
                    optimized_lines.append('')
            else:
                optimized_lines.append(line)

        return '\n'.join(optimized_lines)


class PromptAnalyzer:
    """
    Analyzes and provides feedback on prompt quality
    """

    @staticmethod
    def analyze_prompt_complexity(prompt: str) -> Dict[str, Any]:
        """
        Analyze the complexity of a prompt
        """
        lines = prompt.split('\n')
        word_count = len(prompt.split())
        line_count = len(lines)

        # Count different types of content
        instruction_count = sum(1 for line in lines if 'Please' in line or 'Respond' in line or 'Provide' in line)
        data_block_count = sum(1 for line in lines if line.strip().startswith('{') or line.strip().endswith('}'))

        return {
            'word_count': word_count,
            'line_count': line_count,
            'instruction_count': instruction_count,
            'data_block_count': data_block_count,
            'complexity_score': min(word_count / 100, 5.0),  # Scale 0-5
            'recommendation': 'Consider simplifying' if word_count > 500 else 'Appropriate complexity'
        }


# Convenience function for quick prompt creation
def create_simple_planning_prompt(command: str, context: Dict[str, Any]) -> str:
    """
    Create a simple planning prompt without using the full framework
    """
    framework = PromptEngineeringFramework()
    return framework.create_planning_prompt(command, context)


# Example usage
if __name__ == "__main__":
    print("Testing Prompt Engineering Framework...")

    # Initialize the framework
    framework = PromptEngineeringFramework()

    # Example context
    context = {
        "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak"],
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

    # Create a planning prompt
    command = "Go to the kitchen and pick up the red ball"
    prompt = framework.create_planning_prompt(command, context)

    print(f"Command: {command}")
    print("\nGenerated Prompt:")
    print("-" * 50)
    print(prompt)
    print("-" * 50)

    # Analyze the prompt
    analysis = PromptAnalyzer.analyze_prompt_complexity(prompt)
    print(f"\nPrompt Analysis: {analysis}")