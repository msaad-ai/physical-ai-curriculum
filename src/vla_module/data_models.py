"""
Base data models for the VLA module.

This module defines the core data structures used in the Vision-Language-Action system,
including voice commands, action sequences, cognitive plans, and feedback records.
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Dict, Any, Optional, Union
from enum import Enum
import uuid
from .ros2_actions import ROS2Action, ActionType


class VoiceCommandStatus(Enum):
    """Status of a voice command during processing."""
    PENDING = "pending"
    PROCESSING = "processing"
    PROCESSED = "processed"
    FAILED = "failed"
    VALIDATED = "validated"


class ActionStatus(Enum):
    """Status of an action during execution."""
    PENDING = "pending"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMEOUT = "timeout"


class VoiceCommand:
    """
    Represents a voice command captured from user input.

    Attributes:
        id: Unique identifier for the command
        raw_audio: Audio data captured from user input
        transcribed_text: Text representation from Whisper processing
        timestamp: When the command was received
        confidence_score: Confidence level of voice recognition (0.0-1.0)
        status: Processing status (pending, processed, failed)
    """
    def __init__(self, raw_audio: Optional[bytes] = None,
                 transcribed_text: Optional[str] = None,
                 confidence_score: float = 0.0):
        self.id = str(uuid.uuid4())
        self.raw_audio = raw_audio
        self.transcribed_text = transcribed_text or ""
        self.timestamp = datetime.now()
        self.confidence_score = confidence_score
        self.status = VoiceCommandStatus.PENDING

    def validate(self) -> bool:
        """Validate that the voice command has required fields."""
        return bool(self.transcribed_text.strip()) and 0.0 <= self.confidence_score <= 1.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "id": self.id,
            "transcribed_text": self.transcribed_text,
            "timestamp": self.timestamp.isoformat(),
            "confidence_score": self.confidence_score,
            "status": self.status.value
        }


@dataclass
class ActionSequence:
    """
    Represents a sequence of actions to be executed.

    Attributes:
        id: Unique identifier for the action sequence
        command_id: Reference to the associated voice command
        actions: List of ROS 2 action objects in execution order
        created_at: Timestamp when sequence was generated
        estimated_duration: Expected time to complete all actions
        validation_status: Whether the sequence is valid for execution
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    command_id: Optional[str] = None
    actions: List[ROS2Action] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)
    estimated_duration: float = 0.0
    validation_status: bool = False

    def add_action(self, action: ROS2Action):
        """Add an action to the sequence."""
        self.actions.append(action)
        # Update estimated duration based on action type
        self.estimated_duration += action.timeout

    def validate(self) -> bool:
        """Validate that the action sequence is executable."""
        if not self.actions:
            return False
        return all(action.action_type in ActionType for action in self.actions)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "id": self.id,
            "command_id": self.command_id,
            "actions": [action.action_type.value for action in self.actions],
            "created_at": self.created_at.isoformat(),
            "estimated_duration": self.estimated_duration,
            "validation_status": self.validation_status
        }


@dataclass
class CognitivePlan:
    """
    Represents a cognitive plan generated from natural language.

    Attributes:
        id: Unique identifier for the cognitive plan
        input_command: Original natural language command
        processed_plan: Structured plan with action sequences
        llm_response: Raw response from LLM with action mapping
        confidence: Confidence level of the plan (0.0-1.0)
        alternatives: List of alternative action sequences if primary fails
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    input_command: str = ""
    processed_plan: Optional[ActionSequence] = None
    llm_response: str = ""
    confidence: float = 0.0
    alternatives: List[ActionSequence] = field(default_factory=list)

    def validate(self) -> bool:
        """Validate that the cognitive plan has required fields."""
        return bool(self.input_command) and 0.0 <= self.confidence <= 1.0

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "id": self.id,
            "input_command": self.input_command,
            "processed_plan_id": self.processed_plan.id if self.processed_plan else None,
            "llm_response": self.llm_response,
            "confidence": self.confidence,
            "alternatives_count": len(self.alternatives)
        }


@dataclass
class SimulationFeedback:
    """
    Represents feedback from the simulation environment.

    Attributes:
        id: Unique identifier for feedback record
        action_id: Reference to the executed action
        robot_state_before: Robot state before action execution
        robot_state_after: Robot state after action execution
        execution_result: Success/failure status with details
        feedback_text: Natural language description of outcome
        timestamp: When feedback was generated
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    action_id: Optional[str] = None
    robot_state_before: Dict[str, Any] = field(default_factory=dict)
    robot_state_after: Dict[str, Any] = field(default_factory=dict)
    execution_result: str = "pending"
    feedback_text: str = ""
    timestamp: datetime = field(default_factory=datetime.now)

    def calculate_success_metrics(self) -> Dict[str, float]:
        """Calculate success metrics based on before/after states."""
        # This would contain logic to calculate metrics like accuracy, efficiency, etc.
        # based on the robot's state changes
        return {
            "accuracy": 1.0,  # Placeholder
            "efficiency": 1.0,  # Placeholder
            "safety_compliance": 1.0  # Placeholder
        }

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "id": self.id,
            "action_id": self.action_id,
            "robot_state_before": self.robot_state_before,
            "robot_state_after": self.robot_state_after,
            "execution_result": self.execution_result,
            "feedback_text": self.feedback_text,
            "timestamp": self.timestamp.isoformat(),
            "success_metrics": self.calculate_success_metrics()
        }


# Relationship helpers
def create_voice_command_to_action_sequence(voice_command: VoiceCommand) -> ActionSequence:
    """
    Create an action sequence linked to a voice command.

    Args:
        voice_command: The source voice command

    Returns:
        ActionSequence linked to the voice command
    """
    return ActionSequence(command_id=voice_command.id)


def create_cognitive_plan_for_command(input_command: str, llm_response: str) -> CognitivePlan:
    """
    Create a cognitive plan for a given command and LLM response.

    Args:
        input_command: The original natural language command
        llm_response: The LLM's response with action mapping

    Returns:
        CognitivePlan instance
    """
    return CognitivePlan(
        input_command=input_command,
        llm_response=llm_response
    )