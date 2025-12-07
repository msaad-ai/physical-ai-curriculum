# Data Model: Module 4: Vision-Language-Action (VLA)

## Core Entities

### VoiceCommand
- **id**: Unique identifier for the command
- **raw_audio**: Audio data captured from user input
- **transcribed_text**: Text representation from Whisper processing
- **timestamp**: When the command was received
- **confidence_score**: Confidence level of voice recognition (0.0-1.0)
- **status**: Processing status (pending, processed, failed)

### ActionSequence
- **id**: Unique identifier for the action sequence
- **command_id**: Reference to the associated voice command
- **actions**: List of ROS 2 action objects in execution order
- **created_at**: Timestamp when sequence was generated
- **estimated_duration**: Expected time to complete all actions
- **validation_status**: Whether the sequence is valid for execution

### ROS2Action
- **id**: Unique identifier for the ROS 2 action
- **action_type**: Type of action (move_to, pick_up, speak, etc.)
- **parameters**: Dictionary of action-specific parameters
- **target_position**: 3D coordinates for movement actions
- **priority**: Execution priority level
- **timeout**: Maximum time allowed for action completion

### CognitivePlan
- **id**: Unique identifier for the cognitive plan
- **input_command**: Original natural language command
- **processed_plan**: Structured plan with action sequences
- **llm_response**: Raw response from LLM with action mapping
- **confidence**: Confidence level of the plan (0.0-1.0)
- **alternatives**: List of alternative action sequences if primary fails

### SimulationFeedback
- **id**: Unique identifier for feedback record
- **action_id**: Reference to the executed action
- **robot_state_before**: Robot state before action execution
- **robot_state_after**: Robot state after action execution
- **execution_result**: Success/failure status with details
- **feedback_text**: Natural language description of outcome
- **timestamp**: When feedback was generated

## Relationships

- VoiceCommand → ActionSequence (one-to-many): One voice command can result in multiple action sequences for complex tasks
- ActionSequence → ROS2Action (one-to-many): One action sequence contains multiple ROS 2 actions
- CognitivePlan → ActionSequence (one-to-many): One cognitive plan can generate multiple action sequences
- ROS2Action → SimulationFeedback (one-to-one): Each action execution generates feedback

## Validation Rules

- VoiceCommand: Must have non-empty transcribed_text and valid confidence_score
- ActionSequence: Must contain at least one valid ROS2Action and have valid status
- ROS2Action: Must have valid action_type and appropriate parameters for the type
- CognitivePlan: Must have non-empty processed_plan and valid confidence level
- SimulationFeedback: Must reference valid action_id and have complete state information

## State Transitions

### VoiceCommand States
- pending → processing: When Whisper begins processing audio
- processing → processed: When transcription is complete
- processing → failed: When Whisper processing fails
- processed → validated: When transcription passes validation

### ROS2Action States
- pending → executing: When action begins execution
- executing → succeeded: When action completes successfully
- executing → failed: When action fails during execution
- executing → timeout: When action exceeds timeout limit