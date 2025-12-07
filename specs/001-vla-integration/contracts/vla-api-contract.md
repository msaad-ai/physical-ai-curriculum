# VLA Module API Contract

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) module that enables voice command processing and cognitive planning for humanoid robots.

## Voice Command Processing Service

### POST /voice/process
Process audio input and convert to text command

**Request:**
```json
{
  "audio_data": "base64_encoded_audio",
  "audio_format": "wav|mp3|flac",
  "language": "en|es|fr|de",
  "user_id": "string"
}
```

**Response (Success):**
```json
{
  "command_id": "uuid",
  "transcribed_text": "string",
  "confidence_score": 0.0-1.0,
  "processing_time_ms": 123,
  "status": "processed"
}
```

**Response (Error):**
```json
{
  "error_code": "AUDIO_PROCESSING_FAILED|INVALID_FORMAT|API_ERROR",
  "message": "string",
  "command_id": "uuid"
}
```

## Cognitive Planning Service

### POST /cognitive/plan
Translate natural language command to ROS 2 action sequence

**Request:**
```json
{
  "command_id": "uuid",
  "natural_language_command": "string",
  "robot_capabilities": ["move_to", "pick_up", "speak", "greet"],
  "environment_context": {
    "available_objects": ["red_ball", "blue_cube"],
    "robot_position": {"x": 0, "y": 0, "z": 0},
    "target_positions": {"kitchen": {"x": 5, "y": 3}}
  }
}
```

**Response (Success):**
```json
{
  "plan_id": "uuid",
  "action_sequence": [
    {
      "action_type": "move_to|pick_up|speak|greet",
      "parameters": {"target_position": {"x": 5, "y": 3}, "object_id": "red_ball"},
      "estimated_duration": 5.5
    }
  ],
  "confidence": 0.0-1.0,
  "reasoning": "string explaining the plan"
}
```

**Response (Error):**
```json
{
  "error_code": "PLANNING_FAILED|INVALID_COMMAND|CAPABILITIES_MISMATCH",
  "message": "string",
  "plan_id": "uuid"
}
```

## Action Execution Service

### POST /actions/execute
Execute a validated action sequence on the robot

**Request:**
```json
{
  "plan_id": "uuid",
  "action_sequence": [{"action_type": "string", "parameters": {}}],
  "simulation_mode": true
}
```

**Response (Success):**
```json
{
  "execution_id": "uuid",
  "status": "executing|completed|failed",
  "estimated_completion": "timestamp",
  "action_results": [
    {
      "action_id": "uuid",
      "action_type": "string",
      "status": "succeeded|failed|timeout",
      "execution_time": 2.3
    }
  ]
}
```

## Simulation Feedback Service

### GET /feedback/{execution_id}
Get feedback about a completed action execution

**Response:**
```json
{
  "execution_id": "uuid",
  "overall_status": "completed|failed|partial",
  "robot_state_before": {"position": {}, "gripper": "open|closed"},
  "robot_state_after": {"position": {}, "gripper": "open|closed"},
  "success_metrics": {
    "accuracy": 0.0-1.0,
    "efficiency": 0.0-1.0,
    "safety_compliance": true
  },
  "natural_language_feedback": "string"
}
```

## Error Codes

| Code | Description |
|------|-------------|
| AUDIO_PROCESSING_FAILED | Whisper API failed to process audio |
| INVALID_FORMAT | Unsupported audio format |
| API_ERROR | External API (Whisper/LLM) error |
| PLANNING_FAILED | LLM failed to generate valid action plan |
| INVALID_COMMAND | Natural language command could not be understood |
| CAPABILITIES_MISMATCH | Robot lacks capabilities to execute command |
| EXECUTION_FAILED | Action execution failed in simulation |
| TIMEOUT | Action exceeded maximum execution time |