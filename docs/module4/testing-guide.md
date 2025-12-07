# VLA Module Testing Guide

## Overview

This guide provides comprehensive testing procedures for the Vision-Language-Action (VLA) module. It covers unit tests, integration tests, and end-to-end testing procedures to ensure the VLA system functions correctly.

## Testing Prerequisites

Before running tests, ensure you have:

1. **API Keys Configured**:
   - OpenAI API key for Whisper integration
   - Anthropic API key for Claude cognitive planning

2. **Dependencies Installed**:
   ```bash
   pip install -r docs/module4/requirements.txt
   ```

3. **Environment Setup**:
   ```bash
   # Create .env file with your API keys
   OPENAI_API_KEY="your-openai-api-key"
   ANTHROPIC_API_KEY="your-anthropic-api-key"
   ```

## Unit Testing

### Testing Voice Processing Components

#### Whisper Integration Tests

```python
# Test basic transcription functionality
from src.vla_module.whisper_integration import WhisperIntegration

def test_transcription():
    whisper = WhisperIntegration()

    # Test with a sample audio file
    transcript = whisper.transcribe_from_file("sample_audio.wav")
    assert transcript is not None
    assert len(transcript) > 0

    print("✓ Whisper transcription test passed")

def test_audio_recording():
    whisper = WhisperIntegration()

    # Test recording functionality (short duration for tests)
    transcript = whisper.record_and_transcribe(duration=2)

    # Should return None or valid text
    assert transcript is None or isinstance(transcript, str)

    print("✓ Audio recording test passed")

def test_audio_validation():
    whisper = WhisperIntegration()

    # Create sample audio data
    import numpy as np
    sample_audio = np.random.random(16000)  # 1 second at 16kHz

    validation_result = whisper.validate_audio_quality(sample_audio)

    assert 'is_valid' in validation_result
    assert 'rms_amplitude' in validation_result

    print("✓ Audio validation test passed")
```

#### Running Unit Tests

```bash
# Run all unit tests
python -m pytest tests/unit/ -v

# Run specific voice processing tests
python -m pytest tests/unit/test_whisper_integration.py -v

# Run cognitive planning tests
python -m pytest tests/unit/test_cognitive_planner.py -v
```

## Integration Testing

### API Service Testing

#### Voice Service API Tests

```python
# Test voice processing API endpoint
import requests
import json

def test_voice_processing_api():
    url = "http://localhost:5001/voice/process"

    # Sample request data
    data = {
        "audio_data": "base64_encoded_audio_data_here",
        "audio_format": "wav",
        "language": "en",
        "user_id": "test_user_123"
    }

    response = requests.post(url, json=data)

    assert response.status_code == 200

    result = response.json()
    assert "command_id" in result
    assert "transcribed_text" in result
    assert "confidence_score" in result

    print("✓ Voice processing API test passed")

def test_api_error_handling():
    url = "http://localhost:5001/voice/process"

    # Test with invalid data
    invalid_data = {
        "audio_format": "invalid_format"
    }

    response = requests.post(url, json=invalid_data)

    assert response.status_code == 400

    error_result = response.json()
    assert "error_code" in error_result

    print("✓ API error handling test passed")
```

#### Cognitive Planning API Tests

```python
def test_cognitive_planning_api():
    url = "http://localhost:5002/cognitive/plan"

    data = {
        "command_id": "test_cmd_123",
        "natural_language_command": "Go to the kitchen",
        "robot_capabilities": ["move_to", "pick_up"],
        "environment_context": {
            "robot_position": {"x": 0, "y": 0, "z": 0},
            "known_locations": {"kitchen": {"x": 5, "y": 3}}
        }
    }

    response = requests.post(url, json=data)

    assert response.status_code == 200

    result = response.json()
    assert "plan_id" in result
    assert "action_sequence" in result
    assert "confidence" in result

    print("✓ Cognitive planning API test passed")
```

### Running Integration Tests

```bash
# Run all integration tests
python -m pytest tests/integration/ -v

# Run specific API integration tests
python -m pytest tests/integration/test_api_services.py -v

# Run VLA pipeline integration tests
python -m pytest tests/integration/test_vla_pipeline.py -v
```

## End-to-End Testing

### Complete VLA Pipeline Test

```python
def test_complete_vla_pipeline():
    """
    Test the complete VLA pipeline: Voice → Text → Plan → Action → Feedback
    """
    import threading
    import time
    from src.vla_module.vla_node import VLAMainNode
    import rclpy

    # Initialize ROS 2
    rclpy.init()

    # Create VLA node
    vla_node = VLAMainNode()

    # Start API services
    vla_node.start_api_services()

    # Simulate a complete command flow
    test_command = "Go to the kitchen and pick up the red ball"

    # Add command to queue (simulating voice input)
    vla_node.command_queue.append({
        'type': 'text',
        'data': test_command,
        'timestamp': time.time()
    })

    # Process the command
    vla_node.process_commands()

    # Verify the command was processed
    assert len(vla_node.command_queue) == 0

    print("✓ Complete VLA pipeline test passed")

    # Cleanup
    vla_node.destroy_node()
    rclpy.shutdown()
```

### Simulation Environment Tests

```python
def test_simulation_integration():
    """
    Test integration with simulation environment
    """
    # This would test actual ROS 2 communication with Gazebo
    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()

    # Create a test node to verify communication
    test_node = rclpy.create_node('vla_test_node')
    cmd_vel_publisher = test_node.create_publisher(Twist, '/cmd_vel', 10)

    # Create and send a simple command
    twist_msg = Twist()
    twist_msg.linear.x = 1.0  # Move forward

    cmd_vel_publisher.publish(tist_msg)

    print("✓ Simulation integration test passed")

    test_node.destroy_node()
    rclpy.shutdown()
```

## Testing Scenarios

### Voice Command Scenarios

| Scenario | Command | Expected Result |
|----------|---------|-----------------|
| Simple Navigation | "Go to the kitchen" | Move to kitchen location |
| Object Interaction | "Pick up the red ball" | Navigate to and pick up red ball |
| Complex Command | "Go to kitchen and pick up red ball" | Sequence of move and pick actions |
| Multi-step | "Go to bedroom, then kitchen" | Execute sequence of navigation actions |
| Invalid Command | Unclear/ambiguous speech | Proper error handling |

### Error Handling Tests

```python
def test_error_scenarios():
    # Test Whisper API failure
    with patch('openai.Audio.transcribe', side_effect=Exception("API Error")):
        whisper = WhisperIntegration()
        result = whisper.record_and_transcribe(duration=1)
        assert result is None  # Should handle gracefully

    # Test Claude API failure
    with patch('anthropic.Anthropic.messages.create', side_effect=Exception("API Error")):
        planner = CognitivePlanner()
        result = planner.plan_action_sequence("test command", {})
        assert not result.is_valid

    # Test invalid action sequence
    validator = ActionValidator()
    is_valid, errors = validator.validate_action_sequence([
        MagicMock(action_type='invalid_action', parameters={})
    ])
    assert not is_valid
    assert len(errors) > 0

    print("✓ Error handling tests passed")
```

## Automated Test Suite

### Running All Tests

```bash
# Run complete test suite
python -m pytest tests/ -v --cov=src.vla_module

# Run with specific markers
python -m pytest -m "not slow"  # Skip slow tests
python -m pytest -m "integration"  # Only integration tests
```

### Continuous Integration Configuration

Create `.github/workflows/test-vla.yml`:

```yaml
name: VLA Module Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: [3.8, 3.9, 3.10]

    steps:
    - uses: actions/checkout@v2

    - name: Set up Python ${{ matrix.python-version }}
      uses: actions/setup-python@v2
      with:
        python-version: ${{ matrix.python-version }}

    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -r docs/module4/requirements.txt
        pip install pytest pytest-cov

    - name: Run unit tests
      run: |
        python -m pytest tests/unit/ -v

    - name: Run integration tests
      run: |
        python -m pytest tests/integration/ -v
```

## Performance Testing

### Load Testing API Services

```python
import concurrent.futures
import time

def test_api_concurrent_load():
    """
    Test API services under concurrent load
    """
    def make_request(i):
        # Make API request
        response = requests.post("http://localhost:5001/voice/process", json={
            "audio_data": "base64_sample_data",
            "audio_format": "wav",
            "language": "en"
        })
        return response.status_code == 200

    # Test with 10 concurrent requests
    with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
        futures = [executor.submit(make_request, i) for i in range(10)]
        results = [future.result() for future in futures]

    success_count = sum(results)
    assert success_count >= 8  # At least 80% success rate

    print(f"✓ Load test passed: {success_count}/10 requests successful")
```

## Testing Best Practices

### 1. Mock External Dependencies

Always mock external APIs in unit tests:

```python
@patch('openai.Audio.transcribe')
def test_transcription_with_mock(mock_transcribe):
    mock_transcribe.return_value = "Mocked transcription"

    whisper = WhisperIntegration()
    result = whisper.transcribe_from_file("dummy.wav")

    assert result == "Mocked transcription"
```

### 2. Test Edge Cases

```python
def test_edge_cases():
    # Empty audio
    result = whisper.record_and_transcribe(duration=0)
    assert result is None

    # Very long command
    long_command = "A" * 1000
    result = planner.plan_action_sequence(long_command, {})
    # Should handle gracefully

    # Invalid JSON in API
    response = requests.post(url, data="invalid_json")
    assert response.status_code == 400
```

### 3. Verify Data Formats

```python
def test_data_format_validation():
    # Test that all API responses follow contract
    response = requests.get("http://localhost:5004/feedback/test_id")
    data = response.json()

    # Verify required fields exist
    assert "execution_id" in data
    assert "overall_status" in data
    assert "success_metrics" in data
```

## Test Coverage Goals

- **Unit Tests**: 80%+ coverage of individual components
- **Integration Tests**: All API endpoints tested
- **End-to-End Tests**: Complete pipeline scenarios
- **Error Handling**: All error conditions tested
- **Performance**: Response times under 5 seconds for 95% of requests

## Troubleshooting Tests

If tests fail:

1. **Check API Keys**: Ensure OPENAI_API_KEY and ANTHROPIC_API_KEY are set
2. **Verify Dependencies**: Run `pip install -r requirements.txt`
3. **Check Service Ports**: Ensure API services are running on expected ports
4. **Review Logs**: Check service logs for error details
5. **Network Issues**: Verify internet connectivity for API calls

## Next Steps

After running tests successfully:

1. Review test coverage reports
2. Address any failing tests
3. Optimize slow tests
4. Add new tests for additional functionality
5. Set up continuous integration