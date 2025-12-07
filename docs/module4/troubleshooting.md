# VLA Module Troubleshooting Guide

## Overview

This guide helps you identify and resolve common issues that may occur when working with the Vision-Language-Action (VLA) module. It covers problems related to voice processing, cognitive planning, API services, and simulation integration.

## Common Issues and Solutions

### 1. API Connection Issues

#### Problem: "API key not found" or "Authentication failed"
**Symptoms:**
- Error messages about missing API keys
- 401 Unauthorized responses from API services

**Solutions:**
1. **Verify API key configuration:**
   ```bash
   # Check if environment variables are set
   echo $OPENAI_API_KEY
   echo $ANTHROPIC_API_KEY
   ```

2. **Create/update .env file:**
   ```bash
   # In project root directory
   touch .env
   echo "OPENAI_API_KEY='your-openai-api-key'" >> .env
   echo "ANTHROPIC_API_KEY='your-anthropic-api-key'" >> .env
   ```

3. **Load environment variables:**
   ```python
   from dotenv import load_dotenv
   load_dotenv()  # Call this before using API integrations
   ```

#### Problem: "Rate limit exceeded" or "API quota exceeded"
**Symptoms:**
- 429 Too Many Requests errors
- API responses with rate limit messages

**Solutions:**
1. **Implement rate limiting in your code:**
   ```python
   import time

   # Add delays between API calls
   time.sleep(1)  # Wait 1 second between calls
   ```

2. **Check your API usage:**
   - Visit OpenAI Dashboard: https://platform.openai.com/usage
   - Visit Anthropic Dashboard: https://console.anthropic.com/
   - Consider upgrading your plan if needed

### 2. Voice Processing Issues

#### Problem: "Audio recording failed" or "No audio input"
**Symptoms:**
- Voice commands not being captured
- Recording errors or empty audio data

**Solutions:**
1. **Check audio input device:**
   ```bash
   # List available audio devices
   python -c "import sounddevice as sd; print(sd.query_devices())"
   ```

2. **Verify microphone permissions:**
   - On Windows: Check Privacy settings for microphone access
   - On macOS: System Preferences > Security & Privacy > Microphone
   - On Linux: Check if `arecord` can record audio

3. **Test audio recording separately:**
   ```python
   import sounddevice as sd
   import numpy as np

   # Test basic recording
   duration = 3  # seconds
   sample_rate = 16000
   audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1)
   sd.wait()  # Wait for recording to complete
   print(f"Recorded {len(audio_data)} samples")
   ```

#### Problem: Poor transcription quality
**Symptoms:**
- Whisper returns incorrect or nonsensical text
- Very low confidence scores

**Solutions:**
1. **Improve audio quality:**
   - Record in a quiet environment
   - Speak clearly and at consistent volume
   - Position microphone appropriately

2. **Validate audio before transcription:**
   ```python
   from src.vla_module.whisper_integration import WhisperIntegration

   whisper = WhisperIntegration()
   validation = whisper.validate_audio_quality(audio_data)

   if not validation['is_valid']:
       print(f"Audio quality issue: {validation['recommendation']}")
   ```

### 3. Cognitive Planning Issues

#### Problem: "Planning failed" or "Invalid command"
**Symptoms:**
- Cognitive planner returns errors
- Action sequences are empty or invalid

**Solutions:**
1. **Check command format:**
   - Ensure commands are in natural language
   - Use simple, clear language
   - Example: "Go to the kitchen" instead of "Kitchen. Go."

2. **Verify context information:**
   ```python
   context = {
       "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak"],
       "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
       "known_locations": {"kitchen": {"x": 5.0, "y": 3.0}},
       "available_objects": [{"id": "red_ball", "name": "red ball"}]
   }
   ```

3. **Test with simple commands first:**
   - Start with basic navigation: "Go to kitchen"
   - Progress to object interaction: "Pick up red ball"
   - Combine commands: "Go to kitchen and pick up red ball"

#### Problem: LLM returns unexpected action types
**Symptoms:**
- Action sequences contain unsupported action types
- Execution fails due to invalid actions

**Solutions:**
1. **Validate action types:**
   ```python
   supported_actions = ["move_to", "pick_up", "put_down", "greet", "speak", "turn", "stop"]

   for action in action_sequence:
       if action.action_type not in supported_actions:
           print(f"Unsupported action: {action.action_type}")
   ```

### 4. API Service Issues

#### Problem: "Connection refused" when calling API services
**Symptoms:**
- Cannot connect to voice service (port 5001)
- Cannot connect to cognitive service (port 5002)
- Connection timeout errors

**Solutions:**
1. **Check if services are running:**
   ```bash
   # Check if services are listening on expected ports
   netstat -an | grep 500[1-4]
   # Or on Windows:
   netstat -an | findstr 500[1-4]
   ```

2. **Start API services manually:**
   ```bash
   # Start voice service
   python -m src.vla_module.api.voice_service

   # Start cognitive service
   python -m src.vla_module.api.cognitive_service

   # Start action service
   python -m src.vla_module.api.action_service

   # Start feedback service
   python -m src.vla_module.api.feedback_service
   ```

3. **Check port availability:**
   ```bash
   # Kill any processes using VLA ports
   # On Linux/Mac:
   lsof -i :5001
   kill -9 [PID]

   # On Windows:
   netstat -ano | findstr :5001
   taskkill /PID [PID] /F
   ```

#### Problem: API service crashes or exits unexpectedly
**Symptoms:**
- Services terminate without error message
- High memory or CPU usage

**Solutions:**
1. **Check logs for errors:**
   ```bash
   # Run with debug mode to see detailed logs
   python -m src.vla_module.api.voice_service --debug
   ```

2. **Verify API keys are valid:**
   - Test API keys directly with OpenAI/Anthropic
   - Check for typos in environment variables

### 5. Simulation and ROS 2 Issues

#### Problem: "No ROS 2 installation found" or "rclpy import error"
**Symptoms:**
- Import errors when running VLA node
- Cannot publish/subscribe to topics

**Solutions:**
1. **Verify ROS 2 installation:**
   ```bash
   # Check ROS 2 installation
   echo $ROS_DISTRO
   source /opt/ros/humble/setup.bash  # Adjust for your ROS version
   python -c "import rclpy; print('rclpy imported successfully')"
   ```

2. **Set up ROS 2 environment:**
   ```bash
   # Add to your .bashrc or .zshrc
   source /opt/ros/humble/setup.bash
   export PYTHONPATH="${PYTHONPATH}:/path/to/your/project/src"
   ```

#### Problem: "Topic not found" or "No subscribers"
**Symptoms:**
- Robot commands not being executed
- No feedback from simulation

**Solutions:**
1. **Check active topics:**
   ```bash
   ros2 topic list
   ros2 topic echo /cmd_vel  # Check if messages are being published
   ```

2. **Verify topic names match:**
   ```python
   # In your VLA node, ensure topic names match simulation
   self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
   # Make sure '/cmd_vel' matches what your simulation expects
   ```

### 6. Performance Issues

#### Problem: Slow response times
**Symptoms:**
- Long delays between command and execution
- API calls taking more than 5 seconds

**Solutions:**
1. **Optimize API calls:**
   - Implement caching for repeated requests
   - Use appropriate model sizes (smaller models are faster)

2. **Check network connectivity:**
   ```bash
   # Test API endpoint connectivity
   curl -I https://api.openai.com/
   curl -I https://api.anthropic.com/
   ```

3. **Monitor resource usage:**
   ```bash
   # Check system resources
   htop  # or task manager on Windows
   ```

## Diagnostic Commands

### Check System Requirements
```bash
# Python version
python --version

# Required packages
pip list | grep -E "(openai|anthropic|sounddevice|rclpy|flask)"

# Check environment variables
env | grep -E "(OPENAI|ANTHROPIC)"
```

### Test Individual Components
```bash
# Test Whisper integration
python -c "from src.vla_module.whisper_integration import WhisperIntegration; print('Whisper OK')"

# Test Cognitive planner
python -c "from src.vla_module.cognitive_planner import CognitivePlanner; print('Cognitive planner OK')"

# Test API services
python -c "from src.vla_module.api.voice_service import create_voice_service; print('Voice service OK')"
```

### Run Health Checks
```bash
# Check if all services are responding
curl http://localhost:5001/health
curl http://localhost:5002/health
curl http://localhost:5003/health
curl http://localhost:5004/health
```

## Prevention Tips

1. **Environment Setup:**
   - Always load environment variables before running VLA components
   - Use virtual environments to avoid dependency conflicts

2. **API Management:**
   - Monitor your API usage to avoid rate limits
   - Implement retry logic with exponential backoff

3. **Code Testing:**
   - Test components individually before integration
   - Use mock services for development without API calls

4. **System Resources:**
   - Ensure sufficient memory and CPU for real-time processing
   - Close unnecessary applications during testing

## Getting Help

If you encounter issues not covered in this guide:

1. **Check the logs:**
   - Look for detailed error messages in console output
   - Check system logs for additional information

2. **Verify your setup:**
   - Ensure all prerequisites are met
   - Check that all required services are running

3. **Test in isolation:**
   - Test individual components separately
   - Use simple test cases before complex scenarios

4. **Consult documentation:**
   - Review the VLA module documentation
   - Check API documentation for OpenAI and Anthropic

5. **Community support:**
   - Check GitHub issues for similar problems
   - Ask questions in the appropriate channels