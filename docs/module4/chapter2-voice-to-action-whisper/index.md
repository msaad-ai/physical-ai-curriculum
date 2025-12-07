# Chapter 2: Voice-to-Action with OpenAI Whisper

## Introduction

In this chapter, we'll explore how to implement voice-to-action capabilities using OpenAI Whisper for speech recognition and conversion to text commands that can be processed by our robotic system. Whisper is a state-of-the-art speech recognition model that can convert spoken language into text with high accuracy.

## Setting up Whisper for Voice Command Processing

### Prerequisites

Before implementing voice command processing, ensure you have:

1. An OpenAI API key for Whisper access
2. Required Python dependencies installed
3. Audio input device (microphone) configured
4. Basic ROS 2 environment set up

### Audio Input Configuration

To capture voice commands, we need to set up audio input using Python's `sounddevice` library:

```python
import sounddevice as sd
import numpy as np
import wave
import io
from pydub import AudioSegment

def record_audio(duration=5, sample_rate=16000, channels=1):
    """
    Record audio from microphone for specified duration
    """
    print(f"Recording for {duration} seconds...")
    audio_data = sd.rec(
        int(duration * sample_rate),
        samplerate=sample_rate,
        channels=channels,
        dtype='float32'
    )
    sd.wait()  # Wait for recording to complete
    return audio_data

def save_audio_to_wav(audio_data, filename, sample_rate=16000):
    """
    Save recorded audio data to WAV file
    """
    # Convert float32 array to int16 for WAV format
    audio_int16 = (audio_data * 32767).astype(np.int16)

    with wave.open(filename, 'wb') as wav_file:
        wav_file.setnchannels(1)  # Mono
        wav_file.setsampwidth(2)  # 2 bytes per sample (16-bit)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio_int16.tobytes())
```

### Whisper API Integration

The Whisper API allows us to convert recorded audio to text. Here's how to integrate it:

```python
import openai
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure OpenAI API
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio_with_whisper(audio_file_path):
    """
    Transcribe audio file using OpenAI Whisper API
    """
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file,
            response_format="text"
        )
    return transcript
```

### Complete Voice Command Processing Pipeline

Now let's put it all together into a complete voice command processing system:

```python
import openai
import sounddevice as sd
import numpy as np
import wave
import os
from dotenv import load_dotenv
import time

class VoiceCommandProcessor:
    def __init__(self):
        load_dotenv()
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.sample_rate = 16000

    def record_voice_command(self, duration=5):
        """
        Record voice command from microphone
        """
        print("Listening for voice command...")
        audio_data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32'
        )
        sd.wait()
        return audio_data

    def save_audio_temp(self, audio_data, temp_filename="temp_voice_command.wav"):
        """
        Save audio data to temporary file for Whisper processing
        """
        audio_int16 = (audio_data * 32767).astype(np.int16)

        with wave.open(temp_filename, 'wb') as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio_int16.tobytes())

        return temp_filename

    def transcribe_voice_command(self, audio_file_path):
        """
        Transcribe voice command using Whisper API
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file,
                    response_format="text"
                )
            return transcript.strip()
        except Exception as e:
            print(f"Error transcribing audio: {e}")
            return None

    def process_voice_command(self, duration=5):
        """
        Complete pipeline: record -> save -> transcribe -> return text
        """
        # Record audio
        audio_data = self.record_voice_command(duration)

        # Save to temporary file
        temp_file = self.save_audio_temp(audio_data)

        # Transcribe using Whisper
        command_text = self.transcribe_voice_command(temp_file)

        # Clean up temporary file
        if os.path.exists(temp_file):
            os.remove(temp_file)

        return command_text

# Example usage
if __name__ == "__main__":
    processor = VoiceCommandProcessor()

    print("Press Enter to start recording voice command...")
    input()

    command = processor.process_voice_command(duration=5)
    if command:
        print(f"Recognized command: {command}")
    else:
        print("Could not recognize voice command")
```

## Converting Voice Commands to ROS 2 Actions

Once we have the transcribed text from Whisper, we need to map it to specific ROS 2 actions. This involves:

1. Natural language processing to extract intent
2. Mapping intent to specific robot actions
3. Validating the action for safety
4. Executing the action in the simulation environment

### Basic Intent Recognition

For basic intent recognition, we can use keyword matching or simple pattern matching:

```python
def extract_intent_from_command(transcribed_text):
    """
    Basic intent extraction from transcribed voice command
    """
    text_lower = transcribed_text.lower()

    # Define action mappings
    action_mappings = {
        'move forward': 'move_forward',
        'move backward': 'move_backward',
        'turn left': 'turn_left',
        'turn right': 'turn_right',
        'stop': 'stop',
        'wave': 'wave',
        'greet': 'greet',
        'pick up': 'pick_up',
        'put down': 'put_down'
    }

    for keyword, action in action_mappings.items():
        if keyword in text_lower:
            return action

    return 'unknown'
```

## Voice Activity Detection

To improve the user experience, we can implement voice activity detection to automatically start recording when speech is detected:

```python
def detect_voice_activity(audio_data, threshold=0.01):
    """
    Simple voice activity detection based on audio amplitude
    """
    # Calculate RMS amplitude
    rms = np.sqrt(np.mean(audio_data**2))
    return rms > threshold
```

## Summary of Key Takeaways

In this chapter, we've covered:

- Setting up audio input for voice command capture
- Integrating with OpenAI Whisper API for speech-to-text conversion
- Creating a complete voice command processing pipeline
- Basic intent recognition to map voice commands to actions
- Voice activity detection for improved user experience

### Key Concepts

1. **Audio Processing Pipeline**: The flow from audio capture → Whisper API → text processing → intent recognition → ROS 2 action mapping
2. **API Integration**: How to properly configure and use the OpenAI Whisper API for real-time voice command processing
3. **Intent Mapping**: Techniques for converting natural language to specific robot actions
4. **Safety Considerations**: Validation of commands before executing them on the robot

### Implementation Best Practices

- Always validate voice commands before executing robot actions
- Implement error handling for API failures and network issues
- Consider audio quality and background noise in real-world scenarios
- Design fallback mechanisms for when voice recognition fails

The next step is to integrate this voice processing system with our ROS 2 action execution system, which we'll cover in the next chapter.