"""
Whisper API Integration Module

This module provides functions for integrating with OpenAI Whisper API
to convert voice commands to text for the VLA (Vision-Language-Action) system.
"""

import openai
import os
import logging
import asyncio
from typing import Optional, Dict, Any
from pathlib import Path
import sounddevice as sd
import numpy as np
import wave
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)

class WhisperIntegration:
    """
    Class for handling Whisper API integration and voice processing
    """

    def __init__(self):
        """
        Initialize Whisper integration with API key
        """
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        openai.api_key = api_key
        self.sample_rate = 16000
        logger.info("WhisperIntegration initialized successfully")

    def record_audio(self, duration: int = 5) -> np.ndarray:
        """
        Record audio from microphone for specified duration

        Args:
            duration: Recording duration in seconds

        Returns:
            numpy array containing audio data
        """
        logger.info(f"Starting audio recording for {duration} seconds...")

        try:
            audio_data = sd.rec(
                int(duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32'
            )
            sd.wait()  # Wait for recording to complete
            logger.info("Audio recording completed successfully")
            return audio_data
        except Exception as e:
            logger.error(f"Error during audio recording: {e}")
            raise

    def save_audio_to_wav(self, audio_data: np.ndarray, filepath: str) -> str:
        """
        Save audio data to WAV file

        Args:
            audio_data: numpy array containing audio data
            filepath: path where WAV file should be saved

        Returns:
            path to saved file
        """
        try:
            # Convert float32 array to int16 for WAV format
            audio_int16 = (audio_data * 32767).astype(np.int16)

            with wave.open(filepath, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 2 bytes per sample (16-bit)
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_int16.tobytes())

            logger.info(f"Audio saved to {filepath}")
            return filepath
        except Exception as e:
            logger.error(f"Error saving audio to WAV: {e}")
            raise

    def transcribe_audio(self, audio_file_path: str) -> Optional[str]:
        """
        Transcribe audio file using OpenAI Whisper API

        Args:
            audio_file_path: path to audio file to transcribe

        Returns:
            transcribed text or None if transcription fails
        """
        try:
            logger.info(f"Starting transcription for file: {audio_file_path}")

            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file,
                    response_format="text"
                )

            logger.info(f"Transcription completed: {transcript[:50]}...")
            return transcript.strip()
        except Exception as e:
            logger.error(f"Error during Whisper transcription: {e}")
            return None

    def transcribe_from_file(self, audio_file_path: str) -> Optional[str]:
        """
        Convenience method to transcribe an existing audio file

        Args:
            audio_file_path: path to existing audio file

        Returns:
            transcribed text or None if transcription fails
        """
        if not os.path.exists(audio_file_path):
            logger.error(f"Audio file does not exist: {audio_file_path}")
            return None

        return self.transcribe_audio(audio_file_path)

    def record_and_transcribe(self, duration: int = 5, temp_filename: str = "temp_recording.wav") -> Optional[str]:
        """
        Complete pipeline: record audio -> save to temp file -> transcribe -> return text

        Args:
            duration: Recording duration in seconds
            temp_filename: temporary filename for audio file

        Returns:
            transcribed text or None if process fails
        """
        try:
            # Record audio
            audio_data = self.record_audio(duration)

            # Save to temporary file
            temp_path = self.save_audio_to_wav(audio_data, temp_filename)

            # Transcribe
            transcript = self.transcribe_audio(temp_path)

            # Clean up temporary file
            if os.path.exists(temp_path):
                os.remove(temp_path)
                logger.debug(f"Temporary file {temp_path} removed")

            return transcript
        except Exception as e:
            logger.error(f"Error in record and transcribe pipeline: {e}")
            # Clean up temporary file if it exists
            if os.path.exists(temp_filename):
                try:
                    os.remove(temp_filename)
                except:
                    pass
            return None

    def validate_audio_quality(self, audio_data: np.ndarray, threshold: float = 0.01) -> Dict[str, Any]:
        """
        Validate audio quality before transcription

        Args:
            audio_data: numpy array containing audio data
            threshold: minimum amplitude threshold for valid audio

        Returns:
            dictionary with validation results
        """
        # Calculate RMS amplitude
        rms = np.sqrt(np.mean(audio_data**2))

        # Check for silence
        is_silent = rms < threshold

        # Check for clipping (values too close to max)
        max_amplitude = np.max(np.abs(audio_data))
        is_clipping = max_amplitude > 0.9  # More than 90% of max possible amplitude

        validation_result = {
            'is_valid': not is_silent,
            'rms_amplitude': rms,
            'is_silent': is_silent,
            'is_clipping': is_clipping,
            'max_amplitude': max_amplitude,
            'recommendation': 'good' if not is_silent and not is_clipping else ('increase_volume' if is_silent else 'decrease_volume' if is_clipping else 'check_audio')
        }

        return validation_result


class AsyncWhisperIntegration(WhisperIntegration):
    """
    Async version of Whisper integration for use in async contexts
    """

    async def transcribe_audio_async(self, audio_file_path: str) -> Optional[str]:
        """
        Async version of audio transcription

        Args:
            audio_file_path: path to audio file to transcribe

        Returns:
            transcribed text or None if transcription fails
        """
        loop = asyncio.get_event_loop()
        try:
            logger.info(f"Starting async transcription for file: {audio_file_path}")

            with open(audio_file_path, "rb") as audio_file:
                transcript = await loop.run_in_executor(
                    None,
                    lambda: openai.Audio.transcribe(
                        model="whisper-1",
                        file=audio_file,
                        response_format="text"
                    )
                )

            logger.info(f"Async transcription completed: {transcript[:50]}...")
            return transcript.strip()
        except Exception as e:
            logger.error(f"Error during async Whisper transcription: {e}")
            return None

    async def record_and_transcribe_async(self, duration: int = 5, temp_filename: str = "temp_recording.wav") -> Optional[str]:
        """
        Async version of complete pipeline: record audio -> save to temp file -> transcribe -> return text

        Args:
            duration: Recording duration in seconds
            temp_filename: temporary filename for audio file

        Returns:
            transcribed text or None if process fails
        """
        try:
            # Record audio (this is blocking, could be improved with threading)
            audio_data = self.record_audio(duration)

            # Save to temporary file
            temp_path = self.save_audio_to_wav(audio_data, temp_filename)

            # Transcribe asynchronously
            transcript = await self.transcribe_audio_async(temp_path)

            # Clean up temporary file
            if os.path.exists(temp_path):
                os.remove(temp_path)
                logger.debug(f"Temporary file {temp_path} removed")

            return transcript
        except Exception as e:
            logger.error(f"Error in async record and transcribe pipeline: {e}")
            # Clean up temporary file if it exists
            if os.path.exists(temp_filename):
                try:
                    os.remove(temp_filename)
                except:
                    pass
            return None


# Convenience function for simple use cases
def transcribe_voice_command(duration: int = 5) -> Optional[str]:
    """
    Simple function to record and transcribe a voice command in one call

    Args:
        duration: Recording duration in seconds

    Returns:
        transcribed text or None if process fails
    """
    try:
        whisper = WhisperIntegration()
        return whisper.record_and_transcribe(duration)
    except Exception as e:
        logger.error(f"Error in simple transcription: {e}")
        return None


if __name__ == "__main__":
    # Example usage
    print("Testing Whisper Integration...")

    try:
        whisper = WhisperIntegration()

        print("Press Enter to start recording a 5-second voice command...")
        input()

        transcript = whisper.record_and_transcribe(duration=5)

        if transcript:
            print(f"Transcribed text: {transcript}")
        else:
            print("Could not transcribe the voice command")

    except Exception as e:
        print(f"Error: {e}")