"""
Voice Service API

This module implements the Voice Command Processing Service API as defined in the VLA API contract.
It handles audio input processing and conversion to text commands using Whisper.
"""

from flask import Flask, request, jsonify
import uuid
import base64
import time
import tempfile
import os
from typing import Dict, Any, Tuple
import logging

from ..whisper_integration import WhisperIntegration

# Configure logging
logger = logging.getLogger(__name__)

def create_voice_service() -> Flask:
    """
    Create and configure the voice service Flask app
    """
    app = Flask(__name__)

    # Initialize Whisper integration
    whisper_integration = WhisperIntegration()

    @app.route('/voice/process', methods=['POST'])
    def process_voice_command():
        """
        Process audio input and convert to text command
        API Contract: POST /voice/process
        """
        start_time = time.time()

        try:
            # Parse request data
            data = request.get_json()

            if not data:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "Request body must be JSON",
                    "command_id": str(uuid.uuid4())
                }), 400

            # Extract required fields
            audio_data = data.get('audio_data')
            audio_format = data.get('audio_format', 'wav')
            language = data.get('language', 'en')
            user_id = data.get('user_id')

            # Validate required fields
            if not audio_data:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "audio_data is required",
                    "command_id": str(uuid.uuid4())
                }), 400

            if audio_format not in ['wav', 'mp3', 'flac']:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": f"Unsupported audio format: {audio_format}",
                    "command_id": str(uuid.uuid4())
                }), 400

            # Create temporary file for audio data
            with tempfile.NamedTemporaryFile(delete=False, suffix=f'.{audio_format}') as temp_file:
                temp_filename = temp_file.name

                # Decode base64 audio data and write to temp file
                try:
                    audio_bytes = base64.b64decode(audio_data)
                    temp_file.write(audio_bytes)
                except Exception as e:
                    logger.error(f"Error decoding audio data: {e}")
                    return jsonify({
                        "error_code": "INVALID_FORMAT",
                        "message": "Invalid base64 audio data",
                        "command_id": str(uuid.uuid4())
                    }), 400

            # Process audio using Whisper
            command_id = str(uuid.uuid4())
            try:
                transcribed_text = whisper_integration.transcribe_from_file(temp_filename)

                if not transcribed_text:
                    return jsonify({
                        "error_code": "AUDIO_PROCESSING_FAILED",
                        "message": "Could not transcribe audio",
                        "command_id": command_id
                    }), 500

                # Calculate processing time
                processing_time = int((time.time() - start_time) * 1000)

                # For now, we'll use a simple confidence estimation based on text length and content
                # In a real implementation, Whisper would provide confidence scores
                confidence_score = min(len(transcribed_text) / 100.0, 1.0)  # Simple estimation

                # Log successful processing
                logger.info(f"Voice command processed successfully for user {user_id}, command_id: {command_id}")

                return jsonify({
                    "command_id": command_id,
                    "transcribed_text": transcribed_text,
                    "confidence_score": round(confidence_score, 2),
                    "processing_time_ms": processing_time,
                    "status": "processed"
                })

            except Exception as e:
                logger.error(f"Error processing audio with Whisper: {e}")
                return jsonify({
                    "error_code": "API_ERROR",
                    "message": f"Whisper API error: {str(e)}",
                    "command_id": command_id
                }), 500

            finally:
                # Clean up temporary file
                if os.path.exists(temp_filename):
                    os.unlink(temp_filename)

        except Exception as e:
            logger.error(f"Unexpected error in process_voice_command: {e}")
            command_id = str(uuid.uuid4())
            return jsonify({
                "error_code": "API_ERROR",
                "message": f"Internal server error: {str(e)}",
                "command_id": command_id
            }), 500

    @app.route('/health', methods=['GET'])
    def health_check():
        """
        Health check endpoint
        """
        return jsonify({
            "status": "healthy",
            "service": "voice-processing",
            "timestamp": time.time()
        })

    return app


# For standalone execution
if __name__ == '__main__':
    import os
    from dotenv import load_dotenv

    # Load environment variables
    load_dotenv()

    # Create and run the voice service
    voice_service = create_voice_service()

    # Get port from environment or default to 5000
    port = int(os.getenv('VOICE_SERVICE_PORT', 5001))

    # Enable debug mode if specified
    debug_mode = os.getenv('VOICE_SERVICE_DEBUG', 'false').lower() == 'true'

    logger.info(f"Starting Voice Service on port {port}")
    voice_service.run(host='0.0.0.0', port=port, debug=debug_mode)