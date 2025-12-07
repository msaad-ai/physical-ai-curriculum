"""
Cognitive Planning Service API

This module implements the Cognitive Planning Service API as defined in the VLA API contract.
It handles translating natural language commands to ROS 2 action sequences using LLMs.
"""

from flask import Flask, request, jsonify
import uuid
import time
import logging
from typing import Dict, Any, Tuple

from ..cognitive_planner import CognitivePlanner

# Configure logging
logger = logging.getLogger(__name__)

def create_cognitive_service() -> Flask:
    """
    Create and configure the cognitive service Flask app
    """
    app = Flask(__name__)

    # Initialize cognitive planner
    cognitive_planner = CognitivePlanner()

    @app.route('/cognitive/plan', methods=['POST'])
    def plan_cognitive_action():
        """
        Translate natural language command to ROS 2 action sequence
        API Contract: POST /cognitive/plan
        """
        start_time = time.time()

        try:
            # Parse request data
            data = request.get_json()

            if not data:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "Request body must be JSON",
                    "plan_id": str(uuid.uuid4())
                }), 400

            # Extract required fields
            command_id = data.get('command_id')
            natural_language_command = data.get('natural_language_command')
            robot_capabilities = data.get('robot_capabilities', [])
            environment_context = data.get('environment_context', {})

            # Validate required fields
            if not natural_language_command:
                return jsonify({
                    "error_code": "INVALID_COMMAND",
                    "message": "natural_language_command is required",
                    "plan_id": str(uuid.uuid4())
                }), 400

            # Create plan ID if not provided
            plan_id = str(uuid.uuid4())

            # Prepare context for planning
            context = {
                'robot_capabilities': robot_capabilities,
                'environment_context': environment_context
            }

            # Plan action sequence using cognitive planner
            try:
                planning_result = cognitive_planner.plan_action_sequence(
                    natural_language_command,
                    context
                )

                if not planning_result.is_valid:
                    return jsonify({
                        "error_code": "PLANNING_FAILED",
                        "message": f"Planning validation failed: {'; '.join(planning_result.validation_errors)}",
                        "plan_id": plan_id
                    }), 400

                if not planning_result.action_sequence:
                    return jsonify({
                        "error_code": "PLANNING_FAILED",
                        "message": "Could not generate action sequence",
                        "plan_id": plan_id
                    }), 500

                # Convert action sequence to the expected format
                action_sequence = []
                for action in planning_result.action_sequence:
                    action_dict = {
                        "action_type": action.action_type,
                        "parameters": action.parameters,
                        "estimated_duration": action.estimated_duration
                    }
                    action_sequence.append(action_dict)

                # Calculate processing time
                processing_time = int((time.time() - start_time) * 1000)

                # Log successful planning
                logger.info(f"Cognitive plan generated successfully for command {command_id}, plan_id: {plan_id}")

                return jsonify({
                    "plan_id": plan_id,
                    "action_sequence": action_sequence,
                    "confidence": round(planning_result.confidence, 2),
                    "reasoning": planning_result.reasoning[:500] + "..." if len(planning_result.reasoning) > 500 else planning_result.reasoning
                })

            except Exception as e:
                logger.error(f"Error in cognitive planning: {e}")
                return jsonify({
                    "error_code": "PLANNING_FAILED",
                    "message": f"Cognitive planning error: {str(e)}",
                    "plan_id": plan_id
                }), 500

        except Exception as e:
            logger.error(f"Unexpected error in plan_cognitive_action: {e}")
            plan_id = str(uuid.uuid4())
            return jsonify({
                "error_code": "API_ERROR",
                "message": f"Internal server error: {str(e)}",
                "plan_id": plan_id
            }), 500

    @app.route('/health', methods=['GET'])
    def health_check():
        """
        Health check endpoint
        """
        return jsonify({
            "status": "healthy",
            "service": "cognitive-planning",
            "timestamp": time.time()
        })

    return app


# For standalone execution
if __name__ == '__main__':
    import os
    from dotenv import load_dotenv

    # Load environment variables
    load_dotenv()

    # Create and run the cognitive service
    cognitive_service = create_cognitive_service()

    # Get port from environment or default to 5000
    port = int(os.getenv('COGNITIVE_SERVICE_PORT', 5002))

    # Enable debug mode if specified
    debug_mode = os.getenv('COGNITIVE_SERVICE_DEBUG', 'false').lower() == 'true'

    logger.info(f"Starting Cognitive Service on port {port}")
    cognitive_service.run(host='0.0.0.0', port=port, debug=debug_mode)