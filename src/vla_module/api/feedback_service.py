"""
Simulation Feedback Service API

This module implements the Simulation Feedback Service API as defined in the VLA API contract.
It provides feedback about completed action executions in the simulation environment.
"""

from flask import Flask, request, jsonify
import uuid
import time
import logging
import random
from typing import Dict, Any, List, Optional
from datetime import datetime

# Configure logging
logger = logging.getLogger(__name__)

# In-memory storage for execution results (in a real system, this would be a database)
execution_results = {}

def create_feedback_service() -> Flask:
    """
    Create and configure the feedback service Flask app
    """
    app = Flask(__name__)

    @app.route('/feedback/<execution_id>', methods=['GET'])
    def get_execution_feedback(execution_id: str):
        """
        Get feedback about a completed action execution
        API Contract: GET /feedback/{execution_id}
        """
        try:
            # Look up execution result
            if execution_id not in execution_results:
                # If not found in memory, create a simulated result
                execution_result = generate_simulated_feedback(execution_id)
                execution_results[execution_id] = execution_result
            else:
                execution_result = execution_results[execution_id]

            # Prepare response based on contract
            response = {
                "execution_id": execution_id,
                "overall_status": execution_result['overall_status'],
                "robot_state_before": execution_result['robot_state_before'],
                "robot_state_after": execution_result['robot_state_after'],
                "success_metrics": execution_result['success_metrics'],
                "natural_language_feedback": execution_result['natural_language_feedback']
            }

            logger.info(f"Feedback retrieved for execution {execution_id}")

            return jsonify(response)

        except Exception as e:
            logger.error(f"Error retrieving feedback for execution {execution_id}: {e}")
            return jsonify({
                "error_code": "FEEDBACK_RETRIEVAL_FAILED",
                "message": f"Could not retrieve feedback: {str(e)}",
                "execution_id": execution_id
            }), 500

    @app.route('/feedback', methods=['POST'])
    def submit_execution_result():
        """
        Submit execution results for feedback processing
        (Not in original contract but useful for system operation)
        """
        try:
            data = request.get_json()

            if not data:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "Request body must be JSON"
                }), 400

            execution_id = data.get('execution_id')
            if not execution_id:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "execution_id is required"
                }), 400

            # Store the execution result
            execution_results[execution_id] = process_execution_result(data)

            return jsonify({
                "execution_id": execution_id,
                "status": "received"
            })

        except Exception as e:
            logger.error(f"Error submitting execution result: {e}")
            return jsonify({
                "error_code": "SUBMISSION_FAILED",
                "message": f"Could not submit result: {str(e)}"
            }), 500

    def generate_simulated_feedback(execution_id: str) -> Dict[str, Any]:
        """
        Generate simulated feedback for an execution ID
        """
        # Generate random robot states
        robot_state_before = {
            "position": {
                "x": round(random.uniform(-5.0, 5.0), 2),
                "y": round(random.uniform(-5.0, 5.0), 2),
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": round(random.uniform(-3.14, 3.14), 2),
                "w": 1.0
            },
            "gripper": random.choice(["open", "closed"]),
            "battery_level": round(random.uniform(20, 100), 1)
        }

        # Generate final state based on some movement
        robot_state_after = {
            "position": {
                "x": round(robot_state_before["position"]["x"] + random.uniform(-2.0, 2.0), 2),
                "y": round(robot_state_before["position"]["y"] + random.uniform(-2.0, 2.0), 2),
                "z": 0.0
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": round(random.uniform(-3.14, 3.14), 2),
                "w": 1.0
            },
            "gripper": random.choice(["open", "closed"]),
            "battery_level": max(0, round(robot_state_before["battery_level"] - random.uniform(0, 5), 1))
        }

        # Determine overall status based on simulated action results
        action_success_rate = random.random()
        if action_success_rate > 0.8:
            overall_status = "completed"
        elif action_success_rate > 0.5:
            overall_status = "partial"
        else:
            overall_status = "failed"

        # Calculate success metrics
        success_metrics = {
            "accuracy": round(min(1.0, action_success_rate + 0.1), 2),
            "efficiency": round(random.uniform(0.6, 1.0), 2),
            "safety_compliance": random.choice([True, True, True, True, False])  # 80% safety compliance
        }

        # Generate natural language feedback
        if overall_status == "completed":
            natural_language_feedback = "All actions completed successfully. Robot performed tasks as requested with high accuracy."
        elif overall_status == "partial":
            natural_language_feedback = "Some actions completed successfully, but some failed. Robot partially achieved the goal."
        else:
            natural_language_feedback = "Execution failed. Robot could not complete the requested tasks. Please check the command or environment."

        return {
            "execution_id": execution_id,
            "overall_status": overall_status,
            "robot_state_before": robot_state_before,
            "robot_state_after": robot_state_after,
            "success_metrics": success_metrics,
            "natural_language_feedback": natural_language_feedback,
            "timestamp": time.time()
        }

    def process_execution_result(data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process execution result data and generate feedback
        """
        execution_id = data.get('execution_id', str(uuid.uuid4()))
        action_results = data.get('action_results', [])
        robot_state_before = data.get('robot_state_before', {})
        robot_state_after = data.get('robot_state_after', {})

        # Calculate success metrics based on action results
        if action_results:
            successful_actions = sum(1 for result in action_results if result.get('status') == 'succeeded')
            success_rate = successful_actions / len(action_results)
        else:
            success_rate = 1.0

        overall_status = "completed" if success_rate == 1.0 else ("partial" if success_rate > 0.5 else "failed")

        success_metrics = {
            "accuracy": round(success_rate, 2),
            "efficiency": round(random.uniform(0.5, 1.0), 2),
            "safety_compliance": data.get('safety_compliance', True)
        }

        # Generate natural language feedback based on results
        if overall_status == "completed":
            feedback = "All actions completed successfully. The robot performed all requested tasks with high accuracy."
        elif overall_status == "partial":
            feedback = f"Partially successful execution. {successful_actions}/{len(action_results)} actions completed successfully."
        else:
            feedback = "Execution failed. The robot could not complete the requested tasks. Please review the action sequence or environment."

        return {
            "execution_id": execution_id,
            "overall_status": overall_status,
            "robot_state_before": robot_state_before or {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "gripper": "open",
            },
            "robot_state_after": robot_state_after or {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "gripper": "open",
            },
            "success_metrics": success_metrics,
            "natural_language_feedback": feedback,
            "timestamp": time.time()
        }

    @app.route('/health', methods=['GET'])
    def health_check():
        """
        Health check endpoint
        """
        return jsonify({
            "status": "healthy",
            "service": "simulation-feedback",
            "timestamp": time.time()
        })

    return app


# For standalone execution
if __name__ == '__main__':
    import os
    from dotenv import load_dotenv

    # Load environment variables
    load_dotenv()

    # Create and run the feedback service
    feedback_service = create_feedback_service()

    # Get port from environment or default to 5000
    port = int(os.getenv('FEEDBACK_SERVICE_PORT', 5004))

    # Enable debug mode if specified
    debug_mode = os.getenv('FEEDBACK_SERVICE_DEBUG', 'false').lower() == 'true'

    logger.info(f"Starting Feedback Service on port {port}")
    feedback_service.run(host='0.0.0.0', port=port, debug=debug_mode)