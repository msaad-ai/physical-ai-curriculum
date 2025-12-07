"""
Action Execution Service API

This module implements the Action Execution Service API as defined in the VLA API contract.
It handles executing validated action sequences on the robot in simulation.
"""

from flask import Flask, request, jsonify
import uuid
import time
import logging
from typing import Dict, Any, List, Tuple
import random

# Configure logging
logger = logging.getLogger(__name__)

def create_action_service() -> Flask:
    """
    Create and configure the action service Flask app
    """
    app = Flask(__name__)

    @app.route('/actions/execute', methods=['POST'])
    def execute_action_sequence():
        """
        Execute a validated action sequence on the robot
        API Contract: POST /actions/execute
        """
        start_time = time.time()

        try:
            # Parse request data
            data = request.get_json()

            if not data:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "Request body must be JSON",
                    "execution_id": str(uuid.uuid4())
                }), 400

            # Extract required fields
            plan_id = data.get('plan_id')
            action_sequence = data.get('action_sequence', [])
            simulation_mode = data.get('simulation_mode', True)

            # Validate required fields
            if not action_sequence:
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "action_sequence is required",
                    "execution_id": str(uuid.uuid4())
                }), 400

            if not isinstance(action_sequence, list):
                return jsonify({
                    "error_code": "INVALID_FORMAT",
                    "message": "action_sequence must be an array",
                    "execution_id": str(uuid.uuid4())
                }), 400

            # Create execution ID
            execution_id = str(uuid.uuid4())

            # Execute the action sequence
            action_results = []
            overall_status = "executing"

            try:
                for i, action in enumerate(action_sequence):
                    action_id = str(uuid.uuid4())

                    # Execute individual action
                    action_result = execute_single_action(action, simulation_mode)

                    action_result['action_id'] = action_id
                    action_result['action_index'] = i

                    action_results.append(action_result)

                    # If any action fails, mark overall status as failed
                    if action_result['status'] == 'failed':
                        overall_status = 'failed'
                        break
                else:
                    # If all actions completed successfully
                    overall_status = 'completed'

                # Calculate total execution time
                total_execution_time = time.time() - start_time

                # Log successful execution
                logger.info(f"Action sequence executed successfully for plan {plan_id}, execution_id: {execution_id}")

                return jsonify({
                    "execution_id": execution_id,
                    "status": overall_status,
                    "estimated_completion": time.time() + 0.1,  # Simple estimation
                    "action_results": action_results
                })

            except Exception as e:
                logger.error(f"Error executing action sequence: {e}")
                return jsonify({
                    "error_code": "EXECUTION_FAILED",
                    "message": f"Action execution error: {str(e)}",
                    "execution_id": execution_id
                }), 500

        except Exception as e:
            logger.error(f"Unexpected error in execute_action_sequence: {e}")
            execution_id = str(uuid.uuid4())
            return jsonify({
                "error_code": "API_ERROR",
                "message": f"Internal server error: {str(e)}",
                "execution_id": execution_id
            }), 500

    def execute_single_action(action: Dict[str, Any], simulation_mode: bool) -> Dict[str, Any]:
        """
        Execute a single action and return result
        """
        action_type = action.get('action_type', 'unknown')
        parameters = action.get('parameters', {})
        estimated_duration = action.get('estimated_duration', 1.0)

        logger.info(f"Executing action: {action_type} with params: {parameters}")

        # Simulate action execution
        start_time = time.time()

        # In simulation mode, we just simulate the action
        # In real mode, this would interface with actual robot controls
        if simulation_mode:
            # Simulate action execution time
            time.sleep(min(estimated_duration, 2.0))  # Cap at 2 seconds for simulation

            # Simulate success/failure based on action type
            success_probability = 0.95  # 95% success rate in simulation
            is_successful = random.random() < success_probability

            status = 'succeeded' if is_successful else 'failed'

            # Add some random execution time variation
            execution_time = estimated_duration * random.uniform(0.8, 1.2)
        else:
            # In real mode, we would execute actual robot commands
            # This is a placeholder for actual robot control code
            status = 'succeeded'  # Assume success in this simplified version
            execution_time = estimated_duration

        result = {
            "action_type": action_type,
            "status": status,
            "execution_time": round(execution_time, 3),
            "parameters_used": parameters
        }

        if status == 'failed':
            result["error_message"] = f"Action {action_type} failed during execution"

        return result

    @app.route('/health', methods=['GET'])
    def health_check():
        """
        Health check endpoint
        """
        return jsonify({
            "status": "healthy",
            "service": "action-execution",
            "timestamp": time.time()
        })

    return app


# For standalone execution
if __name__ == '__main__':
    import os
    from dotenv import load_dotenv

    # Load environment variables
    load_dotenv()

    # Create and run the action service
    action_service = create_action_service()

    # Get port from environment or default to 5000
    port = int(os.getenv('ACTION_SERVICE_PORT', 5003))

    # Enable debug mode if specified
    debug_mode = os.getenv('ACTION_SERVICE_DEBUG', 'false').lower() == 'true'

    logger.info(f"Starting Action Service on port {port}")
    action_service.run(host='0.0.0.0', port=port, debug=debug_mode)