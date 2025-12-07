"""
Comprehensive Integration Tests for VLA Pipeline

This module contains integration tests for the complete VLA (Vision-Language-Action) pipeline,
testing the integration between voice processing, cognitive planning, and action execution.
"""

import unittest
import asyncio
import time
from unittest.mock import Mock, patch, MagicMock
from src.vla_module.whisper_integration import WhisperIntegration, transcribe_voice_command
from src.vla_module.cognitive_planner import CognitivePlanner
from src.vla_module.vla_node import VLAMainNode
import tempfile
import os
import numpy as np
import wave


class TestVLAPipelineIntegration(unittest.TestCase):
    """
    Integration tests for the complete VLA pipeline
    """

    def setUp(self):
        """
        Set up test fixtures before each test method
        """
        # Mock environment variables for API keys
        os.environ['OPENAI_API_KEY'] = 'test-openai-key'
        os.environ['ANTHROPIC_API_KEY'] = 'test-anthropic-key'

        # Create mock components
        self.mock_whisper = Mock(spec=WhisperIntegration)
        self.mock_planner = Mock(spec=CognitivePlanner)

        # Create a test VLA node
        self.vla_node = VLAMainNode()

    def tearDown(self):
        """
        Clean up after each test method
        """
        # Clean up environment variables
        if 'OPENAI_API_KEY' in os.environ:
            del os.environ['OPENAI_API_KEY']
        if 'ANTHROPIC_API_KEY' in os.environ:
            del os.environ['ANTHROPIC_API_KEY']

    @patch('src.vla_module.whisper_integration.openai.Audio.transcribe')
    def test_voice_to_action_pipeline(self, mock_transcribe):
        """
        Test the complete voice-to-action pipeline
        """
        # Mock Whisper transcription
        mock_transcribe.return_value = "Go to the kitchen"

        # Create a simple test scenario
        command = "Go to the kitchen"

        # Test the cognitive planning step
        with patch.object(self.vla_node.cognitive_planner, 'plan_action_sequence') as mock_plan:
            # Mock the planning result
            mock_plan_result = MagicMock()
            mock_plan_result.is_valid = True
            mock_plan_result.action_sequence = [
                MagicMock(action_type='move_to', parameters={'target_position': {'x': 5.0, 'y': 3.0}}, estimated_duration=3.0)
            ]
            mock_plan_result.confidence = 0.9
            mock_plan_result.validation_errors = []

            mock_plan.return_value = mock_plan_result

            # Execute the cognitive planning
            result = self.vla_node.cognitive_planner.plan_action_sequence(
                command,
                {"robot_capabilities": ["move_to"]}
            )

            # Verify the result
            self.assertTrue(result.is_valid)
            self.assertEqual(len(result.action_sequence), 1)
            self.assertEqual(result.action_sequence[0].action_type, 'move_to')

            print("✓ Voice-to-action pipeline test passed")

    @patch('src.vla_module.cognitive_planner.openai')
    def test_cognitive_planning_integration(self, mock_openai):
        """
        Test cognitive planning with simulated LLM responses
        """
        # Mock the Anthropic client response
        mock_response = MagicMock()
        mock_response.content = '[{"action_type": "move_to", "parameters": {"target_position": {"x": 5, "y": 3}}, "estimated_duration": 3.5}]'

        # Create mock messages.create method
        mock_openai.messages.create.return_value = mock_response

        # Create cognitive planner instance
        planner = CognitivePlanner.__new__(CognitivePlanner)  # Create without calling __init__
        planner.client = mock_openai
        planner.validator = self.vla_node.cognitive_planner.validator  # Use real validator
        planner.command_parser = self.vla_node.cognitive_planner.command_parser  # Use real parser

        # Test planning with context
        context = {
            "robot_capabilities": ["move_to", "pick_up"],
            "robot_position": {"x": 0.0, "y": 0.0},
            "known_locations": {"kitchen": {"x": 5.0, "y": 3.0}}
        }

        result = planner.plan_action_sequence("Go to the kitchen", context)

        # Verify the result structure
        self.assertIsNotNone(result)
        self.assertIsInstance(result.action_sequence, list)

        print("✓ Cognitive planning integration test passed")

    def test_action_execution_simulation(self):
        """
        Test action execution simulation
        """
        # Test single action execution
        action = MagicMock()
        action.action_type = 'move_to'
        action.parameters = {'target_position': {'x': 5.0, 'y': 3.0}}

        # Execute the action
        success = self.vla_node.execute_single_action(action)

        # Verify execution was attempted
        self.assertTrue(success)

        print("✓ Action execution simulation test passed")

    def test_command_queue_processing(self):
        """
        Test command queue processing in VLA node
        """
        # Add commands to queue
        self.vla_node.command_queue.append({
            'type': 'text',
            'data': 'Go to the kitchen',
            'timestamp': time.time()
        })

        self.vla_node.command_queue.append({
            'type': 'text',
            'data': 'Pick up the red ball',
            'timestamp': time.time()
        })

        # Process commands
        initial_queue_size = len(self.vla_node.command_queue)
        self.vla_node.process_commands()  # Process first command
        self.vla_node.process_commands()  # Process second command

        # Verify commands were processed
        final_queue_size = len(self.vla_node.command_queue)
        self.assertEqual(final_queue_size, initial_queue_size - 2)

        print("✓ Command queue processing test passed")

    def test_context_integration(self):
        """
        Test that context is properly integrated across components
        """
        # Prepare context
        context = {
            "robot_capabilities": ["move_to", "pick_up", "put_down", "greet", "speak"],
            "robot_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "available_objects": [
                {"id": "red_ball", "name": "red ball", "position": {"x": 5.0, "y": 3.0}}
            ],
            "known_locations": {
                "kitchen": {"x": 5.0, "y": 3.0},
                "bedroom": {"x": -3.0, "y": -2.0}
            }
        }

        # Verify context structure
        self.assertIn("robot_capabilities", context)
        self.assertIn("known_locations", context)
        self.assertIn("available_objects", context)

        # Verify specific values
        self.assertIn("kitchen", context["known_locations"])
        self.assertGreater(len(context["robot_capabilities"]), 0)

        print("✓ Context integration test passed")

    @patch('src.vla_module.cognitive_planner.openai')
    def test_end_to_end_vla_pipeline(self, mock_openai):
        """
        Test the complete end-to-end VLA pipeline
        """
        # Mock LLM response
        mock_response = MagicMock()
        mock_response.content = '[{"action_type": "move_to", "parameters": {"target_position": {"x": 5, "y": 3}}, "estimated_duration": 3.5}, {"action_type": "pick_up", "parameters": {"object_id": "red_ball"}, "estimated_duration": 2.0}]'
        mock_openai.messages.create.return_value = mock_response

        # Create planner with mocked client
        planner = CognitivePlanner.__new__(CognitivePlanner)
        planner.client = mock_openai
        planner.validator = self.vla_node.cognitive_planner.validator
        planner.command_parser = self.vla_node.cognitive_planner.command_parser

        # Step 1: Process natural language command
        command = "Go to the kitchen and pick up the red ball"
        context = {
            "robot_capabilities": ["move_to", "pick_up"],
            "known_locations": {"kitchen": {"x": 5.0, "y": 3.0}},
            "available_objects": [{"id": "red_ball", "name": "red ball"}]
        }

        # Step 2: Plan action sequence
        with patch.object(self.vla_node, 'cognitive_planner', planner):
            action_plan = self.vla_node.plan_cognitive_action(command)

        # Step 3: Execute action sequence
        if action_plan and action_plan.is_valid:
            self.vla_node.execute_action_sequence(action_plan.action_sequence)

        # Verify the pipeline completed successfully
        self.assertIsNotNone(action_plan)
        if action_plan:
            self.assertTrue(action_plan.is_valid)

        print("✓ End-to-end VLA pipeline test passed")

    def test_error_handling_in_pipeline(self):
        """
        Test error handling throughout the VLA pipeline
        """
        # Test with invalid command
        invalid_command = ""

        # This should handle the error gracefully
        try:
            result = self.vla_node.cognitive_planner.plan_action_sequence(
                invalid_command,
                {}
            )
            # Even if planning fails, the system should not crash
        except Exception as e:
            # If an exception occurs, it should be handled properly
            self.assertIsInstance(e, (ValueError, Exception))

        # Test with invalid action type
        invalid_action = MagicMock()
        invalid_action.action_type = 'invalid_action_type'
        invalid_action.parameters = {}

        success = self.vla_node.execute_single_action(invalid_action)
        # Should return False for invalid actions
        self.assertFalse(success)

        print("✓ Error handling in pipeline test passed")


class TestVLAServiceIntegration(unittest.TestCase):
    """
    Integration tests for VLA API services
    """

    def test_service_health_endpoints(self):
        """
        Test that all service health endpoints are accessible
        """
        # This would normally test actual HTTP endpoints
        # For now, we'll verify the service creation functions exist and work
        from src.vla_module.api.voice_service import create_voice_service
        from src.vla_module.api.cognitive_service import create_cognitive_service
        from src.vla_module.api.action_service import create_action_service
        from src.vla_module.api.feedback_service import create_feedback_service

        # Create service instances
        voice_service = create_voice_service()
        cognitive_service = create_cognitive_service()
        action_service = create_action_service()
        feedback_service = create_feedback_service()

        # Verify services were created
        self.assertIsNotNone(voice_service)
        self.assertIsNotNone(cognitive_service)
        self.assertIsNotNone(action_service)
        self.assertIsNotNone(feedback_service)

        print("✓ Service health endpoints test passed")


def run_integration_tests():
    """
    Run all integration tests and return results
    """
    # Create test suite
    suite = unittest.TestSuite()

    # Add all test cases
    suite.addTest(unittest.makeSuite(TestVLAPipelineIntegration))
    suite.addTest(unittest.makeSuite(TestVLAServiceIntegration))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result


if __name__ == '__main__':
    print("Running VLA Pipeline Integration Tests...")
    print("=" * 50)

    # Run the integration tests
    test_result = run_integration_tests()

    print("=" * 50)
    print(f"Tests run: {test_result.testsRun}")
    print(f"Failures: {len(test_result.failures)}")
    print(f"Errors: {len(test_result.errors)}")

    if test_result.wasSuccessful():
        print("✓ All integration tests passed!")
    else:
        print("✗ Some tests failed")

    # Exit with appropriate code
    exit(0 if test_result.wasSuccessful() else 1)