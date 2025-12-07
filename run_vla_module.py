#!/usr/bin/env python3
"""
VLA Module Startup Script

This script provides a simple way to run the VLA (Vision-Language-Action) module.
"""

import sys
import os

# Add the src directory to the path so we can import vla_module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def main():
    print("Starting VLA (Vision-Language-Action) Module...")
    print("Checking dependencies...")

    # Check for required dependencies
    try:
        import rclpy
        print("✓ ROS2 Python client (rclpy) found")
    except ImportError:
        print("✗ ROS2 Python client (rclpy) not found!")
        print("Please install ROS2 and ensure 'rclpy' is available in your Python environment.")
        print("Visit: https://docs.ros.org/en/humble/Installation.html")
        return 1

    try:
        import openai
        print("✓ OpenAI library found")
    except ImportError:
        print("⚠ OpenAI library not found (optional dependency)")

    try:
        import flask
        print("✓ Flask library found")
    except ImportError:
        print("⚠ Flask library not found (optional dependency)")

    print("\nStarting VLA Main Node...")

    # Import and run the VLA node
    try:
        from src.vla_module.vla_node import main as vla_main
        print("VLA Module is ready!")
        print("Listening for voice and text commands on ROS2 topics...")
        vla_main()
    except Exception as e:
        print(f"Error starting VLA module: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())