#!/usr/bin/env python3
"""
Synthetic Data Generation Script for NVIDIA Isaac Sim and Isaac ROS

This script demonstrates how to generate synthetic training data for AI models
using Isaac Sim's simulation capabilities and Isaac ROS perception nodes.

The script will:
1. Connect to Isaac Sim
2. Configure sensors to collect data
3. Execute predefined robot behaviors to generate diverse scenarios
4. Save collected data in standard formats for AI training
"""

import os
import json
import time
import numpy as np
import cv2
from datetime import datetime
import argparse
import subprocess
import yaml

try:
    import carb
    import omni
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.utils.prims import get_prim_at_path
    from omni.isaac.core.utils.nucleus import get_assets_root_path
    from omni.isaac.core.utils.viewports import set_camera_view
    from omni.isaac.sensor import Camera
    import omni.replicator.core as rep
except ImportError:
    print("Isaac Sim Python modules not available. This script is meant to run within Isaac Sim.")
    print("Run this script using: python generate_synthetic_data.py")


class SyntheticDataGenerator:
    """
    A class to generate synthetic data for AI model training using Isaac Sim.
    """

    def __init__(self, output_dir="synthetic_data", num_scenarios=10):
        """
        Initialize the synthetic data generator.

        Args:
            output_dir (str): Directory to save generated data
            num_scenarios (int): Number of different scenarios to generate
        """
        self.output_dir = output_dir
        self.num_scenarios = num_scenarios
        self.world = None
        self.cameras = []
        self.data_counter = 0

        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "metadata"), exist_ok=True)

        print(f"Initialized synthetic data generator. Output directory: {output_dir}")

    def setup_world(self):
        """
        Set up the Isaac Sim world with a humanoid robot and environment.
        """
        print("Setting up Isaac Sim world...")

        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)

        # Add a ground plane
        self.world.scene.add_default_ground_plane()

        # Add a simple humanoid robot (you would load your specific robot model here)
        # For this example, we'll use a simple cube as placeholder
        from omni.isaac.core.objects import DynamicCuboid
        robot = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Robot",
                name="robot",
                position=np.array([0, 0, 1.0]),
                size=0.5,
                color=np.array([0.0, 0.0, 1.0])
            )
        )

        # Add some objects to detect
        for i in range(5):
            obj = self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Object_{i}",
                    name=f"object_{i}",
                    position=np.array([2.0 + i, 0, 0.5]),
                    size=0.3,
                    color=np.array([1.0, 0.0, 0.0]) if i % 2 == 0 else np.array([0.0, 1.0, 0.0])
                )
            )

        # Add a camera sensor
        camera = Camera(
            prim_path="/World/Camera",
            position=np.array([3.0, 0.0, 2.0]),
            look_at_target=np.array([0, 0, 1.0])
        )
        camera.initialize()
        self.cameras.append(camera)

        print("World setup complete.")

    def capture_data(self, scenario_id, step_id):
        """
        Capture sensor data for the current scenario and step.

        Args:
            scenario_id (int): Current scenario ID
            step_id (int): Current step ID within the scenario
        """
        print(f"Capturing data for scenario {scenario_id}, step {step_id}")

        # Step the world to update sensors
        self.world.step(render=True)

        # Capture data from each camera
        for i, camera in enumerate(self.cameras):
            # Get RGB image
            rgb_image = camera.get_rgb()

            # Get depth information
            depth_data = camera.get_depth()

            # Get segmentation data (if available)
            try:
                seg_data = camera.get_segmentation()
            except:
                seg_data = None

            # Generate unique filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename_base = f"{timestamp}_scenario{scenario_id}_step{step_id}_cam{i}"

            # Save RGB image
            image_path = os.path.join(self.output_dir, "images", f"{filename_base}.png")
            if rgb_image is not None:
                # Convert from Isaac Sim format to OpenCV format
                rgb_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                cv2.imwrite(image_path, rgb_bgr)

            # Save depth data
            depth_path = os.path.join(self.output_dir, "images", f"{filename_base}_depth.npy")
            if depth_data is not None:
                np.save(depth_path, depth_data)

            # Save segmentation data if available
            if seg_data is not None:
                seg_path = os.path.join(self.output_dir, "images", f"{filename_base}_seg.npy")
                np.save(seg_path, seg_data)

            # Create metadata
            metadata = {
                "timestamp": timestamp,
                "scenario_id": scenario_id,
                "step_id": step_id,
                "camera_id": i,
                "image_path": image_path,
                "depth_path": depth_path,
                "camera_pose": {
                    "position": camera.get_world_pose()[0].tolist(),
                    "orientation": camera.get_world_pose()[1].tolist()
                },
                "camera_intrinsics": camera.get_intrinsics().tolist() if camera.get_intrinsics() is not None else None
            }

            # Save metadata
            metadata_path = os.path.join(self.output_dir, "metadata", f"{filename_base}_meta.json")
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)

            print(f"Saved data: {image_path}")

        self.data_counter += 1

    def execute_scenario(self, scenario_id):
        """
        Execute a specific scenario by moving objects and changing the environment.

        Args:
            scenario_id (int): ID of the scenario to execute
        """
        print(f"Executing scenario {scenario_id}")

        # Move the robot to a new position based on scenario ID
        robot_position = np.array([
            0.5 * np.cos(scenario_id * 0.5),
            0.5 * np.sin(scenario_id * 0.5),
            1.0
        ])

        # Move objects to new positions
        for i in range(5):
            obj_position = np.array([
                2.0 + (i * 0.5) + (scenario_id * 0.1),
                (i - 2) * 0.5,
                0.5 + (scenario_id % 3) * 0.2
            ])

            # Update object position (this is a simplified example)
            # In a real implementation, you would use Isaac Sim's API to move objects
            pass

        # Capture data at different steps within the scenario
        for step in range(10):  # Capture 10 frames per scenario
            self.capture_data(scenario_id, step)

            # Move robot slightly for next frame (simplified movement)
            # In a real implementation, you would use proper robot control
            self.world.step(render=True)
            time.sleep(0.1)  # Small delay to allow for rendering

    def generate_data(self):
        """
        Main method to generate synthetic data across multiple scenarios.
        """
        print(f"Starting synthetic data generation for {self.num_scenarios} scenarios...")

        # Setup the world
        self.setup_world()

        # Start the world
        self.world.reset()

        # Generate data for each scenario
        for scenario_id in range(self.num_scenarios):
            self.execute_scenario(scenario_id)

        print(f"Synthetic data generation complete! Generated {self.data_counter} data samples.")

        # Create a summary file
        summary = {
            "total_samples": self.data_counter,
            "scenarios": self.num_scenarios,
            "timestamp": datetime.now().isoformat(),
            "output_directory": self.output_dir,
            "camera_count": len(self.cameras)
        }

        summary_path = os.path.join(self.output_dir, "summary.json")
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)

        print(f"Summary saved to: {summary_path}")

    def cleanup(self):
        """
        Clean up resources.
        """
        if self.world:
            self.world.clear()
            print("World cleared.")


def main():
    """
    Main function to run the synthetic data generator.
    """
    parser = argparse.ArgumentParser(description="Generate synthetic data using Isaac Sim")
    parser.add_argument("--output_dir", type=str, default="synthetic_data",
                        help="Directory to save generated data")
    parser.add_argument("--num_scenarios", type=int, default=10,
                        help="Number of different scenarios to generate")
    parser.add_argument("--headless", action="store_true",
                        help="Run in headless mode (no GUI)")

    args = parser.parse_args()

    print("Starting synthetic data generation...")
    print(f"Output directory: {args.output_dir}")
    print(f"Number of scenarios: {args.num_scenarios}")
    print(f"Headless mode: {args.headless}")

    # Initialize the data generator
    generator = SyntheticDataGenerator(
        output_dir=args.output_dir,
        num_scenarios=args.num_scenarios
    )

    try:
        # Generate the synthetic data
        generator.generate_data()
    except KeyboardInterrupt:
        print("Data generation interrupted by user.")
    except Exception as e:
        print(f"Error during data generation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up resources
        generator.cleanup()


if __name__ == "__main__":
    main()