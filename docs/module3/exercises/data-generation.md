# Data Generation for Training Documentation

## Overview

This document provides guidance on generating synthetic training data for AI models using NVIDIA Isaac Sim and Isaac ROS. Synthetic data generation is a crucial component of developing robust AI perception systems for robotics, allowing for the creation of diverse, labeled datasets without requiring physical hardware or real-world data collection.

## Importance of Synthetic Data Generation

### Benefits
- **Safety**: Generate diverse scenarios without risk to physical robots or environments
- **Cost-Effective**: Eliminate the need for expensive real-world data collection campaigns
- **Controlled Conditions**: Create specific scenarios with known ground truth
- **Scalability**: Generate large datasets quickly with consistent quality
- **Variety**: Create edge cases and rare scenarios that are difficult to capture in reality
- **Privacy**: No concerns about capturing sensitive real-world information

### Applications in Robotics
- Training object detection models
- Developing SLAM algorithms
- Testing navigation systems
- Validating sensor fusion techniques
- Creating simulation environments for reinforcement learning

## Isaac Sim Capabilities for Data Generation

### Replicator Framework
Isaac Sim includes NVIDIA's Replicator framework for synthetic data generation, which provides:

- **Procedural Scene Generation**: Automatically create diverse environments
- **Variation Engine**: Randomize materials, lighting, and object placement
- **Sensor Simulation**: Accurate simulation of cameras, LiDAR, IMU, and other sensors
- **Annotation Tools**: Automatic generation of ground truth labels
- **Domain Randomization**: Systematic variation of visual and physical properties

### Supported Data Types
- RGB images with segmentation masks
- Depth maps
- Point clouds from simulated LiDAR
- 3D bounding boxes
- Pose information
- Optical flow
- Normal maps

## Implementation Approach

### 1. Environment Setup
```python
import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.sensor import Camera

# Initialize replicator
rep.orchestrator._orchestrator = None  # Reset orchestrator if needed
rep.orchestrator.setup_camera()  # Setup default camera
```

### 2. Camera Configuration
```python
# Configure camera for data collection
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([3.0, 0.0, 2.0]),
    look_at_target=np.array([0, 0, 1.0])
)
camera.set_focal_length(24.0)  # Set focal length in mm
camera.set_horizontal_aperture(20.955)  # Set horizontal aperture in mm
camera.set_vertical_aperture(15.29)  # Set vertical aperture in mm
```

### 3. Annotation Configuration
```python
# Configure output annotations
with rep.new_layer():
    # Annotate with RGB, depth, and segmentation
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach([camera])

    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annotator.attach([camera])

    seg_annotator = rep.AnnotatorRegistry.get_annotator("instance_segmentation")
    seg_annotator.attach([camera])
```

### 4. Variation Engine Setup
```python
# Create variation function
def randomize_objects():
    # Randomize object positions
    with rep.randomizer:
        # Randomize positions of objects
        objects = rep.get.prims(prim_types=["Mesh"])
        with objects.randomize_position():
            positions = rep.distribution.uniform((-5, -5, 0), (5, 5, 0))
            rep.modify.pose(position=positions)

    # Randomize materials
    with rep.randomizer:
        # Apply random materials to objects
        materials = rep.get.materials()
        rep.randomizer.materials.assign(materials)

# Register variation function
rep.randomizer.register("randomize_objects", randomize_objects)
```

## Data Generation Pipeline

### 1. Basic Data Collection Script
```python
import numpy as np
import cv2
import json
import os
from datetime import datetime

class IsaacSimDataCollector:
    def __init__(self, output_dir="synthetic_data"):
        self.output_dir = output_dir
        self.frame_counter = 0

        # Create output directories
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "metadata"), exist_ok=True)

    def capture_frame(self, camera, frame_id):
        """Capture a single frame with all annotations"""
        # Get RGB image
        rgb_data = camera.get_rgb()

        # Get depth data
        depth_data = camera.get_depth()

        # Get segmentation data
        seg_data = camera.get_segmentation()

        # Get camera pose
        pose = camera.get_world_pose()

        # Save RGB image
        rgb_path = os.path.join(self.output_dir, "images", f"rgb_{frame_id:06d}.png")
        if rgb_data is not None:
            rgb_bgr = cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR)
            cv2.imwrite(rgb_path, rgb_bgr)

        # Save depth data
        depth_path = os.path.join(self.output_dir, "images", f"depth_{frame_id:06d}.npy")
        if depth_data is not None:
            np.save(depth_path, depth_data)

        # Save segmentation data
        seg_path = os.path.join(self.output_dir, "images", f"seg_{frame_id:06d}.npy")
        if seg_data is not None:
            np.save(seg_path, seg_data)

        # Create metadata
        metadata = {
            "frame_id": frame_id,
            "timestamp": datetime.now().isoformat(),
            "camera_pose": {
                "position": pose[0].tolist(),
                "orientation": pose[1].tolist()
            },
            "camera_intrinsics": camera.get_intrinsics().tolist()
        }

        meta_path = os.path.join(self.output_dir, "metadata", f"meta_{frame_id:06d}.json")
        with open(meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        return {
            "rgb_path": rgb_path,
            "depth_path": depth_path,
            "seg_path": seg_path,
            "meta_path": meta_path
        }
```

### 2. Domain Randomization Script
```python
def setup_domain_randomization():
    """Setup domain randomization for diverse data generation"""

    # Randomize lighting conditions
    lights = rep.get.light()
    with lights.randomize_color():
        rep.modify.color(color=rep.distribution.uniform((0.5, 0.5, 0.5), (1.0, 1.0, 1.0)))

    # Randomize background environments
    backgrounds = rep.get.prim_at_path("/World/Background")
    with backgrounds.randomize_material():
        materials = rep.utils.get_materials_from_directory("/Isaac/Props/Backgrounds")
        rep.randomizer.materials.assign(materials)

    # Randomize object textures
    objects = rep.get.prim_at_path("/World/Objects")
    with objects.randomize_material():
        materials = rep.utils.get_materials_from_directory("/Isaac/Environments/Simple_Rooms")
        rep.randomizer.materials.assign(materials)

    # Randomize camera parameters
    cameras = rep.get.camera()
    with cameras.randomize_focal_length():
        rep.modify.focal_length(rep.distribution.uniform(18.0, 50.0))
```

## Best Practices for Data Generation

### 1. Data Quality Assurance
- Validate that generated data matches expected format and range
- Check for artifacts or unrealistic features
- Verify that annotations are accurate and complete
- Ensure consistent naming conventions

### 2. Dataset Balance
- Ensure balanced representation of different object classes
- Vary lighting, backgrounds, and object poses systematically
- Include diverse scenarios and edge cases
- Maintain appropriate train/validation/test splits

### 3. Ground Truth Accuracy
- Use high-precision annotations from simulation
- Validate annotation quality against known ground truth
- Include uncertainty estimates where appropriate
- Document annotation methodology

### 4. Performance Optimization
- Batch processing to improve efficiency
- Use appropriate image resolutions for target applications
- Optimize scene complexity for generation speed
- Monitor resource usage during generation

## Data Format Standards

### Image Data
- RGB images: PNG or JPEG format, 8-bit or 16-bit
- Depth maps: NPY format with metric depth values
- Segmentation: NPY format with class IDs
- Normal maps: NPY format with unit normal vectors

### Annotation Format
```json
{
  "info": {
    "version": "1.0",
    "description": "Synthetic dataset for object detection",
    "date_created": "2025-12-07"
  },
  "images": [
    {
      "id": 1,
      "width": 640,
      "height": 480,
      "file_name": "rgb_000001.png",
      "camera_pose": {
        "position": [3.0, 0.0, 2.0],
        "orientation": [0.0, 0.0, 0.0, 1.0]
      }
    }
  ],
  "annotations": [
    {
      "id": 1,
      "image_id": 1,
      "category_id": 1,
      "bbox": [100, 150, 200, 180],  // [x, y, width, height]
      "area": 36000,
      "iscrowd": 0
    }
  ],
  "categories": [
    {
      "id": 1,
      "name": "red_cube",
      "supercategory": "object"
    }
  ]
}
```

## Integration with Training Pipelines

### 1. Data Preprocessing
```python
import torch
from torch.utils.data import Dataset
import numpy as np
import cv2

class IsaacSimDataset(Dataset):
    def __init__(self, data_dir, transform=None):
        self.data_dir = data_dir
        self.transform = transform
        self.data_list = self._load_data_list()

    def _load_data_list(self):
        """Load list of available data samples"""
        # Load from annotation file or directory structure
        pass

    def __len__(self):
        return len(self.data_list)

    def __getitem__(self, idx):
        sample = self.data_list[idx]

        # Load RGB image
        rgb_path = os.path.join(self.data_dir, "images", sample["rgb_file"])
        image = cv2.imread(rgb_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Load annotations
        annotations = sample["annotations"]

        # Apply transforms
        if self.transform:
            image, annotations = self.transform(image, annotations)

        return {
            "image": torch.from_numpy(image).float(),
            "annotations": annotations
        }
```

### 2. Training Script Integration
```python
def train_with_synthetic_data():
    """Example training script using synthetic data"""

    # Load synthetic dataset
    dataset = IsaacSimDataset("synthetic_data")
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Initialize model
    model = ObjectDetectionModel()

    # Training loop
    for epoch in range(num_epochs):
        for batch_idx, batch in enumerate(dataloader):
            # Forward pass
            outputs = model(batch["image"])

            # Compute loss
            loss = compute_loss(outputs, batch["annotations"])

            # Backward pass
            loss.backward()
            optimizer.step()

            # Log progress
            if batch_idx % 100 == 0:
                print(f"Epoch {epoch}, Batch {batch_idx}, Loss: {loss.item():.4f}")
```

## Validation and Quality Assessment

### 1. Cross-Validation with Real Data
- Test synthetic-trained models on real-world validation sets
- Compare performance metrics between synthetic and real data
- Identify domain gaps and adjust generation parameters

### 2. Quality Metrics
- Annotation accuracy verification
- Data diversity assessment
- Model performance comparison
- Domain gap analysis

### 3. Iterative Improvement
- Analyze failure cases to identify data generation improvements
- Adjust domain randomization parameters based on validation results
- Generate additional data for underperforming scenarios

## Troubleshooting Common Issues

### 1. Performance Issues
- Reduce scene complexity during generation
- Use lower resolution cameras for faster processing
- Batch generation operations
- Monitor GPU memory usage

### 2. Data Quality Issues
- Verify camera calibration parameters
- Check for lighting artifacts
- Validate annotation accuracy
- Ensure consistent coordinate frames

### 3. Compatibility Issues
- Ensure generated data format matches training pipeline requirements
- Validate file permissions and paths
- Check for missing dependencies
- Verify Isaac Sim version compatibility

## Advanced Techniques

### 1. Active Learning Integration
- Use model uncertainty to guide data generation
- Focus on scenarios where model performance is poor
- Prioritize generation of challenging examples

### 2. Physics-Based Data Augmentation
- Simulate different physical properties
- Add realistic noise models
- Include sensor-specific artifacts
- Model environmental conditions

### 3. Multi-Modal Data Generation
- Generate synchronized data from multiple sensors
- Ensure temporal consistency
- Validate cross-modal relationships
- Support sensor fusion training

## Conclusion

Synthetic data generation with Isaac Sim provides a powerful approach to creating high-quality training datasets for AI robotics applications. By following the guidelines and best practices outlined in this document, you can create diverse, accurate, and useful synthetic datasets that improve the performance and robustness of your AI models.

The key to successful synthetic data generation is to balance the need for diversity and realism with computational efficiency, while maintaining accurate ground truth annotations that enable effective model training.