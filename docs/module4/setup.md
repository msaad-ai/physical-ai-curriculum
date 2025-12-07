# VLA Module Setup Guide

## API Configuration

### OpenAI API Setup

To use Whisper for voice-to-text conversion, you'll need an OpenAI API key:

1. Go to [OpenAI API Keys](https://platform.openai.com/api-keys)
2. Create a new secret key
3. Save the key in a `.env` file in your project root:

```bash
OPENAI_API_KEY="your-api-key-here"
```

### Anthropic Claude API Setup

To use Claude for cognitive planning and action mapping, you'll need an Anthropic API key:

1. Go to [Anthropic API Keys](https://console.anthropic.com/)
2. Create a new API key
3. Add it to your `.env` file:

```bash
ANTHROPIC_API_KEY="your-anthropic-api-key-here"
```

## Environment Configuration

Create a `.env` file in your project root with the following content:

```bash
# API Keys
OPENAI_API_KEY="your-openai-api-key-here"
ANTHROPIC_API_KEY="your-anthropic-api-key-here"

# Configuration
ROS_DOMAIN_ID=4  # Domain ID for VLA module
VLA_DEBUG=true   # Enable debug mode for development
```

## Python Dependencies

Install the required Python dependencies:

```bash
pip install -r docs/module4/requirements.txt
```

## ROS 2 Environment

Make sure ROS 2 Humble is properly installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

For simulation, ensure Gazebo is installed and accessible:

```bash
gz sim --version
```

## Simulation Environment Setup

### Gazebo Installation

For Ubuntu 22.04 with ROS 2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-ignition-*
```

### Robot Model Setup

1. Create a humanoid robot model in URDF format:
   - Place URDF files in `~/.gazebo/models/humanoid_robot/`
   - Include visual, collision, and joint definitions

2. Or use existing models like:
   - ROS-Industrial robot models
   - Fetch Robotics models
   - Custom humanoid models

### Simulation Configuration

Create a launch file for your simulation environment:

```xml
<launch>
  <include file="$(find-pkg-share gazebo_ros)/launch/gz_sim.launch.py">
    <arg name="gz_args" value="-r empty.sdf"/>
  </include>

  <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher" args="$(find-pkg-share your_robot_description)/urdf/robot.urdf"/>

  <node name="spawn_entity" pkg="gazebo_ros" exec="spawn_entity.py" args="-topic robot_description -entity humanoid_robot"/>
</launch>
```

### Testing Simulation

Launch your simulation environment:

```bash
ros2 launch your_simulation_package simulation.launch.py
```

Verify the robot is visible in Gazebo and can receive commands:

```bash
# Check available topics
ros2 topic list | grep cmd

# Send a simple command
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

## Testing the Setup

To verify your setup is working correctly:

```bash
# Test API access
python -c "import openai; print('OpenAI import successful')"
python -c "import anthropic; print('Anthropic import successful')"

# Test ROS 2 access
python -c "import rclpy; print('ROS 2 import successful')"

# Test simulation environment
gz sim --version
```