# Quickstart: ROS 2 Nervous System Module

## Getting Started

This quickstart guide helps you get up and running with the ROS 2 Nervous System module content.

### Prerequisites

- Ubuntu 22.04 or compatible Linux system
- ROS 2 Humble Hawksbill installed
- Python 3.10 or later
- Git for version control
- Basic command line familiarity

### Setup Instructions

1. **Install ROS 2 Humble** (if not already installed):
   ```bash
   # Add ROS 2 repository and install
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-rosdep2
   source /opt/ros/humble/setup.bash
   ```

2. **Set up your ROS 2 workspace**:
   ```bash
   mkdir -p ~/ros2_workspace/src
   cd ~/ros2_workspace
   colcon build
   source install/setup.bash
   ```

3. **Navigate to the textbook content**:
   ```bash
   cd docs/module1/
   ```

### Learning Path

1. Start with `ros2-foundations.md` to understand core ROS 2 concepts
2. Proceed to `ros2-environment.md` to set up your development environment
3. Continue with `ros2-communication.md` to learn about node communication
4. Complete with `exercises-project.md` to apply your knowledge

### Running Code Examples

1. Each code example is in the `examples/` directory
2. Navigate to the example directory
3. Source your ROS 2 environment: `source ~/ros2_workspace/install/setup.bash`
4. Run the example: `python3 example_name.py`

### Validation

All code examples have been validated in ROS 2 Humble simulation environment. If you encounter issues:
1. Verify your ROS 2 installation
2. Check that you've sourced the setup.bash file
3. Ensure you're using Python 3.10+
4. Refer to the specific chapter for detailed setup instructions