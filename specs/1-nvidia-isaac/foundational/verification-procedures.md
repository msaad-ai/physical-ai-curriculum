# Isaac Sim Environment Verification Procedures

## Pre-Installation Verification

### System Compatibility Check
1. Verify GPU compute capability:
   ```bash
   nvidia-smi
   # Look for GPU model and check NVIDIA documentation for compute capability
   ```

2. Check available system resources:
   ```bash
   # Check RAM
   free -h

   # Check available disk space
   df -h

   # Check CPU cores
   nproc
   ```

3. Verify NVIDIA driver and CUDA installation:
   ```bash
   # Check driver version
   nvidia-smi

   # Check CUDA version
   nvcc --version
   ```

## Post-Installation Verification

### Isaac Sim Verification
1. **Basic Launch Test**:
   ```bash
   # Launch Isaac Sim in headless mode
   isaac-sim --no-window --headless --/isaac/enable_navmesh_generation=False
   ```

2. **Python API Verification**:
   ```bash
   python3 -c "import omni; print('Isaac Sim Python API accessible')"
   ```

3. **Basic Scene Loading**:
   ```bash
   # Test with a simple scene
   isaac-sim --summary-cache-clear --no-window --/app/window/dockInParent=false --/app/window/enabled=false --/isaac/enable_navmesh_generation=False
   ```

### Isaac ROS Verification
1. **Environment Setup Verification**:
   ```bash
   # Source ROS 2 and Isaac ROS
   source /opt/ros/humble/setup.bash
   source /opt/isaac/isaac_ros-dev/setup.bash

   # Verify packages are available
   ros2 pkg list | grep isaac
   ```

2. **Basic Node Verification**:
   ```bash
   # Check Isaac ROS nodes
   ros2 node list | grep isaac
   ```

3. **Topic Communication Verification**:
   ```bash
   # Check available topics
   ros2 topic list

   # Echo a common topic (e.g., clock)
   ros2 topic echo /clock builtin_interfaces/msg/Time --field stamp
   ```

## Simulation Environment Verification

### Robot Model Loading
1. **Basic Robot Model Test**:
   - Launch Isaac Sim
   - Load a simple robot model (e.g., Carter or ALOHA)
   - Verify the model appears correctly in the scene

2. **Sensor Configuration Test**:
   - Add basic sensors to the robot (camera, IMU, LIDAR)
   - Verify sensors are properly configured
   - Check that sensor data is being published

### Perception Pipeline Verification
1. **VSLAM Pipeline Test**:
   - Launch Isaac Sim with a robot equipped with camera and IMU
   - Start the VSLAM pipeline
   - Move the robot and verify map creation and localization

2. **Object Detection Test**:
   - Set up a scene with recognizable objects
   - Launch object detection pipeline
   - Verify objects are detected and classified correctly

### Navigation Pipeline Verification
1. **Nav2 Integration Test**:
   - Launch Nav2 stack for Isaac Sim
   - Set up costmaps with Isaac Sim sensor data
   - Verify path planning and execution

2. **Obstacle Avoidance Test**:
   - Create a scene with obstacles
   - Plan a path through the environment
   - Verify robot successfully avoids obstacles

## Performance Verification

### Resource Utilization Check
1. **Monitor GPU Usage**:
   ```bash
   # Monitor GPU utilization during simulation
   watch -n 1 nvidia-smi
   ```

2. **Monitor CPU and Memory**:
   ```bash
   # Monitor system resources
   htop
   # Or use system monitor tools
   ```

### Simulation Stability Test
1. **Long-Running Simulation**:
   - Run a basic simulation for 10-15 minutes
   - Monitor for crashes or performance degradation
   - Check for memory leaks

2. **Multi-Process Test**:
   - Run Isaac Sim with multiple ROS 2 nodes
   - Verify stable communication between processes
   - Monitor resource usage

## Troubleshooting Verification Steps

### Common Issues and Checks
1. **Isaac Sim Won't Launch**:
   - Verify GPU drivers and CUDA installation
   - Check system requirements
   - Try headless mode to isolate graphics issues

2. **ROS 2 Nodes Not Communicating**:
   - Verify ROS_DOMAIN_ID consistency
   - Check network configuration
   - Confirm Isaac ROS packages are properly sourced

3. **Perception Pipeline Not Processing**:
   - Verify sensor data publication
   - Check Isaac ROS perception packages
   - Confirm computational resources adequacy

### Final Verification Checklist
- [ ] Isaac Sim launches successfully
- [ ] Isaac ROS packages accessible
- [ ] Robot model loads correctly
- [ ] Sensors configured and publishing data
- [ ] Perception pipeline operational
- [ ] Navigation system functional
- [ ] All exercises from Module 3 run successfully
- [ ] Performance meets requirements (stable frame rate, no crashes)
- [ ] Documentation examples execute without errors