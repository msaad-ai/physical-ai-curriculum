# Basic Simulation Environment Configuration

## Gazebo Environment Setup

### Default World Configuration
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- GUI configuration -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>4.9243100000000002 -4.3678900000000003 2.0999000000000001 0.0000000000000000 0.2306726471660684 2.4583304054844045</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

### Educational Environment Configuration
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="edu_environment">
    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Ground plane with grid -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
            <bounce/>
            <contact>
              <ode/>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <uri>file://media/materials/textures</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Physics configuration for educational purposes -->
    <physics type="ode" name="default_physics" default="0">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- GUI configuration -->
    <gui fullscreen="0">
      <plugin name="world_stats" filename="libgazebo_ros_world_stats.so"/>
      <camera name="user_camera">
        <pose>5 -5 2 0 0.5 1.5708</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

## Unity Environment Setup

### Scene Configuration Template
```
# Unity Scene Configuration for Robotics Simulation
# This represents the basic structure of a Unity scene for robotics

Scene: RoboticsEnvironment
  - Main Camera (with appropriate settings for robotics)
  - Directional Light (simulating sun)
  - Ground Plane (with grid material for orientation)
  - Robot Prefab (loaded from URDF)
  - Lighting Setup (realistic but performant)
  - Post-Processing Stack (for visual enhancement)
  - Event System (for UI interactions)

Camera Settings:
  - Field of View: 60
  - Clipping Planes: Near 0.1, Far 1000
  - Render Pipeline: Built-in or URP (for performance)

Lighting Settings:
  - Intensity: 1.0
  - Shadow Type: Soft Shadows
  - Color: White (255, 255, 255)
```

### Physics Configuration for Unity
```
# Unity Physics Configuration for Robotics Simulation

Physics Settings:
  - Gravity: (0, -9.81, 0)
  - Default Material: Bounce Combine = Average, Friction Combine = Average
  - Solver Iteration Count: 6 (performance vs accuracy balance)
  - Solver Velocity Iteration Count: 1
  - Sleep Threshold: 0.005
  - Default Contact Offset: 0.01
  - Default Solver Resolution: 0.02

For educational purposes, these settings provide a good balance between realistic physics simulation and computational performance.
```

## ROS/ROS2 Integration Configuration

### Launch File for Complete Simulation
```xml
<?xml version="1.0"?>
<launch>
  <!-- Start Gazebo with educational environment -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find your_robot_description)/worlds/edu_environment.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Load robot description -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find your_robot_description)/urdf/robot.urdf.xacro'" />

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_entity.py" respawn="false" output="screen"
    args="-entity robot -x 0 -y 0 -z 1 -file $(find your_robot_description)/urdf/robot.urdf"/>

  <!-- Robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen">
    <param name="publish_frequency" type="double" value="50.0" />
  </node>
</launch>
```

## Quality Settings for Educational Use

### Performance Configuration
```
# Configuration optimized for educational use
# Balances visual quality with performance for learning

Gazebo Settings:
  - Real-time factor: 1.0 (matches real-time for learning)
  - Update rate: 1000 Hz (accurate physics simulation)
  - Visualization: Enabled (students can see robot behavior)
  - CPU usage: Limited to prevent system overload during learning

Unity Settings:
  - Target Frame Rate: 60 FPS (smooth visualization)
  - Quality Level: Medium (balance quality and performance)
  - Anti-aliasing: Disabled (performance over quality for learning)
  - Shadows: Enabled (helps with spatial understanding)

General Settings:
  - Logging: Enabled (for debugging and learning)
  - Visualization: Maximum enabled (for educational value)
  - Safety limits: Enforced (to prevent unrealistic behaviors)
```