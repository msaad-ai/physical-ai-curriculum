# Physical AI & Humanoid Robotics Curriculum

This project contains a comprehensive curriculum for building next-generation intelligent humanoid systems, combining both documentation and practical VLA (Vision-Language-Action) modules.

## Project Structure

- `docs/` - Contains curriculum modules (1-4)
- `src/vla_module/` - Python-based VLA system using ROS2
- `examples/` - ROS2 and Isaac examples
- `specs/` - Specification files

## Prerequisites

### For Documentation Website
- Node.js (v18 or higher)
- npm or yarn

### For VLA Module
- ROS2 (Humble Hawksbill or later)
- Python 3.8+
- Required Python packages (see requirements.txt)

## Running the Documentation Website

1. Install dependencies:
```bash
npm install
# or
yarn install
```

2. Start the development server:
```bash
npm start
# or
yarn start
```

3. Open your browser to http://localhost:3000

## Running the VLA Module

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Make sure ROS2 is installed and sourced:
```bash
source /opt/ros/humble/setup.bash  # Adjust for your ROS2 installation
```

3. Run the VLA module:
```bash
python run_vla_module.py
```

Or directly:
```bash
cd src/vla_module
python vla_node.py
```

## Curriculum Modules

The curriculum is organized into 4 comprehensive modules:

### Module 1: ROS 2 Foundations
- ROS2 concepts and development
- Communication systems
- Environment setup

### Module 2: Digital Twin (Gazebo & Unity)
- Simulation environments
- Sensor integration
- Digital twin concepts

### Module 3: AI-Robot Brain (NVIDIA Isaac)
- Perception pipelines
- Navigation and planning
- Deep learning integration

### Module 4: Vision-Language-Action Systems
- VLA system integration
- Voice-to-action systems
- Cognitive planning
- End-to-end projects

## Troubleshooting

### Documentation Issues
- If modules don't appear, ensure you've run `npm install` and `npm start`
- Check that all module directories exist in `docs/`

### VLA Module Issues
- Ensure ROS2 is properly installed and sourced
- Verify Python dependencies are installed
- Check that required services (audio, etc.) have proper permissions

## Building for Production

To build the documentation website for production:
```bash
npm run build
```

The built site will be in the `build/` directory.

## Contributing

This curriculum is designed to be extensible. You can add new modules by:
1. Creating a new directory in `docs/` (e.g., `docs/module5/`)
2. Adding content files (Markdown format)
3. Updating `sidebars.ts` to include the new module in the navigation