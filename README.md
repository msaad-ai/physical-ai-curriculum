# Physical AI & Humanoid Robotics Curriculum

This project contains a comprehensive curriculum for building next-generation intelligent humanoid systems, combining both documentation and practical VLA (Vision-Language-Action) modules. The curriculum covers everything from ROS 2 fundamentals to advanced Vision-Language-Action systems for intelligent robots.

## 🚀 Features

- **Complete 4-Module Curriculum**: From ROS 2 basics to advanced VLA systems
- **Hands-on Projects**: Practical exercises and implementation guides
- **VLA Integration**: Vision-Language-Action system with ROS2 integration
- **NVIDIA Isaac Support**: AI-Robot Brain implementation with Isaac Sim
- **Digital Twin**: Simulation environments with Gazebo and Unity
- **Deployed Website**: Ready-to-use Docusaurus-based curriculum site

## 📚 Curriculum Overview

The curriculum is organized into 4 comprehensive modules:

### Module 1: The Robotic Nervous System (ROS 2)
- ROS2 concepts and development
- Communication systems and environment setup
- Node development and inter-process communication
- Practical exercises and project work

### Module 2: Digital Twin (Gazebo & Unity)
- Simulation environment setup and configuration
- Sensor integration and data processing
- Unity for advanced visualization
- Mini-project implementation

### Module 3: AI-Robot Brain (NVIDIA Isaac)
- Perception pipeline development with VSLAM
- Navigation and planning algorithms for bipedal robots
- Deep learning integration with Isaac ROS
- Exercises and implementation projects

### Module 4: Vision-Language-Action (VLA)
- VLA concept and architecture overview
- Voice-to-action systems with OpenAI Whisper
- Cognitive planning and instruction translation
- Complete VLA system integration and testing

## 🗂️ Project Structure

- `docs/` - Contains complete curriculum modules (1-4) with exercises
- `src/vla_module/` - Python-based VLA system using ROS2
- `src/css/custom.css` - Custom styling for the curriculum website
- `examples/` - ROS2 and Isaac examples for practical learning
- `specs/` - Specification files and project plans
- `history/` - Architectural decision records and prompt history
- `docusaurus.config.ts` - Website configuration
- `sidebars.ts` - Navigation structure
- `vercel.json` - Vercel deployment configuration

## 🛠️ Prerequisites

### For Documentation Website
- Node.js (v18 or higher)
- npm or yarn

### For VLA Module
- ROS2 (Humble Hawksbill or later)
- Python 3.8+
- Required Python packages (see requirements.txt)

## 🌐 Deployment

### Vercel Deployment (Recommended)
This curriculum is designed for easy deployment on Vercel:
1. Push your code to the GitHub repository
2. Connect your repository to Vercel
3. Use the following build settings:
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Root Directory: `/`

### Local Development
To run the documentation website locally:

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

### Running the VLA Module

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

## 📚 Curriculum Modules

The curriculum is organized into 4 comprehensive modules with extensive exercises and implementation guides:

### Module 1: The Robotic Nervous System (ROS 2)
- ROS2 concepts and development fundamentals
- Communication systems and message passing
- Environment setup and configuration
- Node development and inter-process communication
- Practical exercises and project work

### Module 2: Digital Twin (Gazebo & Unity)
- Simulation environment setup and configuration
- Sensor integration and data processing
- Unity for advanced visualization
- Mini-project implementation with realistic scenarios

### Module 3: AI-Robot Brain (NVIDIA Isaac)
- Perception pipeline development with VSLAM
- Navigation and planning algorithms for bipedal robots
- Deep learning integration with Isaac ROS
- Exercises and implementation projects
- Mini-project implementation guide with detailed steps

### Module 4: Vision-Language-Action (VLA)
- VLA concept and architecture overview
- Voice-to-action systems with OpenAI Whisper
- Cognitive planning and instruction translation
- Complete VLA system integration and testing
- End-to-end projects and implementation

## 🔧 Troubleshooting

### Documentation Issues
- If modules don't appear, ensure you've run `npm install` and `npm start`
- Check that all module directories exist in `docs/`
- Verify that the `sidebars.ts` file includes all modules in navigation

### VLA Module Issues
- Ensure ROS2 is properly installed and sourced
- Verify Python dependencies are installed via `pip install -r requirements.txt`
- Check that required services (audio, etc.) have proper permissions
- Confirm ROS2 environment is sourced: `source /opt/ros/humble/setup.bash`

### Deployment Issues
- Ensure the `vercel.json` configuration is properly set for static builds
- Check that the `build` directory is correctly specified as output
- Verify GitHub integration settings in Vercel dashboard

## 🏗️ Building for Production

To build the documentation website for production:
```bash
npm run build
```

The built site will be in the `build/` directory and is optimized for deployment.

## 🤝 Contributing

This curriculum is designed to be extensible. You can add new modules by:
1. Creating a new directory in `docs/` (e.g., `docs/module5/`)
2. Adding content files in Markdown format
3. Updating `sidebars.ts` to include the new module in the navigation
4. Adding appropriate exercises and implementation guides

## 📄 License

This curriculum is open source and available under the MIT License.

## 🎯 Learning Outcomes

By completing this curriculum, learners will be able to:
- Design and implement complex robotic systems using ROS 2
- Create realistic simulation environments for robot testing and development
- Integrate AI models for perception, planning, and control
- Build Vision-Language-Action (VLA) systems for human-robot interaction
- Deploy and test complete robotic systems in both simulation and real-world environments