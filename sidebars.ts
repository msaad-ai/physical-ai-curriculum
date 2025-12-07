import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main tutorial sidebar that includes all modules
  tutorialSidebar: [
    'intro',
    'About the Author',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/intro',
        'module1/ros2-foundations',
        'module1/ros2-environment',
        'module1/ros2-communication',
        'module1/exercises-project'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module2/intro',
        'module2/gazebo-setup',
        'module2/unity-setup',
        'module2/sensors-integration',
        'module2/mini-project'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/intro',
        'module3/perception-pipelines',
        'module3/navigation-planning',
        'module3/mini-project',
        'module3/mini-project-implementation',
        {
          type: 'category',
          label: 'Exercises',
          items: [
            'module3/exercises/basic-exercises',
            'module3/exercises/integrated-exercises',
            'module3/exercises/review-questions',
            'module3/exercises/solution-hints',
            'module3/exercises/project-evaluation',
            'module3/exercises/data-generation',
            'module3/exercises/glossary'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/module4',
        'module4/chapter1-vla-concept-overview/index',
        'module4/chapter2-voice-to-action-whisper/index',
        'module4/chapter3-cognitive-planning/index',
        'module4/chapter4-exercises-mini-project/index',
        'module4/setup',
        'module4/glossary',
        'module4/checklist',
        'module4/testing-guide',
        'module4/troubleshooting',
        'module4/educational-content-review',
      ],
    },
  ],

  // Manual sidebar for Module 1 (for module-specific view)
  module1Sidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/intro',
        'module1/ros2-foundations',
        'module1/ros2-environment',
        'module1/ros2-communication',
        'module1/exercises-project'
      ],
    },
  ],

  // Manual sidebar for Module 2 (for module-specific view)
  module2Sidebar: [
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module2/intro',
        'module2/gazebo-setup',
        'module2/unity-setup',
        'module2/sensors-integration',
        'module2/mini-project'
      ],
    },
  ],

  // Manual sidebar for Module 3 (for module-specific view)
  module3Sidebar: [
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/intro',
        'module3/perception-pipelines',
        'module3/navigation-planning',
        'module3/mini-project',
        'module3/mini-project-implementation',
        {
          type: 'category',
          label: 'Exercises',
          items: [
            'module3/exercises/basic-exercises',
            'module3/exercises/integrated-exercises',
            'module3/exercises/review-questions',
            'module3/exercises/solution-hints',
            'module3/exercises/project-evaluation',
            'module3/exercises/data-generation',
            'module3/exercises/glossary'
          ],
        },
      ],
    },
  ],

  // Manual sidebar for Module 4 (for module-specific view)
  module4Sidebar: [
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/module4',
        'module4/chapter1-vla-concept-overview/index',
        'module4/chapter2-voice-to-action-whisper/index',
        'module4/chapter3-cognitive-planning/index',
        'module4/chapter4-exercises-mini-project/index',
        'module4/setup',
        'module4/glossary',
        'module4/checklist',
        'module4/testing-guide',
        'module4/troubleshooting',
        'module4/educational-content-review',
      ],
    },
  ],
};

export default sidebars;