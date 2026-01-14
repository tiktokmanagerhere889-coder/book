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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2: The Robotic Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/ros2-humanoid-control/index',
        'modules/ros2-humanoid-control/chapter-1-ros2-fundamentals',
        'modules/ros2-humanoid-control/chapter-2-python-agents-rclpy',
        'modules/ros2-humanoid-control/chapter-3-urdf-modeling'
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Simulation: Gazebo & Unity',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/digital-twin-simulation/index',
        'modules/digital-twin-simulation/chapter-1-gazebo-physics-simulation',
        'modules/digital-twin-simulation/chapter-2-unity-interaction-visualization',
        'modules/digital-twin-simulation/chapter-3-digital-twin-concepts'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/isaac-ai-brain/index',
        'modules/isaac-ai-brain/chapter-1-nvidia-isaac-sim',
        'modules/isaac-ai-brain/chapter-2-isaac-ros-perception',
        'modules/isaac-ai-brain/chapter-3-navigation-with-nav2'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/vla-integration/index',
        'modules/vla-integration/chapter-1-voice-to-action',
        'modules/vla-integration/chapter-2-cognitive-planning',
        'modules/vla-integration/chapter-3-autonomous-humanoid'
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
