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
  // Manual sidebar structure for the 4 modules
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'chapter-1-introduction/overview',
        'chapter-1-introduction/physical-ai',
        'chapter-1-introduction/role-of-ros2'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'chapter-2-ros2-basics/nodes',
        'chapter-2-ros2-basics/topics',
        'chapter-2-ros2-basics/services',
        'chapter-2-ros2-basics/actions'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'chapter-3-python-agents-rclpy/intro-to-rclpy',
        'chapter-3-python-agents-rclpy/python-agent-setup',
        'chapter-3-python-agents-rclpy/joint-control-examples',
        'chapter-3-python-agents-rclpy/sensor-integration'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'chapter-4-urdf-humanoids/urdf-basics',
        'chapter-4-urdf-humanoids/links-joints',
        'chapter-4-urdf-humanoids/sensors',
        'chapter-4-urdf-humanoids/gazebo-integration'
      ],
    },
    {
      type: 'category',
      label: 'Practical System & Summary',
      items: [
        'chapter-5-practical-system/building-control-pipeline',
        'chapter-5-practical-system/ros2-composition',
        'chapter-5-practical-system/python-command-to-movement',
        'chapter-6-summary/key-takeaways',
        'chapter-6-summary/exercises',
        'chapter-6-summary/references'
      ],
    },
  ],
};

export default sidebars;
