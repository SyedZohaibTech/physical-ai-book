import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Configure the sidebars for the Docusaurus site.
 *
 * This file defines the structure of the navigation sidebar. We are creating a
 * manual sidebar to ensure the modules and chapters are displayed in the
 * correct pedagogical order.
 */
const sidebars: SidebarsConfig = {
  // Define the main textbook sidebar
  tutorialSidebar: [
    // Link to the introduction page
    'intro',
    // Module 1: ROS 2
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/intro-to-ros2',
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/rclpy-for-control',
        'module-1-ros2/humanoid-urdf-models',
        'module-1-ros2/ai-agents-to-controllers',
        'module-1-ros2/exercises',
      ],
    },
    // Module 2: Digital Twin
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/what-is-a-digital-twin',
        'module-2-digital-twin/gazebo-physics',
        'module-2-digital-twin/unity-for-hri',
        'module-2-digital-twin/sensor-simulation',
        'module-2-digital-twin/complete-humanoid-simulation',
        'module-2-digital-twin/exercises',
      ],
    },
    // Module 3: NVIDIA Isaac
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-isaac/intro-to-isaac',
        'module-3-isaac/isaac-sim-photorealism',
        'module-3-isaac/isaac-ros-perception',
        'module-3-isaac/vslam-and-depth',
        'module-3-isaac/nav2-path-planning',
        'module-3-isaac/ai-controlled-humanoid-brain',
        'module-3-isaac/exercises',
      ],
    },
    // Module 4: VLA
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/vla-future-of-robotics',
        'module-4-vla/whisper-voice-commands',
        'module-4-vla/llm-to-ros-actions',
        'module-4-vla/cognitive-planning-pipelines',
        'module-4-vla/integrating-vla',
        'module-4-vla/exercises',
      ],
    },
    // Capstone Project
    {
      type: 'category',
      label: 'Capstone Project: The Autonomous Humanoid',
      items: [
        'capstone-project/project-walkthrough',
      ],
    },
    // Glossary
    'glossary',
    // Appendix
    'appendix',
  ],
};

export default sidebars;

