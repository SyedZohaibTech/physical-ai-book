module.exports = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 - Robotic Nervous System',
      collapsed: false,
      items: [
        'module1/introduction',
        'module1/ros2-setup',
        'module1/nodes-topics-services',
        'module1/python-rclpy',
        'module1/urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo & Unity - Digital Twin',
      collapsed: false,
      items: [
        'module2/introduction',
        'module2/gazebo-environment',
        'module2/physics-simulation',
        'module2/unity-rendering',
        'module2/sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac - AI Robot Brain',
      collapsed: false,
      items: [
        'module3/introduction',
        'module3/isaac-sim',
        'module3/isaac-ros',
        'module3/visual-slam',
        'module3/nav2-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: false,
      items: [
        'module4/introduction',
        'module4/voice-to-action',
        'module4/llm-planning',
        'module4/capstone-project',
      ],
    },
  ],
};
