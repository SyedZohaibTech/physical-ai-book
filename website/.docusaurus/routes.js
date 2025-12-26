import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'b2d'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'd49'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '4cb'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '89a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/introduction',
                component: ComponentCreator('/docs/module1/introduction', '8b3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/nodes-topics-services',
                component: ComponentCreator('/docs/module1/nodes-topics-services', '423'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/python-rclpy',
                component: ComponentCreator('/docs/module1/python-rclpy', '68b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-setup',
                component: ComponentCreator('/docs/module1/ros2-setup', '7da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/urdf-humanoids',
                component: ComponentCreator('/docs/module1/urdf-humanoids', '958'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/gazebo-environment',
                component: ComponentCreator('/docs/module2/gazebo-environment', 'e34'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/introduction',
                component: ComponentCreator('/docs/module2/introduction', 'cee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/physics-simulation',
                component: ComponentCreator('/docs/module2/physics-simulation', '874'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/sensor-simulation',
                component: ComponentCreator('/docs/module2/sensor-simulation', '793'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/unity-rendering',
                component: ComponentCreator('/docs/module2/unity-rendering', 'dec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/introduction',
                component: ComponentCreator('/docs/module3/introduction', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac-ros',
                component: ComponentCreator('/docs/module3/isaac-ros', 'd4e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac-sim',
                component: ComponentCreator('/docs/module3/isaac-sim', '1f1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/nav2-planning',
                component: ComponentCreator('/docs/module3/nav2-planning', '62b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/visual-slam',
                component: ComponentCreator('/docs/module3/visual-slam', 'acd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/capstone-project',
                component: ComponentCreator('/docs/module4/capstone-project', '258'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/introduction',
                component: ComponentCreator('/docs/module4/introduction', 'b3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/llm-planning',
                component: ComponentCreator('/docs/module4/llm-planning', 'e9b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/voice-to-action',
                component: ComponentCreator('/docs/module4/voice-to-action', '7b5'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
