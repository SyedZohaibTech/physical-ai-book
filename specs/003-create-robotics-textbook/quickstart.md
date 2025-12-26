# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This guide will help you get started with the "Physical AI & Humanoid Robotics" textbook project. The project uses Docusaurus to create a comprehensive, interactive textbook covering ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems.

## Prerequisites
Before starting, ensure you have:
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git version control system
- A text editor or IDE (VS Code recommended)
- Basic knowledge of JavaScript/TypeScript

## Setting Up the Development Environment

### 1. Clone the Repository
```bash
git clone <repository-url>
cd physical-ai-book
```

### 2. Navigate to the Website Directory
```bash
cd website
```

### 3. Install Dependencies
```bash
npm install
# or
yarn install
```

### 4. Start the Development Server
```bash
npm start
# or
yarn start
```
This command starts a local development server and opens the textbook in your default browser. Most changes are reflected live without restarting the server.

## Project Structure
```
website/
├── docs/                # Textbook content (MD/MDX files organized by modules)
│   ├── intro/
│   ├── module-1-ros2/
│   ├── module-2-digital-twin/
│   ├── module-3-isaac/
│   ├── module-4-vla/
│   ├── capstone-project/
│   ├── appendix/
│   ├── glossary/
│   └── practice-labs/
├── src/
│   ├── components/      # Custom React components for textbook
│   └── pages/           # Additional pages beyond docs
├── static/              # Static assets (images, diagrams, code samples)
├── docusaurus.config.js # Docusaurus configuration
├── sidebars.js          # Navigation sidebar configuration
├── package.json         # Project dependencies and scripts
└── README.md            # Project overview
```

## Creating New Content

### Adding a New Chapter
1. Create a new MD/MDX file in the appropriate module directory:
   ```
   website/docs/module-1-ros2/new-chapter.md
   ```

2. Add frontmatter to your file:
   ```md
   ---
   title: Chapter Title
   sidebar_position: 3
   description: Brief description of the chapter
   ---
   ```

3. Add your content using MD/MDX syntax:
   ```md
   # Chapter Title
   
   ## Learning Objectives
   
   - Objective 1
   - Objective 2
   
   ## Introduction
   
   Your chapter content here...
   
   import CodeBlock from '@theme/CodeBlock';
   import { useState } from 'react';
   
   export const Counter = () => {
     const [count, setCount] = useState(0);
     return <button onClick={() => setCount(count + 1)}>Count: {count}</button>;
   };
   
   <Counter />
   
   ## Exercises
   
   1. Exercise description
   2. Another exercise
   ```

### Adding Code Examples
Include code examples using Docusaurus's CodeBlock component:

```md
import CodeBlock from '@theme/CodeBlock';
import python from 'prism-react-renderer/themes/github';

<CodeBlock language="python" title="publisher_node.py" showLineNumbers>
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
</CodeBlock>
```

### Adding Diagrams
Place diagrams in the `static/` directory and reference them:

```md
![Diagram Title](/img/diagram-name.png)
```

## Building the Textbook

To build the static site for production:

```bash
npm run build
# or
yarn build
```

This command generates a `build/` directory with the static HTML files ready for deployment.

## Local Deployment Test

To test the production build locally:

```bash
npm run serve
# or
yarn serve
```

## Adding to Navigation

Update `sidebars.js` to add new content to the navigation:

```js
module.exports = {
  textbook: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/intro',
        'module-1-ros2/nodes-and-topics',
        'module-1-ros2/services-actions', // Add your new chapter here
      ],
    },
    // ... other modules
  ],
};
```

## Content Guidelines

### Writing Style
- Use clear, concise language appropriate for students
- Explain complex concepts with analogies and examples
- Include practical exercises after theoretical sections
- Provide code examples with explanations

### Code Examples
- Ensure all code examples are syntactically correct
- Include comments explaining key parts
- Provide complete, runnable examples when possible
- Mention required dependencies

### Diagrams
- Use clear, legible diagrams
- Include alt text for accessibility
- Reference diagrams in the text
- Keep diagrams relevant to the content

## Running Tests

The textbook project includes validation checks:

```bash
npm test
# or
yarn test
```

## Deployment

The textbook is designed for deployment to GitHub Pages:

1. Ensure your `docusaurus.config.js` has the correct deployment settings
2. Build the site: `npm run build`
3. The `build/` directory contains the static site ready for deployment

For GitHub Pages deployment, follow the [Docusaurus deployment guide](https://docusaurus.io/docs/deployment).

## Getting Help

- Review the Docusaurus documentation: https://docusaurus.io/docs
- Check the project's GitHub repository for issues and discussions
- Refer to the textbook's development documentation in the `specs/` directory