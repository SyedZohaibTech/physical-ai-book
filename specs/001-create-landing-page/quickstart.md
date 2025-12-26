# Quickstart Guide: Physical AI & Humanoid Robotics Landing Page

## Overview
This guide will help you get started with the Physical AI & Humanoid Robotics textbook landing page project. The landing page is built with React and integrated into the Docusaurus documentation site.

## Prerequisites
Before starting, ensure you have:
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git version control system
- A text editor or IDE (VS Code recommended)

## Setting Up the Development Environment

### 1. Clone the Repository
```bash
git clone <repository-url>
cd physical-ai-book
```

### 2. Navigate to the Project Directory
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
This command starts a local development server and opens the landing page in your default browser. Most changes are reflected live without restarting the server.

## Project Structure
```
website/
├── src/
│   ├── pages/
│   │   ├── index.js              # Main landing page React component
│   │   └── index.module.css      # CSS module for landing page styling
│   └── components/
│       ├── HomepageHeader.js     # Hero section component
│       ├── Features.js           # Features section component
│       ├── AboutBook.js          # About section component
│       └── CustomFooter.js       # Custom footer component
├── docusaurus.config.js          # Docusaurus configuration
├── sidebars.js                   # Navigation sidebar configuration
├── package.json                  # Project dependencies and scripts
└── README.md                     # Project overview
```

## Creating the Landing Page Components

### Main Landing Page (index.js)
The main landing page component imports and uses Docusaurus's Layout component while adding custom sections:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageHeader from './components/HomepageHeader';
import Features from './components/Features';
import AboutBook from './components/AboutBook';
import CustomFooter from './components/CustomFooter';

import styles from './index.module.css';

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into the meta tag">
      <HomepageHeader />
      <main>
        <Features />
        <AboutBook />
        <CustomFooter />
      </main>
    </Layout>
  );
}
```

### CSS Module (index.module.css)
The CSS module provides scoped styling for the landing page:

```css
.heroBanner {
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

@media screen and (max-width: 996px) {
  .heroBanner {
    padding: 2rem;
  }
}

.buttons {
  display: flex;
  align-items: center;
  justify-content: center;
}

.featuresSection {
  padding: 4rem 0;
  background-color: #f9f9f9;
}

.featuresGrid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 2rem;
  max-width: 1200px;
  margin: 0 auto;
  padding: 0 1rem;
}

.featureCard {
  background: white;
  border-radius: 8px;
  padding: 1.5rem;
  text-align: center;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  transition: transform 0.3s ease, box-shadow 0.3s ease;
}

.featureCard:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 15px rgba(0, 0, 0, 0.1);
}

.fadeInUp {
  animation: fadeInUp 0.8s ease-out forwards;
  opacity: 0;
  transform: translateY(20px);
}

@keyframes fadeInUp {
  to {
    opacity: 1;
    transform: translateY(0);
  }
}
```

### Component Structure
Each section of the landing page is implemented as a separate React component:
- `HomepageHeader.js`: Hero section with title, subtitle, description, and CTA
- `Features.js`: Grid of 4 feature cards showcasing the core modules
- `AboutBook.js`: Section explaining the textbook's value proposition
- `CustomFooter.js`: Footer with attribution and copyright information

## Building the Landing Page

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

## Development Workflow

1. Make changes to components in `src/pages/` or `src/components/`
2. The development server will automatically update the page
3. Test responsive design using browser dev tools
4. Verify all links and interactions work correctly
5. Run build command to check for any issues

## Testing

To verify the landing page works correctly:

1. Start the development server with `npm start`
2. Navigate to the homepage and verify all sections appear
3. Test the "Start Learning" button links to `/docs/intro`
4. Test responsive design on different screen sizes
5. Verify animations work smoothly
6. Check that the page loads within 3 seconds

## Deployment

The landing page is designed for deployment to GitHub Pages:

1. Ensure your `docusaurus.config.js` has the correct deployment settings
2. Build the site: `npm run build`
3. The `build/` directory contains the static site ready for deployment

For GitHub Pages deployment, follow the [Docusaurus deployment guide](https://docusaurus.io/docs/deployment).

## Getting Help

- Review the Docusaurus documentation: https://docusaurus.io/docs
- Check the project's GitHub repository for issues and discussions
- Refer to the landing page's development documentation in the `specs/` directory