# Quickstart: Physical AI & Humanoid Robotics Textbook

This quickstart guide provides instructions on how to set up, build, and deploy the "Physical AI & Humanoid Robotics" textbook, generated using Docusaurus.

## 1. Project Setup

This project is a Docusaurus static site. Ensure you have Node.js (version 18 or higher) and npm/yarn installed on your system.

## 2. Install Dependencies

Navigate to the `website/` directory and install the necessary Node.js dependencies:

```bash
cd website
npm install
# or yarn install
```

## 3. Develop Locally

To start the local development server and preview the textbook in your browser:

```bash
cd website
npm start
# or yarn start
```

This command starts a local development server and opens a browser window. Most changes are reflected live without restarting the server.

## 4. Build the Static Site

To build the static HTML, CSS, and JavaScript files for production deployment:

```bash
cd website
npm run build
# or yarn build
```

This command generates static content into the `website/build` directory. This content can be served by any static hosting service (e.g., GitHub Pages, Netlify).

## 5. Deployment (GitHub Pages)

This textbook is designed for deployment on GitHub Pages. After building the site (Step 4), you can deploy it by following Docusaurus's deployment guide for GitHub Pages. Typically, this involves configuring your `docusaurus.config.js` and pushing the `build` directory to a `gh-pages` branch.

Alternatively, you can use the `deploy` script (if configured in `package.json`):

```bash
cd website
npm run deploy
# or yarn deploy
```

## 6. Content Structure

The textbook content (`.mdx` or `.md` files) is organized within the `website/docs/` directory, mirroring the logical structure of the book's modules and chapters. The sidebar navigation is configured in `website/sidebars.js`.
