# Landing Page Interface Contracts

## Page Structure Contract

### Main Landing Page (index.js)
- **Purpose**: Entry point for the Physical AI & Humanoid Robotics textbook
- **Interface**:
  - Input: None (loads context from Docusaurus)
  - Output: Complete landing page with all required sections
  - Dependencies: Docusaurus Layout, useDocusaurusContext, Link

### HomepageHeader Component
- **Purpose**: Hero section with title, subtitle, description and CTA
- **Interface**:
  - Props: None required (uses hardcoded content per spec)
  - Output: Hero section with gradient background
  - Styling: Uses CSS module classes for gradient and animations

### Features Component
- **Purpose**: Grid of 4 feature cards showcasing core modules
- **Interface**:
  - Props: None required (uses hardcoded content per spec)
  - Output: Grid of 4 feature cards with icons and descriptions
  - Styling: Uses CSS module classes for grid layout and hover effects

### AboutBook Component
- **Purpose**: Section explaining the textbook's value proposition
- **Interface**:
  - Props: None required (uses hardcoded content per spec)
  - Output: Section with title and 3 paragraphs about the book
  - Styling: Uses CSS module classes for typography and layout

### CustomFooter Component
- **Purpose**: Footer with attribution and copyright
- **Interface**:
  - Props: None required (uses hardcoded content per spec)
  - Output: Footer with styled attribution and copyright notice
  - Styling: Uses CSS module classes for gradient text

## CSS Contract (index.module.css)

### Hero Section Classes
- `.heroBanner`: Main hero container with gradient background
- `.heroTitle`: Styles for the main title
- `.heroSubtitle`: Styles for the subtitle
- `.heroDescription`: Styles for the 2-line description
- `.heroCtaButton`: Styles for the "Start Learning" button with hover effects

### Features Section Classes
- `.featuresSection`: Container for the features grid
- `.featuresGrid`: Grid layout for the 4 feature cards
- `.featureCard`: Individual feature card styling
- `.featureIcon`: Styles for the emoji icons
- `.featureTitle`: Styles for the feature titles
- `.featureDescription`: Styles for the feature descriptions
- `.featureCard:hover`: Hover effects for feature cards

### About Section Classes
- `.aboutSection`: Container for the about section
- `.aboutTitle`: Styles for the "About This Textbook" title
- `.aboutParagraph`: Styles for the 3 paragraphs about the book

### Footer Classes
- `.footer`: Main footer container
- `.authorAttribution`: Styles for "Created by Syed Zohaib" with gradient text
- `.copyrightNotice`: Styles for the copyright information

### Animation Classes
- `.fadeInUp`: Animation class for fade-in-up effect
- `@keyframes fadeInUp`: Keyframe definition for the animation

### Responsive Classes
- Media query classes for mobile, tablet, and desktop layouts
- Responsive units and breakpoints for different screen sizes

## Component Interface Contract

### HomepageHeader
```
interface HomepageHeaderProps {}
interface HomepageHeader {
  render(): JSX.Element;
}
```

### Features
```
interface FeaturesProps {}
interface Features {
  render(): JSX.Element;
}
```

### AboutBook
```
interface AboutBookProps {}
interface AboutBook {
  render(): JSX.Element;
}
```

### CustomFooter
```
interface CustomFooterProps {}
interface CustomFooter {
  render(): JSX.Element;
}
```

## Functional Requirements Mapping

### FR-001 to FR-004 (Hero Section)
- Component: HomepageHeader
- CSS Classes: .heroBanner, .heroTitle, .heroSubtitle, .heroDescription, .heroCtaButton
- Gradient: #667eea to #764ba2

### FR-005 to FR-009 (Features Section)
- Component: Features
- CSS Classes: .featuresSection, .featuresGrid, .featureCard, .featureIcon, .featureTitle, .featureDescription
- Cards: 4 feature cards with icons and descriptions

### FR-010 (About Section)
- Component: AboutBook
- CSS Classes: .aboutSection, .aboutTitle, .aboutParagraph
- Content: 3 paragraphs explaining textbook value

### FR-011 to FR-012 (Footer Section)
- Component: CustomFooter
- CSS Classes: .footer, .authorAttribution, .copyrightNotice
- Styling: Gradient text for attribution

### FR-013 to FR-015 (Styling & Animations)
- CSS Classes: Hover effects, responsive layouts, fadeInUp animation
- Implementation: In index.module.css