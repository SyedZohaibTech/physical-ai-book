# Data Model: Physical AI & Humanoid Robotics Landing Page

## Core Entities

### 1. LandingPage
- **Description**: The main landing page component containing all required sections
- **Fields**:
  - title: string (e.g., "Physical AI & Humanoid Robotics")
  - subtitle: string (e.g., "Master the Future of Embodied Intelligence")
  - description: string (2-line description about building humanoid robots)
  - ctaText: string (e.g., "Start Learning")
  - ctaLink: string (e.g., "/docs/intro")
  - heroBackground: string (gradient colors, e.g., "#667eea to #764ba2")
  - sections: Section[] (collection of all page sections)
  - authorAttribution: string (e.g., "Created by Syed Zohaib")
  - copyrightNotice: string (copyright information)

### 2. Section
- **Description**: A section of the landing page (hero, features, about, footer)
- **Fields**:
  - id: string (unique identifier, e.g., "hero", "features", "about", "footer")
  - title: string (section title)
  - content: string[] | Section[] (section content, either text lines or subsections)
  - styles: string (CSS classes or styles specific to the section)
  - position: number (order in which the section appears)

### 3. FeatureCard
- **Description**: One of four cards representing core textbook modules
- **Fields**:
  - id: string (unique identifier, e.g., "ros2", "gazebo", "isaac", "vla")
  - title: string (e.g., "ROS 2 Fundamentals")
  - icon: string (emoji or icon identifier, e.g., "🤖")
  - description: string (brief description of the module)
  - position: number (order in the feature grid)
  - styles: string (CSS classes for styling)

### 4. AboutSection
- **Description**: Section explaining the textbook's value proposition
- **Fields**:
  - id: string (e.g., "about-book")
  - title: string (e.g., "About This Textbook")
  - paragraphs: string[] (3 paragraphs explaining comprehensive guide, hands-on learning, industry skills)
  - styles: string (CSS classes for styling)

### 5. Footer
- **Description**: Bottom section containing attribution and copyright information
- **Fields**:
  - id: string (e.g., "footer")
  - authorAttribution: string (e.g., "Created by Syed Zohaib")
  - authorStyle: string (styling for author text: italic, gradient)
  - copyrightNotice: string (copyright information)
  - position: string (positioning, e.g., "right corner")

## Relationships

### LandingPage contains Sections
- **Relationship**: One-to-Many
- **Description**: A landing page contains multiple sections
- **Cardinality**: 1 landing page → 4 sections (hero, features, about, footer)

### Section contains FeatureCards (for features section)
- **Relationship**: One-to-Many
- **Description**: The features section contains multiple feature cards
- **Cardinality**: 1 features section → 4 feature cards

## Validation Rules

### LandingPage Validation
- Title must be provided and not empty
- Subtitle must be provided and not empty
- Description must be provided and contain exactly 2 lines
- CTA text must be provided and not empty
- CTA link must be a valid path (e.g., starting with /)
- All sections must be defined

### FeatureCard Validation
- Title must be provided and not empty
- Icon must be provided (emoji or icon identifier)
- Description must be provided and not empty
- Position must be between 1 and 4

### AboutSection Validation
- Title must be provided and not empty
- Must contain exactly 3 paragraphs
- Each paragraph must be provided and not empty

### Footer Validation
- Author attribution must be provided
- Copyright notice must be provided

## State Transitions

### LandingPage States
- DRAFT → REVIEW → APPROVED → PUBLISHED
- Description: Content progresses from initial draft through review and approval to final publication