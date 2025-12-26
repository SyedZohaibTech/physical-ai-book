# Data Model: Physical AI & Humanoid Robotics Textbook

## Core Entities

### 1. Textbook
- **Description**: The complete educational resource on Physical AI & Humanoid Robotics
- **Fields**:
  - title: string (e.g., "Physical AI & Humanoid Robotics")
  - version: string (semantic versioning)
  - description: string (educational overview)
  - modules: Module[] (collection of all modules)
  - authors: string[] (authors of the textbook)
  - publication_date: date (date of publication)
  - prerequisites: string[] (knowledge required before starting)

### 2. Module
- **Description**: A collection of related chapters focusing on a specific aspect of robotics
- **Fields**:
  - id: string (unique identifier, e.g., "module-1-ros2")
  - title: string (e.g., "The Robotic Nervous System (ROS 2)")
  - description: string (overview of the module)
  - chapters: Chapter[] (collection of chapters in the module)
  - learning_objectives: string[] (what students will learn)
  - prerequisites: string[] (knowledge required for this module)
  - estimated_duration: string (time to complete module)
  - difficulty_level: enum (beginner, intermediate, advanced)

### 3. Chapter
- **Description**: A single document containing specific educational content
- **Fields**:
  - id: string (unique identifier, e.g., "module-1-chapter-3")
  - title: string (e.g., "ROS 2 Nodes and Communication")
  - content: string (the actual text content in MDX format)
  - module_id: string (reference to parent module)
  - objectives: string[] (what students will achieve)
  - prerequisites: string[] (knowledge required for this chapter)
  - code_examples: CodeExample[] (associated code examples)
  - diagrams: Diagram[] (visual aids)
  - exercises: Exercise[] (practice problems)
  - duration: string (estimated time to complete)
  - keywords: string[] (important terms covered)

### 4. CodeExample
- **Description**: A code snippet demonstrating a concept
- **Fields**:
  - id: string (unique identifier)
  - title: string (description of the example)
  - language: string (programming language)
  - code: string (the actual code)
  - explanation: string (description of what the code does)
  - chapter_id: string (reference to parent chapter)
  - dependencies: string[] (required libraries/packages)

### 5. Diagram
- **Description**: A visual representation of a concept
- **Fields**:
  - id: string (unique identifier)
  - title: string (description of the diagram)
  - type: enum (flowchart, architecture, process, etc.)
  - description: string (explanation of the diagram)
  - source: string (file path to the diagram)
  - chapter_id: string (reference to parent chapter)

### 6. Exercise
- **Description**: A practice problem for students
- **Fields**:
  - id: string (unique identifier)
  - title: string (description of the exercise)
  - description: string (detailed instructions)
  - type: enum (coding, theoretical, practical)
  - difficulty: enum (easy, medium, hard)
  - chapter_id: string (reference to parent chapter)
  - solution: string (solution or hints)
  - estimated_time: string (time to complete)

### 7. CapstoneProject
- **Description**: The final comprehensive project integrating all learned concepts
- **Fields**:
  - id: string (unique identifier)
  - title: string (e.g., "The Autonomous Humanoid Robot")
  - description: string (overview of the project)
  - objectives: string[] (what students will achieve)
  - requirements: string[] (technical requirements)
  - phases: ProjectPhase[] (step-by-step breakdown)
  - evaluation_criteria: string[] (how the project will be assessed)
  - resources: string[] (additional materials needed)

### 8. ProjectPhase
- **Description**: A step in the capstone project
- **Fields**:
  - id: string (unique identifier)
  - title: string (phase name)
  - description: string (what to do in this phase)
  - deliverables: string[] (what needs to be completed)
  - capstone_project_id: string (reference to parent project)
  - estimated_duration: string (time to complete phase)

## Relationships

### Textbook contains Modules
- **Relationship**: One-to-Many
- **Description**: A textbook contains multiple modules
- **Cardinality**: 1 textbook → n modules

### Module contains Chapters
- **Relationship**: One-to-Many
- **Description**: A module contains multiple chapters
- **Cardinality**: 1 module → n chapters

### Chapter contains CodeExamples
- **Relationship**: One-to-Many
- **Description**: A chapter may contain multiple code examples
- **Cardinality**: 1 chapter → n code examples

### Chapter contains Diagrams
- **Relationship**: One-to-Many
- **Description**: A chapter may contain multiple diagrams
- **Cardinality**: 1 chapter → n diagrams

### Chapter contains Exercises
- **Relationship**: One-to-Many
- **Description**: A chapter may contain multiple exercises
- **Cardinality**: 1 chapter → n exercises

### CapstoneProject contains ProjectPhases
- **Relationship**: One-to-Many
- **Description**: A capstone project is broken down into multiple phases
- **Cardinality**: 1 capstone project → n project phases

## Validation Rules

### Textbook Validation
- Title must be provided and not empty
- Must contain at least one module
- Version must follow semantic versioning format

### Module Validation
- Title must be provided and not empty
- Must contain at least one chapter
- Learning objectives must be defined
- Difficulty level must be specified

### Chapter Validation
- Title must be provided and not empty
- Content must be provided and not empty
- Must belong to a valid module
- Duration must be specified

### CodeExample Validation
- Title must be provided and not empty
- Code must be provided and not empty
- Language must be specified
- Must belong to a valid chapter

### Exercise Validation
- Title must be provided and not empty
- Description must be provided and not empty
- Difficulty level must be specified
- Must belong to a valid chapter

## State Transitions

### Chapter States
- DRAFT → REVIEW → APPROVED → PUBLISHED
- Description: Content progresses from initial draft through review and approval to final publication

### Module States
- PLANNING → IN_PROGRESS → REVIEW → COMPLETE
- Description: Module development progresses through planning, implementation, review, and completion

### Textbook States
- CONCEPT → DEVELOPMENT → REVIEW → PUBLISHED
- Description: The textbook progresses from concept through development and review to final publication