# Data Model: Physical AI & Humanoid Robotics Textbook

## Entities

### 1. Textbook
- **Description**: The overarching container for the entire educational content. Represents the complete "Physical AI & Humanoid Robotics" book.
- **Fields**:
    - `title`: String (e.g., "Physical AI & Humanoid Robotics")
    - `modules`: List of `Module` entities
    - `introduction`: Reference to an `Introduction` Chapter
    - `capstone_project`: Reference to a `Capstone Project` Chapter
    - `appendix`: Reference to an `Appendix` Chapter
    - `glossary`: Reference to a `Glossary` Chapter
    - `practice_labs`: List of `Practice Lab` Chapters
- **Relationships**: Contains multiple `Module` entities and specific `Chapter` types.

### 2. Module
- **Description**: A logical grouping of related `Chapter` entities, forming a major section of the textbook.
- **Fields**:
    - `id`: String (e.g., "module-1-ros2")
    - `title`: String (e.g., "Module 1: The Robotic Nervous System (ROS 2)")
    - `chapters`: List of `Chapter` entities
    - `learning_objectives`: List of Strings
- **Relationships**: Belongs to `Textbook`, contains multiple `Chapter` entities.

### 3. Chapter
- **Description**: A single `.mdx` or `.md` file containing specific educational content. This is the primary unit of content in the textbook.
- **Fields**:
    - `id`: String (e.g., "intro-to-ros2", "nodes-topics-services")
    - `title`: String (e.g., "Introduction to ROS 2", "ROS 2 Nodes, Topics, and Services")
    - `file_path`: String (relative path within Docusaurus docs, e.g., `docs/module-1-ros2/intro-to-ros2.mdx`)
    - `content`: Markdown/MDX content, including:
        - `sections`: Hierarchical headings (`##`, `###`, etc.)
        - `text_explanations`: Rich text
        - `diagrams`: ASCII art or references to image files (SVG, PNG)
        - `code_blocks`: Fenced code blocks with language specifiers (Python, XML, YAML)
        - `practice_questions`: Markdown-formatted questions
        - `real_world_examples`: Markdown-formatted examples
    - `front_matter`: YAML object (required for Docusaurus, e.g., `title`, `sidebar_label`, `slug`)
- **Relationships**: Belongs to a `Module` (or `Textbook` for special chapters like Introduction, Appendix, Glossary).

### 4. Capstone Project
- **Description**: A specialized `Chapter` entity providing a detailed, step-by-step guide for the final project.
- **Fields**: Inherits all fields from `Chapter`, plus:
    - `project_overview`: String
    - `implementation_steps`: List of Strings (detailed instructions)
    - `evaluation_rubric`: String (assessment criteria)
    - `simulation_setup`: String (instructions for setting up simulation environment)
- **Relationships**: A special type of `Chapter`, belongs to the `Textbook`.

### 5. Practice Lab
- **Description**: A specialized `Chapter` entity dedicated to hands-on exercises and practical application of concepts.
- **Fields**: Inherits all fields from `Chapter`, plus:
    - `lab_objectives`: List of Strings
    - `setup_instructions`: String
    - `exercise_steps`: List of Strings (detailed instructions for lab activities)
    - `expected_outcomes`: List of Strings
- **Relationships**: A special type of `Chapter`, belongs to the `Textbook`.

## Data Flow (Conceptual)

1. **Input**: Initial user prompt defining the textbook scope and structure.
2. **Generation Process**: AI agent interprets the `Textbook` structure, iterates through `Module`s and `Chapter`s.
3. **Content Generation**: For each `Chapter`, AI generates `content` based on `learning_objectives`, `topic`, and `requirements` (including code, diagrams, exercises).
4. **Output**: `.mdx` or `.md` files written to the Docusaurus `docs/` directory structure, along with updated `sidebars.js` configuration.
