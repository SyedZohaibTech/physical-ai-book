---
id: 4
title: Implement Core Textbook Content for Modules 1-3
stage: green
date: 2025-12-09
surface: agent
model: gemini-pro
feature: robotics-textbook
branch: 1-robotics-textbook
user: user
command: /sp.implement
labels: [implementation, docusaurus, content-generation]
links:
  spec: specs/1-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - website/docusaurus.config.ts
  - website/src/pages/index.tsx
  - website/sidebars.ts
  - website/docs/intro.md
  - website/docs/module-1-ros2/1-intro-to-ros2.md
  - website/docs/module-1-ros2/2-nodes-topics-services.md
  - website/docs/module-1-ros2/3-rclpy-for-control.md
  - website/docs/module-1-ros2/4-humanoid-urdf-models.md
  - website/docs/module-1-ros2/5-ai-agents-to-controllers.md
  - website/docs/module-1-ros2/6-exercises.md
  - website/docs/module-2-digital-twin/1-what-is-a-digital-twin.md
  - website/docs/module-2-digital-twin/2-gazebo-physics.md
  - website/docs/module-2-digital-twin/3-unity-for-hri.md
  - website/docs/module-2-digital-twin/4-sensor-simulation.md
  - website/docs/module-2-digital-twin/5-complete-humanoid-simulation.md
  - website/docs/module-2-digital-twin/6-exercises.md
  - website/docs/module-3-isaac/1-intro-to-isaac.md
  - website/docs/module-3-isaac/2-isaac-sim-photorealism.md
  - website/docs/module-3-isaac/3-isaac-ros-perception.md
  - specs/1-robotics-textbook/tasks.md
tests:
  - "none"
---

## Prompt

/sp.implement # IMPLEMENT: Step-by-Step Instructions for Building the Textbook
# Using Docusaurus + Spec-Kit Plus + Claude Code
# Title: Physical AI & Humanoid Robotics

-----------------------------------------
GENERAL IMPLEMENTATION INSTRUCTIONS
-----------------------------------------
1. **All content must be Docusaurus-ready**:
   - Output `.md` or `.mdx` files with front-matter
   - Sidebar position and title included
   - Include code blocks, diagrams, exercises, and examples

2. **Follow modular task execution**:
   - Implement tasks in the order specified in `/sp.task`
   - Each module and chapter must be self-contained
   - Ensure consistency across modules and chapters

3. **Focus on clarity and learning outcomes**:
   - Provide concise explanations
   - Include step-by-step tutorials
   - Add practical exercises and labs
   - Use real robotics APIs, ROS 2 commands, Isaac Sim, and VLA pipelines

4. **Use technical precision**:
   - Avoid generic or hypothetical examples
   - Use actual ROS 2, Gazebo, Unity, Isaac ROS, Nav2, and Whisper commands
   - Provide sample Python code with correct syntax

-----------------------------------------
IMPLEMENTATION STEPS PER MODULE
-----------------------------------------

MODULE 1 — The Robotic Nervous System (ROS 2)
- Write chapter content: introduction, concepts, theory
- Include ROS 2 Nodes, Topics, Services, Actions
- Add rclpy code examples to control humanoid joints
- Include URDF model examples
- Add exercises for creating and connecting ROS agents

MODULE 2 — The Digital Twin (Gazebo & Unity)
- Explain Digital Twin concepts and importance
- Include Gazebo physics simulation examples (gravity, collisions)
- Add Unity high-fidelity environment examples
- Simulate sensors: LiDAR, Depth Cameras, IMUs
- Include human-robot interaction simulation examples
- Add step-by-step exercises for building simulation scenarios

MODULE 3 — The AI-Robot Brain (NVIDIA Isaac™)
- Explain Isaac Sim and Isaac ROS stack
- Add photorealistic simulation examples and synthetic data generation
- Include VSLAM navigation examples
- Show Nav2 path planning and bipedal movement code
- Provide step-by-step labs with complete implementation instructions

MODULE 4 — Vision-Language-Action (VLA)
- Explain LLM → Robot action pipelines
- Implement Whisper voice-to-action examples
- Demonstrate cognitive planning for translating natural language to ROS 2 actions
- Include visual perception integration examples
- Add exercises where a robot executes multi-step tasks

CAPSTONE PROJECT — The Autonomous Humanoid
- Combine all modules into a full project
- Implement voice command reception
- Plan and execute ROS 2 actions based on commands
- Navigate obstacles and manipulate objects using computer vision
- Provide complete step-by-step instructions, diagrams, and code
- Include setup instructions for simulation and optional real-world testing
- Provide an evaluation rubric and practice exercises

-----------------------------------------
VISUALS AND CODE IMPLEMENTATION
-----------------------------------------
- Use ASCII or markdown diagrams for architecture, ROS graphs, URDF structures
- Provide code snippets for Python, ROS 2, Nav2, Isaac Sim, Unity, and Whisper
- Include sample configuration files, launch files, and simulation examples
- Add notes and comments for students in each code block

-----------------------------------------
FINAL IMPLEMENTATION RULES
-----------------------------------------
1. Each step must result in **ready-to-use Docusaurus content**
2. Do not generate extra commentary or unrelated text
3. Ensure consistency in terminology, formatting, and examples
4. Verify that all cross-references, links, and diagrams work in Docusaurus
5. Follow the `/sp.specify`, `/sp.plane`, and `/sp.task` instructions precisely

-----------------------------------------
END OF IMPLEMENTATION
-----------------------------------------

## Response snapshot

I have executed the implementation plan, starting with Phase 3 (creating a navigable textbook shell) and proceeding through the content generation tasks of Phase 4.

Key actions performed:
- Updated the main Docusaurus configuration (`docusaurus.config.ts`) and homepage (`index.tsx`).
- Created a manual, hierarchical sidebar in `sidebars.ts` for all modules.
- Wrote and populated all chapter content for Module 1 (ROS 2), Module 2 (Digital Twin), and the first three chapters of Module 3 (NVIDIA Isaac).
- Each chapter was created as a new markdown file in the `website/docs` directory with appropriate Docusaurus front-matter.
- Marked each completed task as `[x]` in `specs/1-robotics-textbook/tasks.md`.

## Outcome

- ✅ Impact: High. The core content for the first three modules of the textbook is now implemented, providing substantial value to the project.
- 🧪 Tests: No automated tests were run. Verification was done by successfully creating and modifying files.
- 📁 Files: 20 files were created or modified.
- 🔁 Next prompts: Continue with the remaining implementation tasks for Module 3 and Module 4.
- 🧠 Reflection: The implementation process was smooth. The tasks were well-defined, and the modular structure allowed for sequential content generation.

## Evaluation notes (flywheel)

- Failure modes observed: The initial `check-prerequisites.ps1` script failed due to incorrect feature detection. This was resolved by setting the `SPECIFY_FEATURE` environment variable. The `create-phr.sh` script also failed due to a missing WSL environment, forcing a fallback to the agent-native PHR creation flow.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
