---
id: 1
title: Update Physical AI Robotics Textbook Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-1.5-pro
feature: none
branch: current
user: user
command: /sp.constitution CONTEXT:
You are a senior AI textbook author tasked with creating a highly technical and comprehensive textbook for the "Physical AI & Humanoid Robotics" course.

GOAL:
To deliver a complete textbook built using the Docusaurus framework that covers the four main modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) and is ready for deployment on GitHub Pages.

PRINCIPLES:
1.  **Academic Standard:** The content must be technically accurate, rigorous, and follow an academic, pedagogical structure suitable for a university-level course.
2.  **Docusaurus Integrity:** The output must strictly adhere to Docusaurus's file structure and Markdown/MDX syntax, including correct sidebar configuration and linking.
3.  **Illustrative Code:** All core technical concepts (ROS 2 communication, URDF structure) must be illustrated with complete and correct Python or XML code examples within code blocks.
4.  **Completeness:** The book must cover all 13 weeks of the course, from foundational ROS 2 to the final Capstone Project on Vision-Language-Action (VLA) systems.
labels: [constitution, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:

---

## Prompt

/sp.constitution CONTEXT:
You are a senior AI textbook author tasked with creating a highly technical and comprehensive textbook for the "Physical AI & Humanoid Robotics" course.

GOAL:
To deliver a complete textbook built using the Docusaurus framework that covers the four main modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) and is ready for deployment on GitHub Pages.

PRINCIPLES:
1.  **Academic Standard:** The content must be technically accurate, rigorous, and follow an academic, pedagogical structure suitable for a university-level course.
2.  **Docusaurus Integrity:** The output must strictly adhere to Docusaurus's file structure and Markdown/MDX syntax, including correct sidebar configuration and linking.
3.  **Illustrative Code:** All core technical concepts (ROS 2 communication, URDF structure) must be illustrated with complete and correct Python or XML code examples within code blocks.
4.  **Completeness:** The book must cover all 13 weeks of the course, from foundational ROS 2 to the final Capstone Project on Vision-Language-Action (VLA) systems.

## Response snapshot

Updated the project constitution in `.specify/memory/constitution.md` with the provided project name, goal, and four core principles. Version bumped to 0.1.0 and last amended date set to 2025-12-07. Marked `RATIFICATION_DATE`, `ADDITIONAL_CONSTRAINTS`, and `DEVELOPMENT_WORKFLOW` as TODOs. Also prepended a Sync Impact Report to the constitution file.

## Outcome

- ✅ Impact: The project constitution has been successfully created/updated with the provided details.
- 🧪 Tests: No tests were run or added in this step.
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: Continue with further project setup or development based on the new constitution.
- 🧠 Reflection: The agent successfully processed the user's detailed input to generate a project constitution, handling placeholder replacements, section removals, and adding TODOs for missing information. The agent also handled the `bash` command not found error by falling back to agent-native PHR creation.

## Evaluation notes (flywheel)

- Failure modes observed: `bash` command not found on Windows, requiring fallback to agent-native PHR creation.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
