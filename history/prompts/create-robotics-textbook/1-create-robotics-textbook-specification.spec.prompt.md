---
id: 1
title: Create Robotics Textbook Specification
stage: spec
date: 2025-12-20
surface: agent
model: gemini
feature: create-robotics-textbook
branch: 003-create-robotics-textbook
user: Pcw
command: /sp.specify
labels: [spec, robotics, textbook]
links:
  spec: specs/003-create-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-create-robotics-textbook/spec.md
 - specs/003-create-robotics-textbook/checklists/requirements.md
tests:
 - none
---

## Prompt

/sp.specify # Project Specification for Textbook Creation # Using Docusaurus, Spec-Kit Plus # Title: Physical AI & Humanoid Robotics This specification defines exactly what the AI must build: A complete, structured, Docusaurus-powered textbook that teaches students how to design, simulate, and deploy AI-powered humanoid robots capable of interacting with the real physical world. -------------------------------------------- PROJECT OVERVIEW -------------------------------------------- Title: Physical AI & Humanoid Robotics Theme: Embodied Intelligence — AI operating in the physical world. Goal: Bridge the gap between the digital brain (AI Models) and the physical body (Robotics Systems). Purpose: Teach students how AI systems interact with physics, sensors, and robotic bodies using ROS 2, Gazebo, Unity, and NVIDIA Isaac. Enable students to build humanoid robots capable of natural behavior, navigation, perception, and action. Target: Capstone quarter students learning advanced AI engineering + robotics. Output Format: - All content must be in Docusaurus `.md` or `.mdx` files. - Must include front-matter. - Folder-structured for GitHub Pages deployment. - Compatible with sidebar navigation. -------------------------------------------- BOOK STRUCTURE REQUIREMENTS -------------------------------------------- The textbook MUST be divided into four major modules: ============================================================ MODULE 1 — THE ROBOTIC NERVOUS SYSTEM (ROS 2) ============================================================ Focus: * Robot middleware foundations * ROS 2 Nodes, Topics, Services, and Actions * rclpy Python control loops * ROS graph concepts * Using Python agents to control joints * URDF for humanoid robots * Launch files * Real-world robot control architecture Chapters Required: 1. Introduction to ROS 2 2. Nodes, Topics, and Services 3. rclpy for Humanoid Control 4. Creating Humanoid URDF Models 5. Connecting AI Agents to ROS Controllers 6. Exercises + Practical Projects ============================================================ MODULE 2 — THE DIGITAL TWIN (GAZEBO & UNITY) ============================================================ Focus: * Physics simulation * Gravity, inertia, friction, torque forces * Building environments for humanoids * Simulating LiDAR, Depth Cameras, RGB cameras, and IMUs * Gazebo for robotics physics * Unity for high-fidelity visual simulation * Human–robot interaction simulations Chapters Required: 1. What is a Digital Twin? 2. Gazebo Physics Deep Dive 3. Unity for Human-Robot Interaction 4. Sensor Simulation Techniques 5. Building a Complete Humanoid Simulation 6. Exercises + Simulation Labs ============================================================ MODULE 3 — THE AI-ROBOT BRAIN (NVIDIA ISAAC™) ============================================================ Focus: * Photorealistic robotics simulation * Synthetic data generation * Hardware-accelerated Isaac ROS pipelines * Isaac ROS perception stack * VSLAM (Visual SLAM) * Navigation2 for humanoids * Bipedal locomotion fundamentals Chapters Required: 1. Introduction to NVIDIA Isaac 2. Isaac Sim: Photorealistic Training 3. Isaac ROS Perception Stack 4. VSLAM + Depth Perception 5. Nav2 Path Planning for Humanoids 6. Building an AI-Controlled Humanoid Brain 7. Exercises + Isaac Projects ============================================================ MODULE 4 — VISION-LANGUAGE-ACTION (VLA) ============================================================ Focus: * Convergence of LLMs + Robotics * Voice-to-Action via Whisper * LLM cognitive planning * Natural language → robot action pipeline * Hierarchical task decomposition * Human-level command execution Chapters Required: 1. VLA: The Future of Robotics 2. Whisper Voice Commands 3. LLM → ROS 2 Action Translation System 4. Cognitive Planning Pipelines 5. Integrating Vision + Language + Action 6. Exercises + VLA Labs -------------------------------------------- CAPSTONE PROJECT (MANDATORY) -------------------------------------------- Title: **The Autonomous Humanoid Assistant** Requirements: * Receives a voice command (via Whisper) * Converts it into a task plan using LLM cognitive planning * Generates a sequence of ROS 2 actions * Navigates a simulated environment (Nav2) * Avoids obstacles * Identifies the correct object using computer vision * Picks it up and places it * Works in simulation (Isaac or Gazebo) * Optional real-world deployment Output Required: * Full project walkthrough * Architecture diagrams * ROS package structure * Pseudo-code * Simulation setup guide * Evaluation rubric * Practice exercises -------------------------------------------- DELIVERABLES -------------------------------------------- The AI must generate: * All textbook chapters * All modules * All exercises * All diagrams (ASCII or markdown-compatible) * All example code * The entire Docusaurus `/docs` structure * Sidebar configuration * Homepage content * Capstone project documentation -------------------------------------------- END OF SPECIFICATION --------------------------------------------

## Response snapshot

Created a new feature specification for "create-robotics-textbook".
- Switched to branch `003-create-robotics-textbook`.
- Created `specs/003-create-robotics-textbook/spec.md`.
- Created `specs/003-create-robotics-textbook/checklists/requirements.md`.
- Created PHR `history/prompts/create-robotics-textbook/1-create-robotics-textbook-specification.spec.prompt.md`.

## Outcome

- ✅ Impact: High. This specification is the foundation for the entire project.
- 🧪 Tests: n/a
- 📁 Files: 3 created
- 🔁 Next prompts: `/sp.plan` to create the implementation plan.
- 🧠 Reflection: The provided specification was very detailed, making it easy to create a comprehensive feature spec. The script failure was a minor setback.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-new-feature.ps1` script failed when the feature description was passed as a multi-line string. Workaround was to save the description to a temp file. The script still failed, but the necessary files and branch were created, so the process could be completed manually.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: Investigate the PowerShell script to handle multi-line strings correctly.
