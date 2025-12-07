# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `specs/1-robotics-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Project Setup

**Purpose**: Initialize the Docusaurus project and create the basic directory structure for the textbook content and examples.

- [x] T001 Create the top-level `examples/` directory for supplementary code.
- [x] T002 [P] Create the content directory structure under `website/docs/` for Module 1: `website/docs/module-1-ros2`.
- [x] T003 [P] Create the content directory structure under `website/docs/` for Module 2: `website/docs/module-2-digital-twin`.
- [x] T004 [P] Create the content directory structure under `website/docs/` for Module 3: `website/docs/module-3-isaac`.
- [x] T005 [P] Create the content directory structure under `website/docs/` for Module 4: `website/docs/module-4-vla`.
- [x] T006 [P] Create the content directory structure under `website/docs/` for the Capstone Project: `website/docs/capstone-project`.
- [x] T007 [P] Create a placeholder for the introduction in `website/docs/intro.md`.

---

## Phase 2: Foundational Content Structure (Completes part of US1)

**Purpose**: Create placeholder files for every chapter to enable sidebar creation and full site navigation.

- [x] T008 [P] [US1] Create placeholder file `website/docs/module-1-ros2/1-intro-to-ros2.md`.
- [x] T009 [P] [US1] Create placeholder file `website/docs/module-1-ros2/2-nodes-topics-services.md`.
- [x] T010 [P] [US1] Create placeholder file `website/docs/module-1-ros2/3-rclpy-for-control.md`.
- [x] T011 [P] [US1] Create placeholder file `website/docs/module-1-ros2/4-humanoid-urdf-models.md`.
- [x] T012 [P] [US1] Create placeholder file `website/docs/module-1-ros2/5-ai-agents-to-controllers.md`.
- [x] T013 [P] [US1] Create placeholder file `website/docs/module-1-ros2/6-exercises.md`.
- [x] T014 [P] [US1] Create placeholder file `website/docs/module-2-digital-twin/1-what-is-a-digital-twin.md`.
- [x] T015 [P] [US1] Create placeholder file `website/docs/module-2-digital-twin/2-gazebo-physics.md`.
- [x] T016 [P] [US1] Create placeholder file `website/docs/module-2-digital-twin/3-unity-for-hri.md`.
- [x] T017 [P] [US1] Create placeholder file `website/docs/module-2-digital-twin/4-sensor-simulation.md`.
- [x] T018 [P] [US1] Create placeholder file `website/docs/module-2-digital-twin/5-complete-humanoid-simulation.md`.
- [x] T019 [P] [US1] Create placeholder file `website/docs/module-2-digital-twin/6-exercises.md`.
- [x] T020 [P] [US1] Create placeholder file `website/docs/module-3-isaac/1-intro-to-isaac.md`.
- [x] T021 [P] [US1] Create placeholder file `website/docs/module-3-isaac/2-isaac-sim-photorealism.md`.
- [x] T022 [P] [US1] Create placeholder file `website/docs/module-3-isaac/3-isaac-ros-perception.md`.
- [x] T023 [P] [US1] Create placeholder file `website/docs/module-3-isaac/4-vslam-and-depth.md`.
- [x] T024 [P] [US1] Create placeholder file `website/docs/module-3-isaac/5-nav2-path-planning.md`.
- [x] T025 [P] [US1] Create placeholder file `website/docs/module-3-isaac/6-ai-controlled-humanoid-brain.md`.
- [x] T026 [P] [US1] Create placeholder file `website/docs/module-3-isaac/7-exercises.md`.
- [x] T027 [P] [US1] Create placeholder file `website/docs/module-4-vla/1-vla-future-of-robotics.md`.
- [x] T028 [P] [US1] Create placeholder file `website/docs/module-4-vla/2-whisper-voice-commands.md`.
- [x] T029 [P] [US1] Create placeholder file `website/docs/module-4-vla/3-llm-to-ros-actions.md`.
- [x] T030 [P] [US1] Create placeholder file `website/docs/module-4-vla/4-cognitive-planning-pipelines.md`.
- [x] T031 [P] [US1] Create placeholder file `website/docs/module-4-vla/5-integrating-vla.md`.
- [x] T032 [P] [US1] Create placeholder file `website/docs/module-4-vla/6-exercises.md`.
- [x] T033 [P] [US1] Create placeholder file `website/docs/capstone-project/1-project-walkthrough.md`.

---

## Phase 3: User Story 1 - A Navigable Textbook Shell (Priority: P1) 🎯 MVP

**Goal**: A student can access the deployed Docusaurus website and navigate the entire, albeit empty, structure of the textbook via the sidebar.
**Independent Test**: Run `npm run start` in the `website` directory. Open `http://localhost:3000` and verify that all modules and chapters are visible in the sidebar and are clickable, leading to an empty page with the correct title.

### Implementation for User Story 1
- [ ] T034 [US1] Update the Docusaurus homepage at `website/src/pages/index.tsx` to reflect the textbook's title and overview.
- [ ] T035 [US1] Configure the sidebar in `website/sidebars.ts` to create a hierarchical navigation menu for all modules and chapters created in Phase 2.
- [ ] T036 [US1] Populate the `website/docs/intro.md` file with a brief introduction to the book.

**Checkpoint**: User Story 1 is now fully functional and independently testable. The textbook exists as a navigable skeleton.

---

## Phase 4: User Story 2 - Content Generation

**Goal**: A student can read all the chapters and complete the practical exercises for every module.
**Independent Test**: For each task below, view the generated page in the Docusaurus site and confirm the content is well-structured, student-friendly, and contains all required elements (text, code, diagrams).

### Implementation for User Story 2 (Module 1)
- [ ] T037 [P] [US2] Write content for `website/docs/module-1-ros2/1-intro-to-ros2.md`.
- [ ] T038 [P] [US2] Write content for `website/docs/module-1-ros2/2-nodes-topics-services.md`.
- [ ] T039 [P] [US2] Write content for `website/docs/module-1-ros2/3-rclpy-for-control.md`.
- [ ] T040 [P] [US2] Write content for `website/docs/module-1-ros2/4-humanoid-urdf-models.md`.
- [ ] T041 [P] [US2] Write content for `website/docs/module-1-ros2/5-ai-agents-to-controllers.md`.
- [ ] T042 [P] [US2] Write content and examples for `website/docs/module-1-ros2/6-exercises.md`.

### Implementation for User Story 2 (Module 2)
- [ ] T043 [P] [US2] Write content for `website/docs/module-2-digital-twin/1-what-is-a-digital-twin.md`.
- [ ] T044 [P] [US2] Write content for `website/docs/module-2-digital-twin/2-gazebo-physics.md`.
- [ ] T045 [P] [US2] Write content for `website/docs/module-2-digital-twin/3-unity-for-hri.md`.
- [ ] T046 [P] [US2] Write content for `website/docs/module-2-digital-twin/4-sensor-simulation.md`.
- [ ] T047 [P] [US2] Write content for `website/docs/module-2-digital-twin/5-complete-humanoid-simulation.md`.
- [ ] T048 [P] [US2] Write content and examples for `website/docs/module-2-digital-twin/6-exercises.md`.

### Implementation for User Story 2 (Module 3)
- [ ] T049 [P] [US2] Write content for `website/docs/module-3-isaac/1-intro-to-isaac.md`.
- [ ] T050 [P] [US2] Write content for `website/docs/module-3-isaac/2-isaac-sim-photorealism.md`.
- [ ] T051 [P] [US2] Write content for `website/docs/module-3-isaac/3-isaac-ros-perception.md`.
- [ ] T052 [P] [US2] Write content for `website/docs/module-3-isaac/4-vslam-and-depth.md`.
- [ ] T053 [P] [US2] Write content for `website/docs/module-3-isaac/5-nav2-path-planning.md`.
- [ ] T054 [P] [US2] Write content for `website/docs/module-3-isaac/6-ai-controlled-humanoid-brain.md`.
- [ ] T055 [P] [US2] Write content and examples for `website/docs/module-3-isaac/7-exercises.md`.

### Implementation for User Story 2 (Module 4)
- [ ] T056 [P] [US2] Write content for `website/docs/module-4-vla/1-vla-future-of-robotics.md`.
- [ ] T057 [P] [US2] Write content for `website/docs/module-4-vla/2-whisper-voice-commands.md`.
- [ ] T058 [P] [US2] Write content for `website/docs/module-4-vla/3-llm-to-ros-actions.md`.
- [ ] T059 [P] [US2] Write content for `website/docs/module-4-vla/4-cognitive-planning-pipelines.md`.
- [ ] T060 [P] [US2] Write content for `website/docs/module-4-vla/5-integrating-vla.md`.
- [ ] T061 [P] [US2] Write content and examples for `website/docs/module-4-vla/6-exercises.md`.

**Checkpoint**: User Story 2 is now complete. All chapters and exercises are populated with content.

---

## Phase 5: User Story 3 - The Capstone Project (Priority: P3)

**Goal**: A student can follow the complete capstone project walkthrough, build the "Autonomous Humanoid Assistant", and run it in a simulated environment.
**Independent Test**: Follow the generated documentation from start to finish. The final simulation should run and perform the voice-activated "pick and place" task successfully.

### Implementation for User Story 3
- [ ] T062 [US3] Write the full project walkthrough in `website/docs/capstone-project/1-project-walkthrough.md`.
- [ ] T063 [US3] Create architecture diagrams (using Mermaid.js) within the walkthrough.
- [ ] T064 [US3] Develop the ROS 2 packages and code for the capstone in `examples/capstone-project/`.
- [ ] T065 [P] [US3] Create the simulation setup guide for Isaac Sim or Gazebo.
- [ ] T066 [P] [US3] Write the evaluation rubric for the project.

**Checkpoint**: User Story 3 is now complete. The capstone project is fully documented and the example code is available.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Add supplementary materials and perform final quality checks.
- [ ] T067 [P] Create a Glossary of terms and add it to the Docusaurus site.
- [ ] T068 [P] Create an Appendix with quick-reference guides.
- [ ] T069 [P] Review and format all generated code examples in the `examples/` directory.
- [ ] T070 Run a full Docusaurus site build (`npm run build` in `website/`) to check for broken links or formatting issues.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** can start immediately.
- **Phase 2 (Foundational)** depends on Phase 1.
- **Phase 3 (US1)** depends on Phase 2. This delivers the MVP.
- **Phase 4 (US2)** depends on Phase 3. The content generation tasks within this phase (T037-T061) are highly parallelizable.
- **Phase 5 (US3)** can run in parallel with Phase 4, but depends on Phase 3.
- **Phase 6 (Polish)** depends on all other phases being complete.

## Implementation Strategy

1.  **MVP First**: Complete Phases 1, 2, and 3 to deliver the navigable textbook skeleton (fulfills User Story 1).
2.  **Incremental Content Delivery**: Proceed with Phase 4, generating content module by module. The tasks for each chapter are parallelizable.
3.  **Capstone Integration**: Work on Phase 5 can begin after the MVP is established.
4.  **Final Polish**: Complete Phase 6 after all content is generated.
