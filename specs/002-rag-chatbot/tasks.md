# Tasks: Interactive RAG Chatbot for Physical AI Textbook

**Input**: Design documents from `/specs/002-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure per plan
- [X] T002 [P] Create frontend directory structure per plan
- [X] T003 [P] Initialize backend requirements.txt with dependencies
- [X] T004 Create backend/.env.example template file
- [X] T005 Create src/components/ChatBot directory structure

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 [P] Create database models in backend/database.py (Conversation, Message)
- [X] T007 Create vector store module in backend/vector_store.py (QdrantClient setup)
- [X] T008 Create embeddings module in backend/embeddings.py (OpenAI integration)
- [X] T009 Create main FastAPI app in backend/main.py (app initialization, CORS)
- [X] T010 Create embedding script in backend/embed_book.py (document processing)
- [X] T011 Create ChatBot component base in src/components/ChatBot/ChatBot.jsx
- [X] T012 Create styling for ChatBot in src/components/ChatBot/ChatBot.css
- [X] T013 Create ChatBot export in src/components/ChatBot/index.js
- [X] T014 Create global integration in src/theme/Root.js
- [X] T015 [P] Create API endpoints in backend/main.py (health check, chat endpoint)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - General Q&A with Textbook Content (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask general questions about the textbook content and receive accurate responses with proper citations.

**Independent Test**: The system should accept a user question, retrieve relevant textbook content, and generate a response that directly addresses the question with proper attribution to the source material.

### Implementation for User Story 1

- [X] T016 [P] [US1] Implement vector store search functionality in backend/vector_store.py
- [X] T017 [P] [US1] Implement RAG pipeline in backend/main.py (embedding → search → prompt → LLM)
- [X] T018 [US1] Connect chat endpoint to RAG pipeline in backend/main.py
- [X] T019 [US1] Implement conversation history storage in backend/main.py
- [X] T020 [US1] Add proper source citation in response in backend/main.py
- [X] T021 [US1] Create basic chat UI in src/components/ChatBot/ChatBot.jsx
- [X] T022 [US1] Implement API call to backend in ChatBot component
- [X] T023 [US1] Add loading indicators to ChatBot UI
- [X] T024 [US1] Add floating chat button to ChatBot component
- [X] T025 [US1] Implement expandable chat window in ChatBot component

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Text Selection Q&A (Priority: P2)

**Goal**: Enable users to select text in the textbook and ask specific questions about that selection.

**Independent Test**: The system should detect text selection, offer to ask about the selection, and then provide answers specifically related to the selected text.

### Implementation for User Story 2

- [X] T026 [P] [US2] Implement text selection detection in src/components/ChatBot/ChatBot.jsx
- [X] T027 [US2] Add "Ask about selection" button to ChatBot component
- [X] T028 [US2] Pass selected text as context to backend in ChatBot component
- [X] T029 [US2] Modify RAG pipeline to consider context in backend/main.py
- [X] T030 [US2] Update chat endpoint to handle context parameter in backend/main.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Conversation History (Priority: P3)

**Goal**: Maintain conversation context across multiple questions to enable natural conversation.

**Independent Test**: The system should maintain conversation context and allow users to reference previous parts of the conversation in follow-up questions.

### Implementation for User Story 3

- [X] T031 [P] [US3] Implement conversation ID management in ChatBot component
- [X] T032 [US3] Update API calls to include conversation ID in ChatBot component
- [X] T033 [US3] Implement message history storage in database in backend/database.py
- [X] T034 [US3] Update backend to retrieve conversation history in backend/main.py
- [X] T035 [US3] Display conversation history in ChatBot UI

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T036 [P] Add error handling and user feedback to backend endpoints
- [X] T037 [P] Add comprehensive error handling to ChatBot frontend
- [X] T038 [P] Add performance monitoring and logging
- [X] T039 [P] Add rate limiting to API endpoints
- [X] T040 [P] Add accessibility features to ChatBot component
- [X] T041 [P] Optimize response times to meet 5-second requirement
- [X] T042 [P] Add mobile responsiveness to ChatBot UI
- [X] T043 [P] Run embedding script to populate vector store
- [X] T044 [P] Add tests for backend functionality
- [X] T045 [P] Add tests for frontend functionality
- [X] T046 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 components
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on US1 components

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence