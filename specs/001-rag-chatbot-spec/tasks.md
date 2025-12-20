# Tasks: RAG Chatbot Implementation

**Input**: Design documents from `/specs/001-rag-chatbot-spec/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create backend project structure in `backend/`
- [x] T002 Initialize Python virtual environment in `backend/venv`
- [x] T003 [P] Create `backend/requirements.txt` with initial dependencies
- [x] T004 [P] Create `.env.example` file in project root
- [x] T005 [P] Create `website/src/components/ChatBot` directory

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [x] T006 Implement `chunk_text`, `create_embeddings`, and `setup_qdrant_collection` functions in `backend/embeddings.py`
- [x] T007 Implement `index_book_content` function in `backend/embeddings.py`
- [x] T008 [P] Implement Neon Postgres connection and schema creation in `backend/database.py`
- [x] T009 [P] Create indexing script `scripts/index_book_content.py` to scan and process markdown files

---

## Phase 3: User Story 1 & 2 - Ask Question & Selected Text Context (Priority: P1) 🎯 MVP

**Goal**: Allow users to ask questions and get answers from the chatbot, with support for selected text context.

**Independent Test**: The chatbot can be opened, a question can be asked (with or without selected text), and a relevant answer with sources is returned.

### Implementation

- [x] T010 [US1] Implement `RAGService` class and `get_answer` method in `backend/rag_service.py` to handle both regular and selected text queries
- [x] T011 [US1] Implement FastAPI app, CORS, and `/chat` and `/health` endpoints in `backend/main.py`
- [x] T012 [P] [US1] Create ChatBot component skeleton in `website/src/components/ChatBot/index.tsx`
- [x] T013 [P] [US1] Implement basic styling for the floating button and chat window in `website/src/components/ChatBot/styles.module.css`
- [x] T014 [US1] Implement `sendMessage` and `getSelectedText` functions and API call logic in `website/src/components/ChatBot/index.tsx`
- [x] T015 [US1] Integrate ChatBot into Docusaurus via `website/src/theme/Root.tsx`

---

## Phase 4: User Story 3 - Chat History (Priority: P2)

**Goal**: Persist and retrieve user conversation history.

**Independent Test**: A user can have a conversation, refresh the page, and see their previous messages.

### Implementation

- [ ] T016 [US3] Implement `save_conversation` and `get_user_history` functions in `backend/database.py`
- [ ] T017 [US3] Update `get_answer` in `backend/rag_service.py` to save conversations
- [ ] T018 [US3] Update `ChatBot` component in `website/src/components/ChatBot/index.tsx` to load and display chat history

---

## Phase 5: User Story 4 - Admin Content Indexing (Priority: P3)

**Goal**: Allow an administrator to trigger a re-indexing of the book content.

**Independent Test**: An admin can make a request to the `/index` endpoint and verify that the content in Qdrant is updated.

### Implementation

- [ ] T019 [US4] Implement the `/index` endpoint in `backend/main.py`
- [ ] T020 [US4] Connect the `/index` endpoint to the `index_book_content` function

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T021 [P] Deploy backend to Railway/Render
- [ ] T022 [P] Update frontend with production backend URL and deploy to GitHub Pages
- [ ] T023 [P] End-to-end testing of all features
- [ ] T024 [P] Create `README.md` for backend and chatbot component

---

## Dependencies & Execution Order

- **Phase 1 & 2**: Must be completed before any user stories.
- **Phase 3 (US1 & US2)**: Can begin after Phase 2 is complete.
- **Phase 4 (US3)**: Depends on Phase 3.
- **Phase 5 (US4)**: Depends on Phase 2.
- **Phase 6**: Can begin after all desired user stories are implemented and tested.

## Implementation Strategy

The project will follow an incremental delivery model:
1.  **Foundation**: Complete Phases 1 & 2.
2.  **MVP**: Complete Phase 3 to deliver the core chatbot functionality.
3.  **Enhancements**: Complete Phases 4 and 5 to add chat history and admin features.
4.  **Launch**: Complete Phase 6 for deployment and final testing.
