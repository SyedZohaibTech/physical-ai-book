# Feature Specification: RAG Chatbot Implementation

**Feature Branch**: `001-rag-chatbot-spec`  
**Created**: 2025-12-16  
**Status**: Draft  
**Input**: User description: "Based on the constitution, create detailed technical specifications for the RAG chatbot implementation. Break down into specific components and modules. SPECIFICATIONS NEEDED: 1. BACKEND SPECIFICATIONS: File: chatbot-backend/main.py - FastAPI application setup - CORS middleware configuration - Route definitions with Pydantic models - Error handling middleware - Health check endpoint - Rate limiting implementation File: chatbot-backend/embeddings.py - Function: chunk_text(text, max_tokens=500) - Uses tiktoken to count tokens - Splits text into overlapping chunks - Returns list of chunks with metadata - Function: create_embeddings(chunks) - Calls OpenAI text-embedding-3-small - Batches requests (up to 100 chunks per batch) - Returns embeddings array - Function: setup_qdrant_collection() - Creates collection with cosine distance - Vector size: 1536 (embedding dimension) - Defines payload schema - Function: index_book_content(directory_path) - Recursively reads all .md files - Extracts frontmatter (title, description) - Processes each file into chunks - Stores in Qdrant with metadata File: chatbot-backend/rag_service.py - Class: RAGService - Method: get_answer(question, selected_text=None) - If selected_text: use as context directly - Else: generate embedding → search Qdrant → retrieve chunks - Format prompt with context - Call GPT-4o-mini - Parse and return response File: chatbot-backend/database.py - Setup Neon Postgres connection - Function: save_conversation(user_id, question, answer, sources) - Function: get_user_history(user_id, limit=10) File: chatbot-backend/requirements.txt - List all dependencies with versions File: chatbot-backend/.env.example - Template for environment variables 2. FRONTEND SPECIFICATIONS: File: src/components/ChatBot/index.tsx - State management: - messages: array of {role, content, timestamp} - input: string - loading: boolean - isExpanded: boolean - selectedText: string | null - Functions: - sendMessage(): handles user input submission - getSelectedText(): detects text selection on page - toggleChat(): expand/collapse chatbot - clearChat(): reset conversation - UI Components: - FloatingButton: bottom-right corner button - ChatContainer: expandable chat window - MessageList: scrollable messages area - InputBox: text input with send button - SourcesList: display retrieved sources File: src/components/ChatBot/styles.module.css - Responsive layout - Smooth animations - Dark/light mode support - Mobile-first design 3. INTEGRATION SPECIFICATIONS: File: docusaurus.config.js modifications - Add chatbot to theme config - Configure client modules for chatbot component File: src/theme/Root.tsx (or Root.js) - Wrap app with ChatBot component - Make it available globally 4. DEPLOYMENT SPECIFICATIONS: Backend (Railway): - Environment: Python 3.11 - Build command: pip install -r requirements.txt - Start command: uvicorn main:app --host 0.0.0.0 --port $PORT - Environment variables setup - Health check configuration Frontend: - Update BACKEND_URL in ChatBot component - Redeploy to GitHub Pages 5. TESTING SPECIFICATIONS: Backend Tests: - Test embedding generation - Test Qdrant search functionality - Test RAG pipeline end-to-end - Test selected text mode - Test API endpoints Frontend Tests: - Test text selection detection - Test message sending - Test UI responsiveness - Test error handling 6. INDEXING SPECIFICATIONS: Script: scripts/index_book_content.py - Read all markdown files from docs/ - Process each file: - Extract metadata - Split into chunks - Generate embeddings - Upload to Qdrant - Progress tracking - Error logging - Success confirmation OUTPUT FORMAT: Create specification.md with: - Detailed function signatures - Input/output examples - API request/response examples - Database query examples - Component props and types - CSS class naming conventions - File structure tree"

## User Scenarios & Testing (mandatory)

### User Story 1 - Ask Question (Priority: P1)

A user wants to ask a question about the textbook content and receive a relevant answer.

**Why this priority**: This is the core functionality of the chatbot and provides immediate value.

**Independent Test**: Can be fully tested by submitting a question and verifying the answer's relevance and source citations.

**Acceptance Scenarios**:

1.  **Given** the chatbot is accessible, **When** a user types a question and submits it, **Then** the chatbot displays a relevant answer.
2.  **Given** the chatbot provided an answer, **Then** the answer includes citations to the book sections used.
3.  **Given** the chatbot is accessible, **When** a user asks a question, **Then** the backend response time is less than 3 seconds.

---

### User Story 2 - Selected Text Context (Priority: P1)

A user wants the chatbot to answer a question based *only* on a specific text selection from the book page.

**Why this priority**: This enhances the chatbot's precision and user control, addressing a key requirement.

**Independent Test**: Can be fully tested by selecting text, asking a question, and verifying the answer only uses the selected text as context, with appropriate citations.

**Acceptance Scenarios**:

1.  **Given** a user has selected text on a Docusaurus page, **When** the user asks a question, **Then** the chatbot uses only the selected text as context to generate an answer.
2.  **Given** the chatbot answered based on selected text, **Then** the answer cites the selected text as the source.

---

### User Story 3 - Chat History (Priority: P2)

A user wants their past conversations to be stored and potentially influence future interactions.

**Why this priority**: This enables personalization and continuity, improving the user experience over time.

**Independent Test**: Can be tested by having a conversation, closing the chatbot, reopening it, and verifying the conversation history is present.

**Acceptance Scenarios**:

1.  **Given** a user has interacted with the chatbot, **When** the chatbot generates an answer, **Then** the conversation (question, answer, sources) is saved to the database.
2.  **Given** a user returns to the chatbot, **Then** their previous conversation history is loaded (if applicable).

---

### User Story 4 - Admin Content Indexing (Priority: P3)

An administrator needs to re-index the book content in the Qdrant database.

**Why this priority**: This is an administrative function, not directly user-facing, but critical for maintenance.

**Independent Test**: Can be tested by invoking the indexing endpoint and verifying that new content is processed and available for search.

**Acceptance Scenarios**:

1.  **Given** an authenticated administrator triggers the `/index` endpoint, **Then** the chatbot backend processes all `.md` files in the `docs/` directory.
2.  **Given** the indexing process completes, **Then** all processed content chunks are stored in Qdrant with correct metadata and embeddings.

---

### Edge Cases

-   What happens when a user's question is entirely outside the scope of the book content? (Chatbot should indicate it cannot answer effectively).
-   How does the system handle very long text selections? (Truncate to token limits or indicate the selection is too long).
-   What if the OpenAI API is unavailable or returns an error? (Graceful error handling, user-friendly message).
-   What if Qdrant returns no relevant chunks? (Graceful fallback, general answer if possible, or "cannot answer").
-   What if the database connection fails? (Log error, retry, or indicate temporary unavailability).

## Requirements (mandatory)

### Functional Requirements

-   **FR-001**: The chatbot backend MUST expose a `POST /chat` endpoint for user questions, accepting an optional `selected_text`.
-   **FR-002**: The chatbot backend MUST generate embeddings for user questions using OpenAI's `text-embedding-3-small`.
-   **FR-003**: The chatbot backend MUST search Qdrant for the top 3 most relevant book content chunks.
-   **FR-004**: The chatbot backend MUST construct a prompt for `gpt-4o-mini` including the user's question and retrieved context chunks.
-   **FR-005**: The chatbot backend MUST generate answers using OpenAI's `gpt-4o-mini`.
-   **FR-006**: The chatbot backend MUST store user questions, chatbot answers, and source citations in Neon Postgres.
-   **FR-007**: The chatbot backend MUST expose a `POST /index` endpoint (admin-only) to trigger content re-indexing.
-   **FR-008**: The chatbot backend MUST implement CORS restrictions to allow requests only from the Docusaurus domain.
-   **FR-009**: The chatbot backend MUST implement rate limiting (100 requests/hour per user).
-   **FR-010**: The chatbot frontend (React component) MUST detect user text selections on the Docusaurus page.
-   **FR-011**: The chatbot frontend MUST provide a floating button to expand/collapse the chat interface.
-   **FR-012**: The chatbot frontend MUST display conversation history.
-   **FR-013**: The chatbot backend MUST read all `.md` files from the `docs/` directory for indexing.
-   **FR-014**: The chatbot backend MUST split book content into chunks (max 500 tokens) and generate embeddings.
-   **FR-015**: The chatbot backend MUST store embeddings and metadata (chapter, section, file path) in Qdrant.

### Key Entities

-   **Conversation**: Represents a single exchange between a user and the chatbot, including question, answer, and sources.
-   **Text Chunk**: A segment of the book content with its embedding and associated metadata (file path, chapter, section, content preview).

## Success Criteria (mandatory)

### Measurable Outcomes

-   **SC-001**: 90% of user questions receive a relevant answer from the chatbot (verified by manual review of a sample set).
-   **SC-002**: Average response time for chatbot answers (POST /chat) is under 3 seconds.
-   **SC-003**: The chatbot accurately uses user-selected text as context for answers in 100% of test cases.
-   **SC-004**: The chat interface renders correctly and is fully functional on both desktop and mobile devices.
-   **SC-005**: All conversation history is correctly persisted and retrieved from the database.
-   **SC-006**: The content indexing process successfully processes all book markdown files and populates the vector database without errors.