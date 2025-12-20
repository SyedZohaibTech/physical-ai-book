---
id: 1
title: Create RAG Chatbot Specification
stage: spec
date: 2025-12-16
surface: agent
model: gemini
feature: 001-rag-chatbot-spec
branch: 001-rag-chatbot-spec
user: user
command: /sp.specify Based on the constitution, create detailed technical specifications for the RAG chatbot implementation...
labels: [spec, rag-chatbot]
links:
  spec: specs/001-rag-chatbot-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/001-rag-chatbot-spec/spec.md
- specs/001-rag-chatbot-spec/checklists/requirements.md
tests:
- "n/a"
---

## Prompt

Based on the constitution, create detailed technical specifications for the RAG chatbot implementation. Break down into specific components and modules. SPECIFICATIONS NEEDED: 1. BACKEND SPECIFICATIONS: File: chatbot-backend/main.py - FastAPI application setup - CORS middleware configuration - Route definitions with Pydantic models - Error handling middleware - Health check endpoint - Rate limiting implementation File: chatbot-backend/embeddings.py - Function: chunk_text(text, max_tokens=500) - Uses tiktoken to count tokens - Splits text into overlapping chunks - Returns list of chunks with metadata - Function: create_embeddings(chunks) - Calls OpenAI text-embedding-3-small - Batches requests (up to 100 chunks per batch) - Returns embeddings array - Function: setup_qdrant_collection() - Creates collection with cosine distance - Vector size: 1536 (embedding dimension) - Defines payload schema - Function: index_book_content(directory_path) - Recursively reads all .md files - Extracts frontmatter (title, description) - Processes each file into chunks - Stores in Qdrant with metadata File: chatbot-backend/rag_service.py - Class: RAGService - Method: get_answer(question, selected_text=None) - If selected_text: use as context directly - Else: generate embedding → search Qdrant → retrieve chunks - Format prompt with context - Call GPT-4o-mini - Parse and return response File: chatbot-backend/database.py - Setup Neon Postgres connection - Function: save_conversation(user_id, question, answer, sources) - Function: get_user_history(user_id, limit=10) File: chatbot-backend/requirements.txt - List all dependencies with versions File: chatbot-backend/.env.example - Template for environment variables 2. FRONTEND SPECIFICATIONS: File: src/components/ChatBot/index.tsx - State management: - messages: array of {role, content, timestamp} - input: string - loading: boolean - isExpanded: boolean - selectedText: string | null - Functions: - sendMessage(): handles user input submission - getSelectedText(): detects text selection on page - toggleChat(): expand/collapse chatbot - clearChat(): reset conversation - UI Components: - FloatingButton: bottom-right corner button - ChatContainer: expandable chat window - MessageList: scrollable messages area - InputBox: text input with send button - SourcesList: display retrieved sources File: src/components/ChatBot/styles.module.css - Responsive layout - Smooth animations - Dark/light mode support - Mobile-first design 3. INTEGRATION SPECIFICATIONS: File: docusaurus.config.js modifications - Add chatbot to theme config - Configure client modules for chatbot component File: src/theme/Root.tsx (or Root.js) - Wrap app with ChatBot component - Make it available globally 4. DEPLOYMENT SPECIFICATIONS: Backend (Railway): - Environment: Python 3.11 - Build command: pip install -r requirements.txt - Start command: uvicorn main:app --host 0.0.0.0 --port $PORT - Environment variables setup - Health check configuration Frontend: - Update BACKEND_URL in ChatBot component - Redeploy to GitHub Pages 5. TESTING SPECIFICATIONS: Backend Tests: - Test embedding generation - Test Qdrant search functionality - Test RAG pipeline end-to-end - Test selected text mode - Test API endpoints Frontend Tests: - Test text selection detection - Test message sending - Test UI responsiveness - Test error handling 6. INDEXING SPECIFICATIONS: Script: scripts/index_book_content.py - Read all markdown files from docs/ - Process each file: - Extract metadata - Split into chunks - Generate embeddings - Upload to Qdrant - Progress tracking - Error logging - Success confirmation OUTPUT FORMAT: Create specification.md with: - Detailed function signatures - Input/output examples - API request/response examples - Database query examples - Component props and types - CSS class naming conventions - File structure tree

## Response snapshot

The feature specification for the RAG Chatbot is now complete and validated.

**Branch Name**: `001-rag-chatbot-spec`
**Spec File Path**: `specs/001-rag-chatbot-spec/spec.md`
**Checklist Path**: `specs/001-rag-chatbot-spec/checklists/requirements.md`

This feature is ready for the next phase.

## Outcome

- ✅ Impact: Created a detailed feature specification for the RAG chatbot, including user scenarios, functional requirements, key entities, and success criteria.
- 🧪 Tests: N/A
- 📁 Files: `specs/001-rag-chatbot-spec/spec.md`, `specs/001-rag-chatbot-spec/checklists/requirements.md`
- 🔁 Next prompts: The feature is ready for the planning phase (`/sp.plan`).
- 🧠 Reflection: Encountered issues with passing long strings to PowerShell script. Resolved by manually creating the branch and files and directly writing content. Successfully validated the spec against the checklist.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell argument parsing for long strings.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): investigate better ways to pass multi-line strings to PowerShell scripts from run_shell_command.
