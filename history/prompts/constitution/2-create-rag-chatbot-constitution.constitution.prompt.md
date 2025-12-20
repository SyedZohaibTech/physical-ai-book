---
id: 2
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: gemini
feature: robotics-textbook
branch: 001-robotics-modules
user: user
command: /sp.constitution Create a comprehensive constitution file for implementing a RAG (Retrieval-Augmented Generation) chatbot...
labels: [constitution, rag, chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
- "n/a"
---

## Prompt

/sp.constitution Create a comprehensive constitution file for implementing a RAG (Retrieval-Augmented Generation) chatbot that will be integrated into our Physical AI & Humanoid Robotics textbook Docusaurus website.

CONSTITUTION REQUIREMENTS:

Project Name: Physical AI Book RAG Chatbot
Project Goal: Build an intelligent chatbot that answers questions about the book content, with special support for answering questions based on user-selected text.

ARCHITECTURE:
1. Backend Stack:
   - FastAPI (Python web framework)
   - OpenAI API (embeddings via text-embedding-3-small, chat via gpt-4o-mini)
   - Qdrant Cloud (vector database for storing book embeddings)
   - Neon Serverless Postgres (for storing user queries and chat history)
   - Python libraries: openai, qdrant-client, psycopg2-binary, tiktoken, python-dotenv

2. Frontend Stack:
   - React/TypeScript component integrated into existing Docusaurus site
   - Fetch API for backend communication
   - CSS Modules for styling
   - Text selection detection API

3. Deployment:
   - Backend: Railway or Render (with environment variables)
   - Frontend: Already deployed on GitHub Pages with existing book

CORE FEATURES:
1. Question Answering: Users ask questions, chatbot retrieves relevant book sections and generates answers
2. Selected Text Mode: If user selects text on the page, chatbot answers based only on that selection
3. Source Citations: Show which book sections were used to generate the answer
4. Chat History: Store conversations in Postgres for future personalization
5. Responsive UI: Mobile-friendly chat interface

TECHNICAL REQUIREMENTS:
1. Book Content Processing:
   - Read all .md files from docs/ folder
   - Split content into chunks (max 500 tokens each)
   - Generate embeddings for each chunk
   - Store in Qdrant with metadata (chapter, section, file path)

2. RAG Pipeline:
   - User question → Generate embedding → Search Qdrant → Retrieve top 3 chunks
   - Pass chunks as context to GPT-4o-mini → Generate answer
   - Return answer + source references

3. API Endpoints:
   - POST /chat: Main chatbot endpoint
     - Input: { question: string, selected_text?: string }
     - Output: { answer: string, sources: array }
   - GET /health: Health check
   - POST /index: Re-index book content (admin only)

4. Security & Best Practices:
   - CORS configuration for Docusaurus domain
   - Environment variables for all API keys
   - Rate limiting (100 requests/hour per user)
   - Input validation and sanitization
   - Error handling with user-friendly messages

5. Database Schema (Neon Postgres):
   - conversations table: id, user_id, timestamp, question, answer, sources
   - embeddings_metadata table: chunk_id, file_path, chapter, content_preview

INTEGRATION WITH EXISTING BOOK:
- Chatbot component should be accessible from every page
- Floating chat button in bottom-right corner
- Can be expanded/collapsed
- Detects selected text automatically
- Should not interfere with existing book navigation

CONSTRAINTS:
- Backend response time < 3 seconds
- Embeddings generation: one-time process, not on every request
- Maximum 3 relevant chunks per query (to stay within GPT context limits)
- Selected text feature must work seamlessly
- Mobile responsive design mandatory

OUTPUT FORMAT:
Create a detailed constitution.md file that documents:
1. System architecture diagram (text-based)
2. Data flow diagrams
3. API specifications
4. Database schemas
5. Component hierarchy
6. Deployment strategy
7. Testing requirements
8. Success criteria

This constitution will guide all subsequent implementation steps.

## Response snapshot

I have successfully updated `.specify/memory/constitution.md` with the detailed specifications for the RAG chatbot. The new constitution is at version 1.0.0 and serves as the foundational document for this project.

As a next step, I will create a Prompt History Record (PHR) to document this action.

Suggested commit message:
`docs: amend constitution to v1.0.0 for RAG chatbot`

## Outcome

- ✅ Impact: Created a new project constitution for the RAG chatbot, overwriting the previous book-focused one.
- 🧪 Tests: No tests were run.
- 📁 Files: Overwrote `.specify/memory/constitution.md` with the new chatbot constitution.
- 🔁 Next prompts: Proceed with implementing the features outlined in the new constitution.
- 🧠 Reflection: The user provided a very detailed request that amounted to a new constitution, which required overwriting the existing one. The process was smooth.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (if applicable): None
