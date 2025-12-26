# Implementation Plan: Interactive RAG Chatbot for Physical AI Textbook

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-24 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The Interactive RAG Chatbot for Physical AI Textbook will implement a Retrieval-Augmented Generation (RAG) system that allows users to ask questions about the textbook content. The system will support two interaction modes: General Q&A (searching entire book) and Text Selection Q&A (answering specific questions about selected passages). The backend will use FastAPI with Qdrant vector store and Neon Postgres database, while the frontend will be a React component embedded in the Docusaurus site.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend
**Primary Dependencies**: FastAPI, uvicorn, openai, qdrant-client, psycopg2-binary, sqlalchemy, React
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (relational database)
**Testing**: pytest for backend, Jest/React Testing Library for frontend
**Target Platform**: Web application (Docusaurus documentation site)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: Total response time under 5 seconds (vector search <500ms, LLM response <3s)
**Constraints**: Must use OpenAI text-embedding-3-small for embeddings, OpenAI GPT-3.5-turbo for responses
**Scale/Scope**: Support concurrent users accessing textbook content via the chat interface

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Verification:
- **RAG Architecture-First**: Backend MUST implement RAG architecture retrieving content from vector DB before LLM generation
- **Dual Mode Functionality**: System MUST support General Q&A and Text Selection Q&A modes
- **Tech Stack Consistency**: Backend MUST use FastAPI with Python; vector DB MUST be Qdrant Cloud; SQL DB MUST be Neon Serverless Postgres; embeddings MUST use OpenAI text-embedding-3-small; LLM MUST use OpenAI GPT-3.5-turbo; Frontend MUST be React component integrated with Docusaurus
- **Embedded Experience**: Chatbot interface MUST be floating button in bottom-right corner that expands to chat window
- **Production-Ready Quality**: All components MUST include error handling, loading states, API rate limiting, and proper logging
- **Performance Optimization**: Response times MUST meet requirements: vector search <500ms, LLM response <3s, total <5 seconds

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # FastAPI app, CORS, endpoints
├── vector_store.py      # Qdrant operations
├── embeddings.py        # OpenAI embedding generation
├── database.py          # Neon Postgres connection
├── embed_book.py        # Script to embed all book content
├── requirements.txt     # Dependencies
└── .env.example         # Environment variables template

src/components/ChatBot/
├── ChatBot.jsx          # Main component
├── ChatBot.css          # Styling
└── index.js             # Export

src/theme/
└── Root.js              # Global wrapper to inject chatbot

website/
├── docusaurus.config.js # Docusaurus configuration
└── src/
    └── pages/           # Additional pages if needed
```

**Structure Decision**: Web application structure with separate backend and frontend components as specified in the user input. The backend uses FastAPI with Python, and the frontend integrates with the existing Docusaurus documentation site via a React component injected globally.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations identified. All constitutional requirements are met with the planned architecture.
